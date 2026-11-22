"""The chiplet module's core model against the reference emulator and the RTL.

Stage 1 of design/chiplet-module.md: the C++ model in chiplet/ runs LM-1
code as the reference emulator (lm1.execute) runs it.  Three batches test
that, through the model's ctypes binding (lm1.chiplet), and a fourth,
chiplet_interface, tests the model's own C interface:

- chiplet_reference runs every program of the reference's own tests on
  both models.  Each test runs once with the reference's Emulator and
  Cluster and once with the chiplet's, so its own assertions hold on each,
  and then every model either run made is compared: registers and thread
  state, memory, the card table, the templates, the inline cache, the
  queues, the nursery and old-gen pointers, the collection count, and how
  each run ended.
- chiplet_rtl runs the RTL testbench's programs (tb/tests/*.hex) from the
  RTL's reset state on both models, compares the two, and checks the
  registers each program's .expected file gives.
- chiplet_streams runs seeded random instruction streams on both models,
  one thread stepped an instruction at a time, the threads of a tile in
  round-robin, and a cluster of tiles, and compares them after every call.
"""

from __future__ import annotations

import importlib
import io
import random
import threading
from pathlib import Path

import lm1.execute as reference
from lm1 import chiplet
from lm1.decode import Op, encode_b, encode_i, encode_r, encode_s, encode_x
from lm1.testing.harness import get_registry, test
from lm1.traps import LM1Trap
from lm1.word import NIL, T, WORD_MASK, make_header, make_ref, tag_fixnum

TESTS_DIR = Path(__file__).resolve().parent
REPO = TESTS_DIR.parents[1]

# The reference's own tests, which register as their modules load.
for _path in sorted(TESTS_DIR.glob("test_*.py")):
    if _path.stem != Path(__file__).stem:
        importlib.import_module("tests." + _path.stem)
_REFERENCE_ENTRIES = [e for e in get_registry() if e.func.__module__ != __name__]

# The comparisons patch the names Emulator and Cluster in a test's module,
# so they run one at a time.
_PATCH_LOCK = threading.Lock()


# ---------------------------------------------------------------------------
# State of a model, read the same way from either
# ---------------------------------------------------------------------------

def _outcome(exc: BaseException | None):
    """How a run ended: None, a trap's code, or an exception's type."""
    if exc is None:
        return None
    if isinstance(exc, LM1Trap):
        return ("trap", exc.code)
    return (type(exc).__name__,)


def _thread_state(t) -> dict:
    return {
        "regs": list(t.regs),
        "pc": t.pc & WORD_MASK,
        "cycle_count": t.cycle_count & WORD_MASK,
        "halted": bool(t.halted),
        "stalled": bool(t.stalled),
        "stall_queue": t.stall_queue,
        "trap_table_base": t.trap_table_base & WORD_MASK,
        "trap_cause": t.trap_cause,
        "trap_pc": t.trap_pc & WORD_MASK,
        "in_trap": bool(t.in_trap),
        "tile_id": t.tile_id,
        "thread_id": t.thread_id,
    }


_TILE_FIELDS = ("nursery_base", "nursery_size", "oldgen_base", "oldgen_size", "oldgen_ptr",
                "card_shift", "card_table_base", "gen_boundary", "card_table_size",
                "gc_count", "instruction_count", "tile_id", "_current_thread_idx")


def _memory(emu) -> bytes:
    if isinstance(emu, chiplet.Emulator):
        return emu.mem.tobytes()
    return emu.mem._data.tobytes()


def tile_state(emu, *, full: bool = True) -> dict:
    """What a tile holds.  Without FULL, only its threads and its fields."""
    state = {
        "threads": [_thread_state(t) for t in emu.threads],
        "fields": {name: getattr(emu, name) for name in _TILE_FIELDS},
        "slices": list(emu.nursery_slices),
    }
    if full:
        state["memory"] = _memory(emu)
        state["cards"] = bytes(emu.card_table)
        state["templates"] = list(emu.thread.header_templates)
        state["ic"] = sorted(emu.ic_table.items())
        state["queues"] = [list(q) for q in emu.queues]
    return state


def _first_difference(path: str, a, b) -> str | None:
    if isinstance(a, dict) and isinstance(b, dict):
        for key in a:
            if key not in b:
                return "%s.%s: only on the reference" % (path, key)
            found = _first_difference("%s.%s" % (path, key), a[key], b[key])
            if found:
                return found
        for key in b:
            if key not in a:
                return "%s.%s: only on the chiplet model" % (path, key)
        return None
    if isinstance(a, bytes) and isinstance(b, bytes):
        if a == b:
            return None
        if len(a) != len(b):
            return "%s: %d bytes against %d" % (path, len(a), len(b))
        at = next(i for i in range(len(a)) if a[i] != b[i])
        word = at & ~7
        return "%s: first difference at byte %#x, word %#x against %#x" % (
            path, at, int.from_bytes(a[word:word + 8], "little"),
            int.from_bytes(b[word:word + 8], "little"))
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return "%s: %d items against %d" % (path, len(a), len(b))
        for i, (x, y) in enumerate(zip(a, b)):
            found = _first_difference("%s[%d]" % (path, i), x, y)
            if found:
                return found
        return None
    if a != b:
        show = lambda v: "%#x" % v if isinstance(v, int) and not isinstance(v, bool) else repr(v)
        return "%s: reference %s, chiplet %s" % (path, show(a), show(b))
    return None


def assert_same(ref_state, chip_state, what: str) -> None:
    found = _first_difference(what, ref_state, chip_state)
    assert found is None, found


# ---------------------------------------------------------------------------
# Batch chiplet_reference: the reference's own tests on both models
# ---------------------------------------------------------------------------

def _recording(emulator_base, cluster_base, made: list):
    """Emulator and Cluster classes that note each model a test makes and
    how each of its runs ends.  Only models made on this thread count, as
    a run of every batch at once may run another test meanwhile."""
    owner = threading.get_ident()

    def recorded(method):
        # Only the test's own calls count: the reference's run loops call
        # _gc_collect themselves, where the chiplet model collects inside.
        def run(self, *args, **kwargs):
            outer = not self._inside
            self._inside = True
            try:
                result = method(self, *args, **kwargs)
            except BaseException as exc:
                if outer:
                    self._outcomes.append(_outcome(exc))
                raise
            finally:
                if outer:
                    self._inside = False
            if outer:
                self._outcomes.append(None)
            return result
        return run

    class Emulator(emulator_base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._outcomes = []
            self._inside = False
            if threading.get_ident() == owner:
                made.append(self)

        run = recorded(emulator_base.run)
        run_round_robin = recorded(emulator_base.run_round_robin)
        _gc_collect = recorded(emulator_base._gc_collect)

    class Cluster(cluster_base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._outcomes = []
            self._inside = False
            if threading.get_ident() == owner:
                made.append(self)

        run = recorded(cluster_base.run)

    return Emulator, Cluster


def _models_state(made: list) -> list:
    states = []
    for model in made:
        tiles = model.tiles if hasattr(model, "tiles") else [model]
        states.append({"outcomes": list(model._outcomes),
                       "tiles": [tile_state(t) for t in tiles]})
    return states


def _run_on(entry, emulator_base, cluster_base) -> tuple[list, BaseException | None]:
    made: list = []
    emulator, cluster = _recording(emulator_base, cluster_base, made)
    names = entry.func.__globals__
    saved = {name: names[name] for name in ("Emulator", "Cluster") if name in names}
    names["Emulator"], names["Cluster"] = emulator, cluster
    try:
        entry.func()
        failure = None
    except BaseException as exc:
        failure = exc
    finally:
        for name in ("Emulator", "Cluster"):
            if name in saved:
                names[name] = saved[name]
            else:
                names.pop(name, None)
    return _models_state(made), failure


def _reference_program(entry):
    def run():
        with _PATCH_LOCK:
            ref_states, ref_failure = _run_on(entry, reference.Emulator, reference.Cluster)
            chip_states, chip_failure = _run_on(entry, chiplet.Emulator, chiplet.Cluster)
        if ref_failure is not None:
            raise AssertionError("fails on the reference: %r" % ref_failure) from ref_failure
        if chip_failure is not None:
            raise AssertionError("fails on the chiplet model: %r" % chip_failure) from chip_failure
        assert len(ref_states) == len(chip_states), \
            "%d models on the reference, %d on the chiplet" % (len(ref_states), len(chip_states))
        for i, (a, b) in enumerate(zip(ref_states, chip_states)):
            assert_same(a, b, "model %d" % i)
    run.__doc__ = "%s (batch %s) on both models." % (entry.name, entry.batch)
    return run


for _entry in _REFERENCE_ENTRIES:
    test("ref.%s.%s" % (_entry.batch, _entry.name), batch="chiplet_reference",
         timeout=120.0)(_reference_program(_entry))


# ---------------------------------------------------------------------------
# Batch chiplet_rtl: the RTL testbench's programs
# ---------------------------------------------------------------------------

TB_TESTS = REPO / "tb" / "tests"
RTL_MAX_INSTRUCTIONS = 100_000   # the bench gives each program 100,000 cycles

# Programs whose bench expectations rest on an RTL deviation from the spec,
# with the entry of docs/test_gap_ideas.md that records it.  The two models
# still run them and must agree on them.
RTL_DEVIATIONS = {
    "35_send_recv": "P0-9: the RTL and the assembler take SEND's and RECV's queue "
                    "from the immediate; the spec and both models read a descriptor "
                    "register",
}


def _rtl_reset(emu, words: list[int]) -> None:
    """The bench's state when reset ends: the image at address 0, every
    register 0, the template table empty, and the barrier off until a
    program sets the generation boundary."""
    emu.mem.load_words(0, words)
    for t in emu.threads:
        for r in range(32):
            t.regs[r] = 0
        t.pc = 0
    for i in range(256):
        emu.thread.header_templates[i] = 0
    emu.gen_boundary = 0


def _rtl_program(hex_path: Path):
    def run():
        words = [int(line, 16) for line in hex_path.read_text().split()]
        expected = {}
        for line in hex_path.with_suffix(".expected").read_text().split():
            reg, value = line.split("=")
            expected[int(reg[1:])] = int(value, 16)
        states = []
        for kind in (reference.Emulator, chiplet.Emulator):
            emu = kind(mem_size=len(words) * 8, stdout=io.StringIO())
            _rtl_reset(emu, words)
            try:
                emu.run(max_instructions=RTL_MAX_INSTRUCTIONS)
                outcome = None
            except (LM1Trap, IndexError, RuntimeError) as exc:
                outcome = _outcome(exc)
            states.append({"outcome": outcome, "tile": tile_state(emu)})
        assert_same(states[0], states[1], hex_path.stem)
        if hex_path.stem in RTL_DEVIATIONS:
            return
        state = states[1]
        assert state["outcome"] is None, "the program ended with %r" % (state["outcome"],)
        thread = state["tile"]["threads"][0]
        assert thread["halted"], "the program did not halt"
        for reg, value in sorted(expected.items()):
            assert thread["regs"][reg] == value, \
                "r%d = %#x, the bench expects %#x" % (reg, thread["regs"][reg], value)
    run.__doc__ = "RTL bench program %s on both models." % hex_path.stem
    return run


for _hex in sorted(TB_TESTS.glob("*.hex")):
    test("rtl.%s" % _hex.stem, batch="chiplet_rtl", timeout=120.0)(_rtl_program(_hex))


@test("rtl.deviations_still_differ", batch="chiplet_rtl")
def test_rtl_deviations_still_differ():
    """Each program set aside in RTL_DEVIATIONS still misses its bench
    registers on the models, so the list shrinks when a deviation is
    settled."""
    for name in RTL_DEVIATIONS:
        hex_path = TB_TESTS / (name + ".hex")
        words = [int(line, 16) for line in hex_path.read_text().split()]
        emu = chiplet.Emulator(mem_size=len(words) * 8, stdout=io.StringIO())
        _rtl_reset(emu, words)
        try:
            emu.run(max_instructions=RTL_MAX_INSTRUCTIONS)
        except LM1Trap:
            continue
        expected = dict(line.split("=") for line in
                        hex_path.with_suffix(".expected").read_text().split())
        regs = emu.thread.regs
        if all(regs[int(r[1:])] == int(v, 16) for r, v in expected.items()):
            raise AssertionError("%s now meets its bench registers" % name)


# ---------------------------------------------------------------------------
# Batch chiplet_streams: seeded random instruction streams
# ---------------------------------------------------------------------------

# The streams' memory, 64 KiB: code, data, stack, trap table, card table,
# nursery and old-gen.  The stack scan of a collection runs from SP up to
# the nursery, over the trap and card tables too.
S_MEM = 0x10000
S_DATA = 0x1000          # data the base registers point into, to 0x5000
S_STACK = 0x6000         # the stack grows down from here
S_TRAP_TABLE = 0x6000    # 256 entries
S_CARDS = 0x6800         # one byte per 64-byte card
S_HANDLER = 0x0F00       # the trap handler's code
S_NURSERY = 0x8000
S_OLDGEN, S_OLDGEN_SIZE = 0xA000, 0x6000
S_CODE_WORDS = 0x0F00 // 4

# Register roles, so that operations mostly meet operands they accept:
# base pointers into the data, objects, fixnums, code addresses, a queue
# descriptor, and anything at all.
R_BASE = (1, 2, 3, 4)
R_OBJ = tuple(range(5, 13))
R_FIX = tuple(range(13, 21))
R_CODE = (21, 22)
R_QUEUE = 23
R_ANY = (24, 31)

_SAFE_TRAPS = (0x80, 0x81, 0x82, 0x83, 0x84, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96,
               0x42, 0x10, 0x03)
_OPS = [  # (opcode, weight)
    (Op.TST, 3), (Op.TST_SHAPE, 2), (Op.ARITH_FIX, 6), (Op.ADD_FIX_IMM, 4),
    (Op.CMP_TAGGED, 3), (Op.LDW, 2), (Op.STW, 2), (Op.ALLOC, 4), (Op.ALLOC_CONS, 8),
    (Op.ALLOCV, 3), (Op.ALLOC_CLOSURE, 2), (Op.LD, 4), (Op.LD_CAR_CDR, 4), (Op.ST, 3),
    (Op.ST_WB, 5), (Op.ST_CAR_CDR, 5), (Op.CALL_IC, 1), (Op.IC_INSTALL, 1),
    (Op.CALL_DIRECT, 1), (Op.CALL_CLOSURE, 1), (Op.RET, 1), (Op.TAILCALL_IC, 1),
    (Op.TAILCALL_DIR, 1), (Op.JR, 1), (Op.PREFETCH_REF, 1), (Op.SEND, 2), (Op.RECV, 2),
    (Op.CAS_TAGGED, 2), (Op.FAA_FENCE, 2), (Op.LDB, 2), (Op.STB, 2), (Op.LDH, 2),
    (Op.STH, 2), (Op.ARITH_RAW, 5), (Op.BITWISE, 5), (Op.LDR, 4), (Op.STR, 4),
    (Op.BR, 2), (Op.BR_COND, 4), (Op.PUSH_POP, 3), (Op.LI, 5), (Op.LUI, 1),
    (Op.PUSH_MULTI, 1), (Op.POP_MULTI, 1), (Op.LI32, 1), (Op.TRAP, 3), (Op.ERET, 0.3),
    (Op.SYS_INFO, 2), (Op.HALT_NOP, 1), (7, 0.15), (40, 0.15),
]
_OP_CHOICES = [op for op, _ in _OPS]
_OP_WEIGHTS = [w for _, w in _OPS]
# The collector's mix: allocation, barriered stores into old and young
# objects, and the loads that follow the refs, so that collections come
# often and find old-to-young refs in dirty cards.
_COLLECTOR_WEIGHTS = [w * {Op.ALLOC_CONS: 4, Op.ALLOC: 2, Op.ALLOCV: 2, Op.ST_WB: 4,
                           Op.ST_CAR_CDR: 4, Op.CAS_TAGGED: 2, Op.LD: 2,
                           Op.LD_CAR_CDR: 2}.get(op, 1) for op, w in _OPS]

# Trap handlers: one skips the instruction that trapped and goes on, one
# retries it after changing a register.
_SKIP_HANDLER = [encode_i(Op.SYS_INFO, 24, 4, 0),             # r24 = trap_pc
                 encode_i(Op.ADD_FIX_IMM, 24, 24, 4),          # past the instruction
                 encode_i(Op.JR, 24, 0, 0)]
_RETRY_HANDLER = [encode_i(Op.LI, 24, 0, 0x77), encode_x(Op.ERET, 0)]
_HANDLED = (0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x10, 0x20, 0x30, 0x42)


class _Stream:
    """One seeded program and the state both models start from."""

    def __init__(self, seed: int, *, threads: int = 1, tiles: int = 1,
                 collector: bool = False):
        rng = self.rng = random.Random(seed)
        self.threads, self.tiles = threads, tiles
        self.collector = collector
        self.weights = _COLLECTOR_WEIGHTS if collector else _OP_WEIGHTS
        # Small nurseries, so that allocation overflows them and collects.
        self.nursery_size = rng.choice((0x180, 0x300, 0x600)) * threads
        self.code = [self.instruction() for _ in range(rng.randrange(16, S_CODE_WORDS))]
        self.handler = "skip" if collector else rng.choices(("skip", "retry", None), (8, 1, 1))[0]
        # A small old-gen now and then, which promotions fill.
        self.oldgen_size = rng.choice((S_OLDGEN_SIZE, S_OLDGEN_SIZE, S_OLDGEN_SIZE, 0x400))
        self.card_table = collector or rng.random() < 0.5
        self.stdin = "".join(chr(rng.randrange(32, 127)) for _ in range(8))
        self.build_seed = rng.getrandbits(32)

    # -- values and fields --

    def value(self, rng: random.Random, objects: list[int]) -> int:
        kind = rng.randrange(12)
        if kind < 2:
            return tag_fixnum(rng.randrange(-16, 16)) & WORD_MASK
        if kind == 2:
            return tag_fixnum(rng.choice((1, -1)) * ((1 << 62) - rng.randrange(1, 4))) & WORD_MASK
        if kind == 3:
            return rng.choice((NIL, T, 0x35 | (rng.randrange(128) << 8)))
        if kind == 4:
            return S_DATA + 8 * rng.randrange(0x800)
        if kind in (5, 6) and objects:
            return rng.choice(objects)
        if kind == 7:
            return make_ref(S_NURSERY + 8 * rng.randrange(self.nursery_size // 8),
                            cons=rng.random() < 0.5)
        if kind == 8:
            return rng.getrandbits(64)
        if kind == 9:
            return 4 * rng.randrange(S_CODE_WORDS)
        return rng.choice((0, 1, 2, 8, 64, 0xFF))

    def pick(self, role) -> int:
        """A register of ROLE, or now and then any register at all."""
        if self.rng.random() < 0.03:
            return self.rng.randrange(32)
        return self.rng.choice(role)

    def offset(self) -> int:
        rng = self.rng
        return (8 * rng.randrange(-4, 16) if rng.random() < 0.85 else rng.randrange(-64, 64)) & 0xFFFF

    # -- instructions --

    def instruction(self) -> int:
        rng, pick = self.rng, self.pick
        op = rng.choices(_OP_CHOICES, self.weights)[0]
        # The collector's streams store objects more often than anything else.
        value = pick(R_OBJ) if self.collector and rng.random() < 0.7 else pick(R_OBJ + R_FIX + R_ANY)
        if op == Op.ARITH_FIX:
            func = rng.randrange(4) if rng.random() < 0.95 else rng.randrange(32)
            return encode_r(op, pick(R_FIX), pick(R_FIX), pick(R_FIX), func)
        if op in (Op.ARITH_RAW, Op.BITWISE):
            func = rng.randrange(7) if rng.random() < 0.95 else rng.randrange(32)
            return encode_r(op, pick(R_FIX + R_ANY), pick(R_FIX + R_ANY), pick(R_FIX + R_ANY), func)
        if op == Op.CMP_TAGGED:
            return encode_r(op, pick(R_ANY), pick(R_FIX + R_OBJ), pick(R_FIX + R_OBJ),
                            rng.randrange(3))
        if op in (Op.LDR, Op.LDB, Op.LDH, Op.LDW):
            return encode_i(op, pick(R_FIX + R_ANY + R_OBJ), pick(R_BASE), self.offset())
        if op in (Op.STR, Op.STB, Op.STH, Op.STW):
            return encode_i(op, pick(R_BASE), value, self.offset())
        if op in (Op.LD, Op.LD_CAR_CDR):
            return encode_i(op, pick(R_OBJ + R_FIX + R_ANY), pick(R_OBJ), rng.randrange(6))
        if op in (Op.TST, Op.TST_SHAPE):
            return encode_i(op, pick(R_ANY), value, rng.choice((0, 1, 2, 3, 4, 5, 6, 7, 42)))
        if op in (Op.ST, Op.ST_WB, Op.ST_CAR_CDR):
            return encode_s(op, pick(R_OBJ), value, rng.randrange(4) if rng.random() < 0.9 else rng.randrange(32))
        if op == Op.ADD_FIX_IMM:
            return encode_i(op, pick(R_FIX), pick(R_FIX), tag_fixnum(rng.randrange(-100, 100)) & 0xFFFF)
        if op == Op.LI:
            imm = tag_fixnum(rng.randrange(-50, 50)) if rng.random() < 0.6 else rng.getrandbits(16)
            return encode_i(op, pick(R_FIX + R_ANY), 0, imm & 0xFFFF)
        if op == Op.LUI:
            return encode_i(op, pick(R_ANY), 0, rng.getrandbits(16))
        if op == Op.LI32:
            return encode_i(op, pick(R_ANY), 0, 0)   # the next word is its immediate
        if op == Op.ALLOC:
            words = rng.randrange(6) if rng.random() < 0.8 else rng.randrange(32)
            return encode_x(op, (pick(R_OBJ) << 21) | (words << 16) | rng.choice((0, 1, 2, 3, 300)))
        if op == Op.ALLOC_CONS:
            return encode_r(op, pick(R_OBJ), value if rng.random() < 0.7 else 0,
                            pick(R_OBJ + R_FIX) if rng.random() < 0.7 else 0)
        if op == Op.ALLOCV:
            return encode_x(op, (pick(R_OBJ) << 21) | (pick(R_FIX) << 16) | rng.choice((0, 1, 2, 5)))
        if op == Op.ALLOC_CLOSURE:
            return encode_x(op, (pick(R_OBJ) << 21) | (pick(R_CODE) << 16) | (rng.randrange(4) << 11))
        if op in (Op.BR, Op.CALL_DIRECT, Op.TAILCALL_DIR):
            return encode_b(op, 0, 0, rng.randrange(-6, 8) & 0xFFFF)
        if op == Op.BR_COND:
            return encode_b(op, pick(R_FIX + R_ANY), rng.randrange(7), rng.randrange(-6, 8) & 0xFFFF)
        if op == Op.PUSH_POP:
            return encode_r(op, pick(R_OBJ + R_FIX + R_ANY), 0, 0, rng.randrange(3))
        if op in (Op.PUSH_MULTI, Op.POP_MULTI):
            return encode_i(op, rng.choice((0, 0, 0, 1, 1, 1, 2)), 0, rng.getrandbits(16) & 0x1FE0)
        if op == Op.TRAP:
            return encode_x(op, rng.choice(_SAFE_TRAPS))
        if op == Op.SYS_INFO:
            return encode_i(op, pick(R_ANY + R_FIX), rng.randrange(16), 0)
        if op == Op.HALT_NOP:
            return encode_x(op, 0 if rng.random() < 0.2 else 1 << 21)
        if op == Op.SEND:
            return encode_s(op, pick((R_QUEUE,)), value, 0)
        if op == Op.RECV:
            func = 0 if rng.random() < 0.3 else rng.choice(R_ANY)
            return encode_r(op, pick(R_FIX + R_ANY), pick((R_QUEUE,)), 0, func)
        if op == Op.CAS_TAGGED:
            return encode_r(op, pick(R_ANY), pick(R_BASE + R_OBJ), pick(R_FIX + R_OBJ), value)
        if op == Op.FAA_FENCE:
            return encode_r(op, pick(R_FIX), pick(R_BASE + R_OBJ), pick(R_FIX),
                            0 if rng.random() < 0.8 else 0x1F)
        if op in (Op.CALL_IC, Op.TAILCALL_IC, Op.CALL_CLOSURE):
            return encode_i(op, pick(R_OBJ), 0, 0)
        if op == Op.JR:
            return encode_i(op, pick(R_CODE), 0, 0)
        if op == Op.IC_INSTALL:
            return encode_r(op, pick(R_CODE), pick(R_OBJ), pick(R_CODE))
        return encode_r(op, pick(R_ANY), pick(R_ANY), pick(R_ANY), rng.randrange(32))

    # -- the models --

    def build(self, kind):
        """A model of either kind, in the stream's starting state."""
        emulator, cluster = kind
        out = io.StringIO()
        layout = dict(mem_size=S_MEM, nursery_base=S_NURSERY, nursery_size=self.nursery_size,
                      oldgen_base=S_OLDGEN, oldgen_size=self.oldgen_size)
        if self.tiles > 1:
            made = cluster(n_tiles=self.tiles, num_threads_per_tile=self.threads, **layout)
            tiles = made.tiles
            for t in tiles:
                t._stdout, t._stdin = out, io.StringIO(self.stdin)
        else:
            made = emulator(stdout=out, stdin=io.StringIO(self.stdin), num_threads=self.threads,
                            **layout)
            tiles = [made]
        rng = random.Random(self.build_seed)
        for tile in tiles:
            self._fill(tile, rng)
        return made, out

    def _fill(self, tile, rng: random.Random) -> None:
        tile.mem.load_instructions(0, self.code)
        if self.handler:
            tile.mem.load_instructions(
                S_HANDLER, _SKIP_HANDLER if self.handler == "skip" else _RETRY_HANDLER)
        templates = tile.thread.header_templates
        templates[3] = make_header(0, 0, 42)
        # Objects an earlier collection left in old-gen, and young ones in
        # each thread's slice of the nursery.
        objects: list[int] = []

        def place(addr: int, sub: int, size: int, shape: int) -> int:
            tile.mem.store_word(addr, make_header(sub, size, shape))
            for i in range(size):
                tile.mem.store_word(addr + 8 * (i + 1), self.value(rng, objects))
            objects.append(make_ref(addr, cons=sub == 1))
            return addr + 8 * (size + 1)

        addr = tile.oldgen_ptr
        for sub, size, shape in ((1, 2, 0), (2, 3, 7), (1, 2, 0), (0, 1, 42), (2, 5, 7)):
            addr = place(addr, sub, size, shape)
        tile.mem.store_word(addr, make_header(4, 2, 1))   # a closure: code entry, one slot
        tile.mem.store_word(addr + 8, 4 * rng.randrange(len(self.code)))
        tile.mem.store_word(addr + 16, NIL)
        objects.append(make_ref(addr))
        tile.oldgen_ptr = addr + 24
        for t in tile.threads:
            young = t.np
            for sub, size, shape in ((1, 2, 0), (2, 1, 7)):
                young = place(young, sub, size, shape)
            t.np = young
        for i in range(S_DATA, S_DATA + 0x200, 8):
            tile.mem.store_word(i, self.value(rng, objects))
        for _ in range(3):   # inline-cache entries some call sites may hit
            tile.ic_table[(4 * rng.randrange(len(self.code)), rng.choice((0, 7, 42)))] = \
                4 * rng.randrange(len(self.code))
        if self.card_table:
            tile.card_table_base = S_CARDS
        for tid, t in enumerate(tile.threads):
            regs = t.regs
            for r in R_BASE:
                regs[r] = S_DATA + 0x400 * (r - 1)
            for r in R_OBJ:
                regs[r] = rng.choice(objects) if rng.random() < 0.85 else self.value(rng, objects)
            for r in R_FIX:
                regs[r] = (tag_fixnum(rng.randrange(-20, 20)) & WORD_MASK
                           if rng.random() < 0.85 else self.value(rng, objects))
            for r in R_CODE:
                regs[r] = 4 * rng.randrange(len(self.code))
            regs[R_QUEUE] = tag_fixnum((rng.choice(range(self.tiles)) << 2) | rng.randrange(4)) \
                if rng.random() < 0.9 else self.value(rng, objects)
            for r in R_ANY:
                regs[r] = self.value(rng, objects)
            t.pc = 4 * ((tid * 37) % len(self.code))
            t.sp = S_STACK - 0x200 * tid
            if self.handler:
                t.trap_table_base = S_TRAP_TABLE
        if self.handler:
            # A handler for TRAP_NURSERY_OVERFLOW only now and then, since
            # one takes the trap from the collection that stands in for an
            # absent runtime.
            for code in _HANDLED:
                if rng.random() < (0.15 if code == 0x10 else 1.0 if self.collector else 0.8):
                    tile.mem.store_word(S_TRAP_TABLE + code * 8, S_HANDLER)


class _GraphStream(_Stream):
    """A straight-line program that builds a graph of objects of known sizes,
    links them through every field, the last ones above all, and allocates
    garbage between the links, so that collections copy objects with young
    refs in all their fields and find old ones in dirty cards."""

    def __init__(self, seed: int):
        rng = self.rng = random.Random(seed)
        self.threads, self.tiles, self.collector = 1, 1, True
        self.nursery_size = rng.choice((0x200, 0x400))
        self.oldgen_size = S_OLDGEN_SIZE
        self.handler = None
        self.card_table = rng.random() < 0.7
        self.stdin = ""
        self.build_seed = rng.getrandbits(32)
        code: list[int] = []
        sizes: dict[int, int] = {}   # register -> payload words of its object
        objects = list(R_OBJ)

        def alloc(r: int) -> None:
            kind = rng.randrange(4)
            if kind == 0:
                code.append(encode_r(Op.ALLOC_CONS, r, 0, 0))
                sizes[r] = 2
            elif kind == 1:
                n = rng.randrange(1, 7)
                code.append(encode_x(Op.ALLOC, (r << 21) | (n << 16) | rng.randrange(4)))
                sizes[r] = n
            elif kind == 2:
                n = rng.randrange(0, 6)
                code.append(encode_i(Op.LI, 20, 0, tag_fixnum(n) & 0xFFFF))
                code.append(encode_x(Op.ALLOCV, (r << 21) | (20 << 16) | 2))
                sizes[r] = n + 1
            else:
                env = rng.randrange(4)
                code.append(encode_x(Op.ALLOC_CLOSURE, (r << 21) | (21 << 16) | (env << 11)))
                sizes[r] = 1 + env

        def link() -> None:
            a, b = rng.choice(objects), rng.choice(objects)
            n = sizes[a]
            field = n - 1 if rng.random() < 0.5 else rng.randrange(n)
            op = rng.choice((Op.ST_WB, Op.ST_WB, Op.ST_WB, Op.ST))
            code.append(encode_s(op, a, b, field))

        for r in objects:
            alloc(r)
        for _ in range(12):
            link()
        for _ in range(rng.randrange(150, 250)):
            x = rng.random()
            if x < 0.4:   # garbage
                if rng.random() < 0.5:
                    code.append(encode_r(Op.ALLOC_CONS, 24, 0, 0))
                else:
                    code.append(encode_x(Op.ALLOC, (24 << 21) | (rng.randrange(1, 6) << 16)))
            elif x < 0.55:
                alloc(rng.choice(objects))
            elif x < 0.85:
                link()
            else:
                a = rng.choice(objects)
                code.append(encode_i(Op.LD, 23, a, rng.randrange(sizes[a])))
        code.append(encode_x(Op.HALT_NOP, 0))
        self.code = code


def _call(fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
        return None
    except (LM1Trap, IndexError, RuntimeError, ValueError) as exc:
        return _outcome(exc)


def _state(made, out, *, full: bool):
    tiles = made.tiles if hasattr(made, "tiles") else [made]
    return {"tiles": [tile_state(t, full=full) for t in tiles], "output": out.getvalue()}


def _stream_seeds(first: int, count: int, *, threads: int, tiles: int, steps: int,
                  collector: bool = False, graph: bool = False):
    def run():
        for seed in range(first, first + count):
            stream = (_GraphStream(seed) if graph else
                      _Stream(seed, threads=threads, tiles=tiles, collector=collector))
            ref, ref_out = stream.build((reference.Emulator, reference.Cluster))
            chip, chip_out = stream.build((chiplet.Emulator, chiplet.Cluster))
            calls = random.Random(seed * 7919)
            what = "seed %d" % seed
            assert_same(_state(ref, ref_out, full=True), _state(chip, chip_out, full=True),
                        what + " at the start")
            for call in range(steps):
                if tiles > 1:
                    budget, quanta = calls.randrange(1, 12), calls.randrange(1, 4)
                    outcomes = [_call(m.run, max_instructions=budget, quanta=quanta)
                                for m in (ref, chip)]
                elif threads > 1:
                    budget, quanta = calls.randrange(1, 12), calls.randrange(1, 4)
                    outcomes = [_call(m.run_round_robin, max_instructions=budget, quanta=quanta)
                                for m in (ref, chip)]
                else:
                    outcomes = [_call(m.run, max_instructions=1) for m in (ref, chip)]
                full = call % 16 == 15 or outcomes[0] is not None
                where = "%s after call %d" % (what, call)
                assert outcomes[0] == outcomes[1], "%s: the reference ended %r, the chiplet %r" % (
                    where, outcomes[0], outcomes[1])
                assert_same(_state(ref, ref_out, full=full), _state(chip, chip_out, full=full),
                            where)
                if outcomes[0] is not None:
                    break
                ended = all(t.halted or t.stalled
                            for tile in (ref.tiles if tiles > 1 else [ref]) for t in tile.threads)
                if ended:
                    break
            assert_same(_state(ref, ref_out, full=True), _state(chip, chip_out, full=True),
                        what + " at the end")
    return run


for _first in range(0, 400, 25):
    test("streams.thread.%03d-%03d" % (_first, _first + 24), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 25, threads=1, tiles=1, steps=600))
for _first in range(1000, 1120, 20):
    test("streams.round_robin.%04d-%04d" % (_first, _first + 19), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 20, threads=3, tiles=1, steps=60))
for _first in range(2000, 2080, 20):
    test("streams.cluster.%04d-%04d" % (_first, _first + 19), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 20, threads=2, tiles=2, steps=60))
for _first in range(3000, 3200, 25):
    test("streams.collector.%04d-%04d" % (_first, _first + 24), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 25, threads=1, tiles=1, steps=600,
                                      collector=True))
for _first in range(5000, 5100, 25):
    test("streams.graph.%04d-%04d" % (_first, _first + 24), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 25, threads=1, tiles=1, steps=2000, graph=True))
for _first in range(4000, 4060, 20):
    test("streams.collector_threads.%04d-%04d" % (_first, _first + 19), batch="chiplet_streams",
         timeout=300.0)(_stream_seeds(_first, 20, threads=3, tiles=1, steps=80,
                                      collector=True))


@test("streams.library_matches_sources", batch="chiplet_streams")
def test_library_matches_sources():
    """The loaded library carries the digest of the checkout's sources."""
    lib = chiplet.library()
    assert lib.lm1c_source_digest().decode() == chiplet.source_digest()
    assert chiplet.built_digest() == chiplet.source_digest()


# ---------------------------------------------------------------------------
# Batch chiplet_interface: the model's own C interface
# ---------------------------------------------------------------------------

@test("interface.defaults_are_the_references", batch="chiplet_interface")
def test_defaults_are_the_references():
    """A model made from an empty configuration has the reference
    emulator's sizes and layout (decision E of design/chiplet-module.md)."""
    model = chiplet.Model("")
    ref = reference.Emulator()
    assert model.tiles == 1
    assert model.tile_get(0, chiplet.TILE_THREADS) == len(ref.threads)
    assert model.tile_get(0, chiplet.TILE_MEM_SIZE) == ref.mem.size
    for field, value in ((chiplet.TILE_NURSERY_BASE, ref.nursery_base),
                         (chiplet.TILE_NURSERY_SIZE, ref.nursery_size),
                         (chiplet.TILE_OLDGEN_BASE, ref.oldgen_base),
                         (chiplet.TILE_OLDGEN_SIZE, ref.oldgen_size),
                         (chiplet.TILE_OLDGEN_PTR, ref.oldgen_ptr),
                         (chiplet.TILE_CARD_SHIFT, ref.card_shift),
                         (chiplet.TILE_GEN_BOUNDARY, ref.gen_boundary),
                         (chiplet.TILE_CARD_TABLE_SIZE, ref.card_table_size)):
        assert model.tile_get(0, field) == value, (field, model.tile_get(0, field), value)
    assert [model.template(0, i) for i in range(256)] == list(ref.thread.header_templates)
    assert model.regs(0, 0) == list(ref.thread.regs)


@test("interface.sizes_come_from_the_configuration", batch="chiplet_interface")
def test_sizes_come_from_the_configuration():
    """Tiles, threads, memory, the layout and the queues' depth come from
    the configuration string, with K, M and G for binary units."""
    model = chiplet.Model("tiles=3, threads=4, mem=1M, tile_id=0x10, nursery_base=64K,"
                          "nursery_size=0x800, oldgen_base=0x20000, oldgen_size=128K,"
                          "card_shift=7, queue_depth=2")
    assert model.tiles == 3
    for tile in range(3):
        assert model.tile_get(tile, chiplet.TILE_ID) == 0x10 + tile
        assert model.tile_get(tile, chiplet.TILE_THREADS) == 4
        assert model.tile_get(tile, chiplet.TILE_MEM_SIZE) == 1 << 20
        assert model.tile_get(tile, chiplet.TILE_CARD_TABLE_SIZE) == (1 << 20) >> 7
        assert model.tile_get(tile, chiplet.TILE_GEN_BOUNDARY) == 0x10800
        # Each thread's slice of the nursery, the last to its end.
        slices = [model.thread_get(tile, t, chiplet.THREAD_SLICE_BASE) for t in range(4)]
        assert slices == [0x10000, 0x10200, 0x10400, 0x10600]
        assert model.reg(tile, 3, 25) == 0x10800
    for value in (1, 2, 3):   # a queue of depth 2 drops its oldest on a third push
        model.queue_push(0, 1, value)
    assert model.queue(0, 1) == [2, 3]


@test("interface.configurations_it_refuses", batch="chiplet_interface")
def test_configurations_it_refuses():
    """A configuration the model cannot hold is refused with its reason."""
    for config, reason in (("speed=3", "unknown configuration key"),
                           ("tiles", "not key=value"),
                           ("mem=12x", "takes a number"),
                           ("mem=-8", "takes a number"),
                           ("mem=12", "8-byte words"),
                           ("tiles=0", "at least one tile"),
                           ("threads=0", "at least one thread"),
                           ("card_shift=64", "at most 63"),
                           ("queue_depth=0", "at least one message"),
                           ("nursery_base=0x80000,oldgen_base=0x30000", "old-gen lies below")):
        try:
            chiplet.Model(config)
        except ValueError as exc:
            assert reason in str(exc), (config, str(exc))
        else:
            raise AssertionError("%r was accepted" % config)


@test("interface.host_callbacks_are_optional", batch="chiplet_interface")
def test_host_callbacks_are_optional():
    """Without a console the console traps stop the run as a host failure,
    and without a block device TRAP 0x82 answers fixnum -1 in r0."""
    model = chiplet.Model("mem=64K")
    code = b"".join(w.to_bytes(4, "little") for w in (
        encode_x(Op.TRAP, 0x82), encode_x(Op.TRAP, 0x80), encode_x(Op.HALT_NOP, 0)))
    model.write(0, 0, code)
    result = model.run(0)
    assert result.stop == chiplet.STOP_HOST and result.executed == 1
    assert model.reg(0, 0, 0) == tag_fixnum(-1) & WORD_MASK
    assert model.thread_get(0, 0, chiplet.THREAD_PC) == 4


@test("interface.bad_indices_are_refused", batch="chiplet_interface")
def test_bad_indices_are_refused():
    """A tile, thread, register, template or memory range the model lacks
    is refused rather than read or written."""
    model = chiplet.Model("tiles=2,threads=2,mem=64K")
    for call in (lambda: model.tile_get(2, chiplet.TILE_ID),
                 lambda: model.thread_get(0, 2, chiplet.THREAD_PC),
                 lambda: model.reg(0, 0, 32),
                 lambda: model.template(0, 256),
                 lambda: model.read(0, 64 * 1024 - 4, 8),
                 lambda: model.tile_set(0, chiplet.TILE_MEM_SIZE, 8),
                 lambda: model.tile_set(0, chiplet.TILE_CARD_SHIFT, 64),
                 lambda: model.run_round_robin(0, 10, 0)):
        try:
            call()
        except ValueError:
            pass
        else:
            raise AssertionError("an index the model lacks was taken")
