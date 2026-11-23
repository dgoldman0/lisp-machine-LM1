"""LM-1's end of the link: the chiplet module as MegaPad attaches it.

Stage 2 of design/chiplet-module.md.  The library's mp64_chiplet_create
makes the model a MegaPad chiplet module (MegaPad's
shared/accel/chiplet_module.h), and these tests drive it through the
binding's Module as MegaPad's port bridge drives one, against a behavioural
MegaPad written here:

- chiplet_link runs the link conformance cases of tb/link/link_cases.txt,
  one test per case: the one list that LM-1's RTL link benches
  (lm1_link_tb, lm1_megapad_link_tb) run too, through tb/link/link_cases.svh;
- chiplet_module tests what the module does beyond that list: the ranges
  and the start its stand-in for LM-1's runtime provides, LM-1 code at both
  ends of the doorbells, the boot fetch's failures, the step budget and the
  schedule, the stop, reset, and save and restore.

The tests that attach the module to MegaPad's own emulator and simulator
are MegaPad test files, as design/chiplet-module.md § 4 places them.
"""

from __future__ import annotations

import hashlib
import shutil
import tempfile
from pathlib import Path

from lm1 import chiplet
from lm1.decode import Op, encode_b, encode_i, encode_r, encode_s, encode_x
from lm1.testing.harness import test
from lm1.word import NIL, T, WORD_MASK, tag_fixnum

REPO = Path(__file__).resolve().parents[2]
CASES = REPO / "tb" / "link" / "link_cases.txt"

READ, WRITE = 1, 2
OK, REFUSED, MEM_ERROR = chiplet.LINK_OK, chiplet.LINK_REFUSED, chiplet.LINK_MEM_ERROR
CODES = {"ok": OK, "refused": REFUSED, "mem_error": MEM_ERROR}
# Units of data and of zeros that TACC.STORE sends for each format.
FORMATS = {"u8": (4, 0), "fp32": (2, 2), "fp64": (1, 3)}

# The behavioural MegaPad's map: it lends LM-1 [LENT, LENT_LIMIT) for
# reading, the image lies inside, and UNLENT lies outside.
LENT, LENT_LIMIT = 0x1000, 0x3000
IMAGE, UNLENT = 0x1200, 0x9000
LOAD_BASE = 0x100
BUDGET = 64          # the port bridge's default budget per step


# ---------------------------------------------------------------------------
# A behavioural MegaPad
# ---------------------------------------------------------------------------

class Megapad:
    """MegaPad's side of the link as its port bridge models it: memory it
    lends for reading or writing, each request admitted whole or refused,
    the doorbells LM-1 rings, the sideband, and the port's register state
    that decides when the module is reset, released, stopped and stepped."""

    def __init__(self, config: str = "", *, lent=((LENT, LENT_LIMIT, READ),)):
        self.memory: dict[int, int] = {}      # by word address
        self.failing: set[int] = set()        # words external memory fails
        self.lent = list(lent)
        self.requests: list[tuple[int, int, int, bool, int]] = []
        self.rung: list[int] = []
        self.ready = self.stopped = False
        self.module = chiplet.Module(config, mem=self._mem, doorbell=self.rung.append,
                                     status=self._status)
        self.model = self.module.model
        self.released = False
        self.stop_req = False
        self.pending: set[int] = set()        # sources whose doorbells wait

    # -- the host services the module calls --

    def _mem(self, addr, size, length, wen, words):
        burst = length > 0
        aligned = (size == 3 and addr % 8 == 0) if burst else addr % (1 << size) == 0
        span = 8 * (length + 1) if burst else 1 << size
        right = WRITE if wen else READ
        admitted = aligned and any(rights & right and base <= addr and addr + span <= limit
                                   for base, limit, rights in self.lent)
        code = OK if admitted else REFUSED
        self.requests.append((addr, size, length, wen, code))
        if not admitted:
            return REFUSED, 0
        for k in range(length + 1):
            at = (addr & ~7) + 8 * k
            if at in self.failing:
                # The bridge ends the burst at the failing word: the words
                # before it moved, and a read gives zeros from it on.
                for rest in range(k, length + 1):
                    if not wen:
                        words[rest] = 0
                self.requests[-1] = (addr, size, length, wen, MEM_ERROR)
                return MEM_ERROR, k
            if not wen:
                words[k] = self.memory.get(at, 0)
            elif burst or size == 3:
                self.memory[at] = words[k]
            else:   # a sub-word write takes its data from the low bits
                shift, mask = 8 * (addr & 7), (1 << (8 << size)) - 1
                old = self.memory.get(at, 0)
                self.memory[at] = (old & ~(mask << shift)) | ((words[k] & mask) << shift)
        return OK, length + 1

    def _status(self, ready, stopped):
        if self.released:
            self.ready, self.stopped = ready, stopped

    # -- the port bridge's side --

    def reset(self) -> None:
        self.module.reset()
        self.released = False
        self.ready = self.stopped = False

    def release(self, boot_addr: int, boot_len: int, boot_arg: int) -> None:
        self.module.release(boot_addr, boot_len, boot_arg)
        self.released = True
        if self.stop_req:
            self.module.stop(True)

    def stop(self, flag: bool) -> None:
        self.stop_req = flag
        if self.released:
            self.module.stop(flag)

    def ring(self, src: int) -> None:
        self.pending.add(src)

    def step(self, budget: int = BUDGET) -> bool:
        """One step as the port takes it: the waiting doorbells offered,
        lowest source first, then the module's work."""
        if not self.released:
            return False
        for src in sorted(self.pending):
            if not self.module.doorbell(src):
                break
            self.pending.discard(src)
        more = self.module.step(budget)
        if self.module.host_error is not None:
            raise self.module.host_error
        return more

    def settle(self, limit: int = 10_000) -> int:
        """Steps until the module rests; how many it took."""
        for count in range(1, limit + 1):
            if not self.step():
                return count
        raise AssertionError("the module never came to rest")

    def put(self, addr: int, *words: int) -> None:
        for k, word in enumerate(words):
            self.memory[addr + 8 * k] = word & WORD_MASK

    def lm1_word(self, addr: int) -> int:
        return int.from_bytes(self.model.read(chiplet.LINK_TILE, addr, 8), "little")


def image_word(seed: int, w: int) -> int:
    """Word w of the image a case's store sends (tb/link/link_cases.txt)."""
    return (seed & 0xFF) << 56 | (w & 0xFF) << 48 | 0x7ACC << 32 | (w * 7 + 1)


def image_units(seed: int, units: int) -> bytes:
    return b"".join(image_word(seed, w).to_bytes(8, "little") for w in range(8 * units))


def program_words(instructions: list[int]) -> list[int]:
    """Instructions packed two to a word, the first in the low half."""
    padded = instructions + [0] * (len(instructions) % 2)
    return [padded[i] | padded[i + 1] << 32 for i in range(0, len(padded), 2)]


# ---------------------------------------------------------------------------
# Batch chiplet_link: the shared conformance cases
# ---------------------------------------------------------------------------

def read_cases(path: Path = CASES) -> list[tuple[str, str, list[tuple[int, list[str]]]]]:
    """The cases of the list, each its name, its text and its operations
    with their line numbers."""
    cases: list[tuple[str, str, list[tuple[int, list[str]]]]] = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        words = line.split()
        if words[0] == "case":
            cases.append((words[1], " ".join(words[2:]), []))
        else:
            assert cases, "%s:%d: an operation before the first case" % (path.name, number)
            cases[-1][2].append((number, words))
    return cases


class CaseRun:
    """One case's operations on the module and the behavioural MegaPad."""

    def __init__(self) -> None:
        self.mp = Megapad("load_base=%#x" % LOAD_BASE)
        self.mark = 0          # requests made before the release
        self.before: dict[int, int] = {}
        self.taken_in = 0      # doorbells LM-1 has taken from queue 0
        self.seen_out = 0      # doorbells MegaPad has taken

    def snapshot(self, first: int, count: int) -> None:
        for k in range(count):
            self.before[first + 8 * k] = self.mp.lm1_word(first + 8 * k)

    def held(self, addr: int) -> bool:
        return self.mp.lm1_word(addr) == self.before[addr]

    def do(self, op: str, args: list[str]) -> None:
        mp, n = self.mp, [int(a, 16) for a in args if a not in CODES and a not in FORMATS
                          and a not in ("done", "failed")]
        if op == "reset":
            mp.reset()
            mp.stop_req = False
            self.taken_in = 0
        elif op == "identity":
            got = mp.module.identity(n[0])
            assert got == n[1], "identity word %d reads %#x" % (n[0], got)
        elif op == "image":
            mp.put(IMAGE, *[n[1] + k for k in range(n[0])])
        elif op in ("release", "release_unlent"):
            self.snapshot(LOAD_BASE, 64)
            self.mark = len(mp.requests)
            mp.release(IMAGE if op == "release" else UNLENT, n[0], n[1])
        elif op == "boot":
            for _ in range(1000):
                if mp.model.link(chiplet.LINK_BOOT) in (chiplet.BOOT_RUNNING, chiplet.BOOT_FAILED):
                    break
                mp.step()
            want = chiplet.BOOT_RUNNING if args[0] == "done" else chiplet.BOOT_FAILED
            assert mp.model.link(chiplet.LINK_BOOT) == want, \
                "the fetch ended in state %d" % mp.model.link(chiplet.LINK_BOOT)
        elif op == "boot_arg":
            # What the sideband gave; the stand-in puts it in thread 0's r1
            # when it starts an image (batch chiplet_module checks that).
            got = mp.model.link(chiplet.LINK_BOOT_ARG)
            assert got == n[0], "the boot argument is %#x" % got
        elif op == "loaded":
            for k in range(n[0]):
                got = mp.lm1_word(LOAD_BASE + 8 * k)
                assert got == n[1] + k, "word %d at the load base is %#x" % (k, got)
            assert self.held(LOAD_BASE + 8 * n[0]), "the word after the image changed"
        elif op == "requests":
            made = len(mp.requests) - self.mark
            assert made == n[0], "the fetch made %d requests" % made
        elif op == "wait":
            for _ in range(4):
                mp.step()
        elif op in ("runtime_ready", "runtime_stop"):
            mp.step()   # the module's stand-in for the runtime acts by itself
        elif op in ("ready", "stopped"):
            # A few steps rather than rest: the module starts a case's image
            # once it is in, and the image is data the benches never run.
            for _ in range(4):
                mp.step()
            got = mp.ready if op == "ready" else mp.stopped
            assert got == bool(n[0]), "MegaPad reads %s as %d" % (op.upper(), got)
        elif op == "stop":
            mp.stop(bool(n[0]))
        elif op == "lend":
            code = mp.model.lend(n[0], n[1], n[2], bool(n[3]))
            # The table ignores an index past it, as the RTL's lend port does.
            assert code == 0 or n[0] >= mp.model.link(chiplet.LINK_RANGES), code
        elif op == "store":
            units, zeros = FORMATS[args[0]]
            addr, seed, want = n[0], n[1], CODES[args[3]]
            self.snapshot(addr - 8, 34)
            got = mp.module.lent_write(addr, image_units(seed, units), zeros)
            assert got == want, "LM-1 answered %d" % got
            assert self.held(addr - 8) and self.held(addr + 256), "a word beside the image changed"
            for w in range(32):
                word = mp.lm1_word(addr + 8 * w)
                if want == OK:
                    expected = image_word(seed, w) if w < 8 * units else 0
                    assert word == expected, "word %d of the image is %#x" % (w, word)
                elif want == REFUSED:
                    assert self.held(addr + 8 * w), "a refused write changed word %d" % w
        elif op == "ring_in":
            mp.ring(n[0])
            mp.step()
        elif op == "rung_in":
            mp.settle()
            items = mp.model.queue(chiplet.LINK_TILE, chiplet.DOORBELL_IN_QUEUE)
            assert len(items) > self.taken_in, "no doorbell reached LM-1"
            assert items[self.taken_in] == tag_fixnum(n[0]), \
                "LM-1's doorbell came from %#x" % items[self.taken_in]
            self.taken_in += 1
        elif op == "ring_out":
            # What LM-1 code's SEND to the link's queue does.
            mp.model.queue_push(chiplet.LINK_TILE, chiplet.DOORBELL_OUT_QUEUE, tag_fixnum(n[0]))
            mp.step()
        elif op == "rung_out":
            mp.settle()
            assert len(mp.rung) > self.seen_out, "no doorbell left LM-1"
            assert mp.rung[self.seen_out] == n[0], "LM-1 rang %d" % mp.rung[self.seen_out]
            self.seen_out += 1
        elif op == "no_ring_out":
            assert len(mp.rung) == self.seen_out, "LM-1 rang %d" % mp.rung[self.seen_out]
        else:
            raise AssertionError("no such operation %r" % op)


def _case_test(name: str, text: str, ops: list[tuple[int, list[str]]]):
    def run():
        # Cases run in order on one machine in the RTL benches; here each
        # case runs on a module of its own, and every case begins with a
        # reset, so the two agree.
        state = CaseRun()
        for number, (op, *args) in ops:
            try:
                state.do(op, args)
            except AssertionError as exc:
                raise AssertionError("%s:%d %s %s: %s" % (
                    CASES.name, number, op, " ".join(args), exc)) from None
    run.__doc__ = text
    return run


for _name, _text, _ops in read_cases():
    test("link.%s" % _name, batch="chiplet_link")(_case_test(_name, _text, _ops))


@test("link.every_case_begins_with_a_reset", batch="chiplet_link")
def test_every_case_begins_with_a_reset():
    """The benches run the cases in order on one machine, so each case must
    begin from reset to mean the same there as on a fresh module here."""
    cases = read_cases()
    assert len(cases) >= 12
    for name, text, ops in cases:
        assert text, name
        assert ops and ops[0][1][0] == "reset", name


# ---------------------------------------------------------------------------
# Batch chiplet_module: the module beyond the shared cases
# ---------------------------------------------------------------------------

def _echo_program() -> list[int]:
    """Ring participant 9 once, then ring back each participant that rings."""
    return program_words([
        encode_i(Op.LI, 2, 0, tag_fixnum(chiplet.DOORBELL_OUT_QUEUE) & 0xFFFF),
        encode_i(Op.LI, 4, 0, tag_fixnum(chiplet.DOORBELL_IN_QUEUE) & 0xFFFF),
        encode_i(Op.LI, 3, 0, tag_fixnum(9) & 0xFFFF),
        encode_s(Op.SEND, 2, 3, 0),
        encode_i(Op.RECV, 5, 4, 0),     # loop: a doorbell's source
        encode_s(Op.SEND, 2, 5, 0),     # ring it back
        encode_b(Op.BR, 0, 0, -2),
    ])


def _booted(config: str, words: list[int], *, boot_arg: int = 0, lent=None) -> Megapad:
    mp = Megapad(config, **({} if lent is None else {"lent": lent}))
    mp.put(IMAGE, *words)
    mp.release(IMAGE, 8 * len(words), boot_arg)
    mp.settle()
    assert mp.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_RUNNING
    return mp


@test("module.interface_and_identity", batch="chiplet_module")
def test_interface_and_identity():
    """The module reports version 2 of MegaPad's interface, fills every
    function, and answers its identity record in reset and running alike."""
    mp = Megapad()
    module = mp.module._module
    assert module.abi == chiplet.CHIPLET_ABI == chiplet.library().lm1c_chiplet_abi()
    for name, _ in chiplet._ChipletModule._fields_[2:]:
        assert getattr(module, name), name
    words = [mp.module.identity(i) for i in range(4)]
    assert words == [0x4348_4950_0000_0001, int.from_bytes(b"LM1", "little"), 0, 0]
    assert mp.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_RESET
    assert not mp.module.step(BUDGET), "a module held in reset does no work"
    assert mp.requests == [] and not mp.module.doorbell(0)
    mp.release(IMAGE, 0, 0)
    mp.settle()
    assert [mp.module.identity(i) for i in range(4)] == words


@test("module.stand_in_lends_and_starts", batch="chiplet_module")
def test_stand_in_lends_and_starts():
    """Once the image is in, the runtime's stand-in lends the configured
    ranges and starts thread 0 at the load base with the boot argument in
    r1; before that, and after a reset, nothing is lent."""
    config = "lend=0x10000:4K,lend=0x20000:0x100,lend_ranges=3,load_base=0x400"
    mp = Megapad(config)
    words = program_words([encode_x(Op.HALT_NOP, 0)])
    mp.put(IMAGE, *words)
    assert mp.model.link(chiplet.LINK_RANGES) == 3
    mp.release(IMAGE, 8, 0x0010_5000)
    assert mp.module.lent_write(0x10000, bytes(64)) == REFUSED, "lent before the fetch"
    mp.settle()
    assert mp.model.lend_range(0) == (0x10000, 0x11000, True)
    assert mp.model.lend_range(1) == (0x20000, 0x20100, True)
    assert mp.model.lend_range(2) == (0, 0, False)
    assert mp.model.reg(0, 0, 1) == 0x0010_5000
    assert mp.model.thread_get(0, 0, chiplet.THREAD_HALTED)       # it ran its HALT
    assert mp.model.thread_get(0, 0, chiplet.THREAD_PC) == 0x404
    assert mp.module.lent_write(0x10FC0, image_units(1, 1)) == OK
    assert mp.module.lent_write(0x10FC0, image_units(1, 2)) == REFUSED, "past the range"
    assert mp.module.lent_write(0x200C0, image_units(1, 1)) == OK
    mp.reset()
    assert mp.model.lend_range(0) == (0, 0, False)
    assert mp.lm1_word(0x10FC0) == 0, "reset clears LM-1's memory"
    for config, reason in (("lend=0x10000", "BASE:LENGTH"),
                           ("lend=1:1,lend=2:2,lend=3:3", "more ranges to lend"),
                           ("lend=0x3FFFFF:2", "outside the first tile's memory"),
                           ("lend=0x1000:0", "empty"),
                           ("burst_words=257", "1 to 256"),
                           ("lend_ranges=0", "1 to 255"),
                           ("load_base=4", "multiple of 8"),
                           ("quanta=0", "at least one instruction")):
        try:
            chiplet.Module(config)
        except ValueError as exc:
            assert reason in str(exc), (config, str(exc))
        else:
            raise AssertionError("%r was accepted" % config)


@test("module.lm1_code_rings_and_answers", batch="chiplet_module")
def test_lm1_code_rings_and_answers():
    """LM-1 code rings MegaPad through queue 1 and takes MegaPad's doorbells
    from queue 0: a booted program rings once, then rings back whoever
    rings, and waits in RECV without using its budget meanwhile."""
    mp = _booted("", _echo_program())
    assert mp.rung == [9] and mp.ready
    executed = mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT)
    assert not mp.step(), "a thread waiting in RECV leaves no work"
    assert mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT) == executed
    for src in (0, 3, 2):
        mp.ring(src)
    mp.settle()
    assert mp.rung == [9, 0, 2, 3], "sources are offered lowest first"
    assert mp.model.link(chiplet.LINK_DOORBELLS_IN) == 3
    assert mp.model.link(chiplet.LINK_DOORBELLS_OUT) == 4


@test("module.doorbells_wait_for_room", batch="chiplet_module")
def test_doorbells_wait_for_room():
    """A doorbell that finds queue 0 full stays pending at MegaPad, and a
    word sent to the link that names no participant is dropped."""
    mp = Megapad("queue_depth=2")
    mp.release(IMAGE, 0, 0)
    mp.settle()
    assert mp.module.doorbell(1) and mp.module.doorbell(2)
    assert not mp.module.doorbell(3), "a full queue declines"
    for word in (NIL, tag_fixnum(256), tag_fixnum(-1), tag_fixnum(7)):
        mp.model.queue_push(0, chiplet.DOORBELL_OUT_QUEUE, word)
        mp.settle()
    assert mp.rung == [7]
    assert mp.model.link(chiplet.LINK_DOORBELLS_DROPPED) == 3


@test("module.fetch_failures", batch="chiplet_module")
def test_fetch_failures():
    """A burst that fails part way loads the words MegaPad answered before
    it and fails the fetch; so does an image that runs past LM-1's memory,
    after the words that fit.  A failed fetch raises no READY and lends
    nothing."""
    mp = Megapad("load_base=0x100,lend=0x1000:0x100")
    mp.put(IMAGE, *range(1, 20))
    mp.failing.add(IMAGE + 8 * 10)       # word 10, the second burst's third
    mp.release(IMAGE, 19 * 8, 0)
    mp.settle()
    assert mp.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_FAILED
    assert [code for *_, code in mp.requests] == [OK, MEM_ERROR]
    assert mp.model.link(chiplet.LINK_FETCHED) == 10
    assert [mp.lm1_word(0x100 + 8 * k) for k in range(11)] == list(range(1, 11)) + [0]
    assert not mp.ready and mp.module.lent_write(0x1000, bytes(64)) == REFUSED

    near_end = 4 * 1024 * 1024 - 16
    mp = Megapad("load_base=%#x" % near_end)
    mp.put(IMAGE, 11, 22, 33, 44)
    mp.release(IMAGE, 32, 0)
    mp.settle()
    assert mp.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_FAILED
    assert mp.model.link(chiplet.LINK_FETCHED) == 2
    assert [mp.lm1_word(near_end), mp.lm1_word(near_end + 8)] == [11, 22]


@test("module.lent_write_past_memory", batch="chiplet_module")
def test_lent_write_past_memory():
    """A range lent past the end of LM-1's memory admits a write there, and
    the write fails with MEM_ERROR after the words that fit, as the RTL's
    memory write port fails a word beyond its memory."""
    mp = Megapad("mem=64K")
    mp.release(IMAGE, 0, 0)
    mp.settle()
    assert mp.model.lend(0, 0xFF00, 0x10100, True) == 0
    assert mp.module.lent_write(0xFF40, image_units(5, 4)) == MEM_ERROR
    assert [mp.lm1_word(0xFF40 + 8 * w) for w in range(24)] == \
        [image_word(5, w) for w in range(24)]
    assert mp.module.lent_write(0xFFC0, image_units(6, 1), 3) == MEM_ERROR
    assert mp.lm1_word(0xFFF8) == image_word(6, 7)
    assert mp.module.lent_write(0x10000, image_units(6, 1)) == MEM_ERROR
    assert mp.model.lend(2, 0, 0x100, True) != 0, "the table has two entries"
    mp.reset()
    assert mp.model.lend(0, 0, 0x100, True) != 0, "the lend port takes nothing in reset"


def _counting_program() -> list[int]:
    """Count r10 up in a loop, storing it at 0x2000, then halt at 200."""
    return program_words([
        encode_i(Op.LI, 10, 0, 0),
        encode_i(Op.LI, 11, 0, 0x2000),
        encode_i(Op.LI, 12, 0, tag_fixnum(-200) & 0xFFFF),
        encode_i(Op.ADD_FIX_IMM, 10, 10, tag_fixnum(1)),      # loop
        encode_s(Op.STR, 11, 10, 0),
        encode_r(Op.ARITH_FIX, 13, 10, 12),
        encode_b(Op.BR_COND, 13, 3, 2),                       # BR.FIX.EQ r13: done
        encode_b(Op.BR, 0, 0, -4),
        encode_x(Op.HALT_NOP, 0),
    ])


@test("module.budget_counts_instructions_and_requests", batch="chiplet_module")
def test_budget_counts_instructions_and_requests():
    """A step does at most its budget of LM-1's units: one memory request
    of the fetch, or one instruction."""
    words = _counting_program() + [0] * 20      # 25 words: four requests of eight
    mp = Megapad()
    mp.put(IMAGE, *words)
    mp.release(IMAGE, 8 * len(words), 0)
    assert mp.step(3) and len(mp.requests) == 3
    assert mp.step(1) and len(mp.requests) == 4
    assert mp.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_RUNNING
    count = lambda: mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT)
    mp.step(5)
    assert count() == 5
    mp.step(0)
    assert count() == 5
    mp.settle()
    assert mp.lm1_word(0x2000) == tag_fixnum(200)


def _logging_program(rounds: int) -> list[int]:
    """Each thread that runs it takes ROUNDS slots of a log with FAA on the
    count at 0x2000 and writes its thread ID into each, so the log records
    the order in which the threads ran."""
    return program_words([
        encode_i(Op.LI, 11, 0, 0x2000),                       # the count
        encode_i(Op.LI, 15, 0, 0x2008),                       # the log
        encode_i(Op.LI, 12, 0, tag_fixnum(1) & 0xFFFF),
        encode_i(Op.LI, 10, 0, tag_fixnum(rounds) & 0xFFFF),
        encode_i(Op.SYS_INFO, 14, 1, 0),                      # the thread ID
        encode_r(Op.FAA_FENCE, 13, 11, 12),                   # loop: a slot
        encode_r(Op.ARITH_RAW, 16, 13, 13),                   # 4 × the slot
        encode_r(Op.ARITH_RAW, 16, 16, 16),                   # 8 × the slot
        encode_r(Op.ARITH_RAW, 16, 16, 15),
        encode_s(Op.STR, 16, 14, 0),
        encode_i(Op.ADD_FIX_IMM, 10, 10, tag_fixnum(-1) & 0xFFFF),
        encode_b(Op.BR_COND, 10, 4, -6),                      # BR.FIX.GT r10: loop
        encode_x(Op.HALT_NOP, 0),
    ])


@test("module.schedule_ignores_the_split", batch="chiplet_module")
def test_schedule_ignores_the_split():
    """The scheduler keeps its place between steps, so LM-1 computes the
    same whatever budget each step brings: three threads whose order of
    running a log records end with the same log and the same state."""
    words = _logging_program(30)

    def run(budgets):
        mp = Megapad("threads=3,quanta=5")
        mp.put(IMAGE, *words)
        mp.release(IMAGE, 8 * len(words), 0)
        mp.step(1)     # one request fetches the image and starts thread 0 ...
        for tid in (1, 2):   # ... and two more threads run it from the start
            mp.model.thread_set(0, tid, chiplet.THREAD_PC, 0)
            mp.model.thread_set(0, tid, chiplet.THREAD_HALTED, 0)
        assert mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT) == 0
        for i in range(100_000):
            if not mp.step(budgets[i % len(budgets)]):
                break
        log = [mp.lm1_word(0x2008 + 8 * k) for k in range(90)]
        return mp.module.save(), log

    whole = run([1_000_000])
    assert whole[1].count(0) == whole[1].count(1) == whole[1].count(2) == 30
    assert whole[1] != sorted(whole[1]), "the threads take turns"
    for budgets in ([1], [7], [3, 11, 2], [64]):
        assert run(budgets) == whole, budgets


@test("module.stop_pauses_lm1", batch="chiplet_module")
def test_stop_pauses_lm1():
    """Asked to stop, LM-1 runs no instruction, makes no request and rings
    nobody, and says STOPPED at its next step; withdrawn, it carries on."""
    mp = Megapad("burst_words=2")
    words = _counting_program()                 # five words: three requests
    mp.put(IMAGE, *words)
    mp.release(IMAGE, 8 * len(words), 0)
    mp.step(1)                                  # the image's first request
    mp.stop(True)
    assert mp.step() is False and mp.stopped and not mp.ready
    requests = len(mp.requests)
    for _ in range(5):
        mp.step()
    assert len(mp.requests) == requests == 1
    mp.stop(False)
    mp.step(2)
    assert not mp.stopped and mp.ready         # the third request ends the fetch
    mp.step(10)
    executed = mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT)
    mp.stop(True)
    mp.model.queue_push(0, chiplet.DOORBELL_OUT_QUEUE, tag_fixnum(4))
    for _ in range(5):
        mp.step()
    assert mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT) == executed
    assert mp.rung == [] and mp.stopped and mp.ready
    mp.stop(False)
    mp.settle()
    assert mp.rung == [4] and not mp.stopped
    assert mp.lm1_word(0x2000) == tag_fixnum(200)


@test("module.a_fault_ends_its_thread", batch="chiplet_module")
def test_a_fault_ends_its_thread():
    """A trap no handler answers ends its thread and is kept for the host;
    the other threads run on, and the console traps, with no console, are
    such traps."""
    mp = Megapad("threads=2")
    mp.put(IMAGE, *program_words([encode_x(Op.TRAP, 0x80), encode_x(Op.HALT_NOP, 0)]))
    mp.release(IMAGE, 8, 0)
    mp.step(0)
    mp.settle()
    assert mp.model.link(chiplet.LINK_FAULTS) == 1
    fault = mp.model.first_fault()
    assert (fault.stop, fault.trap, fault.tile, fault.thread) == (chiplet.STOP_TRAP, 0x80, 0, 0)
    assert mp.model.thread_get(0, 0, chiplet.THREAD_PC) == 0
    # Another thread, started by hand, runs although thread 0 has ended.
    mp.model.thread_set(0, 1, chiplet.THREAD_PC, 4)
    mp.model.thread_set(0, 1, chiplet.THREAD_HALTED, 0)
    mp.settle()
    assert mp.model.thread_get(0, 1, chiplet.THREAD_PC) == 8
    assert mp.model.link(chiplet.LINK_FAULTS) == 1


@test("module.release_starts_over", batch="chiplet_module")
def test_release_starts_over():
    """Reset returns the model to its configured state, every thread
    halted, and a release that finds LM-1 running starts it over."""
    mp = _booted("", _echo_program())
    mp.ring(1)
    mp.settle()
    assert mp.rung == [9, 1]
    mp.module.release(IMAGE, 8 * len(_echo_program()), 0)   # no reset between
    mp.settle()
    assert mp.rung == [9, 1, 9] and mp.model.link(chiplet.LINK_DOORBELLS_IN) == 0
    mp.reset()
    assert all(mp.model.thread_get(0, 0, f) == v for f, v in (
        (chiplet.THREAD_HALTED, 1), (chiplet.THREAD_PC, 0), (chiplet.THREAD_INSTRET, 0)))
    assert mp.model.tile_get(0, chiplet.TILE_INSTRUCTION_COUNT) == 0
    assert mp.model.queue(0, 0) == [] and mp.model.queue(0, 1) == []
    assert mp.lm1_word(0) == 0


@test("module.save_and_restore", batch="chiplet_module")
def test_save_and_restore():
    """A saved state taken back by another module of the same configuration
    carries on exactly as the first: memory, threads, queues, lent ranges,
    the scheduler's place and the sideband alike.  A state of another
    configuration, or a damaged one, is refused and leaves the module in
    reset."""
    config = "threads=2,quanta=3,lend=0x4000:0x800"
    first = _booted(config, _echo_program() + _counting_program(), boot_arg=0x77)
    first.model.thread_set(0, 1, chiplet.THREAD_PC, 8 * len(_echo_program()))
    first.model.thread_set(0, 1, chiplet.THREAD_HALTED, 0)
    first.step(41)
    assert first.module.lent_write(0x4000, image_units(3, 4)) == OK
    first.ring(5)
    first.step(0)       # the doorbell is taken, not yet answered
    state = first.module.save()
    assert len(state) < 64 * 1024, "zero runs keep the state small"

    second = Megapad(config)
    second.released, second.ready = True, first.ready      # the port restores its own part
    assert second.module.restore(state)
    assert second.module.save() == state
    for mp in (first, second):
        mp.settle()
    assert first.module.save() == second.module.save()
    assert first.rung[-1:] == second.rung == [5]
    assert second.lm1_word(0x2000) == tag_fixnum(200)
    assert second.lm1_word(0x4000) == image_word(3, 0)

    for bad in (state[:-1], state + b"\0", b"LM1CHIPX" + state[8:], b""):
        third = Megapad(config)
        third.module.release(IMAGE, 0, 0)
        assert not third.module.restore(bad)
        assert third.model.link(chiplet.LINK_BOOT) == chiplet.BOOT_RESET
    other = Megapad("threads=2,quanta=4,lend=0x4000:0x800")
    assert not other.module.restore(state), "a state of another configuration"


@test("module.digest_covers_megapads_header", batch="chiplet_module")
def test_digest_covers_megapads_header():
    """The library builds against MegaPad's chiplet_module.h, so the digest
    that decides whether it is current covers that header's content, and
    names it by its path inside MegaPad wherever the checkout lies."""
    header = chiplet.megapad_root() / chiplet.MP64_HEADER
    with tempfile.TemporaryDirectory() as folder:
        copy = Path(folder) / "megapad"
        (copy / "shared" / "accel").mkdir(parents=True)
        shutil.copyfile(header, copy / chiplet.MP64_HEADER)
        assert chiplet.source_digest(copy) == chiplet.source_digest()
        with open(copy / chiplet.MP64_HEADER, "a") as f:
            f.write("\n")
        assert chiplet.source_digest(copy) != chiplet.source_digest()
        try:
            chiplet.source_digest(Path(folder) / "absent")
        except chiplet.ChipletBuildError as exc:
            assert "MEGAPAD_ROOT" in str(exc)
        else:
            raise AssertionError("a missing header gave a digest")
    lib = chiplet.library()
    assert lib.lm1c_source_digest().decode() == chiplet.source_digest()
    listing = b"".join(b"%s  %s\n" % (hashlib.sha256((chiplet.CHIPLET_DIR / n).read_bytes())
                                      .hexdigest().encode(), n.encode())
                       for n in chiplet.source_files())
    listing += b"%s  %s\n" % (hashlib.sha256(header.read_bytes()).hexdigest().encode(),
                              chiplet.MP64_HEADER.encode())
    assert hashlib.sha256(listing).hexdigest() == chiplet.source_digest()


@test("module.runtime_stand_in_needs_a_control_block", batch="chiplet_module")
def test_runtime_stand_in_needs_a_control_block():
    """The stand-in for LM-1's runtime is off by default, so the module reads
    nothing past the image.  With runtime=1 it reads the control block that
    the boot argument names, once, and a block that is no CHIPCTRL block, or
    none at all, leaves it serving nothing: LM-1 code still takes the
    doorbells on queue 0."""
    words = _echo_program()
    for config, boot_arg, reads in (("", 0x2040, []), ("runtime=1", 0, []),
                                    ("runtime=1", LENT, [LENT])):
        mp = _booted(config, words, boot_arg=boot_arg)
        mp.settle()
        beyond = [r[0] for r in mp.requests if not IMAGE <= r[0] < IMAGE + 8 * len(words)]
        assert beyond == reads, (config, mp.requests)
        if reads:
            assert mp.model.md(chiplet.MD_PHASE) == 5      # broken: no CHIPCTRL there
        mp.ring(9)
        mp.settle()
        assert mp.rung[-1:] == [9], "LM-1 code answered the doorbell"


@test("module.hooks_find_the_module_and_check_their_arguments", batch="chiplet_module")
def test_hooks_find_the_module_and_check_their_arguments():
    """A test finds the model inside a chiplet module by its hook_id, the
    most recent one made with it, until the module goes; the hooks refuse
    a mode past 4 or an address that is not aligned."""
    first = chiplet.Module("hook_id=77")
    second = chiplet.Module("hook_id=77")
    found = chiplet.attached_model(77)
    assert found is not None and found._handle == second.model._handle
    second.close()
    assert chiplet.attached_model(77)._handle == first.model._handle
    first.close()
    assert chiplet.attached_model(77) is None
    model = chiplet.Module("runtime=1").model
    for call in (lambda: model.md_hold(5), lambda: model.md_write(0x1004, 0),
                 lambda: model.md_churn(0x1001, 0, 0, 1)):
        try:
            call()
        except ValueError:
            pass
        else:
            raise AssertionError("a hook took an argument it cannot")
    model.md_hold(3, 1000)
    assert model.md(chiplet.MD_PHASE) == 0 and model.md(chiplet.MD_EPOCH) == 0
