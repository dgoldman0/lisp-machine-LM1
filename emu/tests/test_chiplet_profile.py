"""The chiplet module's per-function profile (docs/profile.md).

A compiled program spends most of its time in one function, busy, which
middle calls, which main calls.  With profile=N the module samples the core
every N cycles of its timing model and charges each sample to the function
holding the PC and to its callers, named by the symbol table the image
carries at its end.
"""

from __future__ import annotations

import io
import json
import tempfile
from pathlib import Path

from lm1 import chiplet, image
from lm1 import execute as reference
from lm1.testing.harness import test
from lm1.word import WORD_MASK

PROGRAM = """
(defun busy (n acc) (if (= n 0) acc (busy (- n 1) (+ acc n))))
(defun middle () (busy 400 0))
(defun light () (+ 1 2))
(defun main () (light) (middle))
"""
MEM = 1024 * 1024
STEPS = 100_000


def _image(*, symbols: bool = True) -> image.Image:
    return image.compile_program(PROGRAM, symbols=symbols)


def _load(emu, img: image.Image) -> None:
    emu.mem.load_words(0, img.words)
    emu.thread.pc = 0


def _profiled(img: image.Image, period: int, *, symbols: bool = True) -> tuple[object, list[dict]]:
    """Run the image with a sample every PERIOD cycles; the emulator and
    the profile's records."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "profile.jsonl"
        emu = chiplet.Emulator(mem_size=MEM, stdout=io.StringIO(),
                               config=f"profile={period},profile_file={path}")
        _load(emu, img)
        if symbols:
            assert emu.profile_symbols(img.tobytes())
        emu.run(max_instructions=STEPS)
        assert emu.thread.halted
        emu.profile_flush()
        records = [json.loads(line) for line in path.read_text().splitlines()]
    return emu, records


def _totals(records: list[dict], key: str) -> dict[str, int]:
    total: dict[str, int] = {}
    for record in records:
        for name, cycles in record[key].items():
            total[name] = total.get(name, 0) + cycles
    return total


@test("hot_function", batch="chiplet_profile")
def test_hot_function():
    """The function the program spends its time in gets the most samples,
    charged with its callers innermost first, in records of MegaPad's
    guest-words shape whose steps are the sampled cycles."""
    emu, records = _profiled(_image(), 7)
    assert records and all(set(r) == {"start_ns", "t_ns", "steps", "samples", "inclusive",
                                      "leaf", "chains", "core"} for r in records)
    leaf, inclusive, chains = (_totals(records, k) for k in ("leaf", "inclusive", "chains"))
    steps = sum(r["steps"] for r in records)
    assert steps == 7 * sum(r["samples"] for r in records)
    assert emu.cycles - 7 < steps <= emu.cycles, (steps, emu.cycles)
    assert max(leaf, key=leaf.get) == "busy", leaf
    assert leaf["busy"] > 0.8 * steps, leaf
    assert max(chains, key=chains.get) == "busy < middle < main < _call_main", chains
    assert inclusive["busy"] <= inclusive["middle"] <= inclusive["main"] <= steps
    assert inclusive.get("light", 0) < inclusive["busy"] / 10


@test("raw_pcs_without_symbols", batch="chiplet_profile")
def test_raw_pcs_without_symbols():
    """An image that carries no symbol table is profiled by PC: each name is
    an address, and the hottest lies in busy's code."""
    named = _image()
    _, records = _profiled(_image(symbols=False), 7, symbols=False)
    leaf = _totals(records, "leaf")
    assert all(name.startswith("0x") for name in leaf), leaf
    addresses = dict((name, addr) for addr, name in named.symbols)
    hottest = int(max(leaf, key=leaf.get), 16)
    assert addresses["busy"] <= hottest < addresses["middle"], (hex(hottest), addresses)


@test("profile_changes_nothing", batch="chiplet_profile")
def test_profile_changes_nothing():
    """A run sampled every cycle computes, cycle for cycle, what a run
    without the profile computes, on the module and on the reference, which
    has none."""
    img = _image()
    plain = chiplet.Emulator(mem_size=MEM, stdout=io.StringIO())
    ref = reference.Emulator(mem_size=MEM, stdout=io.StringIO())
    for emu in (plain, ref):
        _load(emu, img)
        emu.run(max_instructions=STEPS)
    sampled, records = _profiled(img, 1)
    assert sum(r["samples"] for r in records) == sampled.cycles

    def state(emu) -> dict:
        t = emu.thread
        return {"cycles": emu.cycles, "issued": emu.instruction_count, "perf": list(emu.perf),
                "regs": list(t.regs), "pc": t.pc & WORD_MASK, "instret": t.instret,
                "memory": [emu.mem.load_word(a) for a in range(0, 0x4000, 8)]}

    assert state(sampled) == state(plain) == state(ref)
