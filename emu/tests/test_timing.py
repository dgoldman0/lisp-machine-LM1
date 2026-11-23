"""The models' architectural timing model and performance counters.

A model has no clock: it charges each instruction the cycles of the table in
chiplet/src/timing.h (spec/02-isa.md section 11.6), and CYCLE reads the
cycles its core spent before the reading instruction.  These tests run on
the reference and, through the chiplet_reference batch, on the chiplet
module.
"""

from __future__ import annotations

import io
import re
from pathlib import Path

from lm1 import timing
from lm1.asm import Assembler
from lm1.decode import (
    Op, encode_i,
    CTR_ALLOC_BYTES, CTR_ALLOC_COUNT, CTR_BARRIER_FILTERED, CTR_BARRIER_FIRES,
    CTR_IC_HITS, CTR_IC_MISSES, CTR_MEMORY_STALLS, CTR_NURSERY_OVERFLOWS,
)
from lm1.execute import Emulator
from lm1.testing.harness import test
from lm1.word import NIL, make_header, tag_fixnum

STACK_TOP = 0x8000
TRAP_TABLE = 0x6000
CARD_TABLE = 0x5000
DATA = 0x7000

ISSUE, LATENCY, TRAP_ENTRY = timing.ISSUE, timing.MEMORY_LATENCY, timing.TRAP_ENTRY


def _counter(rd: int, number: int) -> str:
    """SYS_INFO Rd, 10 with the counter's number, which has no mnemonic."""
    return f".u32 {encode_i(Op.SYS_INFO, rd, 10, number):#010x}"


def _run(source: str, **kw) -> Emulator:
    """Assemble SOURCE at address 0 and run it to its HALT on a 1 MiB
    emulator, whose nursery and old generation take their defaults unless
    KW names others."""
    binary = Assembler().assemble(source)
    binary += b"\0" * (-len(binary) % 8)
    emu = Emulator(mem_size=1024 * 1024, stdout=io.StringIO(), **kw)
    emu.mem.load_words(0, [int.from_bytes(binary[i:i + 8], "little")
                           for i in range(0, len(binary), 8)])
    emu.thread.pc = 0
    emu.thread.sp = STACK_TOP
    emu.run(max_instructions=10_000)
    assert emu.thread.halted, "the program did not halt"
    return emu


@test("costs", batch="timing_model")
def test_costs():
    """Each instruction costs its issue or its own entry; a load adds one
    memory latency, while POP, which reads the stack, adds none.  CYCLE reads
    the cycles before it, and counter 8 the latency charged."""
    emu = _run(f"""
    NOP
    LI  r2, {DATA}
    LDR r3, r2, 0          ; a load: the issue and one latency
    LI  r5, 0
    MUL.FIX r4, r5, r5
    LI  r8, 1
    DIV r6, r7, r8
    PUSH r1
    POP r9                 ; the stack's reads wait no latency
    CYCLE r10
    {_counter(11, 8)}
    HALT
""")
    before = 7 * ISSUE + LATENCY + timing.MUL + timing.DIV
    assert emu.thread.regs[10] == before, f"CYCLE read {emu.thread.regs[10]}, not {before}"
    assert emu.thread.regs[11] == LATENCY
    assert emu.cycles == before + 3 * ISSUE
    assert emu.perf[CTR_MEMORY_STALLS] == LATENCY
    assert emu.instruction_count == 12


@test("allocation_and_barrier", batch="timing_model")
def test_allocation_and_barrier():
    """ALLOCV costs its own entry, and a store whose barrier marks a card
    adds the mark.  The counters count the allocation, its bytes, the mark
    and the store the barrier filtered."""
    emu = _run(f"""
    LI  r1, {CARD_TABLE}
    TRAP 0x92
    LI  r1, {tag_fixnum(2)}
    ALLOCV r2, r1, 2       ; a young vector of two elements: 32 bytes
    LI32 r3, {0x80000 | 1} ; a ref to an old object, at old-gen's base
    ST.WB r3, r2, 0        ; a young ref into an old word: marks its card
    LI  r5, {tag_fixnum(9)}
    ST.WB r3, r5, 1        ; a fixnum: filtered
    HALT
""")
    assert emu.perf[CTR_ALLOC_COUNT] == 1 and emu.perf[CTR_ALLOC_BYTES] == 32
    assert emu.perf[CTR_BARRIER_FIRES] == 1 and emu.perf[CTR_BARRIER_FILTERED] == 1
    expected = 8 * ISSUE + timing.ALLOCV + timing.BARRIER_MARK
    assert emu.cycles == expected, f"{emu.cycles} cycles, not {expected}"


@test("trap_entry", batch="timing_model")
def test_trap_entry():
    """A fault costs its issue and the trap's entry, and its instruction its
    own cycles again when it runs; an explicit TRAP costs the same entry; a
    fault after a read adds the read's latency."""
    emu = _run(f"""
    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, mend
    LI  r15, {TRAP_TABLE + 0x03 * 8}    ; TRAP_DIVIDE_BY_ZERO
    STR r15, r14, 0
    LI  r14, back
    LI  r15, {TRAP_TABLE + 0x42 * 8}
    STR r15, r14, 0
    LI  r14, skip
    LI  r15, {TRAP_TABLE + 0x07 * 8}    ; TRAP_NOT_CLOSURE
    STR r15, r14, 0
    LI  r8, 0
    DIV r6, r7, r8         ; faults, and runs again after the handler
    TRAP 0x42
    LI  r9, {DATA + 1}     ; a ref to a word that is no closure's header
    CALL.CLOSURE r9        ; reads the header, then faults
    HALT

mend:
    LI  r8, 1
    ERET

back:
    ERET

skip:
    ERET.NEXT
""")
    setup = 12 * ISSUE
    divide = (ISSUE + TRAP_ENTRY) + 2 * ISSUE + timing.DIV
    explicit = (ISSUE + TRAP_ENTRY) + ISSUE
    closure = ISSUE + (ISSUE + LATENCY + TRAP_ENTRY) + ISSUE
    expected = setup + divide + explicit + closure + ISSUE
    assert emu.cycles == expected, f"{emu.cycles} cycles, not {expected}"
    assert emu.perf[CTR_MEMORY_STALLS] == LATENCY


@test("inline_cache_and_overflow_counters", batch="timing_model")
def test_inline_cache_and_overflow_counters():
    """The models count an inline-cache miss, the hit that follows its
    handler's IC.INSTALL, and a nursery overflow, whose allocation counts
    once, when it succeeds."""
    emu = _run(f"""
    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, install
    LI  r15, {TRAP_TABLE + 0x20 * 8}    ; TRAP_IC_MISS
    STR r15, r14, 0
    LI  r1, 1
    LI32 r2, {make_header(0, 0, 9)}
    TRAP 0x91
    ALLOC r10, 2, 1        ; an object of shape 9
    CALL.IC r10            ; misses, then hits once its entry is installed
    MOV np, nl             ; the thread's chunk of the nursery is full
    ALLOC.CONS r12, r0, r0 ; overflows; the model takes a fresh chunk
    HALT

install:
    TRAP.PC r12
    LI  r13, method
    IC.INSTALL r12, r10, r13
    ERET

method:
    RET
""")
    assert emu.perf[CTR_IC_MISSES] == 1 and emu.perf[CTR_IC_HITS] == 1
    assert emu.perf[CTR_NURSERY_OVERFLOWS] == 1
    assert emu.perf[CTR_ALLOC_COUNT] == 2 and emu.thread.regs[12] != NIL


@test("one_table", batch="timing_model")
def test_one_table():
    """The spec states the table the models read from chiplet/src/timing.h,
    entry for entry."""
    spec = (Path(__file__).resolve().parents[2] / "spec" / "02-isa.md").read_text()
    table = spec.split("| Entry | Cycles |", 1)[1].split("\n\n", 1)[0]
    stated = {name: int(n) for name, n in re.findall(r"^\| `(\w+)` \| (\d+) \|", table, re.M)}
    assert stated == timing.CYCLES, stated
