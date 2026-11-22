"""Phase 3 tests — traps, PUSH.MULTI/POP.MULTI, write barrier, nursery GC.

Milestone: allocate until nursery overflows 10×, live objects survive.
"""

import io

from lm1.testing.harness import test
from lm1.execute import Emulator, EMU_TRAP_PUTCHAR
from lm1.decode import (
    Op, encode_r, encode_i, encode_s, encode_b, encode_x,
    FUNC_ADD, FUNC_SUB, FUNC_DIV,
    FUNC_ADD_FIX, FUNC_SUB_FIX,
    FUNC_CMP, FUNC_EQ,
    FUNC_PUSH, FUNC_POP,
    FUNC_TRAP_CAUSE, FUNC_TRAP_PC,
    FUNC_ERET_NEXT,
    BR_T, BR_NIL, BR_FIX_EQ, BR_FIX_GT,
)
from lm1.word import (
    NIL, T, tag_fixnum, untag_fixnum,
    is_cons_ref, is_ref, is_any_ref, ref_address, make_ref,
    make_header, HDR_CONS, HDR_VECTOR, HDR_CLOSURE,
    header_shape_id, header_subtype, header_size,
    is_header, WORD_MASK,
)
from lm1.traps import (
    LM1Trap, TRAP_NURSERY_OVERFLOW, TRAP_DIVIDE_BY_ZERO, TRAP_UNIMPLEMENTED,
)


# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------

# Memory layout for Phase 3 tests:
#   0x00000 - 0x07FFF  : code + data (32 KiB)
#   0x08000 - 0x0FFFF  : stack (grows down from 0x10000)
#   0x10000 - 0x1FFFF  : nursery (64 KiB)
#   0x20000 - 0x3FFFF  : old-gen (128 KiB)
#   0x40000 - 0x40FFF  : trap table (4 KiB, enough for 256 entries × 8)
#   0x50000 - 0x51FFF  : card table, where a test configures one in memory
#                        (one byte per 64-byte card of the 512 KiB)
NURSERY_BASE = 0x1_0000
NURSERY_SIZE = 0x1_0000
OLDGEN_BASE  = 0x2_0000
OLDGEN_SIZE  = 0x2_0000
STACK_TOP    = 0x1_0000   # stack grows down from here
TRAP_TABLE   = 0x4_0000
CARD_TABLE   = 0x5_0000
CODE_BASE    = 0x0


def _emu(**kw):
    """Create an Emulator with nursery + old-gen for testing."""
    return Emulator(
        mem_size=512 * 1024,       # 512 KiB
        nursery_base=NURSERY_BASE,
        nursery_size=NURSERY_SIZE,
        oldgen_base=OLDGEN_BASE,
        oldgen_size=OLDGEN_SIZE,
        **kw,
    )


def _load_and_run(emu, program, *, base=CODE_BASE, max_instructions=100_000):
    """Load instructions at `base` and run."""
    for i, insn in enumerate(program):
        emu.mem.store_u32(base + i * 4, insn)
    emu.thread.pc = base
    emu.thread.sp = STACK_TOP
    emu.run(max_instructions=max_instructions)


def _alloc_cons_x(rd):
    """Encode ALLOC.CONS instruction."""
    payload = (rd & 0x1F) << 21
    return encode_x(Op.ALLOC_CONS, payload)


def _alloc_x(rd, n_words, tmpl_idx):
    """Encode ALLOC instruction."""
    payload = ((rd & 0x1F) << 21) | ((n_words & 0x1F) << 16) | (tmpl_idx & 0xFFFF)
    return encode_x(Op.ALLOC, payload)


def _allocv_x(rd, rs_len, tmpl_idx):
    """Encode ALLOCV instruction."""
    payload = ((rd & 0x1F) << 21) | ((rs_len & 0x1F) << 16) | (tmpl_idx & 0xFFFF)
    return encode_x(Op.ALLOCV, payload)


def _run_at(emu, program, *, base=0x200, max_instructions=100):
    """Load a further program at `base`, clear of the first, and run it with
    the registers the first one left."""
    for i, insn in enumerate(program):
        emu.mem.store_u32(base + i * 4, insn)
    emu.thread.pc = base
    emu.thread.halted = False
    emu.run(max_instructions=max_instructions)


def _old_vector(emu, n_fields):
    """Place an n-field vector of nils in old-gen and return its ref.

    The allocator puts new objects in the nursery, and only a collection
    moves them to old-gen.  Writing one at old-gen's bump pointer stands in
    for an object an earlier collection promoted, and moving the pointer
    past it keeps a later collection from copying over it.
    """
    addr = emu.oldgen_ptr
    emu.mem.store_word(addr, make_header(HDR_VECTOR, n_fields, 0))
    for i in range(n_fields):
        emu.mem.store_word(addr + (i + 1) * 8, NIL)
    emu.oldgen_ptr = addr + (n_fields + 1) * 8
    return make_ref(addr)


def _old_cons(emu):
    """Place a cons of two nils in old-gen, as `_old_vector` places a vector."""
    addr = emu.oldgen_ptr
    emu.mem.store_word(addr, emu.thread.header_templates[0])
    emu.mem.store_word(addr + 8, NIL)
    emu.mem.store_word(addr + 16, NIL)
    emu.oldgen_ptr = addr + 24
    return make_ref(addr, cons=True)


# ===================================================================
# Batch: phase3_trap — trap table dispatch and ERET
# ===================================================================

@test("trap_dispatch_basic", batch="phase3_trap")
def test_trap_dispatch_basic():
    """TRAP instruction dispatches through trap table, handler runs, ERET returns."""
    emu = _emu()
    t = emu.thread

    # Install a trap handler for TRAP_DIVIDE_BY_ZERO (code 0x03)
    # Handler at address 0x100: just sets r5 = 42 and ERETes
    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + 0x03 * 8, handler_addr)
    t.trap_table_base = TRAP_TABLE

    # Handler code at 0x100:
    handler = [
        encode_i(Op.LI, 5, 0, tag_fixnum(42) & 0xFFFF),  # r5 = fixnum(42)
        # Advance trap_pc past the faulting instruction (ARITH_RAW DIV)
        # Read trap_pc into r6, add 4, write back... actually ERET returns
        # to trap_pc which is the faulting instruction. We need to skip it.
        # SYS_INFO: rd=r6, sub=TRAP_PC(4)
        encode_i(Op.SYS_INFO, 6, FUNC_TRAP_PC, 0),  # r6 = trap_pc
        encode_r(Op.ARITH_RAW, 6, 6, 7, FUNC_ADD),   # r6 = r6 + r7 (r7=4)
        # Store r6 back as trap_pc... we can't write trap_pc from LM-1.
        # Simpler: just ERET with trap_pc pointing at the NOP after the div.
        encode_x(Op.ERET, 0),
    ]
    for i, insn in enumerate(handler):
        emu.mem.store_u32(handler_addr + i * 4, insn)

    # Main code: try to divide by zero → trap → handler sets r5=42 → ERET
    # ERET returns to the faulting instruction (DIV by zero), which will
    # trap again. To avoid infinite loop, put something the handler can fix.
    #
    # Better approach: the main code does:
    #   r1 = 10, r2 = 0 → DIV (traps)
    #   → handler sets r5 = 42, ERET returns to the DIV instruction
    #   → it traps again... infinite loop.
    #
    # Solution: have the handler skip the faulting instruction.
    # Since we can't write trap_pc, let's do it differently.
    # The handler can modify the divisor so the retry succeeds.
    #
    # Handler: set r2 = 2 (fix the divisor), then ERET (retries the DIV)
    handler2 = [
        encode_i(Op.LI, 5, 0, tag_fixnum(42) & 0xFFFF),  # r5 = marker
        encode_i(Op.LI, 2, 0, 2),                         # r2 = 2 (fix divisor)
        encode_x(Op.ERET, 0),
    ]
    for i, insn in enumerate(handler2):
        emu.mem.store_u32(handler_addr + i * 4, insn)

    # Main code
    main = [
        encode_i(Op.LI, 1, 0, 10),           # r1 = 10
        encode_i(Op.LI, 2, 0, 0),            # r2 = 0
        encode_r(Op.ARITH_RAW, 3, 1, 2, FUNC_DIV),  # r3 = r1 / r2 → trap!
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, main)

    assert t.regs[5] == tag_fixnum(42) & WORD_MASK, f"Handler marker not set: r5={t.regs[5]:#x}"
    assert t.regs[3] == 5, f"DIV 10/2 should give 5, got {t.regs[3]}"


@test("trap_cause_read", batch="phase3_trap")
def test_trap_cause_read():
    """Trap handler can read trap_cause via SYS_INFO."""
    emu = _emu()
    t = emu.thread

    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + 0x03 * 8, handler_addr)
    t.trap_table_base = TRAP_TABLE

    # Handler: read trap cause into r10, fix divisor, ERET
    handler = [
        encode_i(Op.SYS_INFO, 10, FUNC_TRAP_CAUSE, 0),  # r10 = trap_cause
        encode_i(Op.LI, 2, 0, 1),                        # r2 = 1 (fix divisor)
        encode_x(Op.ERET, 0),
    ]
    for i, insn in enumerate(handler):
        emu.mem.store_u32(handler_addr + i * 4, insn)

    main = [
        encode_i(Op.LI, 1, 0, 10),
        encode_i(Op.LI, 2, 0, 0),
        encode_r(Op.ARITH_RAW, 3, 1, 2, FUNC_DIV),  # trap: div by zero
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, main)

    assert t.regs[10] == TRAP_DIVIDE_BY_ZERO, \
        f"trap_cause should be {TRAP_DIVIDE_BY_ZERO:#x}, got {t.regs[10]:#x}"


@test("eret_resumes_correctly", batch="phase3_trap")
def test_eret_resumes_correctly():
    """ERET returns to the faulting instruction and execution continues."""
    emu = _emu()
    t = emu.thread

    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + 0x03 * 8, handler_addr)
    t.trap_table_base = TRAP_TABLE

    # Handler: fix divisor and ERET
    handler = [
        encode_i(Op.LI, 2, 0, 5),   # r2 = 5
        encode_x(Op.ERET, 0),
    ]
    for i, insn in enumerate(handler):
        emu.mem.store_u32(handler_addr + i * 4, insn)

    # Main: div by zero traps, handler fixes, retry succeeds, then set r4
    main = [
        encode_i(Op.LI, 1, 0, 100),          # r1 = 100
        encode_i(Op.LI, 2, 0, 0),            # r2 = 0
        encode_r(Op.ARITH_RAW, 3, 1, 2, FUNC_DIV),  # r3 = 100/0 → trap → 100/5
        encode_i(Op.LI, 4, 0, 0x55),         # r4 = 0x55 (proves we continued)
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, main)

    assert t.regs[3] == 20, f"Expected 100/5=20, got {t.regs[3]}"
    assert t.regs[4] == 0x55, f"Execution didn't continue past trap: r4={t.regs[4]:#x}"


@test("explicit_trap_resumes_after_itself", batch="phase3_trap")
def test_explicit_trap_resumes_after_itself():
    """ERET after an explicit TRAP resumes at the instruction after it, as
    a system call returns, while ERET after a fault retries the faulting
    instruction."""
    emu = _emu()
    t = emu.thread
    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + 0x42 * 8, handler_addr)
    t.trap_table_base = TRAP_TABLE
    for i, insn in enumerate([encode_i(Op.LI, 5, 0, 0x55), encode_x(Op.ERET, 0)]):
        emu.mem.store_u32(handler_addr + i * 4, insn)
    _load_and_run(emu, [
        encode_x(Op.TRAP, 0x42),
        encode_i(Op.LI, 6, 0, 0x66),
        encode_x(Op.HALT_NOP, 0),
    ], max_instructions=100)
    assert t.halted, "ERET returned to the TRAP and ran it again"
    assert t.regs[5] == 0x55 and t.regs[6] == 0x66
    assert t.trap_pc == 4 and t.trap_cause == 0x42
    assert emu.instruction_count == 5   # TRAP, LI, ERET, LI, HALT


@test("nursery_overflow_runs_installed_handler", batch="phase3_trap")
def test_nursery_overflow_runs_installed_handler():
    """A runtime that installed a handler for TRAP_NURSERY_OVERFLOW gets the
    trap, and the emulator's own collection, which stands in for an absent
    runtime, stays out of the way.  The handler here empties the nursery
    and ERET retries the allocation."""
    emu = Emulator(mem_size=512 * 1024, nursery_base=NURSERY_BASE, nursery_size=48,
                   oldgen_base=OLDGEN_BASE, oldgen_size=OLDGEN_SIZE)
    t = emu.thread
    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + TRAP_NURSERY_OVERFLOW * 8, handler_addr)
    t.trap_table_base = TRAP_TABLE
    for i, insn in enumerate([
        encode_i(Op.LI, 20, 0, 0x77),
        encode_i(Op.LUI, 26, 0, NURSERY_BASE >> 16),   # np = nursery base
        encode_x(Op.ERET, 0),
    ]):
        emu.mem.store_u32(handler_addr + i * 4, insn)
    _load_and_run(emu, [
        _alloc_cons_x(1),
        _alloc_cons_x(2),
        _alloc_cons_x(3),          # overflows: the handler runs, the retry fits
        encode_x(Op.HALT_NOP, 0),
    ])
    assert t.halted
    assert emu.gc_count == 0, f"The emulator collected {emu.gc_count} times"
    assert t.regs[20] == 0x77, "The installed handler did not run"
    assert t.trap_cause == TRAP_NURSERY_OVERFLOW and t.trap_pc == 8
    assert ref_address(t.regs[3]) == NURSERY_BASE


@test("eret_next_resumes_after_the_fault", batch="phase3_trap")
def test_eret_next_resumes_after_the_fault():
    """ERET.NEXT resumes at the saved PC + 4, so a handler that carries out
    the faulting instruction itself, as a generic + does, goes on after it
    (spec/03-core.md section 6.3).  The fault still saves its own PC, and
    ERET.NEXT leaves the handler as ERET does."""
    emu = _emu()
    t = emu.thread
    handler_addr = 0x100
    emu.mem.store_word(TRAP_TABLE + 0x01 * 8, handler_addr)   # TRAP_NOT_FIXNUM
    t.trap_table_base = TRAP_TABLE
    for i, insn in enumerate([
        encode_r(Op.ARITH_RAW, 3, 1, 2, FUNC_ADD),             # r3 = r1 + r2, raw
        encode_r(Op.ERET, 0, 0, 0, FUNC_ERET_NEXT),
    ]):
        emu.mem.store_u32(handler_addr + i * 4, insn)
    _load_and_run(emu, [
        encode_i(Op.LI, 1, 0, 7),                               # no fixnum
        encode_i(Op.LI, 2, 0, tag_fixnum(5) & 0xFFFF),
        encode_r(Op.ARITH_FIX, 3, 1, 2, FUNC_ADD_FIX),          # traps at 8
        encode_i(Op.ADD_FIX_IMM, 4, 4, tag_fixnum(1) & 0xFFFF), # runs once
        encode_x(Op.HALT_NOP, 0),
    ], max_instructions=100)
    assert t.halted, "ERET.NEXT went back to the ADD.FIX"
    assert t.regs[3] == 17 and t.regs[4] == tag_fixnum(1)
    assert t.trap_pc == 8 and t.trap_cause == 0x01 and not t.in_trap
    assert emu.instruction_count == 7   # LI, LI, ADD.FIX, ADD, ERET.NEXT, ADD.FIX.IMM, HALT


@test("eret_outside_handler_or_reserved_func_traps", batch="phase3_trap")
def test_eret_outside_handler_or_reserved_func_traps():
    """ERET.NEXT outside a handler traps as ERET does, and an ERET whose
    func names neither ERET nor ERET.NEXT is reserved and traps
    TRAP_UNIMPLEMENTED, leaving the handler's state as it was."""
    emu = _emu()
    try:
        _load_and_run(emu, [encode_r(Op.ERET, 0, 0, 0, FUNC_ERET_NEXT),
                            encode_x(Op.HALT_NOP, 0)], max_instructions=10)
    except LM1Trap as trap:
        assert trap.code == TRAP_UNIMPLEMENTED, f"Wrong trap: {trap.code:#x}"
    else:
        raise AssertionError("ERET.NEXT outside a handler returned")
    emu = _emu()
    t = emu.thread
    t.in_trap, t.trap_pc = True, 0x200
    try:
        _load_and_run(emu, [encode_r(Op.ERET, 0, 0, 0, 2), encode_x(Op.HALT_NOP, 0)],
                      max_instructions=10)
    except LM1Trap as trap:
        assert trap.code == TRAP_UNIMPLEMENTED, f"Wrong trap: {trap.code:#x}"
    else:
        raise AssertionError("ERET with func 2 returned")
    assert t.in_trap and t.pc == 0, "the reserved ERET left the handler"


# ===================================================================
# Batch: phase3_pushpop — PUSH.MULTI / POP.MULTI
# ===================================================================

@test("push_multi_basic", batch="phase3_pushpop")
def test_push_multi_basic():
    """PUSH.MULTI saves multiple registers to the stack."""
    emu = _emu()
    t = emu.thread
    t.sp = STACK_TOP

    # Set up registers
    t.regs[1] = 0xAA
    t.regs[2] = 0xBB
    t.regs[3] = 0xCC

    mask = 0x000E  # r1, r2, r3
    bank = 0
    # PUSH_MULTI is its own opcode (Format I): rd=bank, rs1=unused, imm16=mask
    program = [
        encode_i(Op.PUSH_MULTI, bank, 0, mask),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    # Stack should have 3 words pushed (r1, r2, r3 in ascending order)
    # r1 pushed first → at highest address, r3 last → at lowest
    assert t.sp == STACK_TOP - 3 * 8, f"SP wrong: expected {STACK_TOP - 24:#x}, got {t.sp:#x}"
    assert emu.mem.load_word(STACK_TOP - 8) == 0xAA
    assert emu.mem.load_word(STACK_TOP - 16) == 0xBB
    assert emu.mem.load_word(STACK_TOP - 24) == 0xCC


@test("pop_multi_basic", batch="phase3_pushpop")
def test_pop_multi_basic():
    """POP.MULTI restores multiple registers from the stack."""
    emu = _emu()
    t = emu.thread

    # Pre-push values onto stack (simulating a previous PUSH.MULTI r1,r2,r3)
    # Must set stack pointers AFTER _load_and_run since it resets SP.
    sp_init = STACK_TOP - 3 * 8
    emu.mem.store_word(STACK_TOP - 8, 0xAA)   # r1
    emu.mem.store_word(STACK_TOP - 16, 0xBB)  # r2
    emu.mem.store_word(STACK_TOP - 24, 0xCC)  # r3

    mask = 0x000E  # r1, r2, r3
    bank = 0
    program = [
        encode_i(Op.POP_MULTI, bank, 0, mask),
        encode_x(Op.HALT_NOP, 0),
    ]
    for i, insn in enumerate(program):
        emu.mem.store_u32(i * 4, insn)
    t.pc = 0
    t.sp = sp_init  # set SP to the pre-pushed state
    emu.run(max_instructions=10)

    assert t.sp == STACK_TOP, f"SP not restored: expected {STACK_TOP:#x}, got {t.sp:#x}"
    assert t.regs[1] == 0xAA, f"r1 = {t.regs[1]:#x}, expected 0xAA"
    assert t.regs[2] == 0xBB, f"r2 = {t.regs[2]:#x}, expected 0xBB"
    assert t.regs[3] == 0xCC, f"r3 = {t.regs[3]:#x}, expected 0xCC"


@test("push_pop_multi_roundtrip", batch="phase3_pushpop")
def test_push_pop_multi_roundtrip():
    """PUSH.MULTI followed by POP.MULTI preserves register values."""
    emu = _emu()
    t = emu.thread

    # Set registers
    for i in range(1, 8):
        t.regs[i] = tag_fixnum(i * 10)

    mask = 0x00FE  # r1-r7
    bank = 0
    program = [
        encode_i(Op.PUSH_MULTI, bank, 0, mask),  # push r1-r7
        # Clobber registers
        encode_i(Op.LI, 1, 0, 0),
        encode_i(Op.LI, 2, 0, 0),
        encode_i(Op.LI, 3, 0, 0),
        encode_i(Op.LI, 4, 0, 0),
        encode_i(Op.LI, 5, 0, 0),
        encode_i(Op.LI, 6, 0, 0),
        encode_i(Op.LI, 7, 0, 0),
        encode_i(Op.POP_MULTI, bank, 0, mask),    # pop r1-r7
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    for i in range(1, 8):
        expected = tag_fixnum(i * 10)
        assert t.regs[i] == expected, \
            f"r{i}: expected {expected:#x}, got {t.regs[i]:#x}"


@test("push_multi_high_bank", batch="phase3_pushpop")
def test_push_multi_high_bank():
    """PUSH.MULTI with bank=1 saves r16-r31 range."""
    emu = _emu()
    t = emu.thread

    t.regs[16] = 0x1616
    t.regs[17] = 0x1717

    mask = 0x0003  # bit 0 = r16, bit 1 = r17
    bank = 1
    program = [
        encode_i(Op.PUSH_MULTI, bank, 0, mask),
        # Clobber
        encode_i(Op.LI, 16, 0, 0),
        encode_i(Op.LI, 17, 0, 0),
        encode_i(Op.POP_MULTI, bank, 0, mask),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    assert t.regs[16] == 0x1616, f"r16 = {t.regs[16]:#x}"
    assert t.regs[17] == 0x1717, f"r17 = {t.regs[17]:#x}"


@test("push_pop_multi_bank_past_one_traps", batch="phase3_pushpop")
def test_push_pop_multi_bank_past_one_traps():
    """The 32 registers form two banks of 16, so PUSH.MULTI and POP.MULTI
    with a bank past 1 name no registers: the instruction traps as
    unimplemented and leaves SP as it was."""
    for op in (Op.PUSH_MULTI, Op.POP_MULTI):
        emu = _emu()
        try:
            _load_and_run(emu, [encode_i(op, 2, 0, 0x0001), encode_x(Op.HALT_NOP, 0)])
        except LM1Trap as trap:
            assert trap.code == TRAP_UNIMPLEMENTED, f"Wrong trap: {trap.code:#x}"
        else:
            raise AssertionError(f"{op.name} with bank 2 returned")
        assert emu.thread.sp == STACK_TOP, f"{op.name} moved SP to {emu.thread.sp:#x}"


# ===================================================================
# Batch: phase3_barrier — write barrier and card table
# ===================================================================

@test("write_barrier_marks_card", batch="phase3_barrier")
def test_write_barrier_marks_card():
    """ST.WB marks the card table when storing a nursery ref into old-gen."""
    emu = _emu()
    t = emu.thread

    _load_and_run(emu, [
        _alloc_cons_x(2),         # r2 = nursery cons
        encode_x(Op.HALT_NOP, 0),
    ])
    assert is_cons_ref(t.regs[2])
    vec = _old_vector(emu, 2)

    # ST.WB: Rs(rd)=r3, Rt(rs1)=r2, field(rs2)=0
    t.regs[3] = vec
    _run_at(emu, [
        encode_s(Op.ST_WB, 3, 2, 0),
        encode_x(Op.HALT_NOP, 0),
    ])

    # The card holding field 0 is marked dirty with 0xFF
    card_idx = (ref_address(vec) + 8) >> emu.card_shift
    assert emu.card_table[card_idx] == 0xFF, \
        f"Card at index {card_idx} should be dirty, got {emu.card_table[card_idx]}"


@test("write_barrier_marks_stored_word_card", batch="phase3_barrier")
def test_write_barrier_marks_stored_word_card():
    """ST.WB marks the card holding the field it stored, not the header's.

    Field 12 of a 16-field vector lies 104 bytes past the header, in the
    next 64-byte card, so only that card records the old-to-young ref
    (spec/02-isa.md § 5.4, spec/05-memory-gc.md § 4.2).
    """
    emu = _emu()
    t = emu.thread

    _load_and_run(emu, [_alloc_cons_x(2), encode_x(Op.HALT_NOP, 0)])
    vec = _old_vector(emu, 16)
    t.regs[3] = vec
    _run_at(emu, [encode_s(Op.ST_WB, 3, 2, 12), encode_x(Op.HALT_NOP, 0)])

    header_card = ref_address(vec) >> emu.card_shift
    field_card = (ref_address(vec) + 13 * 8) >> emu.card_shift
    assert field_card != header_card
    assert emu.card_table[field_card] == 0xFF, \
        f"Field 12's card {field_card} should be dirty, got {emu.card_table[field_card]}"
    assert emu.card_table[header_card] == 0, \
        f"The header's card {header_card} holds no new ref, got {emu.card_table[header_card]}"


@test("st_cdr_marks_stored_word_card", batch="phase3_barrier")
def test_st_cdr_marks_stored_word_card():
    """ST.CAR and ST.CDR are barriered stores (spec/02-isa.md § 5.5).

    A five-field vector puts an old cons 48 bytes into old-gen, so its car
    ends the first card and its cdr starts the next.  Storing a nursery
    cons into that cdr marks the cdr's card.
    """
    emu = _emu()
    t = emu.thread

    _load_and_run(emu, [_alloc_cons_x(2), encode_x(Op.HALT_NOP, 0)])
    _old_vector(emu, 5)
    old = _old_cons(emu)
    t.regs[3] = old
    _run_at(emu, [encode_s(Op.ST_CAR_CDR, 3, 2, 1), encode_x(Op.HALT_NOP, 0)])

    car_card = (ref_address(old) + 8) >> emu.card_shift
    cdr_card = (ref_address(old) + 16) >> emu.card_shift
    assert cdr_card == car_card + 1
    assert emu.card_table[cdr_card] == 0xFF, \
        f"The cdr's card {cdr_card} should be dirty, got {emu.card_table[cdr_card]}"
    assert emu.card_table[car_card] == 0, \
        f"The car's card {car_card} holds no new ref, got {emu.card_table[car_card]}"


@test("barrier_boundary_needs_old_gen_above_nursery", batch="phase3_barrier")
def test_barrier_boundary_needs_old_gen_above_nursery():
    """The barrier's one boundary starts at the nursery's end, and a layout
    with old-gen below the nursery, which no single boundary separates, is
    refused when the emulator is made."""
    emu = _emu()
    assert emu.gen_boundary == NURSERY_BASE + NURSERY_SIZE
    try:
        Emulator(mem_size=512 * 1024, nursery_base=OLDGEN_BASE, nursery_size=NURSERY_SIZE,
                 oldgen_base=NURSERY_BASE, oldgen_size=NURSERY_SIZE)
    except ValueError:
        pass
    else:
        raise AssertionError("A layout with old-gen below the nursery was accepted")


@test("write_barrier_intra_nursery_no_mark", batch="phase3_barrier")
def test_write_barrier_intra_nursery_no_mark():
    """ST.WB within nursery does NOT mark the card table."""
    emu = _emu()
    t = emu.thread

    # Allocate two cons cells in nursery
    program = [
        _alloc_cons_x(1),   # r1 = cons A
        _alloc_cons_x(2),   # r2 = cons B
        # ST.WB r1.car = r2 (both in nursery)
        encode_s(Op.ST_WB, 1, 2, 0),  # field 0 = car
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    # No cards should be dirty
    assert all(c == 0 for c in emu.card_table), \
        "No card should be dirty for intra-nursery store"


# ===================================================================
# Batch: phase3_gc — nursery garbage collection
# ===================================================================

@test("gc_basic_survival", batch="phase3_gc")
def test_gc_basic_survival():
    """A single live cons cell survives GC and is accessible."""
    emu = _emu()
    t = emu.thread

    # Allocate a cons, set car to fixnum 99
    program = [
        _alloc_cons_x(1),                                    # r1 = cons
        encode_i(Op.LI, 2, 0, tag_fixnum(99) & 0xFFFF),    # r2 = 99
        encode_s(Op.ST_CAR_CDR, 1, 2, 0),                   # car(r1) = 99
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    # Verify the cons is in the nursery
    ref = t.regs[1]
    assert is_cons_ref(ref)
    addr = ref_address(ref)
    assert emu._is_nursery_addr(addr), "Cons should be in nursery"

    # Manually trigger GC
    emu._gc_collect()

    # r1 should now point to old-gen
    ref2 = t.regs[1]
    assert is_cons_ref(ref2), f"After GC, r1 should still be cons-ref: {ref2:#x}"
    addr2 = ref_address(ref2)
    assert not emu._is_nursery_addr(addr2), \
        f"After GC, cons should be in old-gen, addr={addr2:#x}"
    assert addr2 >= OLDGEN_BASE and addr2 < OLDGEN_BASE + OLDGEN_SIZE

    # car should still be fixnum 99
    car = emu.mem.load_word(addr2 + 8)
    assert car == tag_fixnum(99), f"car should be 99, got {untag_fixnum(car)}"


@test("gc_linked_list_survives", batch="phase3_gc")
def test_gc_linked_list_survives():
    """A linked list of cons cells survives GC with correct structure."""
    emu = _emu()
    t = emu.thread

    # Build a list: (1 2 3) = cons(1, cons(2, cons(3, nil)))
    # We build bottom-up: first cons(3, nil), then cons(2, that), then cons(1, that)
    program = [
        # cons(3, nil) → r3
        _alloc_cons_x(3),
        encode_i(Op.LI, 10, 0, tag_fixnum(3) & 0xFFFF),
        encode_s(Op.ST_CAR_CDR, 3, 10, 0),  # car = 3
        # cdr stays nil

        # cons(2, r3) → r2
        _alloc_cons_x(2),
        encode_i(Op.LI, 10, 0, tag_fixnum(2) & 0xFFFF),
        encode_s(Op.ST_CAR_CDR, 2, 10, 0),  # car = 2
        encode_s(Op.ST_CAR_CDR, 2, 3, 1),   # cdr = r3

        # cons(1, r2) → r1
        _alloc_cons_x(1),
        encode_i(Op.LI, 10, 0, tag_fixnum(1) & 0xFFFF),
        encode_s(Op.ST_CAR_CDR, 1, 10, 0),  # car = 1
        encode_s(Op.ST_CAR_CDR, 1, 2, 1),   # cdr = r2

        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    # Trigger GC
    emu._gc_collect()

    # Walk the list
    def walk_list(ref):
        values = []
        while ref != NIL:
            assert is_cons_ref(ref) or is_any_ref(ref), f"Not a ref: {ref:#x}"
            addr = ref_address(ref)
            car = emu.mem.load_word(addr + 8)
            values.append(untag_fixnum(car))
            ref = emu.mem.load_word(addr + 16)  # cdr
        return values

    result = walk_list(t.regs[1])
    assert result == [1, 2, 3], f"List should be [1,2,3], got {result}"


@test("gc_garbage_reclaimed", batch="phase3_gc")
def test_gc_garbage_reclaimed():
    """Dead objects (no refs from roots) are NOT copied to old-gen."""
    emu = _emu()
    t = emu.thread

    # Allocate 10 cons cells, only keep the last one in r1
    program = []
    for i in range(10):
        program.append(_alloc_cons_x(1))  # each overwrites r1
    program.append(encode_x(Op.HALT_NOP, 0))
    _load_and_run(emu, program)

    # 10 cons cells × 24 bytes = 240 bytes used in nursery
    assert t.np == NURSERY_BASE + 240

    old_ptr_before = emu.oldgen_ptr
    emu._gc_collect()
    old_ptr_after = emu.oldgen_ptr

    # Only 1 cons should survive (24 bytes copied to old-gen)
    copied = old_ptr_after - old_ptr_before
    assert copied == 24, f"Should copy 24 bytes (1 cons), copied {copied}"


@test("gc_nursery_reset", batch="phase3_gc")
def test_gc_nursery_reset():
    """After GC, nursery pointer resets to base."""
    emu = _emu()
    t = emu.thread

    program = [
        _alloc_cons_x(1),
        _alloc_cons_x(2),
        _alloc_cons_x(3),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    assert t.np > NURSERY_BASE
    emu._gc_collect()
    assert t.np == NURSERY_BASE, f"NP should reset to {NURSERY_BASE:#x}, got {t.np:#x}"


@test("gc_keeps_young_object_held_by_old_field", batch="phase3_gc")
def test_gc_keeps_young_object_held_by_old_field():
    """A young cons reachable only through field 12 of an old vector survives.

    The field lies past the header's card.  After the collection it holds
    the cons's old-gen copy with its car intact, and the field's card is
    clean again in both card tables: the emulator's own and the one the
    runtime configured in memory.
    """
    emu = _emu()
    t = emu.thread
    emu.card_table_base = CARD_TABLE

    _load_and_run(emu, [
        _alloc_cons_x(2),                                   # r2 = young cons
        encode_i(Op.LI, 5, 0, tag_fixnum(7) & 0xFFFF),     # r5 = 7
        encode_s(Op.ST_CAR_CDR, 2, 5, 0),                   # car(r2) = 7
        encode_x(Op.HALT_NOP, 0),
    ])
    vec = _old_vector(emu, 16)
    t.regs[3] = vec
    _run_at(emu, [
        encode_s(Op.ST_WB, 3, 2, 12),                       # vec[12] = r2
        encode_i(Op.LI, 2, 0, 0),                           # drop r2 as a root
        encode_x(Op.HALT_NOP, 0),
    ])
    field_addr = ref_address(vec) + 13 * 8
    card_idx = field_addr >> emu.card_shift
    assert emu.mem.load_byte(CARD_TABLE + card_idx) == 0xFF

    emu._gc_collect()

    moved = emu.mem.load_word(field_addr)
    assert is_cons_ref(moved), f"Field 12 should hold a cons ref, got {moved:#x}"
    assert not emu._is_nursery_addr(ref_address(moved)), \
        f"Field 12 still points into the nursery: {ref_address(moved):#x}"
    assert emu.mem.load_word(ref_address(moved) + 8) == tag_fixnum(7)
    assert emu.card_table[card_idx] == 0
    assert emu.mem.load_byte(CARD_TABLE + card_idx) == 0


@test("gc_keeps_young_car_of_old_cons", batch="phase3_gc")
def test_gc_keeps_young_car_of_old_cons():
    """ST.CAR's barrier lets a collection keep the young car of an old cons."""
    emu = _emu()
    t = emu.thread

    _load_and_run(emu, [
        _alloc_cons_x(2),                                   # r2 = young cons
        encode_i(Op.LI, 5, 0, tag_fixnum(9) & 0xFFFF),     # r5 = 9
        encode_s(Op.ST_CAR_CDR, 2, 5, 0),                   # car(r2) = 9
        encode_x(Op.HALT_NOP, 0),
    ])
    old = _old_cons(emu)
    t.regs[3] = old
    _run_at(emu, [
        encode_s(Op.ST_CAR_CDR, 3, 2, 0),                   # car(old) = r2
        encode_i(Op.LI, 2, 0, 0),                           # drop r2 as a root
        encode_x(Op.HALT_NOP, 0),
    ])

    emu._gc_collect()

    car = emu.mem.load_word(ref_address(old) + 8)
    assert is_cons_ref(car), f"The old cons's car should be a cons ref, got {car:#x}"
    assert not emu._is_nursery_addr(ref_address(car)), \
        f"The old cons's car still points into the nursery: {ref_address(car):#x}"
    assert emu.mem.load_word(ref_address(car) + 8) == tag_fixnum(9)


@test("gc_keeps_every_threads_objects", batch="phase3_gc")
def test_gc_keeps_every_threads_objects():
    """The threads of a tile allocate from their own slices of its nursery
    with the tile's one template table, and a collection keeps the young
    objects that every thread's registers hold (spec/05-memory-gc.md
    section 5.1), then empties every slice."""
    emu = _emu(num_threads=2)
    program = [
        _alloc_cons_x(1),                                   # r1 = young cons
        encode_i(Op.SYS_INFO, 3, 1, 0),                     # r3 = thread id
        encode_r(Op.ARITH_RAW, 3, 3, 3, FUNC_ADD),          # r3 = fixnum(id)
        encode_s(Op.ST_CAR_CDR, 1, 3, 0),                   # car(r1) = id
        encode_x(Op.HALT_NOP, 0),
    ]
    for i, insn in enumerate(program):
        emu.mem.store_u32(i * 4, insn)
    slices = (NURSERY_BASE, NURSERY_BASE + NURSERY_SIZE // 2)
    for tid, t in enumerate(emu.threads):
        t.pc = 0
        t.sp = STACK_TOP - tid * 0x1000
        assert t.np == slices[tid], f"Thread {tid} allocates from {t.np:#x}"
    emu.run_round_robin(max_instructions=100)

    for tid, t in enumerate(emu.threads):
        assert t.halted
        assert ref_address(t.regs[1]) == slices[tid]
    emu._gc_collect()

    moved = [ref_address(t.regs[1]) for t in emu.threads]
    assert moved[0] != moved[1]
    for tid, (t, addr) in enumerate(zip(emu.threads, moved)):
        assert is_cons_ref(t.regs[1]) and not emu._is_nursery_addr(addr), \
            f"Thread {tid}'s cons was not promoted: {t.regs[1]:#x}"
        hdr = emu.mem.load_word(addr)
        assert header_subtype(hdr) == HDR_CONS and header_size(hdr) == 2, \
            f"Thread {tid}'s cons has header {hdr:#x}"
        assert emu.mem.load_word(addr + 8) == tag_fixnum(tid)
        assert t.np == slices[tid], f"Thread {tid}'s slice was not emptied"


@test("gc_overflow_triggers_gc", batch="phase3_gc")
def test_gc_overflow_triggers_gc():
    """Nursery overflow automatically triggers GC and allocation succeeds."""
    # Use a tiny nursery that can hold exactly 2 cons cells (48 bytes)
    emu = Emulator(
        mem_size=512 * 1024,
        nursery_base=NURSERY_BASE,
        nursery_size=48,          # fits exactly 2 cons cells
        oldgen_base=OLDGEN_BASE,
        oldgen_size=OLDGEN_SIZE,
    )
    t = emu.thread

    # Allocate 3 cons cells — the 3rd should trigger GC
    # We keep all 3 in registers so the first 2 survive GC
    program = [
        _alloc_cons_x(1),         # fits
        _alloc_cons_x(2),         # fits
        _alloc_cons_x(3),         # overflow → GC → retry → fits
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    assert emu.gc_count >= 1, "GC should have been triggered"
    # All 3 refs should be valid
    assert is_cons_ref(t.regs[1]), f"r1 not cons-ref: {t.regs[1]:#x}"
    assert is_cons_ref(t.regs[2]), f"r2 not cons-ref: {t.regs[2]:#x}"
    assert is_cons_ref(t.regs[3]), f"r3 not cons-ref: {t.regs[3]:#x}"


@test("gc_10_overflows", batch="phase3_gc")
def test_gc_10_overflows():
    """Allocate until nursery overflows 10×, verify live objects survive.
    
    This is the Phase 3 milestone test.
    """
    # Small nursery: 1 cons cell = 24 bytes
    # Every 2nd allocation triggers GC (nursery fills after 1 alloc).
    # With 25 iterations, we get ~ 12-13 GC cycles.
    emu = Emulator(
        mem_size=1024 * 1024,     # 1 MiB
        nursery_base=NURSERY_BASE,
        nursery_size=24,           # fits 1 cons cell
        oldgen_base=OLDGEN_BASE,
        oldgen_size=OLDGEN_SIZE,
    )
    t = emu.thread

    # Build a linked list of 50 fixnums.
    # With nursery of 5 cons cells, after every 5 allocs we overflow → GC.
    # 50 allocs → at least 10 GC cycles.
    #
    # Program: r1 = list head (starts as nil), loop 50 times:
    #   alloc cons → r2
    #   st.car r2 = counter
    #   st.cdr r2 = r1
    #   r1 = r2
    #   counter -= 2 (fixnum sub)
    #   if counter != 0 goto loop
    program = [
        # r1 = NIL (list head)
        encode_i(Op.LI, 1, 0, NIL & 0xFFFF),
        # r5 = loop counter = 50 (fixnum)
        encode_i(Op.LI, 5, 0, tag_fixnum(50) & 0xFFFF),
        # r6 = fixnum(1) for car values (will increment)
        encode_i(Op.LI, 6, 0, tag_fixnum(1) & 0xFFFF),
        # r7 = fixnum(2) — step for counter
        encode_i(Op.LI, 7, 0, tag_fixnum(2) & 0xFFFF),
        # r8 = fixnum(1) for increment
        encode_i(Op.LI, 8, 0, tag_fixnum(1) & 0xFFFF),

        # LOOP (offset 5):
        _alloc_cons_x(2),                           # r2 = new cons
        encode_s(Op.ST_CAR_CDR, 2, 6, 0),           # car(r2) = r6 (counter value)
        encode_s(Op.ST_CAR_CDR, 2, 1, 1),           # cdr(r2) = r1 (old head)
        # r1 = r2 (new head)
        encode_r(Op.ARITH_RAW, 1, 2, 0, FUNC_ADD),  # r1 = r2 + 0 = r2
        # r6 = r6 + r8 (increment car value)
        encode_r(Op.ARITH_FIX, 6, 6, 8, FUNC_ADD_FIX),
        # r5 = r5 - r7 (counter -= 2)
        encode_r(Op.ARITH_FIX, 5, 5, 7, FUNC_SUB_FIX),
        # branch if r5 > 0 → loop
        # BR_COND: rd=register(r5), rs1=cond(BR_FIX_GT), offset=back to LOOP
        # LOOP is at instruction 5, current is at 11, offset = 5 - 12 = -7
        encode_b(Op.BR_COND, 5, BR_FIX_GT, -7 & 0xFFFF),

        # Done — walk the list, sum all car values
        # r3 = sum = 0
        encode_i(Op.LI, 3, 0, 0),
        # r4 = current = r1
        encode_r(Op.ARITH_RAW, 4, 1, 0, FUNC_ADD),

        # WALK (offset 14):
        # if r4 == nil, done
        encode_b(Op.BR_COND, 4, BR_NIL, 5),  # jump to HALT (+5)
        # r9 = car(r4)
        encode_i(Op.LD_CAR_CDR, 9, 4, 0),    # car
        # r3 = r3 + r9
        encode_r(Op.ARITH_FIX, 3, 3, 9, FUNC_ADD_FIX),
        # r4 = cdr(r4)
        encode_i(Op.LD_CAR_CDR, 4, 4, 1),    # cdr
        # goto WALK
        encode_b(Op.BR, 0, 0, -4 & 0xFFFF),

        # HALT (offset 19):
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program, max_instructions=500_000)

    # Verify GC happened at least 10 times
    assert emu.gc_count >= 10, \
        f"Expected >= 10 GC cycles, got {emu.gc_count}"

    # The list has 25 elements (50/2 = 25 iterations) with car values 1..25
    # Wait, counter starts at 50 and decrements by 2 each iteration,
    # checking > 0 after subtraction. So iterations: 50,48,...,2 → 25 iterations.
    # car values: 1,2,3,...,25 (r6 increments by 1 each time)
    # Sum = 25 * 26 / 2 = 325
    expected_sum = tag_fixnum(325)
    actual_sum = t.regs[3]
    assert actual_sum == expected_sum, \
        f"Sum should be {325}, got {untag_fixnum(actual_sum)}"

    # Verify the list head is valid and points to old-gen
    ref = t.regs[1]
    assert is_any_ref(ref), f"r1 should be a ref: {ref:#x}"


@test("gc_stack_refs_updated", batch="phase3_gc")
def test_gc_stack_refs_updated():
    """Refs on the stack are updated during GC."""
    emu = Emulator(
        mem_size=512 * 1024,
        nursery_base=NURSERY_BASE,
        nursery_size=48,          # 2 cons cells
        oldgen_base=OLDGEN_BASE,
        oldgen_size=OLDGEN_SIZE,
    )
    t = emu.thread

    # Allocate a cons, push to stack, then allocate more to trigger GC
    program = [
        _alloc_cons_x(1),                                  # r1 = cons A
        encode_i(Op.LI, 10, 0, tag_fixnum(77) & 0xFFFF),
        encode_s(Op.ST_CAR_CDR, 1, 10, 0),                # car(A) = 77
        encode_r(Op.PUSH_POP, 1, 0, 0, FUNC_PUSH),        # push r1
        _alloc_cons_x(2),                                  # r2 = cons B (fills nursery)
        _alloc_cons_x(3),                                  # r3 = cons C → GC
        encode_r(Op.PUSH_POP, 1, 0, 0, FUNC_POP),         # pop into r1
        # Verify r1 is still valid by reading car
        encode_i(Op.LD_CAR_CDR, 11, 1, 0),                # r11 = car(r1)
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)

    assert emu.gc_count >= 1
    car = t.regs[11]
    assert car == tag_fixnum(77), \
        f"car of stack-saved cons should be 77, got {untag_fixnum(car)}"
