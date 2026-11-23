"""Phase 1 tests — scalar execution, branches, I/O, tagged arith, type tests.

Each test is a self-contained LM-1 program that is hand-assembled,
loaded into a fresh emulator instance, and run to completion.
"""

from __future__ import annotations

import io

from lm1.testing.harness import test
from lm1.asm import Assembler
from lm1.decode import Op, encode_i, encode_r, encode_b, encode_x
from lm1.execute import Emulator, EMU_TRAP_PUTCHAR, EMU_TRAP_MEM_BYTE
from lm1.traps import LM1Trap, TRAP_FIXNUM_OVERFLOW
from lm1.word import tag_fixnum, untag_fixnum, T, NIL


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _emu(**kwargs) -> Emulator:
    """Create a small emulator with captured stdout."""
    stdout = kwargs.pop("stdout", io.StringIO())
    return Emulator(mem_size=64 * 1024, stdout=stdout, **kwargs)


def _emu_with_output(**kwargs) -> tuple[Emulator, io.StringIO]:
    """Create emulator + its captured output buffer."""
    buf = io.StringIO()
    emu = Emulator(mem_size=64 * 1024, stdout=buf, **kwargs)
    return emu, buf


def _load_and_run(emu: Emulator, program: list[int], **kwargs) -> None:
    emu.mem.load_instructions(0, program)
    emu.thread.pc = 0
    emu.thread.sp = 64 * 1024
    emu.run(**kwargs)


# ===================================================================
# Batch: phase1_io
# ===================================================================

@test("print_lm1", batch="phase1_io")
def test_print_lm1():
    """Prints 'LM-1\\n' via TRAP #0x80 (putchar)."""
    instructions: list[int] = []
    for ch in "LM-1\n":
        val = ord(ch) << 1  # tagged fixnum
        instructions.append(encode_i(Op.LI, 1, 0, val & 0xFFFF))
        instructions.append(encode_x(Op.TRAP, EMU_TRAP_PUTCHAR))
    instructions.append(encode_x(Op.HALT_NOP, 0))

    emu, out = _emu_with_output()
    _load_and_run(emu, instructions)

    result = out.getvalue()
    assert result == "LM-1\n", f"Expected 'LM-1\\n', got {result!r}"


@test("print_empty", batch="phase1_io")
def test_print_empty():
    """Just HALT — should produce no output."""
    emu, out = _emu_with_output()
    _load_and_run(emu, [encode_x(Op.HALT_NOP, 0)])
    assert out.getvalue() == "", f"Expected empty output, got {out.getvalue()!r}"


# ===================================================================
# Batch: phase1_scalar
# ===================================================================

@test("scalar_add_sub_mul", batch="phase1_scalar")
def test_scalar_add_sub_mul():
    """Raw ADD, SUB, MUL on untagged 64-bit values."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 42),
        encode_i(Op.LI, 2, 0, 10),
        encode_r(Op.ARITH_RAW, 3, 1, 2, 0),   # ADD → 52
        encode_r(Op.ARITH_RAW, 4, 1, 2, 1),   # SUB → 32
        encode_r(Op.ARITH_RAW, 5, 1, 2, 2),   # MUL → 420
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 52
    assert emu.thread.regs[4] == 32
    assert emu.thread.regs[5] == 420


@test("scalar_div_mod", batch="phase1_scalar")
def test_scalar_div_mod():
    """Raw DIV and MOD."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 100),
        encode_i(Op.LI, 2, 0, 7),
        encode_r(Op.ARITH_RAW, 3, 1, 2, 3),   # DIV → 14
        encode_r(Op.ARITH_RAW, 4, 1, 2, 4),   # MOD → 2
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 14
    assert emu.thread.regs[4] == 2


@test("bitwise_and_or_xor", batch="phase1_scalar")
def test_bitwise():
    """AND, OR, XOR."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0xFF),
        encode_i(Op.LI, 2, 0, 0x0F),
        encode_r(Op.BITWISE, 3, 1, 2, 0),   # AND → 0x0F
        encode_r(Op.BITWISE, 4, 1, 2, 1),   # OR  → 0xFF
        encode_r(Op.BITWISE, 5, 1, 2, 2),   # XOR → 0xF0
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 0x0F
    assert emu.thread.regs[4] == 0xFF
    assert emu.thread.regs[5] == 0xF0


@test("bitwise_shifts", batch="phase1_scalar")
def test_shifts():
    """SHL, SHR."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0xFF),
        encode_i(Op.LI, 2, 0, 4),
        encode_r(Op.BITWISE, 3, 1, 2, 3),   # SHL → 0xFF0
        encode_r(Op.BITWISE, 4, 1, 2, 4),   # SHR → 0x0F
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 0xFF0
    assert emu.thread.regs[4] == 0x0F


@test("bitwise_not", batch="phase1_scalar")
def test_not():
    """NOT (bitwise complement)."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0),
        encode_r(Op.BITWISE, 2, 1, 0, 6),   # NOT 0 → all ones
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[2] == 0xFFFF_FFFF_FFFF_FFFF


@test("li_sign_extend", batch="phase1_scalar")
def test_li_sign_extend():
    """LI with negative immediate (sign extension)."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, (-1) & 0xFFFF),     # LI r1, -1
        encode_i(Op.LI, 2, 0, (-100) & 0xFFFF),   # LI r2, -100
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[1] == 0xFFFF_FFFF_FFFF_FFFF, f"got {emu.thread.regs[1]:#x}"
    assert emu.thread.regs[2] == ((-100) & 0xFFFF_FFFF_FFFF_FFFF), f"got {emu.thread.regs[2]:#x}"


@test("lui", batch="phase1_scalar")
def test_lui():
    """LUI sets bits 31:16."""
    emu = _emu()
    program = [
        encode_i(Op.LUI, 1, 0, 0xABCD),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[1] == 0xABCD_0000, f"got {emu.thread.regs[1]:#x}"


@test("li_lui_compose", batch="phase1_scalar")
def test_li_lui_compose():
    """Compose a 32-bit value from LI + LUI + OR."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0x5678),
        encode_i(Op.LUI, 2, 0, 0x1234),
        encode_r(Op.BITWISE, 1, 1, 2, 1),   # OR → 0x1234_5678
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[1] == 0x1234_5678


@test("li32_load_large", batch="phase1_scalar")
def test_li32_load_large():
    """LI32 loads a full 32-bit immediate from the following word."""
    emu = _emu()
    # LI32 opcode word: rd=1, followed by the 32-bit immediate 0x00FF0000
    program = [
        encode_i(Op.LI32, 1, 0, 0),   # LI32 r1 (opcode word)
        0x00FF0000,                     # immediate word
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[1] == 0x00FF0000, f"got {emu.thread.regs[1]:#x}"


@test("li32_asm_auto_expand", batch="phase1_scalar")
def test_li32_asm_auto_expand():
    """LI auto-expands to LI32 when value exceeds 16-bit signed range."""
    asm = Assembler()
    words = asm.assemble_to_words("""\
_start:
    LI r1, 0xFF0000
    HALT
""")
    # Should emit 3 words: LI32 opcode, 0xFF0000 immediate, HALT
    assert len(words) == 3, f"Expected 3 words, got {len(words)}"
    # Confirm the opcode is LI32
    opcode = (words[0] >> 26) & 0x3F
    assert opcode == Op.LI32, f"Expected LI32 opcode, got {opcode}"
    # Confirm the immediate word
    assert words[1] == 0xFF0000, f"Expected 0xFF0000, got {words[1]:#x}"


@test("li32_asm_small_stays_li", batch="phase1_scalar")
def test_li32_asm_small_stays_li():
    """LI stays as single-word LI when value fits in 16-bit signed."""
    asm = Assembler()
    words = asm.assemble_to_words("""\
_start:
    LI r1, 42
    HALT
""")
    # Should emit 2 words: LI, HALT
    assert len(words) == 2, f"Expected 2 words, got {len(words)}"
    # Confirm the opcode is plain LI
    opcode = (words[0] >> 26) & 0x3F
    assert opcode == Op.LI, f"Expected LI opcode, got {opcode}"


# ===================================================================
# Batch: phase1_memory
# ===================================================================

@test("load_store_word", batch="phase1_memory")
def test_load_store_word():
    """STR then LDR round-trips a 64-bit value."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0x5678),
        encode_i(Op.LUI, 2, 0, 0x1234),
        encode_r(Op.BITWISE, 1, 1, 2, 1),    # r1 = 0x1234_5678
        encode_i(Op.LI, 10, 0, 0x1000),      # r10 = data address
        encode_i(Op.STR, 10, 1, 0),           # mem[r10] = r1
        encode_i(Op.LI, 1, 0, 0),             # clear r1
        encode_i(Op.LDR, 3, 10, 0),           # r3 = mem[r10]
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 0x1234_5678


@test("load_store_offset", batch="phase1_memory")
def test_load_store_offset():
    """STR/LDR with non-zero offset."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 0x3EEF),        # fits in 15 bits (positive)
        encode_i(Op.LI, 10, 0, 0x1000),
        encode_i(Op.STR, 10, 1, 8),           # mem[r10+8] = r1
        encode_i(Op.LDR, 3, 10, 8),           # r3 = mem[r10+8]
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[3] == 0x3EEF


@test("mem_byte_address_wraps", batch="phase1_memory")
def test_mem_byte_address_wraps():
    """TRAP 0x84 adds its offset to its address in 64 bits.

    Address 0 with offset -1 names the last byte of the 64-bit address
    space, which lies outside memory, and not the last byte of memory.
    """
    emu = _emu()
    emu.mem.store_byte(64 * 1024 - 1, 0xAB)   # the last byte of memory
    emu.thread.regs[1] = tag_fixnum(0)        # load
    emu.thread.regs[2] = 0                    # address
    emu.thread.regs[3] = tag_fixnum(-1)       # offset
    try:
        _load_and_run(emu, [encode_x(Op.TRAP, EMU_TRAP_MEM_BYTE), encode_x(Op.HALT_NOP, 0)])
    except IndexError:
        pass
    else:
        raise AssertionError(f"Read byte {untag_fixnum(emu.thread.regs[1]):#x} "
                             "below address 0")
    assert emu.thread.regs[1] == tag_fixnum(0)


# ===================================================================
# Batch: phase1_branch
# ===================================================================

@test("branch_unconditional", batch="phase1_branch")
def test_br_unconditional():
    """BR skips over an instruction."""
    emu, out = _emu_with_output()
    program = [
        # 0x00: BR +3     → skip to 0x0C
        encode_b(Op.BR, 0, 0, 3),
        # 0x04: LI r1, 84  (should be skipped)
        encode_i(Op.LI, 1, 0, 84),
        # 0x08: TRAP putchar (should be skipped)
        encode_x(Op.TRAP, EMU_TRAP_PUTCHAR),
        # 0x0C: HALT
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert out.getvalue() == "", "Should have skipped the print"


@test("branch_loop_countdown", batch="phase1_branch")
def test_branch_loop():
    """Count down from 5, printing '*' each time → '*****'."""
    emu, out = _emu_with_output()
    program = [
        # 0x00: LI r10, 10    (fixnum 5)
        encode_i(Op.LI, 10, 0, 10),
        # 0x04: LI r11, 2     (fixnum 1)
        encode_i(Op.LI, 11, 0, 2),
        # 0x08: LI r1, 84     ('*' as fixnum)
        encode_i(Op.LI, 1, 0, 84),
        # 0x0C: BR.FIX.EQ r10, +5   → if r10==0 goto HALT
        encode_b(Op.BR_COND, 10, 3, 5),
        # 0x10: TRAP putchar
        encode_x(Op.TRAP, EMU_TRAP_PUTCHAR),
        # 0x14: SUB r10, r10, r11
        encode_r(Op.ARITH_RAW, 10, 10, 11, 1),
        # 0x18: BR -3   → back to 0x0C
        encode_b(Op.BR, 0, 0, (-3) & 0xFFFF),
        # 0x1C: NOP
        encode_x(Op.HALT_NOP, 1 << 21),
        # 0x20: HALT
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert out.getvalue() == "*****"


@test("branch_br_t", batch="phase1_branch")
def test_br_t():
    """BR.T: branch taken on truthy, not taken on nil/0."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, T & 0xFFFF),
        # BR.T r1, +2  → skip next insn
        encode_b(Op.BR_COND, 1, 0, 2),
        # r5 = 999 (should be skipped)
        encode_i(Op.LI, 5, 0, 999),
        # r6 = 1 (marker that we got here)
        encode_i(Op.LI, 6, 0, 1),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[5] == 0, "BR.T should have skipped LI r5"
    assert emu.thread.regs[6] == 1, "Should have reached LI r6"


@test("branch_br_nil", batch="phase1_branch")
def test_br_nil():
    """BR.NIL: branch taken on nil."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, NIL & 0xFFFF),
        # BR.NIL r1, +2  → skip next insn
        encode_b(Op.BR_COND, 1, 1, 2),
        encode_i(Op.LI, 5, 0, 999),
        encode_i(Op.LI, 6, 0, 1),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[5] == 0, "BR.NIL should have skipped LI r5"
    assert emu.thread.regs[6] == 1


@test("branch_below_zero_wraps", batch="phase1_branch")
def test_branch_below_zero_wraps():
    """The PC is a 64-bit register, so a branch below address 0 wraps to
    the top of the address space, and the fetch there lies outside memory."""
    emu = _emu()
    try:
        _load_and_run(emu, [encode_b(Op.BR, 0, 0, (-1) & 0xFFFF)], max_instructions=10)
    except IndexError:
        pass
    else:
        raise AssertionError(f"Fetched from {emu.thread.pc:#x} and ran on")
    assert emu.thread.pc == 0xFFFF_FFFF_FFFF_FFFC, f"PC = {emu.thread.pc:#x}"
    assert emu.instruction_count == 1


# ===================================================================
# Batch: phase1_tagged
# ===================================================================

@test("tagged_add_fix", batch="phase1_tagged")
def test_add_fix():
    """ADD.FIX: 30 + 12 = 42 (as tagged fixnums)."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, tag_fixnum(30) & 0xFFFF),
        encode_i(Op.LI, 2, 0, tag_fixnum(12) & 0xFFFF),
        encode_r(Op.ARITH_FIX, 3, 1, 2, 0),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert untag_fixnum(emu.thread.regs[3]) == 42


@test("tagged_sub_fix", batch="phase1_tagged")
def test_sub_fix():
    """SUB.FIX: 30 - 12 = 18."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, tag_fixnum(30) & 0xFFFF),
        encode_i(Op.LI, 2, 0, tag_fixnum(12) & 0xFFFF),
        encode_r(Op.ARITH_FIX, 3, 1, 2, 1),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert untag_fixnum(emu.thread.regs[3]) == 18


@test("tagged_add_fix_imm", batch="phase1_tagged")
def test_add_fix_imm():
    """ADD.FIX.IMM: 10 + imm(5) = 15."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, tag_fixnum(10) & 0xFFFF),
        encode_i(Op.ADD_FIX_IMM, 2, 1, tag_fixnum(5) & 0xFFFF),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert untag_fixnum(emu.thread.regs[2]) == 15


@test("tst_fixnum", batch="phase1_tagged")
def test_tst_fixnum():
    """TST: fixnum predicate."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, tag_fixnum(42) & 0xFFFF),
        encode_i(Op.TST, 2, 1, 0),   # TAG_FIXNUM → T
        encode_i(Op.TST, 3, 1, 1),   # TAG_REF    → NIL
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[2] == T
    assert emu.thread.regs[3] == NIL


@test("tst_nil", batch="phase1_tagged")
def test_tst_nil():
    """TST: nil predicate."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, NIL & 0xFFFF),
        encode_i(Op.TST, 2, 1, 4),   # TAG_NIL → T
        encode_i(Op.TST, 3, 1, 0),   # TAG_FIXNUM → NIL (nil is not fixnum)
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[2] == T
    assert emu.thread.regs[3] == NIL


@test("eq_identity", batch="phase1_tagged")
def test_eq():
    """EQ: word-level identity."""
    emu = _emu()
    program = [
        encode_i(Op.LI, 1, 0, 42),
        encode_i(Op.LI, 2, 0, 42),
        encode_i(Op.LI, 3, 0, 99),
        encode_r(Op.CMP_TAGGED, 4, 1, 2, 1),   # EQ → T
        encode_r(Op.CMP_TAGGED, 5, 1, 3, 1),   # EQ → NIL
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[4] == T
    assert emu.thread.regs[5] == NIL


def _run_expecting_trap(emu, program, code):
    try:
        _load_and_run(emu, program)
    except LM1Trap as trap:
        assert trap.code == code, f"Wrong trap: {trap.code:#x}"
    else:
        raise AssertionError(f"No trap {code:#x}")


@test("mul_fix_range", batch="phase1_tagged")
def test_mul_fix_range():
    """MUL.FIX returns a product inside the fixnum range and traps on one
    outside it (spec/02-isa.md section 3.2).

    -2**31 times 2**31 is -2**62, the most negative fixnum.  2**31 times
    2**31 is 2**62, one past the largest.  2**40 times 2**40 is 2**80, whose
    low 64 bits are all zero, so the product's truncated word alone would
    read as fixnum 0.
    """
    for a, b, product in ((-(1 << 31), 1 << 31, -(1 << 62)), (-3, 5, -15)):
        emu = _emu()
        emu.thread.regs[1] = tag_fixnum(a)
        emu.thread.regs[2] = tag_fixnum(b)
        _load_and_run(emu, [encode_r(Op.ARITH_FIX, 3, 1, 2, 2), encode_x(Op.HALT_NOP, 0)])
        assert emu.thread.regs[3] == tag_fixnum(product), \
            f"{a} * {b}: got {untag_fixnum(emu.thread.regs[3])}"
    for a, b in ((1 << 31, 1 << 31), (1 << 40, 1 << 40), (-(1 << 62), -1)):
        emu = _emu()
        emu.thread.regs[1] = tag_fixnum(a)
        emu.thread.regs[2] = tag_fixnum(b)
        _run_expecting_trap(emu, [encode_r(Op.ARITH_FIX, 3, 1, 2, 2),
                                  encode_x(Op.HALT_NOP, 0)], TRAP_FIXNUM_OVERFLOW)
        assert emu.thread.regs[3] == 0, f"{a} * {b} wrote {emu.thread.regs[3]:#x}"


@test("div_fix_exact", batch="phase1_tagged")
def test_div_fix_exact():
    """DIV.FIX truncates toward zero and is exact for the largest fixnums,
    which a division through floating point rounds; the most negative
    fixnum divided by -1 leaves the fixnum range and traps."""
    big = (1 << 62) - 1
    for a, b, quotient in ((big, 1, big), (big, -1, -big), (-7, 2, -3), (7, -2, -3),
                           (-(1 << 62), 1, -(1 << 62))):
        emu = _emu()
        emu.thread.regs[1] = tag_fixnum(a)
        emu.thread.regs[2] = tag_fixnum(b)
        _load_and_run(emu, [encode_r(Op.ARITH_FIX, 3, 1, 2, 3), encode_x(Op.HALT_NOP, 0)])
        assert emu.thread.regs[3] == tag_fixnum(quotient), \
            f"{a} / {b}: got {untag_fixnum(emu.thread.regs[3])}"
    emu = _emu()
    emu.thread.regs[1] = tag_fixnum(-(1 << 62))
    emu.thread.regs[2] = tag_fixnum(-1)
    _run_expecting_trap(emu, [encode_r(Op.ARITH_FIX, 3, 1, 2, 3),
                              encode_x(Op.HALT_NOP, 0)], TRAP_FIXNUM_OVERFLOW)


# ===================================================================
# Batch: phase1_system
# ===================================================================

@test("halt", batch="phase1_system")
def test_halt():
    """HALT stops execution."""
    emu = _emu()
    _load_and_run(emu, [encode_x(Op.HALT_NOP, 0)])
    assert emu.thread.halted


@test("nop", batch="phase1_system")
def test_nop():
    """NOP followed by HALT."""
    emu = _emu()
    program = [
        encode_x(Op.HALT_NOP, 1 << 21),   # NOP
        encode_x(Op.HALT_NOP, 1 << 21),   # NOP
        encode_x(Op.HALT_NOP, 0),          # HALT
    ]
    _load_and_run(emu, program)
    assert emu.thread.halted
    assert emu.instruction_count == 3


@test("cycle_count", batch="phase1_system")
def test_cycle_count():
    """CYCLE reads the cycles the core spent before it, which the timing
    model gives a NOP one each (spec/02-isa.md section 11.6)."""
    emu = _emu()
    program = [
        encode_x(Op.HALT_NOP, 1 << 21),   # NOP
        encode_x(Op.HALT_NOP, 1 << 21),   # NOP
        encode_x(Op.HALT_NOP, 1 << 21),   # NOP
        # SYS_INFO: Rd=r1, sub=2 (CYCLE)
        encode_i(Op.SYS_INFO, 1, 2, 0),
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    assert emu.thread.regs[1] == 3, f"CYCLE: expected 3, got {emu.thread.regs[1]}"
    assert emu.instruction_count == 5 and emu.cycles == 5


@test("cycle_count_is_the_cores", batch="phase1_system")
def test_cycle_count_is_the_cores():
    """The cycle count belongs to the core, so both threads of a tile read
    one count, which advances with every instruction either of them issues,
    while each thread's INSTRET counts its own.  Every instruction here
    costs one cycle."""
    emu = _emu(num_threads=2)
    nop, halt = encode_x(Op.HALT_NOP, 1 << 21), encode_x(Op.HALT_NOP, 0)
    cycle = encode_i(Op.SYS_INFO, 1, 2, 0)     # CYCLE r1
    instret = encode_i(Op.SYS_INFO, 2, 15, 0)  # INSTRET r2
    emu.mem.load_instructions(0, [nop, cycle, instret, halt])
    emu.mem.load_instructions(0x100, [nop, nop, cycle, instret, halt])
    emu.threads[0].pc, emu.threads[1].pc = 0, 0x100
    emu.run_round_robin(max_instructions=100)
    # One instruction of each thread in turn: thread 0's CYCLE is the third
    # instruction the core issues, and thread 1's the sixth.
    assert emu.threads[0].regs[1] == 2 and emu.threads[0].regs[2] == 2
    assert emu.threads[1].regs[1] == 5 and emu.threads[1].regs[2] == 3
    assert emu.instruction_count == 9 and emu.cycles == 9
    assert emu.threads[0].instret == 4 and emu.threads[1].instret == 5


@test("sys_info_writes_only_rd", batch="phase1_system")
def test_sys_info_writes_only_rd():
    """SYS_INFO writes its result to Rd and leaves every other register,
    including the one its selector bits would name, as it was."""
    emu = _emu()
    for r in range(4):
        emu.thread.regs[r] = 0x1000 + r
    program = [
        encode_i(Op.SYS_INFO, 1, 2, 0),   # CYCLE r1
        encode_i(Op.SYS_INFO, 2, 0, 0),   # TILE.ID r2
        encode_i(Op.SYS_INFO, 1, 3, 0),   # TRAP.CAUSE r1
        encode_x(Op.HALT_NOP, 0),
    ]
    _load_and_run(emu, program)
    regs = emu.thread.regs
    assert regs[0] == 0x1000, f"r0 = {regs[0]:#x}"
    assert regs[1] == 0, f"r1 = {regs[1]:#x}"       # no trap yet
    assert regs[2] == 0, f"r2 = {regs[2]:#x}"       # tile 0
    assert regs[3] == 0x1003, f"r3 = {regs[3]:#x}"


@test("max_instructions_limit", batch="phase1_system", timeout=5.0)
def test_max_instructions():
    """max_instructions parameter stops execution."""
    emu = _emu()
    # Infinite NOP loop: NOP then BR -1
    program = [
        encode_x(Op.HALT_NOP, 1 << 21),       # NOP
        encode_b(Op.BR, 0, 0, (-1) & 0xFFFF), # BR -1 (back to NOP)
    ]
    _load_and_run(emu, program, max_instructions=100)
    assert emu.instruction_count == 100
    assert not emu.thread.halted
