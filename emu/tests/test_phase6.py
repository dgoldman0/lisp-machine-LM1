"""Phase 6 tests — assembler.

Tests that assembly source text assembles to correct binary and runs
correctly on the emulator.
"""

from lm1.testing.harness import test
from lm1.asm import Assembler, AsmError
from lm1.execute import Emulator
from lm1.decode import decode, Op
from lm1.word import (
    NIL, T, tag_fixnum, untag_fixnum,
    is_fixnum, is_cons_ref, ref_address,
    make_header, HDR_CONS, HDR_INSTANCE,
    WORD_MASK,
)

# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------

NURSERY_BASE = 0x1_0000
NURSERY_SIZE = 0x1_0000
OLDGEN_BASE  = 0x2_0000
OLDGEN_SIZE  = 0x2_0000
STACK_TOP    = 0x1_0000


def _emu(**kw):
    defaults = dict(
        mem_size=512 * 1024,
        nursery_base=NURSERY_BASE,
        nursery_size=NURSERY_SIZE,
        oldgen_base=OLDGEN_BASE,
        oldgen_size=OLDGEN_SIZE,
    )
    defaults.update(kw)
    return Emulator(**defaults)


def _asm_and_run(source: str, **kw) -> Emulator:
    """Assemble source, load into emulator, run, return emulator."""
    asm = Assembler()
    words = asm.assemble_to_words(source)
    emu = _emu(**kw)
    for i, w in enumerate(words):
        emu.mem.store_u32(i * 4, w)
    emu.thread.pc = 0
    emu.thread.sp = STACK_TOP
    # Apply any .template directives
    for idx, sub, size, shape in asm.templates:
        emu.thread.header_templates[idx] = make_header(sub, size, shape)
    emu.run(max_instructions=10_000)
    return emu


# ===================================================================
# Batch: phase6_basic — basic assembly and label resolution
# ===================================================================

@test("asm_li_halt", batch="phase6_basic")
def test_asm_li_halt():
    """Assemble LI and HALT, run on emulator."""
    source = """
        LI r5, 42
        HALT
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[5] == 42


@test("asm_add_sub", batch="phase6_basic")
def test_asm_add_sub():
    """ADD and SUB produce correct results."""
    source = """
        LI r1, 100
        LI r2, 30
        ADD r3, r1, r2
        SUB r4, r1, r2
        HALT
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[3] == 130
    assert emu.thread.regs[4] == 70


@test("asm_labels_branch", batch="phase6_basic")
def test_asm_labels_branch():
    """Labels and forward/backward branches resolve correctly."""
    source = """
        LI r1, 0
        BR skip
        LI r1, 99       ; should be skipped
    skip:
        LI r2, 0xAA
        HALT
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[1] == 0
    assert emu.thread.regs[2] == 0xAA


@test("asm_loop_backward_branch", batch="phase6_basic")
def test_asm_loop_backward_branch():
    """Backward branch implements a counted loop using fixnum arithmetic."""
    source = """
    ; Sum fixnum(1)..fixnum(5) = fixnum(15) using tagged add
    .equ FIX_0,  0    ; tag_fixnum(0)
    .equ FIX_1,  2    ; tag_fixnum(1) = 1<<1
    .equ FIX_6, 12    ; tag_fixnum(6) = 6<<1
        LI r1, FIX_0       ; sum
        LI r2, FIX_1       ; counter = fixnum(1)
        LI r3, FIX_6       ; limit = fixnum(6)
    loop:
        ADD.FIX r1, r1, r2
        LI r4, FIX_1
        ADD.FIX r2, r2, r4
        ; CMP r2, r3 → r5 (< → negative fixnum)
        CMP r5, r2, r3
        BR.FIX.LT r5, loop
        HALT
    """
    emu = _asm_and_run(source)
    # fixnum(15) = 30
    assert emu.thread.regs[1] == 30, f"Sum should be fixnum(15)=30, got {emu.thread.regs[1]}"


@test("asm_equ_directive", batch="phase6_basic")
def test_asm_equ_directive():
    """.equ defines named constants usable in instructions."""
    source = """
    .equ MAGIC, 0xDEAD
        LI r5, MAGIC
        HALT
    """
    emu = _asm_and_run(source)
    # 0xDEAD exceeds signed-16 range → LI auto-expands to LI32,
    # loading the exact 32-bit value.
    assert emu.thread.regs[5] == 0xDEAD, f"got {emu.thread.regs[5]:#x}"


@test("asm_nop", batch="phase6_basic")
def test_asm_nop():
    """NOP doesn't affect registers or halt."""
    source = """
        LI r1, 5
        NOP
        NOP
        LI r2, 10
        HALT
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[1] == 5
    assert emu.thread.regs[2] == 10


# ===================================================================
# Batch: phase6_tagged — tagged ops in assembly
# ===================================================================

@test("asm_fixnum_add", batch="phase6_tagged")
def test_asm_fixnum_add():
    """ADD.FIX on tagged fixnums works correctly."""
    source = """
    .equ FIX_10, 20   ; tag_fixnum(10) = 10<<1 = 20
    .equ FIX_20, 40   ; tag_fixnum(20) = 20<<1 = 40
        LI r1, FIX_10
        LI r2, FIX_20
        ADD.FIX r3, r1, r2
        HALT
    """
    emu = _asm_and_run(source)
    # fixnum(10) + fixnum(20) = fixnum(30) = 60
    assert emu.thread.regs[3] == tag_fixnum(30), \
        f"Expected fixnum(30)={tag_fixnum(30)}, got {emu.thread.regs[3]}"


@test("asm_alloc_cons", batch="phase6_tagged")
def test_asm_alloc_cons():
    """ALLOC.CONS creates a cons cell, LD.CAR/LD.CDR access it."""
    source = """
    .equ FIX_1, 2     ; tag_fixnum(1) = 2
    .equ FIX_2, 4     ; tag_fixnum(2) = 4
        LI r1, FIX_1
        LI r2, FIX_2
        ALLOC.CONS r3, r1, r2
        LD.CAR r4, r3
        LD.CDR r5, r3
        HALT
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[4] == tag_fixnum(1), "CAR should be fixnum(1)"
    assert emu.thread.regs[5] == tag_fixnum(2), "CDR should be fixnum(2)"


# ===================================================================
# Batch: phase6_dispatch — call/ret in assembly
# ===================================================================

@test("asm_call_direct_ret", batch="phase6_dispatch")
def test_asm_call_direct_ret():
    """CALL.DIRECT and RET work with labels."""
    source = """
        CALL.DIRECT my_func
        LI r6, 0xDD
        HALT
    my_func:
        LI r5, 0xAA
        RET
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[5] == 0xAA
    assert emu.thread.regs[6] == 0xDD


@test("asm_tailcall_direct", batch="phase6_dispatch")
def test_asm_tailcall_direct():
    """TAILCALL.DIRECT reuses the frame."""
    source = """
        CALL.DIRECT func_a
        LI r6, 0xDD
        HALT
    func_a:
        LI r5, 1
        TAILCALL.DIRECT func_b
    func_b:
        LI r7, 0xEE
        RET
    """
    emu = _asm_and_run(source)
    assert emu.thread.regs[5] == 1
    assert emu.thread.regs[7] == 0xEE
    assert emu.thread.regs[6] == 0xDD


# ===================================================================
# Batch: phase6_memory — raw loads and stores with offsets
# ===================================================================

_OFFSETS = (0, 8, -8, -1, 1023, 1024, 2047, 2048, 0x7FFF, -0x8000)


@test("asm_raw_access_offsets_take_sixteen_bits", batch="phase6_memory")
def test_asm_raw_access_offsets_take_sixteen_bits():
    """Every raw load and store carries its signed byte offset in bits 15:0,
    the field the machines read (spec/07-encoding.md section 5.8), so a
    negative offset and one of 1,024 or more come back whole, and the
    stores keep the base in bits 25:21 and the value in bits 20:16."""
    for mn, op in (("STR", Op.STR), ("STW", Op.STW), ("STH", Op.STH), ("STB", Op.STB),
                   ("LDR", Op.LDR), ("LDW", Op.LDW), ("LDH", Op.LDH), ("LDB", Op.LDB)):
        for off in _OFFSETS:
            word, = Assembler().assemble_to_words(f"{mn} r3, r4, {off}")
            d = decode(word)
            assert (d.opcode, d.rd, d.rs1, d.imm16) == (op, 3, 4, off), \
                f"{mn} r3, r4, {off} decodes as rd={d.rd} rs1={d.rs1} imm16={d.imm16}"


@test("asm_raw_access_offset_past_sixteen_bits_refused", batch="phase6_memory")
def test_asm_raw_access_offset_past_sixteen_bits_refused():
    """An offset that sixteen signed bits cannot hold is refused rather than
    cut down to another offset."""
    for line in ("STR r1, r2, 32768", "STB r1, r2, -32769", "LDR r1, r2, 0x10000"):
        try:
            Assembler().assemble(line)
        except AsmError as e:
            assert "16 signed bits" in str(e), str(e)
        else:
            raise AssertionError("%r assembled" % line)


@test("asm_stores_at_negative_and_far_offsets", batch="phase6_memory")
def test_asm_stores_at_negative_and_far_offsets():
    """Stores at negative offsets and at offsets of 1,024 or more land where
    the loads at the same offsets find them, and nowhere else."""
    source = """
        LI32 r1, 0x40000        ; the base
        LI   r2, 0x1111
        LI   r3, 0x2222
        LI   r4, 0x3333
        LI   r5, 0x44
        LI   r6, 0x5555
        STR  r1, r2, -8
        STR  r1, r3, 1024
        STW  r1, r4, -1028
        STB  r1, r5, 2047
        STH  r1, r6, -0x8000
        STR  r1, r2, 0x7FF8
        LDR  r10, r1, -8
        LDR  r11, r1, 1024
        LDW  r12, r1, -1028
        LDB  r13, r1, 2047
        LDH  r14, r1, -0x8000
        LDR  r15, r1, 0x7FF8
        LDR  r16, r1, 0         ; untouched
        HALT
    """
    emu = _asm_and_run(source)
    r = emu.thread.regs
    assert (r[10], r[11], r[12], r[13], r[14], r[15]) == (
        0x1111, 0x2222, 0x3333, 0x44, 0x5555, 0x1111), [hex(v) for v in r[10:16]]
    assert r[16] == 0, "a store landed at the base itself"
    mem = emu.mem
    assert mem.load_word(0x40000 - 8) == 0x1111
    assert mem.load_word(0x40000 + 1024) == 0x2222
    assert mem.load_u32_data(0x40000 - 1028) == 0x3333
    assert mem.load_byte(0x40000 + 2047) == 0x44
    assert mem.load_u16(0x40000 - 0x8000) == 0x5555


# ===================================================================
# Batch: phase6_messaging — SEND, RECV and TRY.RECV name descriptors
# ===================================================================

@test("asm_send_recv_descriptors", batch="phase6_messaging")
def test_asm_send_recv_descriptors():
    """SEND, RECV and TRY.RECV take a register holding a queue descriptor,
    the fixnum tile * 4 + queue (spec/02-isa.md section 8), in the fields
    spec/07-encoding.md section 5.6 gives it, and run on the emulator."""
    source = f"""
        LI r1, {tag_fixnum(0 * 4 + 3)}    ; tile 0, queue 3
        LI r2, {tag_fixnum(42)}
        SEND r1, r2
        TRY.RECV r3, r4, r1
        TRY.RECV r5, r6, r1
        SEND r1, r2
        RECV r7, r1
        HALT
    """
    words = Assembler().assemble_to_words(source)
    send, try_recv, recv = decode(words[2]), decode(words[3]), decode(words[6])
    assert (send.opcode, send.rd, send.rs1, send.rs2, send.imm11) == (Op.SEND, 1, 2, 0, 0)
    assert (try_recv.opcode, try_recv.rd, try_recv.rs1, try_recv.func) == (Op.RECV, 3, 1, 4)
    assert (recv.opcode, recv.rd, recv.rs1, recv.func, recv.imm16) == (Op.RECV, 7, 1, 0, 0)
    emu = _asm_and_run(source)
    r = emu.thread.regs
    assert (r[3], r[4]) == (tag_fixnum(42), T), "TRY.RECV takes the word SEND left"
    assert (r[5], r[6]) == (NIL, NIL), "TRY.RECV on the empty queue"
    assert r[7] == tag_fixnum(42), "RECV takes the second word"


@test("asm_try_recv_status_not_r0", batch="phase6_messaging")
def test_asm_try_recv_status_not_r0():
    """TRY.RECV's status register goes in func, where 0 encodes RECV, so
    the assembler refuses r0 for it."""
    try:
        Assembler().assemble("TRY.RECV r3, r0, r1")
    except AsmError as e:
        assert "r0" in str(e)
    else:
        raise AssertionError("TRY.RECV with its status in r0 assembled")


@test("asm_send_takes_a_register", batch="phase6_messaging")
def test_asm_send_takes_a_register():
    """A queue number in place of the descriptor register is refused."""
    for line in ("SEND r1, 0", "RECV r1, 0", "TRY.RECV r1, r2, 0"):
        try:
            Assembler().assemble(line)
        except AsmError:
            continue
        raise AssertionError("%r assembled" % line)


# ===================================================================
# Batch: phase6_milestone — milestone test in assembly
# ===================================================================

@test("asm_sum_1_to_10", batch="phase6_milestone")
def test_asm_sum_1_to_10():
    """MILESTONE: sum 1..10 as fixnums → fixnum(55) in assembly."""
    source = """
    ; Sum fixnum(1) through fixnum(10) → r5
    .equ FIX_0,  0        ; tag_fixnum(0)
    .equ FIX_1,  2        ; tag_fixnum(1)
    .equ FIX_11, 22       ; tag_fixnum(11)

        LI r5, FIX_0      ; sum = 0
        LI r2, FIX_1      ; i = 1
        LI r3, FIX_11     ; limit = 11
        LI r4, FIX_1      ; step = 1
    loop:
        ADD.FIX r5, r5, r2   ; sum += i
        ADD.FIX r2, r2, r4   ; i += 1
        CMP r7, r2, r3       ; r7 = cmp(i, limit)
        BR.FIX.LT r7, loop   ; if i < limit, loop
        HALT
    """
    emu = _asm_and_run(source)
    result = emu.thread.regs[5]
    assert result == tag_fixnum(55), \
        f"Sum should be fixnum(55)={tag_fixnum(55)}, got {result} ({untag_fixnum(result)})"


@test("asm_cons_list_walk", batch="phase6_milestone")
def test_asm_cons_list_walk():
    """Build a list of 5 cons cells in assembly, walk it summing values."""
    source = """
    ; Build list: (1 2 3 4 5)
    ; Then walk it summing car values → 15
    .equ FIX_1, 2
    .equ FIX_2, 4
    .equ FIX_3, 6
    .equ FIX_4, 8
    .equ FIX_5, 10
    .equ NIL_VAL, 5       ; nil = 0x05

        ; Build list from end: (5 . nil), (4 . prev), ...
        LI r10, NIL_VAL   ; nil
        LI r1, FIX_5
        ALLOC.CONS r11, r1, r10    ; (5 . nil)
        LI r1, FIX_4
        ALLOC.CONS r11, r1, r11    ; (4 5)
        LI r1, FIX_3
        ALLOC.CONS r11, r1, r11    ; (3 4 5)
        LI r1, FIX_2
        ALLOC.CONS r11, r1, r11    ; (2 3 4 5)
        LI r1, FIX_1
        ALLOC.CONS r11, r1, r11    ; (1 2 3 4 5)

        ; Walk list: r5 = sum, r12 = current
        LI r5, 0          ; sum = fixnum(0)
        MOV r12, r11      ; current = list head
    walk:
        ; check if nil (EQ for cross-tag comparison)
        EQ r7, r12, r10   ; r7 = T if current==nil, else NIL
        BR.T r7, done
        ; car → r1
        LD.CAR r1, r12
        ADD.FIX r5, r5, r1
        ; cdr → r12
        LD.CDR r12, r12
        BR walk
    done:
        HALT
    """
    emu = _asm_and_run(source)
    result = emu.thread.regs[5]
    assert result == tag_fixnum(15), \
        f"Sum should be fixnum(15), got {untag_fixnum(result)}"


@test("asm_data_directive", batch="phase6_milestone")
def test_asm_data_directive():
    """.word and .u32 directives emit data correctly."""
    asm = Assembler()
    source = """
    .word 0xDEADBEEF12345678
    .u32  0xCAFEBABE
    """
    binary = asm.assemble(source)
    # First 8 bytes: 64-bit word (little-endian)
    val64 = int.from_bytes(binary[0:8], 'little')
    assert val64 == 0xDEADBEEF12345678, f"got {val64:#x}"
    # Next 4 bytes: 32-bit word
    val32 = int.from_bytes(binary[8:12], 'little')
    assert val32 == 0xCAFEBABE, f"got {val32:#x}"


@test("asm_error_unknown_mnemonic", batch="phase6_milestone")
def test_asm_error_unknown_mnemonic():
    """Unknown mnemonic raises AsmError."""
    asm = Assembler()
    caught = False
    try:
        asm.assemble("XYZZY r1, r2")
    except AsmError as e:
        caught = True
        assert "unknown mnemonic" in str(e).lower()
    assert caught, "Should raise AsmError for unknown mnemonic"
