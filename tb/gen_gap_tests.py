#!/usr/bin/env python3
"""
Generate tests for identified coverage gaps in the LM-1 RTL testbench.

Tests cover:
  30: DIV/MOD (raw unsigned)
  31: DIV.FIX (signed fixnum division)
  32: TST all tag types (0-7)
  33: LI32 / large immediates
  34: Sub-word memory ops (LDB/STB/LDH/STH/LDW/STW)
  35: SEND/RECV (message queue round-trip)
  36: CAS.TAGGED (compare-and-swap)
  37: FAA (fetch-and-add)
  38: PUSH.MULTI / POP.MULTI
  39: TST.SHAPE (header-based type test)
  40: JR / TAILCALL.DIR
  41: CALL.CLOSURE
  42: Backward branches and BR.T edge cases
  45: MUL.FIX and DIV.FIX trap when the exact result leaves the fixnum range
  46: An ALU fault saves its own PC, so ERET retries it
  47: ALLOCV and ALLOC.CLOSURE zero every payload word; ALLOCV's length is a fixnum
  48: FAA traps on a word or a delta that is not a fixnum
  49: ERET.NEXT resumes after the faulting instruction; ERET retries it
  50: CALL.REG calls the address in a register, with the frame every call makes
  51: reserved encodings trap TRAP_UNIMPLEMENTED; TRAP 0x70 and 0x7F vector
  52: ALLOC.CONS takes car and cdr registers, read before np moves; r0 gives nil
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'emu'))
from lm1.asm import Assembler
from lm1.decode import Op, encode_r, encode_i, encode_s, encode_b, encode_x
from lm1.word import tag_fixnum, make_header, make_ref, make_char, NIL, T

TB_DIR = os.path.join(os.path.dirname(__file__), 'tests')

# Memory layout constants
NURSERY_BASE   = 0x7000
NURSERY_LIMIT  = 0x7FF0
STACK_TOP      = 0x4000
TRAP_TABLE     = 0x5000
CARD_TABLE     = 0x6000
CARD_SHIFT     = 6
GEN_BOUNDARY_V = 0x7800


# A handler that counts the trap in r20, keeps its cause in r22 and goes on
# after the faulting instruction.  It uses r22 to r24.
SKIP_HANDLER = """\
skip_handler:
    TRAP.CAUSE r22
    LI  r23, 1
    ADD r20, r20, r23
    TRAP.PC r24
    LI  r23, 4
    ADD r24, r24, r23
    JR  r24
"""

# The nursery's words before a program allocates, so that a word an
# allocation leaves unzeroed shows.  0xBAD5 is odd, so no fixnum.
NURSERY_FILL = 0xBAD5


def nursery_garbage(words: int) -> str:
    return f"    .org {NURSERY_BASE:#x}\n" + f"    .word {NURSERY_FILL:#x}\n" * words


def emit_test(name: str, source: str, expected: dict[str, int]):
    """Assemble source, write .hex and .expected files."""
    asm = Assembler()
    binary = asm.assemble(source)
    while len(binary) % 8:
        binary += b'\x00'
    words = []
    for i in range(0, len(binary), 8):
        chunk = binary[i:i+8].ljust(8, b'\x00')
        words.append(int.from_bytes(chunk, 'little'))
    while len(words) < (1 << 16):
        words.append(0)

    hex_path = os.path.join(TB_DIR, f'{name}.hex')
    exp_path = os.path.join(TB_DIR, f'{name}.expected')

    with open(hex_path, 'w') as f:
        for w in words:
            f.write(f'{w:016x}\n')
    with open(exp_path, 'w') as f:
        for reg, val in sorted(expected.items()):
            f.write(f'{reg}=0x{val:x}\n')
    print(f"  {name}: generated")


# ===========================================================================
# Test 30: DIV / MOD — raw unsigned division
# ===========================================================================
def gen_test_30():
    source = """\
    ; Test 30: Raw DIV and MOD
    ;
    ; r1 = 100 / 10 = 10
    ; r2 = 100 % 10 = 0
    ; r3 = 17 / 5 = 3
    ; r4 = 17 % 5 = 2
    ; r5 = 0 / 7 = 0
    ; r6 = 7 / 1 = 7
    ; r7 = 255 / 255 = 1
    ; r8 = 255 % 255 = 0

    LI r10, 100
    LI r11, 10
    LI r12, 17
    LI r13, 5
    LI r14, 0
    LI r15, 7
    LI r16, 1
    LI r17, 255

    DIV r1, r10, r11    ; 100/10 = 10
    MOD r2, r10, r11    ; 100%10 = 0
    DIV r3, r12, r13    ; 17/5 = 3
    MOD r4, r12, r13    ; 17%5 = 2
    DIV r5, r14, r15    ; 0/7 = 0
    DIV r6, r15, r16    ; 7/1 = 7
    DIV r7, r17, r17    ; 255/255 = 1
    MOD r8, r17, r17    ; 255%255 = 0

    HALT
"""
    emit_test('30_div_mod_raw', source, {
        'r1': 10,
        'r2': 0,
        'r3': 3,
        'r4': 2,
        'r5': 0,
        'r6': 7,
        'r7': 1,
        'r8': 0,
    })


# ===========================================================================
# Test 31: DIV.FIX — signed tagged fixnum division
# ===========================================================================
def gen_test_31():
    # tag_fixnum(x) = (x << 1) & WORD_MASK, which produces unsigned values.
    # For negative fixnums, use raw signed value so LI sign-extends correctly.
    def stf(x):
        """Signed tagged fixnum: x*2 as a signed Python int."""
        return x * 2

    source = f"""\
    ; Test 31: Tagged fixnum division (DIV.FIX)
    ;
    ;  12 /  3 =  4       (positive / positive)
    ; -12 /  3 = -4       (negative / positive)
    ;  12 / -3 = -4       (positive / negative)
    ; -12 / -3 =  4       (negative / negative)
    ;  7  /  2 =  3       (truncate toward zero)
    ; -7  /  2 = -3       (truncate toward zero)

    ; Small tagged values fit in signed 16-bit immediate and LI sign-extends.
    ; tag_fixnum(-12)= -24,  tag_fixnum(-3)= -6,  tag_fixnum(-7)= -14
    LI r10, {stf(12)}        ; tagged 12
    LI r11, {stf(3)}         ; tagged 3
    LI r12, {stf(-12)}       ; tagged -12 (sign-extended by LI)
    LI r13, {stf(-3)}        ; tagged -3
    LI r14, {stf(7)}         ; tagged 7
    LI r15, {stf(2)}         ; tagged 2
    LI r16, {stf(-7)}        ; tagged -7

    DIV.FIX r1, r10, r11  ; 12/3 = 4
    DIV.FIX r2, r12, r11  ; -12/3 = -4
    DIV.FIX r3, r10, r13  ; 12/-3 = -4
    DIV.FIX r4, r12, r13  ; -12/-3 = 4
    DIV.FIX r5, r14, r15  ; 7/2 = 3 (truncate)
    DIV.FIX r6, r16, r15  ; -7/2 = -3 (truncate)

    HALT
"""
    emit_test('31_div_fix_signed', source, {
        'r1': tag_fixnum(4),
        'r2': tag_fixnum(-4),  # unsigned 64-bit
        'r3': tag_fixnum(-4),
        'r4': tag_fixnum(4),
        'r5': tag_fixnum(3),
        'r6': tag_fixnum(-3),
    })


# ===========================================================================
# Test 32: TST all tag types
# ===========================================================================
def gen_test_32():
    # Construct values of each type
    char_A = make_char(65)  # 'A' = (65 << 8) | 0x35 = 0x4135
    # A short-float: low byte = 0x3D
    sfloat_val = 0x12_3D  # arbitrary value with low byte 0x3D
    # A header word
    hdr_val = make_header(0, 3, 42)  # subtype 0, size 3, shape 42
    # A ref
    ref_val = make_ref(0x1000)  # general ref to addr 0x1000
    # A cons ref
    cons_val = make_ref(0x2000, cons=True)

    source = f"""\
    ; Test 32: TST with all tag type constants (0-7)
    ;
    ; Test each tag type returns T for the right value and NIL for wrong

    ; --- Build test values ---
    LI r10, 42           ; tagged fixnum (42<<1 = 84 = 0x54)
    LI r11, 0            ; also fixnum (0)
    ; r12 = a general ref (low bits = 001)
    LI r12, {ref_val & 0xFFFF}
    LUI r18, {(ref_val >> 16) & 0xFFFF}
    OR  r12, r12, r18
    ; r13 = a cons ref (low bits = 011)
    LI r13, {cons_val & 0xFFFF}
    LUI r18, {(cons_val >> 16) & 0xFFFF}
    OR  r13, r13, r18
    ; r14 = NIL (0x05, low bits = 101)
    LI r14, 0x05
    ; r15 = a char (low byte = 0x35)
    LI r15, {char_A & 0xFFFF}
    ; r16 = a short-float (low byte = 0x3D)
    LI r16, {sfloat_val & 0xFFFF}
    ; r17 = a header (low bits = 111)
    LI r17, {hdr_val & 0xFFFF}
    LUI r18, {(hdr_val >> 16) & 0xFFFF}
    OR  r17, r17, r18

    ; --- TST.FIX ---
    TST.FIX r1, r10       ; fixnum 84 -> T (0x0D)

    ; --- TST.REF ---
    TST.REF r2, r12       ; ref -> T

    ; --- TST.CONS ---
    TST.CONS r3, r13      ; cons-ref -> T

    ; --- TST.SPECIAL ---
    TST.SPECIAL r4, r14   ; NIL (0x05) has bits 101 -> T (special tag)

    ; --- TST.NIL ---
    TST.NIL r5, r14       ; NIL -> T
    TST.NIL r6, r10       ; fixnum -> NIL (not NIL)

    ; --- TST.CHAR ---
    TST.CHAR r7, r15      ; char -> T

    ; --- TST.HDR ---
    TST.HDR r8, r17       ; header -> T

    ; --- Cross-checks: wrong types ---
    TST.FIX r9, r12       ; ref is NOT fixnum -> NIL
    ; r19 = TST.REF on fixnum -> should be NIL
    TST.REF r19, r10      ; fixnum is NOT ref -> NIL
    ; r20 = TST.CONS on ref -> should be NIL (001 != 011)
    TST.CONS r20, r12     ; ref is NOT cons -> NIL

    HALT
"""
    emit_test('32_tst_all_tags', source, {
        'r1': T,     # TST.FIX on fixnum -> T
        'r2': T,     # TST.REF on ref -> T
        'r3': T,     # TST.CONS on cons -> T
        'r4': T,     # TST.SPECIAL on NIL -> T
        'r5': T,     # TST.NIL on NIL -> T
        'r6': NIL,   # TST.NIL on fixnum -> NIL
        'r7': T,     # TST.CHAR on char -> T
        'r8': T,     # TST.HDR on header -> T
        'r9': NIL,   # TST.FIX on ref -> NIL
        'r19': NIL,  # TST.REF on fixnum -> NIL
        'r20': NIL,  # TST.CONS on ref -> NIL
    })


# ===========================================================================
# Test 33: LI32 — large immediates
# ===========================================================================
def gen_test_33():
    source = """\
    ; Test 33: LI32 (large immediates, auto-expanded from LI)
    ;
    ; LI auto-expands to LI32 for values outside [-32768, 32767]
    ; LI32 zero-extends from 32 bits

    LI r1, 0x12345       ; > 32767 -> LI32
    LI r2, 100000        ; 0x186A0 -> LI32
    LI r3, 0xDEAD        ; 0xDEAD = 57005 -> LI32
    LI r4, -1000         ; fits in 16-bit signed -> regular LI (sign-extend)
    LI r5, 32767         ; max 16-bit signed -> regular LI
    LI r6, 32768         ; just over -> LI32 (zero-extend from 32)
    LI r7, 0xFFFFFFFF    ; max 32-bit -> LI32
    LI r8, 0             ; zero -> regular LI

    HALT
"""
    emit_test('33_li32', source, {
        'r1': 0x12345,
        'r2': 100000,
        'r3': 0xDEAD,
        'r4': 0xFFFF_FFFF_FFFF_FC18,  # -1000 sign-extended to 64 bits
        'r5': 32767,
        'r6': 32768,      # zero-extended from 32 bits
        'r7': 0xFFFFFFFF,  # zero-extended from 32 bits (NOT sign-extended)
        'r8': 0,
    })


# ===========================================================================
# Test 34: Sub-word memory ops
# ===========================================================================
def gen_test_34():
    source = f"""\
    ; Test 34: Sub-word memory operations (LDB/STB, LDH/STH, LDW/STW)

    ; Use memory at 0x2000 for scratch
    LI r10, 0x2000

    ; --- STR a known 64-bit pattern, then read back sub-words ---
    ; Store 0x0807060504030201 at address 0x2000
    LI r11, 0x0201
    LUI r12, 0x0403
    OR  r11, r11, r12
    LUI r12, 0x0605
    ; Need upper 32 bits: 0x08070605 — but we can only LI32 up to 32 bits
    ; Just use the lower 32 bits for now: 0x04030201
    STR r10, r11, 0      ; store 0x04030201 at 0x2000

    ; --- Byte loads (LDB) ---
    LDB r1, r10, 0       ; byte at offset 0 -> 0x01
    LDB r2, r10, 1       ; byte at offset 1 -> 0x02
    LDB r3, r10, 2       ; byte at offset 2 -> 0x03
    LDB r4, r10, 3       ; byte at offset 3 -> 0x04

    ; --- Halfword loads (LDH) ---
    LDH r5, r10, 0       ; halfword at offset 0 -> 0x0201
    LDH r6, r10, 2       ; halfword at offset 2 -> 0x0403

    ; --- Word loads (LDW) ---
    LDW r7, r10, 0       ; word at offset 0 -> 0x04030201

    ; --- STB: store a single byte, verify others unchanged ---
    LI r13, 0xFF
    STB r10, r13, 1      ; store 0xFF at byte offset 1
    LDB r8, r10, 0       ; byte 0 should still be 0x01
    LDB r9, r10, 1       ; byte 1 should now be 0xFF

    ; --- STH: store halfword ---
    LI r14, 0x2000
    LI r15, 8            ; offset 8 -> address 0x2008
    ADD r14, r14, r15
    LI r15, 0xBEEF
    STH r14, r15, 0      ; store 0xBEEF at 0x2008 offset 0
    LDH r19, r14, 0      ; read back -> 0xBEEF

    ; --- STW: store 32-bit word ---
    LI r14, 0x2000
    LI r15, 16
    ADD r14, r14, r15    ; addr 0x2010
    LI r15, 0x1234
    LUI r16, 0x5678
    OR  r15, r15, r16   ; 0x56781234
    STW r14, r15, 0
    LDW r20, r14, 0     ; read back -> 0x56781234

    HALT
"""
    emit_test('34_subword_mem', source, {
        'r1': 0x01,
        'r2': 0x02,
        'r3': 0x03,
        'r4': 0x04,
        'r5': 0x0201,
        'r6': 0x0403,
        'r7': 0x04030201,
        'r8': 0x01,         # unchanged after STB at offset 1
        'r9': 0xFF,         # overwritten by STB
        'r19': 0xBEEF,      # STH round-trip
        'r20': 0x56781234,  # STW round-trip
    })


# ===========================================================================
# Test 35: SEND / RECV — queue descriptors
# ===========================================================================
def gen_test_35():
    source = f"""\
    ; Test 35: SEND, RECV and TRY.RECV name their queue by a descriptor in a
    ; register (spec/02-isa.md section 8), a fixnum whose value is
    ; tile * 4 + queue.  The bench's core is tile 0 alone: descriptors 0 to
    ; 3 name its own four queues, and one naming another tile reaches no
    ; queue, so SEND traps TRAP_QUEUE_FULL and TRY.RECV finds no word.  A
    ; negative value names no tile at all.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, skip_handler
    LI  r15, {TRAP_TABLE + 0x30 * 8}   ; slot 0x30 = TRAP_QUEUE_FULL
    STR r15, r14, 0
    LI  r20, 0

    LI  r15, {tag_fixnum(0 * 4 + 0)}   ; tile 0, queue 0
    LI  r16, {tag_fixnum(0 * 4 + 2)}   ; tile 0, queue 2
    LI  r17, {tag_fixnum(1 * 4 + 0)}   ; tile 1, queue 0

    ; --- TRY.RECV on an empty queue -> nil, nil ---
    TRY.RECV r1, r2, r15

    ; --- SEND and RECV on queue 0 ---
    LI  r10, 42
    SEND r15, r10
    RECV r3, r15          ; 42

    ; --- FIFO order: A then B comes back A then B ---
    LI  r11, 100
    LI  r12, 200
    SEND r15, r11
    SEND r15, r12
    RECV r4, r15          ; 100
    RECV r5, r15          ; 200

    ; --- TRY.RECV on a queue holding a word -> the word, t ---
    LI  r13, 999
    SEND r15, r13
    TRY.RECV r6, r7, r15
    TRY.RECV r8, r9, r15  ; empty again -> nil, nil

    ; --- the queues are apart: queue 2 keeps its word, queue 0 stays empty ---
    LI  r14, 77
    SEND r16, r14
    TRY.RECV r18, r19, r15
    RECV r21, r16         ; 77

    ; --- another tile, and no tile: SEND traps, TRY.RECV finds nothing ---
    LI  r14, -4           ; fixnum -2: tile -1, queue 2
    LI  r11, 5
    SEND r17, r11         ; traps TRAP_QUEUE_FULL
    SEND r14, r11         ; traps TRAP_QUEUE_FULL
    TRY.RECV r10, r11, r17
    TRY.RECV r16, r17, r14
    MOV r12, r20
    MOV r13, r22
    HALT

{SKIP_HANDLER}"""
    emit_test('35_send_recv', source, {
        'r1': NIL,   # TRY.RECV empty -> value = NIL
        'r2': NIL,   # TRY.RECV empty -> status = NIL
        'r3': 42,    # RECV got 42
        'r4': 100,   # FIFO order: first in
        'r5': 200,   # FIFO order: second in
        'r6': 999,   # TRY.RECV success -> value
        'r7': T,     # TRY.RECV success -> status = T
        'r8': NIL,   # TRY.RECV empty again
        'r9': NIL,
        'r18': NIL,  # queue 0 empty while queue 2 holds 77
        'r19': NIL,
        'r21': 77,   # queue 2's word
        'r10': NIL,  # TRY.RECV on another tile's queue finds no word
        'r11': NIL,
        'r16': NIL,  # nor on no tile's
        'r17': NIL,
        'r12': 2,    # both SENDs trapped
        'r13': 0x30, # TRAP_QUEUE_FULL
    })


# ===========================================================================
# Test 36: CAS.TAGGED — compare-and-swap
# ===========================================================================
def gen_test_36():
    source = f"""\
    ; Test 36: CAS.TAGGED (compare-and-swap)
    ;
    ; CAS.TAGGED rd, rs_addr, rs_expected, rs_new
    ;   Reads mem[addr], compares to expected.
    ;   If match -> store new, rd = T.
    ;   If mismatch -> no store, rd = NIL.

    ; Set up memory at 0x2000 with initial value 0xAAAA
    LI r10, 0x2000
    LI r11, 0x0AAA
    STR r10, r11, 0

    ; --- CAS match: expected=0xAAAA, new=0xBBBB ---
    LI r12, 0x0AAA       ; expected
    LI r13, 0x0BBB       ; new value
    CAS.TAGGED r1, r10, r12, r13
    ; r1 should be T (match -> swap succeeded)

    ; Verify memory was actually updated
    LDR r2, r10, 0        ; should be 0x0BBB

    ; --- CAS mismatch: expected=0xAAAA (stale), actual is now 0xBBBB ---
    LI r14, 0x0AAA       ; expected (wrong now)
    LI r15, 0x0CCC       ; new value (should NOT be written)
    CAS.TAGGED r3, r10, r14, r15
    ; r3 should be NIL (mismatch -> no swap)

    ; Verify memory was NOT changed
    LDR r4, r10, 0        ; should still be 0x0BBB

    ; --- CAS match again with the correct current value ---
    LI r16, 0x0BBB       ; expected (correct)
    LI r17, 0x0DDD       ; new value
    CAS.TAGGED r5, r10, r16, r17
    ; r5 should be T
    LDR r6, r10, 0        ; should be 0x0DDD

    HALT
"""
    emit_test('36_cas_tagged', source, {
        'r1': T,       # first CAS matched
        'r2': 0x0BBB,  # memory updated
        'r3': NIL,     # second CAS mismatched
        'r4': 0x0BBB,  # memory unchanged
        'r5': T,       # third CAS matched
        'r6': 0x0DDD,  # memory updated again
    })


# ===========================================================================
# Test 37: FAA — fetch-and-add
# ===========================================================================
def gen_test_37():
    source = f"""\
    ; Test 37: FAA (fetch-and-add)
    ;
    ; FAA rd, rs_addr, rs_delta
    ;   old = mem[addr]
    ;   mem[addr] = old + delta
    ;   rd = old
    ;
    ; The word in memory and the delta are fixnums, which FAA requires
    ; (spec/02-isa.md section 8.5): fixnum n is the raw word 2n.

    ; Set up memory at 0x2000 with fixnum 10
    LI r10, 0x2000
    LI r11, {tag_fixnum(10)}
    STR r10, r11, 0

    ; FAA: delta=5 -> old=10, new=15
    LI r12, {tag_fixnum(5)}
    FAA r1, r10, r12      ; r1 = 10 (old value)
    LDR r2, r10, 0        ; should be 15

    ; FAA again: delta=3 -> old=15, new=18
    LI r13, {tag_fixnum(3)}
    FAA r3, r10, r13      ; r3 = 15 (old value)
    LDR r4, r10, 0        ; should be 18

    ; FAA with delta=0 -> should return old, no change
    LI r14, {tag_fixnum(0)}
    FAA r5, r10, r14      ; r5 = 18 (old value)
    LDR r6, r10, 0        ; should still be 18

    HALT
"""
    emit_test('37_faa', source, {
        'r1': tag_fixnum(10),   # old value returned
        'r2': tag_fixnum(15),   # memory updated: 10+5
        'r3': tag_fixnum(15),   # old value after first FAA
        'r4': tag_fixnum(18),   # memory: 15+3
        'r5': tag_fixnum(18),   # old value, delta=0
        'r6': tag_fixnum(18),   # memory unchanged
    })


# ===========================================================================
# Test 38: PUSH.MULTI / POP.MULTI
# ===========================================================================
def gen_test_38():
    source = f"""\
    ; Test 38: PUSH.MULTI / POP.MULTI
    ;
    ; Push registers r1,r2,r3 (bank 0, mask = 0b1110 = 0x0E)
    ; Clobber them, then pop back.
    ;
    ; Also test: empty mask (no-op), single register

    ; Init stack
    LI sp, {STACK_TOP}

    ; Set up values
    LI r1, 11
    LI r2, 22
    LI r3, 33

    ; Push r1,r2,r3 (bank 0, mask bits 1,2,3 = 0x000E)
    PUSH.MULTI 0, 0x000E

    ; Clobber
    LI r1, 0
    LI r2, 0
    LI r3, 0

    ; Pop r1,r2,r3 back
    POP.MULTI 0, 0x000E

    ; r1,r2,r3 should be restored
    ; Save to r4,r5,r6 so we can test additional cases
    MOV r4, r1
    MOV r5, r2
    MOV r6, r3

    ; --- Empty mask (no-op) ---
    LI r7, 77
    PUSH.MULTI 0, 0x0000  ; no registers -> no stack change
    POP.MULTI 0, 0x0000   ; no registers -> no stack change
    ; r7 should still be 77

    ; --- Single register (bit 8 -> r8) ---
    LI r8, 88
    PUSH.MULTI 0, 0x0100  ; push r8 only
    LI r8, 0              ; clobber
    POP.MULTI 0, 0x0100   ; pop r8 back
    ; r8 should be 88

    HALT
"""
    emit_test('38_push_pop_multi', source, {
        'r4': 11,   # r1 restored to 11
        'r5': 22,   # r2 restored to 22
        'r6': 33,   # r3 restored to 33
        'r7': 77,   # untouched by empty mask
        'r8': 88,   # single-register push/pop
    })


# ===========================================================================
# Test 39: TST.SHAPE — header-based type testing
# ===========================================================================
def gen_test_39():
    source = f"""\
    ; Test 39: TST.SHAPE (header-based type test)
    ;
    ; SET_TEMPLATE (TRAP 0x91) takes r1=index, r2=full header word.
    ; Allocate objects via templates with known shape IDs, then TST.SHAPE.

    LI sp, {STACK_TOP}
    LI np, {NURSERY_BASE}
    LI nl, {NURSERY_LIMIT}

    ; Configure template 0: shape_id=42 (full header in r2)
    LI r1, 0
    LI32 r2, {make_header(0, 0, 42)}
    TRAP 0x91

    ; Configure template 1: shape_id=99
    LI r1, 1
    LI32 r2, {make_header(0, 0, 99)}
    TRAP 0x91

    ; Allocate object with template 0 (shape 42)
    ALLOC r10, 3, 0

    ; Allocate object with template 1 (shape 99)
    ALLOC r11, 3, 1

    ; TST.SHAPE r10 against shape 42 -> T
    TST.SHAPE r1, r10, 42

    ; TST.SHAPE r10 against shape 99 -> NIL
    TST.SHAPE r2, r10, 99

    ; TST.SHAPE r11 against shape 99 -> T
    TST.SHAPE r3, r11, 99

    ; TST.SHAPE on fixnum -> NIL (fast path)
    LI r12, 84
    TST.SHAPE r4, r12, 42

    ; TST.SHAPE on NIL -> NIL (fast path)
    LI r13, 0x05
    TST.SHAPE r5, r13, 42

    HALT
"""
    emit_test('39_tst_shape', source, {
        'r1': T,     # TST.SHAPE on shape-42 object, query 42 -> T
        'r2': NIL,   # TST.SHAPE on shape-42 object, query 99 -> NIL
        'r3': T,     # TST.SHAPE on shape-99 object, query 99 -> T
        'r4': NIL,   # fixnum -> NIL (fast path)
        'r5': NIL,   # NIL -> NIL (fast path)
    })


# ===========================================================================
# Test 40: JR / TAILCALL.DIR
# ===========================================================================
def gen_test_40():
    source = f"""\
    ; Test 40: JR (jump register) and TAILCALL.DIR
    ;
    ; JR: load target address, jump to it.
    ; TAILCALL.DIR: direct tail call (no frame push, just PC-relative jump)

    LI sp, {STACK_TOP}

    ; --- JR: jump to target1 ---
    LI r10, target1
    JR r10
    LI r1, 0xFF          ; should be skipped
    BR done               ; should be skipped

target1:
    LI r1, 11            ; should execute

    ; --- TAILCALL.DIR: tail-call to target2 ---
    ; Save SP to verify no frame push
    MOV r8, sp
    TAILCALL.DIRECT target2
    LI r2, 0xFF          ; should be skipped

target2:
    LI r2, 22            ; should execute
    MOV r9, sp
    SUB r3, r8, r9       ; r3 = 0 if SP unchanged (no frame pushed)

done:
    HALT
"""
    emit_test('40_jr_tailcall', source, {
        'r1': 11,    # reached via JR
        'r2': 22,    # reached via TAILCALL.DIR
        'r3': 0,     # SP unchanged (no frame push)
    })


# ===========================================================================
# Test 41: CALL.CLOSURE
# ===========================================================================
def gen_test_41():
    source = f"""\
    ; Test 41: CALL.CLOSURE
    ;
    ; Allocate a closure object, install code entry and env slots,
    ; then call it. The closure body accesses its env via LD.FLD.

    LI sp, {STACK_TOP}
    LI fp, {STACK_TOP}
    LI np, {NURSERY_BASE}
    LI nl, {NURSERY_LIMIT}

    ; Initialize closure template (entry 1):
    ; HDR_CLOSURE = subtype 4, size patched by allocator
    LI r1, 1
    LI r2, {make_header(4, 0, 0)}
    TRAP 0x91

    ; Allocate closure with 2 env slots
    LI r10, closure_body
    ALLOC.CLOSURE r11, r10, 2    ; r11 = closure ref, 2 env slots

    ; Store environment values
    LI r12, 100
    ST.FLD r11, r12, 1          ; env slot 0 (field 1, after code entry) = 100
    LI r13, 200
    ST.FLD r11, r13, 2          ; env slot 1 (field 2) = 200

    ; Call the closure
    CALL.CLOSURE r11
    ; After return, r1 should be 300 (100 + 200)
    BR done

closure_body:
    ; r11 still holds the closure ref (preserved across CALL.CLOSURE)
    LD.FLD r20, r11, 1          ; env slot 0 -> 100
    LD.FLD r21, r11, 2          ; env slot 1 -> 200
    ADD r1, r20, r21            ; r1 = 300
    RET

done:
    HALT
"""
    emit_test('41_call_closure', source, {
        'r1': 300,  # 100 + 200 from env slots
    })


# ===========================================================================
# Test 42: Backward branches and BR.T edge cases
# ===========================================================================
def gen_test_42():
    source = f"""\
    ; Test 42: Backward branches and BR.T edge cases
    ;
    ; Count-up loop using backward branch.
    ; BR.T edge cases: val=1 (truthy), val=NIL (not), val=0 (not).

    ; --- Backward branch: count-up loop ---
    ; Avoid counting DOWN because raw 5 == NIL (0x05) would
    ; cause BR.T to exit early. Instead, count UP and compare
    ; against limit: (counter - limit) is 0 when done → falsy.
    LI r1, 0              ; iteration count
    LI r2, 10             ; limit
bloop:
    LI r10, 1
    ADD r1, r1, r10       ; r1++
    SUB r10, r1, r2       ; r10 = r1 - 10  (negative = large u64 → truthy)
    BR.T r10, bloop       ; loop while r1 != 10
    ; r1 = 10 (executed 10 times)

    ; --- BR.T edge cases ---
    ; val = 1: truthy (non-zero, non-NIL)
    LI r10, 1
    BR.T r10, brt_1_taken
    LI r3, 0              ; skipped
    BR brt_1_done
brt_1_taken:
    LI r3, 1              ; taken
brt_1_done:

    ; val = NIL: not truthy
    LI r10, 5             ; NIL = 0x05
    BR.T r10, brt_nil_taken
    LI r4, 0              ; NOT taken (NIL is falsy)
    BR brt_nil_done
brt_nil_taken:
    LI r4, 1              ; should NOT reach here
brt_nil_done:

    ; val = 0: not truthy
    LI r10, 0
    BR.T r10, brt_0_taken
    LI r5, 0              ; NOT taken (zero is falsy)
    BR brt_0_done
brt_0_taken:
    LI r5, 1              ; should NOT reach here
brt_0_done:

    HALT
"""
    emit_test('42_backward_br', source, {
        'r1': 10,   # loop ran 10 times
        'r3': 1,    # BR.T on val=1: taken (truthy)
        'r4': 0,    # BR.T on val=NIL: not taken
        'r5': 0,    # BR.T on val=0: not taken
    })


# ===========================================================================
# Test 45: MUL.FIX and DIV.FIX overflow
# ===========================================================================
def gen_test_45():
    source = f"""\
    ; Test 45: MUL.FIX and DIV.FIX trap TRAP_FIXNUM_OVERFLOW when the exact
    ; result leaves the fixnum range, -2**62 to 2**62 - 1 (spec/02-isa.md
    ; section 3.2).  Fixnum n is the raw word 2n.
    ;
    ; The handler counts the trap in r20, keeps the trap PC in r21 and the
    ; cause in r22, and resumes after the faulting instruction, whose
    ; destination keeps the 0x77 put there first.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, ovf_handler
    LI  r15, {TRAP_TABLE + 0x02 * 8}   ; slot 0x02 = TRAP_FIXNUM_OVERFLOW
    STR r15, r14, 0
    LI  r20, 0
    LI  r9, 1

    ; --- 2**40 * 2**40 = 2**80: the low 64 bits are 0, the product overflows ---
    LI  r10, 41
    SHL r10, r9, r10      ; fixnum 2**40
    LI  r1, 0x77
mul_big:
    MUL.FIX r1, r10, r10  ; traps
    LI  r11, mul_big
    SUB r2, r21, r11      ; 0: the trap saved the MUL.FIX's own PC

    ; --- 2**61 * 2 = 2**62, one past the largest fixnum ---
    LI  r10, 62
    SHL r10, r9, r10      ; fixnum 2**61
    LI  r11, 4            ; fixnum 2
    LI  r3, 0x77
    MUL.FIX r3, r10, r11  ; traps

    ; --- -2**61 * 2 = -2**62, the most negative fixnum, fits ---
    LI  r12, 0
    SUB r10, r12, r10     ; fixnum -2**61
    MUL.FIX r4, r10, r11  ; fixnum -2**62

    ; --- -3 * 5 = -15 ---
    LI  r10, -6
    LI  r11, 10
    MUL.FIX r5, r10, r11

    ; --- the most negative fixnum over -1 = 2**62: overflows ---
    LI  r10, 63
    SHL r10, r9, r10      ; fixnum -2**62
    LI  r11, -2           ; fixnum -1
    LI  r6, 0x77
div_min:
    DIV.FIX r6, r10, r11  ; traps
    LI  r12, div_min
    SUB r7, r21, r12      ; 0: the trap saved the DIV.FIX's own PC

    ; --- the most negative fixnum over 1 and over -2 fit ---
    LI  r11, 2            ; fixnum 1
    DIV.FIX r8, r10, r11  ; fixnum -2**62
    LI  r11, -4           ; fixnum -2
    DIV.FIX r13, r10, r11 ; fixnum 2**61
    MOV r16, r22          ; the last trap's cause
    HALT

ovf_handler:
    TRAP.PC r21
    TRAP.CAUSE r22
    LI  r23, 1
    ADD r20, r20, r23
    LI  r23, 4
    ADD r24, r21, r23     ; the instruction after the faulting one
    JR  r24
"""
    emit_test('45_fix_overflow', source, {
        'r1': 0x77,                   # 2**40 * 2**40 trapped, r1 unwritten
        'r2': 0,                      # at the MUL.FIX's own PC
        'r3': 0x77,                   # 2**61 * 2 trapped
        'r4': tag_fixnum(-(1 << 62)),  # -2**61 * 2 fits
        'r5': tag_fixnum(-15),
        'r6': 0x77,                   # -2**62 / -1 trapped
        'r7': 0,                      # at the DIV.FIX's own PC
        'r8': tag_fixnum(-(1 << 62)),  # -2**62 / 1
        'r13': tag_fixnum(1 << 61),   # -2**62 / -2
        'r16': 0x02,                  # TRAP_FIXNUM_OVERFLOW
        'r20': 3,                     # three overflows trapped
    })


# ===========================================================================
# Test 46: ALU faults save their own PC
# ===========================================================================
def gen_test_46():
    source = f"""\
    ; Test 46: a fault saves the faulting instruction's PC, so ERET runs it
    ; again (spec/03-core.md section 6.3), the ALU's faults among them.  The
    ; handler for TRAP_NOT_FIXNUM makes r10 fixnum 3 and returns with ERET,
    ; so each faulting instruction runs again and succeeds.  A DIV.FIX that
    ; faults leaves the divider idle, so the retried DIV.FIX and the raw DIV
    ; after it each get their own quotients.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, fix_handler
    LI  r15, {TRAP_TABLE + 0x01 * 8}   ; slot 0x01 = TRAP_NOT_FIXNUM
    STR r15, r14, 0
    LI  r20, 0

    ; --- ADD.FIX on an odd word ---
    LI  r10, 7            ; not a fixnum
    LI  r11, 10           ; fixnum 5
add_site:
    ADD.FIX r1, r10, r11  ; traps, then fixnum 3 + 5
    LI  r12, add_site
    SUB r2, r21, r12      ; 0: the trap saved the ADD.FIX's own PC

    ; --- DIV.FIX on an odd word, then a raw DIV ---
    LI  r10, 0x7FFF       ; not a fixnum
    LI  r11, 6            ; fixnum 3
    DIV.FIX r3, r10, r11  ; traps, then fixnum 3 / 3
    LI  r12, 100
    LI  r13, 7
    DIV r4, r12, r13      ; 100 / 7
    MOV r5, r20
    HALT

fix_handler:
    TRAP.PC r21
    LI  r23, 1
    ADD r20, r20, r23
    LI  r10, 6            ; fixnum 3
    ERET
"""
    emit_test('46_fault_retry', source, {
        'r1': tag_fixnum(8),   # the retried ADD.FIX
        'r2': 0,               # its trap PC
        'r3': tag_fixnum(1),   # the retried DIV.FIX
        'r4': 14,              # the raw DIV's own quotient
        'r5': 2,               # two traps
    })


# ===========================================================================
# Test 47: ALLOCV and ALLOC.CLOSURE zero their payload
# ===========================================================================
def gen_test_47():
    source = f"""\
    ; Test 47: ALLOCV zeroes every word of its vector and traps
    ; TRAP_NOT_FIXNUM on a length that is not a fixnum (spec/02-isa.md
    ; section 4.3), and ALLOC.CLOSURE zeroes all 31 environment slots of a
    ; closure.  The nursery starts full of {NURSERY_FILL:#x} words.

    LI  np, {NURSERY_BASE}
    LI  nl, {NURSERY_LIMIT}
    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, skip_handler
    LI  r15, {TRAP_TABLE + 0x01 * 8}   ; slot 0x01 = TRAP_NOT_FIXNUM
    STR r15, r14, 0
    LI  r20, 0

    ; Template 2 makes vectors, template 1 closures
    LI  r1, 2
    LI32 r2, {make_header(2, 0, 2)}
    TRAP 0x91
    LI  r1, 1
    LI32 r2, {make_header(4, 0, 1)}
    TRAP 0x91

    ; --- a vector of 40 elements: header, length, 40 elements ---
    LI  r11, {tag_fixnum(40)}
    ALLOCV r10, r11, 2
    LI  r12, -8
    AND r13, r10, r12     ; its address
    LDR r4, r13, 0        ; the header
    LDR r5, r13, 8        ; the length
    LI  r12, {tag_fixnum(40)}   ; elements left
    LI  r15, 16
    ADD r13, r13, r15     ; element 0
    LI  r3, 0
vloop:
    LDR r16, r13, 0
    OR  r3, r3, r16       ; every element ORed into r3
    LI  r15, 8
    ADD r13, r13, r15
    ADD.FIX.IMM r12, r12, -2   ; fixnum -1
    BR.FIX.GT r12, vloop

    ; --- a closure of 31 environment slots: header, code, 31 slots ---
    LI  r21, 0x1234       ; its code entry
    ALLOC.CLOSURE r17, r21, 31
    LI  r12, -8
    AND r13, r17, r12
    LDR r7, r13, 0        ; the header
    LDR r8, r13, 8        ; the code entry
    LI  r12, {tag_fixnum(31)}   ; slots left
    LI  r15, 16
    ADD r13, r13, r15     ; slot 0
    LI  r6, 0
cloop:
    LDR r16, r13, 0
    OR  r6, r6, r16       ; every slot ORed into r6
    LI  r15, 8
    ADD r13, r13, r15
    ADD.FIX.IMM r12, r12, -2   ; fixnum -1
    BR.FIX.GT r12, cloop

    ; --- a length that is not a fixnum: traps and allocates nothing ---
    MOV r18, np
    LI  r11, 7
    ALLOCV r19, r11, 2
    SUB r9, np, r18       ; 0: np unchanged
    MOV r1, r20
    MOV r2, r22
    HALT

{SKIP_HANDLER}
{nursery_garbage(96)}"""
    emit_test('47_alloc_zero', source, {
        'r1': 1,                          # one trap
        'r2': 0x01,                       # TRAP_NOT_FIXNUM
        'r3': 0,                          # all 40 elements zero
        'r4': make_header(2, 41, 2),      # size: the length and 40 elements
        'r5': tag_fixnum(40),
        'r6': 0,                          # all 31 slots zero
        'r7': make_header(4, 32, 1),      # size: the code and 31 slots
        'r8': 0x1234,
        'r9': 0,                          # the trapped ALLOCV took no words
    })


# ===========================================================================
# Test 48: FAA's tag check
# ===========================================================================
def gen_test_48():
    source = f"""\
    ; Test 48: FAA traps TRAP_NOT_FIXNUM when the word in memory or the
    ; delta is not a fixnum (spec/02-isa.md section 8.5), and stores and
    ; writes nothing then.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, skip_handler
    LI  r15, {TRAP_TABLE + 0x01 * 8}   ; slot 0x01 = TRAP_NOT_FIXNUM
    STR r15, r14, 0
    LI  r20, 0
    LI  r10, 0x2000

    ; --- the word in memory is not a fixnum ---
    LI  r11, 7
    STR r10, r11, 0
    LI  r12, {tag_fixnum(5)}
    LI  r1, 0x77
    FAA r1, r10, r12      ; traps
    LDR r2, r10, 0        ; memory keeps 7

    ; --- the delta is not a fixnum ---
    LI  r11, {tag_fixnum(10)}
    STR r10, r11, 0
    LI  r13, 3
    LI  r3, 0x77
    FAA r3, r10, r13      ; traps
    LDR r4, r10, 0        ; memory keeps fixnum 10

    ; --- both fixnums ---
    LI  r13, -8           ; fixnum -4
    FAA r5, r10, r13      ; fixnum 10, the old word
    LDR r6, r10, 0        ; fixnum 6
    MOV r7, r20
    MOV r8, r22
    HALT

{SKIP_HANDLER}"""
    emit_test('48_faa_tags', source, {
        'r1': 0x77,             # FAA trapped, r1 unwritten
        'r2': 7,                # memory unchanged
        'r3': 0x77,             # FAA trapped, r3 unwritten
        'r4': tag_fixnum(10),   # memory unchanged
        'r5': tag_fixnum(10),
        'r6': tag_fixnum(6),
        'r7': 2,                # two traps
        'r8': 0x01,             # TRAP_NOT_FIXNUM
    })


# ===========================================================================
# Test 49: ERET.NEXT resumes after the faulting instruction
# ===========================================================================
def gen_test_49():
    source = f"""\
    ; Test 49: a handler that carries out the faulting instruction itself,
    ; as a generic + or a bignum promotion does (spec/02-isa.md section
    ; 14.3), returns with ERET.NEXT, which resumes at the saved PC + 4,
    ; after the instruction that faulted (spec/03-core.md section 6.3).
    ; Plain ERET still runs the faulting instruction again, and after an
    ; explicit TRAP, whose saved PC is the next instruction's, ERET.NEXT
    ; skips that one.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, add_handler
    LI  r15, {TRAP_TABLE + 0x01 * 8}   ; TRAP_NOT_FIXNUM
    STR r15, r14, 0
    LI  r14, promote_handler
    LI  r15, {TRAP_TABLE + 0x02 * 8}   ; TRAP_FIXNUM_OVERFLOW
    STR r15, r14, 0
    LI  r14, mend_handler
    LI  r15, {TRAP_TABLE + 0x03 * 8}   ; TRAP_DIVIDE_BY_ZERO
    STR r15, r14, 0
    LI  r14, call_handler
    LI  r15, {TRAP_TABLE + 0x42 * 8}   ; the code TRAP 0x42 raises
    STR r15, r14, 0
    LI  r20, 0                         ; handlers run

    ; --- generic +: r1 is no fixnum, so the handler adds the raw words ---
    LI  r1, 7
    LI  r2, {tag_fixnum(5)}
fault1:
    ADD.FIX r3, r1, r2    ; the handler writes r3 = 17
    LI  r4, 0x44          ; runs once, after the handler
    MOV r5, r24           ; the saved PC the handler read

    ; --- overflow: the handler leaves nil where a bignum would go ---
    LI  r6, 1
    LI  r7, 62
    SHL r6, r6, r7        ; fixnum 2**61
    ADD.FIX r8, r6, r6    ; 2**62 leaves the fixnum range
    LI  r9, 0x99

    ; --- plain ERET retries: the handler mends the divisor ---
    LI  r10, {tag_fixnum(42)}
    LI  r11, {tag_fixnum(0)}
    DIV.FIX r12, r10, r11 ; traps, then runs again: fixnum 21

    ; --- after an explicit TRAP, ERET.NEXT skips the next instruction ---
    LI  r13, 0
    TRAP 0x42
    LI  r13, 0x13         ; skipped
    LI  r16, 0x16
    HALT

add_handler:
    TRAP.PC r24
    ADD r3, r1, r2
    LI  r23, 1
    ADD r20, r20, r23
    ERET.NEXT

promote_handler:
    LI  r8, {NIL}
    LI  r23, 1
    ADD r20, r20, r23
    ERET.NEXT

mend_handler:
    LI  r11, {tag_fixnum(2)}
    LI  r23, 1
    ADD r20, r20, r23
    ERET

call_handler:
    LI  r23, 1
    ADD r20, r20, r23
    ERET.NEXT
"""
    asm = Assembler()
    asm.assemble(source)
    emit_test('49_eret_next', source, {
        'r3': 17,                 # the handler's sum, written once
        'r4': 0x44,
        'r5': asm.labels['fault1'],  # a fault saves its own PC
        'r8': NIL,                # the overflow handler's result
        'r9': 0x99,
        'r11': tag_fixnum(2),
        'r12': tag_fixnum(21),    # DIV.FIX ran again after plain ERET
        'r13': 0,                 # skipped after TRAP 0x42
        'r16': 0x16,
        'r20': 4,                 # each handler ran once
    })


# ===========================================================================
# Test 50: CALL.REG calls the address in a register
# ===========================================================================
def gen_test_50():
    source = f"""\
    ; Test 50: CALL.REG Rs calls the address Rs holds with the frame every
    ; call makes (spec/02-isa.md section 6.8): it pushes LR and FP, points
    ; FP at them and sets LR to the next instruction, and RET undoes it.
    ; It reaches a function far beyond CALL.DIRECT's 128 KiB, one whose
    ; address is computed, and, through LR itself, the address LR held
    ; before the call.

    LI  sp, {STACK_TOP}
    LI  fp, 0x3333
    LI  lr, 0x2222

    ; --- a function at 192 KiB ---
    LI32 r1, far_fn
    CALL.REG r1
back:
    MOV r10, sp

    ; --- an address read from a table ---
    LI32 r2, fn_table
    LDR r4, r2, 8         ; the table's second entry, fn_b
    CALL.REG r4

    ; --- LR itself: the target is LR's value before the call ---
    LI32 lr, fn_c
    CALL.REG lr
    HALT

fn_b:
    LI  r11, 0xbb
    RET

fn_c:
    LDR r12, fp, 8        ; the caller's LR the frame saved: fn_c itself
    RET

    .align 8
fn_table:
    .word 0
    .word fn_b

    .org 0x30000
far_fn:
    LI  r5, 0x55
    MOV r6, lr            ; the return address
    MOV r7, fp
    MOV r8, sp
    LDR r9, sp, 8         ; the caller's LR
    LDR r13, sp, 0        ; the caller's FP
    RET
"""
    asm = Assembler()
    asm.assemble(source)
    emit_test('50_call_reg', source, {
        'r5': 0x55,
        'r6': asm.labels['back'],     # LR is the instruction after the call
        'r7': STACK_TOP - 16,         # FP points at the saved FP
        'r8': STACK_TOP - 16,
        'r9': 0x2222,                 # the frame holds the caller's LR
        'r13': 0x3333,                # and its FP
        'r10': STACK_TOP,             # RET took the frame off again
        'r11': 0xbb,
        'r12': asm.labels['fn_c'],
        'r28': asm.labels['fn_c'],    # RET restored the caller's LR
        'r29': 0x3333,                # and FP
        'r30': STACK_TOP,
    })


# ===========================================================================
# Test 51: reserved encodings trap TRAP_UNIMPLEMENTED
# ===========================================================================
def gen_test_51():
    def word(w: int) -> str:
        return f"    .u32 {w:#010x}"
    reserved = [
        ("ARITH.FIX func 4",     encode_r(Op.ARITH_FIX, 3, 1, 2, 4)),
        ("CMP.TAGGED func 2",    encode_r(Op.CMP_TAGGED, 4, 1, 2, 2)),
        ("LD.CAR/CDR imm 2",     encode_i(Op.LD_CAR_CDR, 5, 16, 2)),
        ("ST.CAR/CDR field 2",   encode_s(Op.ST_CAR_CDR, 16, 2, 2)),
        ("FAA func 1",           encode_r(Op.FAA_FENCE, 6, 17, 1, 1)),
        ("ARITH.RAW func 5",     encode_r(Op.ARITH_RAW, 7, 1, 2, 5)),
        ("BITWISE func 7",       encode_r(Op.BITWISE, 8, 1, 2, 7)),
        ("BR.COND cond 6",       encode_b(Op.BR_COND, 1, 6, 2)),
        ("PUSH/POP func 2",      encode_r(Op.PUSH_POP, 10, 0, 0, 2)),
        ("PUSH.MULTI bank 2",    encode_i(Op.PUSH_MULTI, 2, 0, 0x0001)),
        ("POP.MULTI bank 3",     encode_i(Op.POP_MULTI, 3, 0, 0x0001)),
        ("ERET func 2",          encode_r(Op.ERET, 0, 0, 0, 2)),
        ("SYS.INFO sub 15",      encode_i(Op.SYS_INFO, 11, 15, 0)),
        ("HALT/NOP selector 2",  encode_x(Op.HALT_NOP, 2 << 21)),
        ("TRAP 0x96",            encode_x(Op.TRAP, 0x96)),
        ("TRAP 0xA0",            encode_x(Op.TRAP, 0xA0)),
        ("opcode 000111",        encode_r(0b000111, 0, 1, 2, 0)),
        ("opcode 010101",        encode_r(0b010101, 0, 1, 2, 0)),   # Family 8
        ("opcode 010110",        encode_r(0b010110, 0, 1, 2, 0)),
    ]
    lines = []
    for i, (what, w) in enumerate(reserved):
        if i == len(reserved) - 1:
            lines.append("last:")
        lines.append(f"{word(w)}   ; {what}")
        if what.startswith("BR.COND"):
            lines.append("    LI  r9, 0x99          ; skipped only if the branch were taken")
    body = "\n".join(lines)
    source = f"""\
    ; Test 51: a value of a selecting field that names no instruction is
    ; reserved (spec/07-encoding.md section 2), and so are an opcode and a
    ; system trap code no instruction has.  Each traps TRAP_UNIMPLEMENTED,
    ; 0x7E, a fault that saves its own PC and changes nothing else: no
    ; register it names, no word of memory, no SP and no branch.  The
    ; handler counts them and returns past each with ERET.NEXT.  Explicit
    ; TRAPs 0x70 and 0x7F, TRAP_STACK_OVERFLOW and TRAP_USER, vector
    ; through the trap table like every code below 0x80.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, unimpl_handler
    LI  r15, {TRAP_TABLE + 0x7E * 8}   ; TRAP_UNIMPLEMENTED
    STR r15, r14, 0
    LI  r14, call_handler
    LI  r15, {TRAP_TABLE + 0x70 * 8}   ; TRAP_STACK_OVERFLOW
    STR r15, r14, 0
    LI  r15, {TRAP_TABLE + 0x7F * 8}   ; TRAP_USER
    STR r15, r14, 0
    LI  sp, {STACK_TOP}
    LI  np, {NURSERY_BASE}
    LI  nl, {NURSERY_LIMIT}
    LI  r19, 0
    LI  r20, 0

    LI  r1, {tag_fixnum(3)}
    LI  r2, {tag_fixnum(4)}
    ALLOC.CONS r16, r1, r2
    LI  r17, 0x2000
    LI  r18, {tag_fixnum(10)}
    STR r17, r18, 0
    LI  r3, 0x77
    LI  r4, 0x77
    LI  r5, 0x77
    LI  r6, 0x77
    LI  r7, 0x77
    LI  r8, 0x77
    LI  r9, 0
    LI  r10, 0x77
    LI  r11, 0x77

{body}

    TRAP 0x70
    TRAP 0x7F
    MOV r12, sp
    LD.CAR r13, r16
    LD.CDR r14, r16
    LI  r23, -8
    AND r15, r16, r23
    LDR r15, r15, 24      ; the word after the cons
    LDR r18, r17, 0
    HALT

unimpl_handler:
    TRAP.CAUSE r22
    TRAP.PC r21
    LI  r23, 1
    ADD r20, r20, r23
    ERET.NEXT

call_handler:
    TRAP.CAUSE r24
    LI  r23, 1
    ADD r19, r19, r23
    ERET
"""
    asm = Assembler()
    asm.assemble(source)
    emit_test('51_reserved', source, {
        'r3': 0x77, 'r4': 0x77, 'r5': 0x77, 'r6': 0x77,   # no destination written
        'r7': 0x77, 'r8': 0x77, 'r10': 0x77, 'r11': 0x77,
        'r9': 0x99,                     # the branch fell through
        'r12': STACK_TOP,               # no push and no pop
        'r13': tag_fixnum(3),           # the cons as ALLOC.CONS made it
        'r14': tag_fixnum(4),
        'r15': 0,                       # nothing stored after it
        'r18': tag_fixnum(10),          # FAA added nothing
        'r20': len(reserved),           # every reserved encoding trapped
        'r21': asm.labels['last'],      # each saved its own PC
        'r22': 0x7E,                    # TRAP_UNIMPLEMENTED
        'r19': 2,                       # TRAP 0x70 and 0x7F reached a handler
        'r24': 0x7F,
    })


# ===========================================================================
# Test 52: ALLOC.CONS takes its car and cdr from registers
# ===========================================================================
def gen_test_52():
    source = f"""\
    ; Test 52: ALLOC.CONS Rd, Rs_car, Rs_cdr makes a cons of two registers'
    ; values, read before the allocation moves np, so np itself names the
    ; address the cons gets (spec/02-isa.md section 4.2); a field of r0
    ; gives nil.

    LI  np, {NURSERY_BASE}
    LI  nl, {NURSERY_LIMIT}
    LI  r1, {tag_fixnum(7)}
    LI  r2, {tag_fixnum(8)}
    ALLOC.CONS r3, r1, r2
    LD.CAR r4, r3
    LD.CDR r5, r3
    ALLOC.CONS r6, np, np     ; both fields: np before the allocation
    LD.CAR r7, r6
    LD.CDR r8, r6
    ALLOC.CONS r9, r0, r1     ; r0 stands for nil
    LD.CAR r10, r9
    LD.CDR r11, r9
    MOV r12, np
    HALT
"""
    emit_test('52_alloc_cons_operands', source, {
        'r3': make_ref(NURSERY_BASE, cons=True),
        'r4': tag_fixnum(7),
        'r5': tag_fixnum(8),
        'r6': make_ref(NURSERY_BASE + 24, cons=True),
        'r7': NURSERY_BASE + 24,      # np as the cons found it
        'r8': NURSERY_BASE + 24,
        'r10': NIL,
        'r11': tag_fixnum(7),
        'r12': NURSERY_BASE + 72,     # three conses of three words each
    })


# ===========================================================================
# Main
# ===========================================================================

if __name__ == '__main__':
    os.makedirs(TB_DIR, exist_ok=True)
    print("Generating gap tests...")
    gen_test_30()
    gen_test_31()
    gen_test_32()
    gen_test_33()
    gen_test_34()
    gen_test_35()
    gen_test_36()
    gen_test_37()
    gen_test_38()
    gen_test_39()
    gen_test_40()
    gen_test_41()
    gen_test_42()
    gen_test_45()
    gen_test_46()
    gen_test_47()
    gen_test_48()
    gen_test_49()
    gen_test_50()
    gen_test_51()
    gen_test_52()
    print("Done. All gap tests generated.")
