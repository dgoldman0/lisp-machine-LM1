#!/usr/bin/env python3
"""
Generate the tests of the two-tile cluster bench (tb/cluster/lm1_cluster_tb.sv).

Each test is a program for each of the cluster's two tiles, tiles 0 and 1,
written as <name>.t0.hex and <name>.t1.hex (one 64-bit word per line, the
tile's whole memory), and <name>.expected, whose lines t<tile>.r<n>=<value>
give the registers each tile must hold once both have halted, and whose
lines t<tile>.write_collisions=<n> and t<tile>.read_collisions=<n> give the
fewest cycles in which the tile's core and the message router must have
met on one of its queues.

Tests:
  01: SEND, RECV and TRY.RECV between the tiles through the cluster's
      message router
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'emu'))
from lm1.asm import Assembler
from lm1.word import tag_fixnum, NIL, T

TESTS_DIR = os.path.join(os.path.dirname(__file__), 'tests')
TILE_MEM_LOG2 = 12          # each tile's own memory: 4096 words, 32 KiB
TRAP_TABLE = 0x5000

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


def desc(tile: int, queue: int) -> int:
    """A queue descriptor: the fixnum tile * 4 + queue (spec/02-isa.md section 8)."""
    return tag_fixnum(tile * 4 + queue)


def image(source: str) -> list[int]:
    binary = Assembler().assemble(source)
    while len(binary) % 8:
        binary += b'\x00'
    words = [int.from_bytes(binary[i:i + 8], 'little') for i in range(0, len(binary), 8)]
    if len(words) > (1 << TILE_MEM_LOG2):
        raise ValueError("the program does not fit a tile's memory")
    return words + [0] * ((1 << TILE_MEM_LOG2) - len(words))


def emit_test(name: str, sources: list[str], expected: dict[str, int]) -> None:
    os.makedirs(TESTS_DIR, exist_ok=True)
    for tile, source in enumerate(sources):
        with open(os.path.join(TESTS_DIR, f'{name}.t{tile}.hex'), 'w') as f:
            for w in image(source):
                f.write(f'{w:016x}\n')
    with open(os.path.join(TESTS_DIR, f'{name}.expected'), 'w') as f:
        for key, val in sorted(expected.items()):
            f.write(f'{key}=0x{val:x}\n')
    print(f'  {name}: generated')


# ===========================================================================
# Test 01: messages between the two tiles
# ===========================================================================
WORDS = 64          # words each tile sends into tile 1's queue 0


def gen_test_01():
    tile0 = f"""\
    ; Tile 0.  It sends {WORDS} words into tile 1's queue 0 while tile 1
    ; sends {WORDS} of its own there, then two words into tile 1's queue 2.
    ; It then empties tile 1's queue 0 alongside tile 1, sends what it took,
    ; a count and a sum, into tile 1's queue 1, waits for tile 1's answer
    ; on its own queue 1, takes the word tile 1 left on tile 1's queue 3,
    ; and finds that queue empty after it.  A tile outside the cluster is
    ; one no route reaches: a SEND there traps TRAP_QUEUE_FULL and a
    ; TRY.RECV finds no word.

    LI  r1, {TRAP_TABLE}
    TRAP 0x90
    LI  r14, skip_handler
    LI  r15, {TRAP_TABLE + 0x30 * 8}   ; TRAP_QUEUE_FULL
    STR r15, r14, 0
    LI  r15, {TRAP_TABLE + 0x31 * 8}   ; TRAP_QUEUE_EMPTY
    STR r15, r14, 0
    LI  r20, 0

    LI  r15, {desc(1, 2)}
    LI  r16, {desc(1, 0)}
    LI  r17, {desc(0, 1)}
    LI  r18, {desc(1, 3)}
    LI  r19, {desc(5, 0)}              ; tile 5: outside the cluster
    LI  r21, {desc(1, 1)}

    ; --- fixnums 1 to {WORDS} into tile 1's queue 0 ---
    LI  r10, {tag_fixnum(1)}
    LI  r11, {tag_fixnum(WORDS + 1)}
fill:
    SEND r16, r10
    ADD.FIX.IMM r10, r10, {tag_fixnum(1)}
    CMP r12, r10, r11
    BR.FIX.LT r12, fill

    ; --- two words into tile 1's queue 2: all the words are in ---
    LI  r10, {tag_fixnum(1234)}
    SEND r15, r10
    LI  r10, {tag_fixnum(5678)}
    SEND r15, r10

    ; --- take words from tile 1's queue 0 until it is empty ---
    LI  r13, 0            ; count
    LI  r14, 0            ; sum
drain:
    TRY.RECV r10, r11, r16
    BR.NIL r11, drained
    ADD.FIX r14, r14, r10
    ADD.FIX.IMM r13, r13, {tag_fixnum(1)}
    BR drain
drained:
    SEND r21, r13
    SEND r21, r14

    ; --- tile 1's answer on the own queue 1 ---
wait:
    TRY.RECV r1, r2, r17
    BR.NIL r2, wait

    ; --- tile 1's word on its queue 3, then that queue empty ---
    TRY.RECV r3, r4, r18
    TRY.RECV r5, r6, r18
    LI  r7, 0x77
    RECV r7, r18          ; traps TRAP_QUEUE_EMPTY
    MOV r8, r22

    ; --- a tile outside the cluster ---
    LI  r10, {tag_fixnum(7)}
    SEND r19, r10         ; traps TRAP_QUEUE_FULL
    MOV r9, r22
    TRY.RECV r10, r11, r19
    MOV r12, r20
    HALT

{SKIP_HANDLER}"""

    tile1 = f"""\
    ; Tile 1.  It sends fixnums 100 to {99 + WORDS} into its own queue 0,
    ; waits for tile 0's two words on its queue 2, takes words from queue 0
    ; until it is empty while tile 0 takes them too, adds tile 0's count and
    ; sum from its queue 1 to its own, leaves a word on its queue 3 for
    ; tile 0 to take, and answers tile 0 on tile 0's queue 1 with the sum
    ; of the two words.

    LI  r15, {desc(1, 2)}
    LI  r16, {desc(1, 0)}
    LI  r17, {desc(0, 1)}
    LI  r18, {desc(1, 3)}
    LI  r19, {desc(1, 1)}

    ; --- fixnums 100 to {99 + WORDS} into the own queue 0 ---
    LI  r10, {tag_fixnum(100)}
    LI  r11, {tag_fixnum(100 + WORDS)}
fill:
    SEND r16, r10
    ADD.FIX.IMM r10, r10, {tag_fixnum(1)}
    CMP r12, r10, r11
    BR.FIX.LT r12, fill

    ; --- tile 0's two words on queue 2 ---
w1:
    TRY.RECV r1, r2, r15
    BR.NIL r2, w1
w2:
    TRY.RECV r3, r4, r15
    BR.NIL r4, w2

    ; --- take words from queue 0 until it is empty ---
    LI  r5, 0             ; count
    LI  r6, 0             ; sum
drain:
    TRY.RECV r7, r8, r16
    BR.NIL r8, drained
    ADD.FIX r6, r6, r7
    ADD.FIX.IMM r5, r5, {tag_fixnum(1)}
    BR drain
drained:

    ; --- tile 0's count and sum on queue 1 ---
w3:
    TRY.RECV r13, r8, r19
    BR.NIL r8, w3
w4:
    TRY.RECV r14, r8, r19
    BR.NIL r8, w4
    ADD.FIX r5, r5, r13
    ADD.FIX r6, r6, r14

    ; --- a word for tile 0 on queue 3, then the answer ---
    LI  r9, {tag_fixnum(4242)}
    SEND r18, r9
    ADD.FIX r10, r1, r3
    SEND r17, r10
    HALT
"""
    emit_test('01_messages', [tile0, tile1], {
        't0.r1': tag_fixnum(1234 + 5678),   # tile 1's answer
        't0.r2': T,
        't0.r3': tag_fixnum(4242),          # taken from tile 1's queue 3
        't0.r4': T,
        't0.r5': NIL,                       # that queue empty after it
        't0.r6': NIL,
        't0.r7': 0x77,                      # the RECV on it trapped
        't0.r8': 0x31,                      # TRAP_QUEUE_EMPTY
        't0.r9': 0x30,                      # TRAP_QUEUE_FULL: tile 5
        't0.r10': NIL,                      # TRY.RECV on tile 5
        't0.r11': NIL,
        't0.r12': 2,                        # two traps
        't1.r1': tag_fixnum(1234),          # tile 0's words, in order
        't1.r3': tag_fixnum(5678),
        't1.r5': tag_fixnum(2 * WORDS),     # every word taken once
        't1.r6': tag_fixnum(sum(range(1, WORDS + 1)) + sum(range(100, 100 + WORDS))),
        't1.r10': tag_fixnum(1234 + 5678),
        # Both tiles' cores met the router on one of tile 1's queues in some
        # cycle, writing and reading
        't1.write_collisions': 1,
        't1.read_collisions': 1,
    })


if __name__ == '__main__':
    print('Generating cluster tests...')
    gen_test_01()
    print('Done.')
