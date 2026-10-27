"""Phase 7 tests — block device emulation and JR.

Tests the emulator's block device trap (0x82), which reads and writes
4 KiB blocks of a host file, and the JR (jump register) instruction.
"""

import os
import tempfile

from lm1.testing.harness import test
from lm1.execute import Emulator
from lm1.asm import Assembler
from lm1.word import tag_fixnum


# ===================================================================
# Batch: phase7_block_io — block device emulation
# ===================================================================

@test("block_io_read", batch="phase7_block_io")
def test_block_io_read():
    """TRAP 0x82 reads a block from host file into emulator memory."""
    with tempfile.NamedTemporaryFile(suffix='.img', delete=False) as f:
        # Write a 4096-byte block with known pattern
        data = bytes(range(256)) * 16  # 4096 bytes
        f.write(data)
        f.flush()
        fname = f.name

    try:
        asm = Assembler()
        source = f"""
            LI r1, 0        ; BLOCK_READ
            LI r2, 0        ; block 0
            LI r3, 0x4000   ; destination
            TRAP 0x82
            HALT
        """
        words = asm.assemble_to_words(source)
        emu = Emulator(
            mem_size=256 * 1024,
            nursery_base=0x30000, nursery_size=0x10000,
            oldgen_base=0x50000, oldgen_size=0x20000,
            block_device=fname,
        )
        for i, w in enumerate(words):
            emu.mem.store_u32(i * 4, w)
        emu.thread.pc = 0
        emu.run(max_instructions=20)

        # Verify first few bytes
        for i in range(16):
            b = emu.mem.load_byte(0x4000 + i)
            assert b == i, f"byte[{i}]={b}, expected {i}"
        # r0 should be tag_fixnum(0) = 0 (success)
        assert emu.thread.regs[0] == tag_fixnum(0)
    finally:
        os.unlink(fname)


@test("block_io_no_device", batch="phase7_block_io")
def test_block_io_no_device():
    """TRAP 0x82 without block device returns error code."""
    asm = Assembler()
    source = """
        LI r1, 0        ; BLOCK_READ
        LI r2, 0        ; block 0
        LI r3, 0x4000   ; destination
        TRAP 0x82
        HALT
    """
    words = asm.assemble_to_words(source)
    emu = Emulator(mem_size=256 * 1024,
                   nursery_base=0x30000, nursery_size=0x10000,
                   oldgen_base=0x50000, oldgen_size=0x20000)
    for i, w in enumerate(words):
        emu.mem.store_u32(i * 4, w)
    emu.thread.pc = 0
    emu.run(max_instructions=20)
    # r0 = tag_fixnum(-1) = -2 (error)
    assert emu.thread.regs[0] == tag_fixnum(-1), \
        f"Expected error code, got {emu.thread.regs[0]:#x}"


@test("block_io_write_read", batch="phase7_block_io")
def test_block_io_write_read():
    """TRAP 0x82 write then read roundtrips data correctly."""
    with tempfile.NamedTemporaryFile(suffix='.img', delete=False) as f:
        fname = f.name

    try:
        asm = Assembler()
        # Write block 0 with pattern, then read it back to a different addr
        source = """
            ; Store test pattern at 0x4000
            LI r5, 0x4000
            LI r6, 0xAB
            STR r5, r6, 0
            ; Write block 0
            LI r1, 1        ; BLOCK_WRITE
            LI r2, 0        ; block 0
            LI r3, 0x4000   ; source
            TRAP 0x82
            ; Read block 0 to 0x5000
            LI r1, 0        ; BLOCK_READ
            LI r2, 0        ; block 0
            LI r3, 0x5000   ; destination
            TRAP 0x82
            ; Load first word from destination
            LI r5, 0x5000
            LDR r7, r5, 0
            HALT
        """
        words = asm.assemble_to_words(source)
        emu = Emulator(
            mem_size=256 * 1024,
            nursery_base=0x30000, nursery_size=0x10000,
            oldgen_base=0x50000, oldgen_size=0x20000,
            block_device=fname,
        )
        for i, w in enumerate(words):
            emu.mem.store_u32(i * 4, w)
        emu.thread.pc = 0
        emu.run(max_instructions=40)
        # r7 should contain the same value as what was stored at 0x4000
        assert emu.thread.regs[7] == 0xAB, \
            f"Roundtrip failed: got {emu.thread.regs[7]:#x}"
    finally:
        os.unlink(fname)


# ===================================================================
# Batch: phase7_jr — JR (jump register) instruction
# ===================================================================

@test("jr_basic", batch="phase7_jr")
def test_jr_basic():
    """JR jumps to address in register."""
    asm = Assembler()
    source = """
        LI r5, target
        JR r5
        HALT            ; should be skipped
    target:
        LI r1, 42
        HALT
    """
    words = asm.assemble_to_words(source)
    emu = Emulator(mem_size=256 * 1024,
                   nursery_base=0x30000, nursery_size=0x10000,
                   oldgen_base=0x50000, oldgen_size=0x20000)
    for i, w in enumerate(words):
        emu.mem.store_u32(i * 4, w)
    emu.thread.pc = 0
    emu.run(max_instructions=20)
    assert emu.thread.regs[1] == 42, f"Expected 42, got {emu.thread.regs[1]}"
