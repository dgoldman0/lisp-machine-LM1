"""LM-1 instruction executor.

Phase 1: scalar ops, raw loads/stores, branches, LI/LUI, NOP, HALT,
TILE.ID, THREAD.ID, CYCLE, console I/O traps, PUSH/POP.

Phase 2: tagged arithmetic (already in Phase 1), nursery allocation,
header template table, tagged field access (LD/ST/LD.CAR/LD.CDR/ST.WB/
ST.CAR/ST.CDR), TST.SHAPE, ALLOC/ALLOC.CONS/ALLOCV/ALLOC.CLOSURE.

Implements the fetch-decode-execute loop for a single tile / single thread.
"""

from __future__ import annotations

import sys
from typing import Optional, TextIO

from .word import (
    WORD_MASK, SIGN_BIT, FIXNUM_LIMIT, NIL, T,
    is_fixnum, tag_fixnum, untag_fixnum,
    is_ref, is_cons_ref, is_any_ref, ref_address, make_ref,
    is_header, make_header, header_size, header_shape_id, header_subtype,
    is_truthy, u64, s64, add64, sub64,
    HDR_CONS, HDR_CLOSURE, HDR_VECTOR,
    REF_ADDR_MASK, TAG_REF, TAG_CONS,
)
from .decode import (
    Op, Instruction, decode, reserved,
    FUNC_ADD, FUNC_SUB, FUNC_MUL, FUNC_DIV, FUNC_MOD,
    FUNC_AND, FUNC_OR, FUNC_XOR, FUNC_SHL, FUNC_SHR, FUNC_ASR, FUNC_NOT,
    FUNC_ADD_FIX, FUNC_SUB_FIX, FUNC_MUL_FIX, FUNC_DIV_FIX,
    FUNC_CMP, FUNC_EQ,
    BR_T, BR_NIL, BR_FIX_LT, BR_FIX_EQ, BR_FIX_GT, BR_EQ,
    FUNC_PUSH, FUNC_POP,
    FUNC_TILE_ID, FUNC_THREAD_ID, FUNC_CYCLE,
    FUNC_HALT, FUNC_NOP,
    FUNC_ERET_NEXT,
)
from .traps import (
    LM1Trap,
    TRAP_NOT_FIXNUM, TRAP_FIXNUM_OVERFLOW, TRAP_DIVIDE_BY_ZERO,
    TRAP_TYPE_MISMATCH,
    TRAP_NURSERY_OVERFLOW, TRAP_BAD_LENGTH, TRAP_NOT_REF,
    TRAP_IC_MISS, TRAP_NOT_CLOSURE,
    TRAP_QUEUE_FULL, TRAP_NO_ROUTE,
    TRAP_UNIMPLEMENTED,
)
from .core import ThreadContext, HEADER_TEMPLATES
from .memory import Memory

from collections import deque


# ---------------------------------------------------------------------------
# Emulator I/O trap codes (emulator-specific, per design/emulator.md § 7)
# ---------------------------------------------------------------------------
EMU_TRAP_PUTCHAR   = 0x80
EMU_TRAP_GETCHAR   = 0x81
EMU_TRAP_BLOCK_IO  = 0x82
EMU_TRAP_MEM_BYTE  = 0x84  # Byte-level memory: r1=sub(0=load,1=store), r2=addr, r3=offset, r4=value
EMU_TRAP_SET_TRAP_TABLE = 0x90    # BIOS: r1 = trap table base address
EMU_TRAP_SET_TEMPLATE   = 0x91   # BIOS: r1 = index, r2 = 64-bit header value
EMU_TRAP_SET_CARD_BASE  = 0x92   # r1 = card table base address
EMU_TRAP_SET_CARD_SHIFT = 0x93   # r1 = card shift (bits 5:0)
EMU_TRAP_SET_GEN_BOUNDARY = 0x94 # r1 = generation boundary address
EMU_TRAP_SET_QUEUE_BASE = 0x95   # r1 = queue base address
EMU_TRAP_DEBUG_PRINT    = 0x9F   # Debug: print string (r1=addr, r2=len)

# The longest vector ALLOCV makes: its payload, the length slot and the
# elements, is the most a header's 16-bit size holds.
ALLOCV_MAX_LENGTH = 0xFFFF - 1

# Block I/O sub-functions (in r1)
BLOCK_IO_READ  = 0
BLOCK_IO_WRITE = 1
BLOCK_SIZE     = 4096  # bytes per block


class Emulator:
    """Single-tile LM-1 functional emulator.

    Phases 1-4: single-thread scalar, tagged, GC, dispatch.
    Phase 5: message queues (SEND/RECV/TRY.RECV), CAS.TAGGED, FAA, FENCE.GC,
             multi-thread round-robin scheduling, multi-tile cluster support.
    """

    # The chunk of the nursery a thread takes at a time, at most an equal
    # slice of it for each thread.
    DEFAULT_NURSERY_CHUNK = 0x1000

    # Default nursery: 64 KiB at the top of the first 256 KiB
    DEFAULT_NURSERY_BASE  = 0x0003_0000   # 192 KiB offset
    DEFAULT_NURSERY_SIZE  = 0x0001_0000   # 64 KiB

    # Default old-gen: 256 KiB starting at 512 KiB offset
    DEFAULT_OLDGEN_BASE   = 0x0008_0000   # 512 KiB offset
    DEFAULT_OLDGEN_SIZE   = 0x0004_0000   # 256 KiB

    # Card table: one byte per card (card size = 1 << card_shift)
    DEFAULT_CARD_SHIFT = 6  # 64 bytes per card (matches RTL default)

    # Queue depth (per queue, per tile)
    QUEUE_DEPTH = 512
    NUM_QUEUES  = 4

    def __init__(
        self,
        mem_size: int = 4 * 1024 * 1024,  # 4 MiB default
        *,
        trace: bool = False,
        stdin: TextIO | None = None,
        stdout: TextIO | None = None,
        nursery_base: int | None = None,
        nursery_size: int | None = None,
        oldgen_base: int | None = None,
        oldgen_size: int | None = None,
        tile_id: int = 0,
        num_threads: int = 1,
        block_device: str | None = None,
        nursery_chunk: int | None = None,
    ):
        self.mem = Memory(mem_size)
        self.tile_id = tile_id

        # Multi-thread support (Phase 5).  The tile's threads share its one
        # header-template table (spec/07-encoding.md section 6), so an object
        # any of them allocates gets the tile's headers.
        templates = [0] * HEADER_TEMPLATES
        self.threads: list[ThreadContext] = []
        for tid in range(num_threads):
            tc = ThreadContext(tile_id=tile_id, thread_id=tid)
            tc.header_templates = templates
            self.threads.append(tc)
        self._current_thread_idx = 0

        # I/O
        self._stdin: TextIO = stdin or sys.stdin
        self._stdout: TextIO = stdout or sys.stdout
        self.trace = trace

        # Nursery region
        self.nursery_base = nursery_base if nursery_base is not None else self.DEFAULT_NURSERY_BASE
        self.nursery_size = nursery_size if nursery_size is not None else self.DEFAULT_NURSERY_SIZE

        # Old-gen region (promotion target for surviving nursery objects)
        self.oldgen_base = oldgen_base if oldgen_base is not None else self.DEFAULT_OLDGEN_BASE
        self.oldgen_size = oldgen_size if oldgen_size is not None else self.DEFAULT_OLDGEN_SIZE
        self.oldgen_ptr = self.oldgen_base  # bump pointer into old-gen

        # Write barrier configuration (matches RTL registers).  The barrier
        # counts addresses at or above gen_boundary as old and those below
        # it as young, so the nursery lies below the boundary and old-gen
        # above it: the boundary starts at the nursery's end.
        nursery_end = self.nursery_base + self.nursery_size
        if self.oldgen_base < nursery_end:
            raise ValueError(
                f"old-gen at {self.oldgen_base:#x} lies below the nursery's end "
                f"{nursery_end:#x}; the barrier's generation boundary needs "
                f"old-gen above the nursery"
            )
        self.card_shift = self.DEFAULT_CARD_SHIFT     # bits; card_size = 1 << card_shift
        self.card_table_base = 0                       # physical address of card table in memory
        self.gen_boundary = nursery_end                 # addresses >= gen_boundary are old-gen
        # The message queues' base, which SET_QUEUE_BASE records and
        # SYS_INFO 8 reads back; the modeled queues do not live in memory.
        self.queue_base = 0

        # Card table: one byte per card.  Covers the entire memory space.
        # A card is "dirty" (0xFF) when a store-with-barrier writes a
        # young ref into an old-gen object.  Uses configurable card_shift.
        card_size = 1 << self.card_shift
        self.card_table_size = mem_size // card_size
        self.card_table = bytearray(self.card_table_size)

        # GC statistics
        self.gc_count = 0

        # Inline cache table: (callsite_pc, shape_id) → code_entry_addr
        # Per-tile hash map used for polymorphic dispatch.
        self.ic_table: dict[tuple[int, int], int] = {}

        # Hardware message queues (Phase 5): 4 queues per tile
        self.queues: list[deque] = [
            deque(maxlen=self.QUEUE_DEPTH) for _ in range(self.NUM_QUEUES)
        ]

        # Cluster reference for cross-tile messaging (set by Cluster)
        self.cluster: Optional['Cluster'] = None

        # The tile's threads share its nursery, as the runtime divides it
        # (spec/04-soc.md section 4.3): each thread bumps its own pointer
        # through a chunk of it, so no two of them hand out the same words.
        # A thread whose chunk runs out takes another from the nursery's free
        # space, and only when none is left does a collection empty the
        # nursery and give every thread a fresh chunk.  A chunk is
        # nursery_chunk bytes, at most an equal slice for each thread, and
        # nursery_slices holds where each thread's chunk begins.
        slice_size = (self.nursery_size // num_threads) & ~7
        wanted = self.DEFAULT_NURSERY_CHUNK if nursery_chunk is None else nursery_chunk
        if wanted < 8 or wanted % 8:
            raise ValueError("nursery_chunk is a multiple of 8 from 8 up, not %r" % (wanted,))
        self.nursery_chunk = min(wanted, slice_size)
        self.nursery_slices: list[int] = [0] * num_threads
        self.nursery_free = 0
        self._give_fresh_chunks()

        # Install default header templates (index 0 = cons)
        self._init_header_templates()

        # Block device (Phase 7)
        self.block_device_path: str | None = block_device

        # Stats
        self.instruction_count = 0

    @property
    def thread(self) -> ThreadContext:
        """Current active thread — backward compatible with single-thread API."""
        return self.threads[self._current_thread_idx]

    @thread.setter
    def thread(self, value: ThreadContext) -> None:
        self.threads[self._current_thread_idx] = value

    def _init_header_templates(self) -> None:
        """Set up the default header-template table entries."""
        t = self.thread
        # Index 0: Cons cell header (hdr_sub=1, size=2, shape_id=0)
        t.header_templates[0] = make_header(HDR_CONS, 2, 0)
        # Index 1: Closure header template (hdr_sub=4, size=0 — filled at alloc)
        t.header_templates[1] = make_header(HDR_CLOSURE, 0, 1)
        # Index 2: Vector header template (hdr_sub=2, size=0 — filled at alloc)
        t.header_templates[2] = make_header(HDR_VECTOR, 0, 2)

    # -- Register / memory convenience ---

    def reg(self, idx: int) -> int:
        return self.thread.regs[idx & 0x1F]

    def set_reg(self, idx: int, val: int) -> None:
        self.thread.regs[idx & 0x1F] = val & WORD_MASK

    # -- Main loop ---

    def run(self, max_instructions: int = 0) -> None:
        """Run until HALT or max_instructions reached (0 = unlimited)."""
        t = self.thread
        mem = self.mem
        count = 0

        while not t.halted and not t.stalled:
            # Fetch
            raw = mem.load_u32(t.pc)
            inst = decode(raw)

            if self.trace:
                self._trace_instruction(t.pc, inst)

            # Advance PC (may be overwritten by branches)
            next_pc = t.pc + 4

            try:
                # Execute
                next_pc = self._execute(inst, next_pc)
            except LM1Trap as trap:
                next_pc = self._take_trap(trap)

            # The PC is a 64-bit register: a branch below address 0 wraps.
            t.pc = next_pc & WORD_MASK
            t.cycle_count += 1
            self.instruction_count += 1
            count += 1

            if max_instructions and count >= max_instructions:
                break

    # -- Execute dispatch ---

    def _execute(self, inst: Instruction, next_pc: int) -> int:
        """Execute one decoded instruction.  Returns the next PC."""
        op = inst.opcode
        t = self.thread

        # A selecting field's value that names no instruction is reserved:
        # it traps before the instruction does anything.
        if reserved(inst):
            raise LM1Trap(TRAP_UNIMPLEMENTED, f"reserved encoding {inst.raw:#010x}")

        # ---- Scalar arithmetic (raw, untagged 64-bit) ----
        if op == Op.ARITH_RAW:
            a = t.regs[inst.rs1]
            b = t.regs[inst.rs2]
            match inst.func:
                case 0:  # ADD
                    t.regs[inst.rd] = (a + b) & WORD_MASK
                case 1:  # SUB
                    t.regs[inst.rd] = (a - b) & WORD_MASK
                case 2:  # MUL
                    t.regs[inst.rd] = (a * b) & WORD_MASK
                case 3:  # DIV
                    if b == 0:
                        raise LM1Trap(TRAP_DIVIDE_BY_ZERO)
                    # Unsigned division for raw
                    t.regs[inst.rd] = (a // b) & WORD_MASK
                case 4:  # MOD
                    if b == 0:
                        raise LM1Trap(TRAP_DIVIDE_BY_ZERO)
                    t.regs[inst.rd] = (a % b) & WORD_MASK

        # ---- Bitwise ----
        elif op == Op.BITWISE:
            a = t.regs[inst.rs1]
            b = t.regs[inst.rs2]
            match inst.func:
                case 0:  # AND
                    t.regs[inst.rd] = a & b
                case 1:  # OR
                    t.regs[inst.rd] = a | b
                case 2:  # XOR
                    t.regs[inst.rd] = a ^ b
                case 3:  # SHL
                    shift = b & 63
                    t.regs[inst.rd] = (a << shift) & WORD_MASK
                case 4:  # SHR (logical)
                    shift = b & 63
                    t.regs[inst.rd] = a >> shift
                case 5:  # ASR (arithmetic)
                    shift = b & 63
                    t.regs[inst.rd] = u64(s64(a) >> shift)
                case 6:  # NOT
                    t.regs[inst.rd] = (~a) & WORD_MASK

        # ---- Raw loads (LDR) ----
        elif op == Op.LDR:
            base = t.regs[inst.rs1]
            offset = inst.imm16  # signed, byte offset
            addr = (base + offset) & WORD_MASK
            # Default: 64-bit (dword) load
            t.regs[inst.rd] = self.mem.load_word(addr & ~7)

        # ---- Raw stores (STR) ----
        elif op == Op.STR:
            base = t.regs[inst.rd]   # Format S: Rs is in rd position
            val = t.regs[inst.rs1]   # Rt is in rs1 position
            offset = inst.imm16
            addr = (base + offset) & WORD_MASK
            self.mem.store_word(addr & ~7, val)

        # ---- Sub-word loads (LDB / LDH / LDW) ----
        elif op == Op.LDB:
            base = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK
            t.regs[inst.rd] = self.mem.load_byte(addr)
        elif op == Op.LDH:
            base = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK & ~1
            t.regs[inst.rd] = self.mem.load_u16(addr)
        elif op == Op.LDW:
            base = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK & ~3
            t.regs[inst.rd] = self.mem.load_u32_data(addr)

        # ---- Sub-word stores (STB / STH / STW) ----
        elif op == Op.STB:
            base = t.regs[inst.rd]
            val = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK
            self.mem.store_byte(addr, val & 0xFF)
        elif op == Op.STH:
            base = t.regs[inst.rd]
            val = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK & ~1
            self.mem.store_u16(addr, val & 0xFFFF)
        elif op == Op.STW:
            base = t.regs[inst.rd]
            val = t.regs[inst.rs1]
            addr = (base + inst.imm16) & WORD_MASK & ~3
            self.mem.store_u32_data(addr, val & 0xFFFF_FFFF)

        # ---- LI (load immediate) ----
        elif op == Op.LI:
            # imm16 is sign-extended to 64 bits
            t.regs[inst.rd] = inst.imm16 & WORD_MASK

        # ---- LI32 (load 32-bit immediate from next word) ----
        elif op == Op.LI32:
            imm32 = self.mem.load_u32(next_pc)
            t.regs[inst.rd] = imm32 & WORD_MASK
            next_pc += 4  # skip the immediate word

        # ---- LUI (load upper immediate) ----
        elif op == Op.LUI:
            # Load imm16 into bits 31:16, zero other bits
            t.regs[inst.rd] = (inst.imm16 & 0xFFFF) << 16

        # ---- Unconditional branch ----
        elif op == Op.BR:
            offset = inst.imm16  # in words
            next_pc = t.pc + (offset * 4)

        # ---- Conditional branches ----
        elif op == Op.BR_COND:
            # B-format: bits 25:21 = Rs1 (register), bits 20:16 = cond
            # Decoder maps 25:21 → rd, 20:16 → rs1
            cond = inst.rs1   # condition type in bits 20:16
            val1 = t.regs[inst.rd]  # register in bits 25:21
            offset = inst.imm16
            taken = False

            match cond:
                case 0:  # BR.T — branch if truthy
                    taken = is_truthy(val1)
                case 1:  # BR.NIL — branch if nil
                    taken = (val1 == NIL)
                case 2:  # BR.FIX.LT — needs second reg; we repurpose func field
                    # For 2-register cond branches, val2 is in a register
                    # specified differently. Let's use: Rs1 = first, 
                    # val2 from the word at func field as register index.
                    # Actually, per the B format, Rs1 and Rs2 ARE both register
                    # fields — but Rs2 is being used as condition selector.
                    # 
                    # Resolution: For BR.FIX.LT/EQ/GT and BR.EQ which need
                    # two registers, we'll use a different encoding scheme:
                    # Rs1 bits 25:21 = register, bits 20:16 encode BOTH
                    # the condition AND the second register.
                    # 
                    # Simpler approach for now: single-register conditions only.
                    # Two-register comparisons use CMP.TAGGED first.
                    taken = is_fixnum(val1) and s64(val1) < 0
                case 3:  # BR.FIX.EQ (== 0)
                    taken = val1 == 0
                case 4:  # BR.FIX.GT (> 0)
                    taken = is_fixnum(val1) and s64(val1) > 0 and val1 != 0
                case 5:  # BR.EQ — word-equal to zero
                    taken = val1 == 0

            if taken:
                next_pc = t.pc + (offset * 4)

        # ---- PUSH / POP ----
        elif op == Op.PUSH_POP:
            match inst.func:
                case 0:  # PUSH
                    t.sp = (t.sp - 8) & WORD_MASK
                    self.mem.store_word(t.sp, t.regs[inst.rd])
                case 1:  # POP
                    t.regs[inst.rd] = self.mem.load_word(t.sp)
                    t.sp = (t.sp + 8) & WORD_MASK

        # ---- PUSH.MULTI (Format I: rd=bank, imm16=mask) ----
        elif op == Op.PUSH_MULTI:
            bank = inst.rd           # 0 or 1: the 32 registers form two banks of 16
            mask = inst.imm16 & 0xFFFF
            base_reg = bank * 16
            for i in range(16):
                if mask & (1 << i):
                    t.sp = (t.sp - 8) & WORD_MASK
                    self.mem.store_word(t.sp, t.regs[base_reg + i])

        # ---- POP.MULTI (Format I: rd=bank, imm16=mask) ----
        elif op == Op.POP_MULTI:
            bank = inst.rd
            mask = inst.imm16 & 0xFFFF
            base_reg = bank * 16
            for i in range(15, -1, -1):
                if mask & (1 << i):
                    t.regs[base_reg + i] = self.mem.load_word(t.sp)
                    t.sp = (t.sp + 8) & WORD_MASK

        # ---- TRAP ----
        elif op == Op.TRAP:
            trap_code = inst.raw26 & 0xFF
            if trap_code < 0x80:
                # A trap for the trap table, the runtime's: it is the
                # instruction's whole work, so ERET resumes after it, as a
                # system call returns.
                trap = LM1Trap(trap_code)
                trap.resume_pc = next_pc
                raise trap
            self._system_trap(trap_code)

        # ---- ERET / ERET.NEXT ----
        elif op == Op.ERET:
            # ERET resumes at the saved PC, so a fault runs again; ERET.NEXT
            # resumes at the saved PC + 4, after a faulting instruction the
            # handler carried out itself (spec/03-core.md section 6.3).
            if not t.in_trap:
                raise LM1Trap(TRAP_UNIMPLEMENTED, "ERET outside of trap handler")
            t.in_trap = False
            next_pc = t.trap_pc + 4 if inst.func == FUNC_ERET_NEXT else t.trap_pc

        # ---- System info ----
        elif op == Op.SYS_INFO:
            # Bits 25:21 name the destination Rd, the only register written,
            # and bits 20:16 select what is read.
            sub = inst.rs1  # sub-function in rs1 position
            match sub:
                case 0:  t.regs[inst.rd] = t.tile_id
                case 1:  t.regs[inst.rd] = t.thread_id
                case 2:  t.regs[inst.rd] = t.cycle_count & WORD_MASK
                case 3:  t.regs[inst.rd] = t.trap_cause    # TRAP_CAUSE
                case 4:  t.regs[inst.rd] = t.trap_pc       # TRAP_PC
                case 5:  t.regs[inst.rd] = self.card_table_base   # CARD_BASE
                case 6:  t.regs[inst.rd] = self.card_shift        # CARD_SHIFT
                case 7:  t.regs[inst.rd] = self.gen_boundary      # GEN_BOUNDARY
                case 8:  t.regs[inst.rd] = self.queue_base        # QUEUE_BASE
                # The emulator models no movement engines and no counters,
                # so it reads 0 for their status and results.
                case 9:  t.regs[inst.rd] = 0                      # GC_STATUS (stub)
                case 10: t.regs[inst.rd] = 0                      # PERF_CTR (stub)
                case 11: t.regs[inst.rd] = 0                      # SCAN_COUNT (stub)
                case 12: t.regs[inst.rd] = 0                      # SCAN_HEAD_OBJ (stub)
                case 13: t.regs[inst.rd] = 0                      # SCAN_HEAD_FIELD (stub)
                case 14: t.regs[inst.rd] = 0                      # SCAN_POP_REF (stub)

        # ---- HALT / NOP ----
        elif op == Op.HALT_NOP:
            sub = (inst.raw26 >> 21) & 0x1F
            if sub == 0:   # HALT
                t.halted = True
            # else: NOP — do nothing

        # ---- Tagged arithmetic (Phase 1 includes these) ----
        elif op == Op.ARITH_FIX:
            a = t.regs[inst.rs1]
            b = t.regs[inst.rs2]
            if not is_fixnum(a) or not is_fixnum(b):
                raise LM1Trap(TRAP_NOT_FIXNUM)
            match inst.func:
                case 0:  # ADD.FIX
                    result, overflow = add64(a, b)
                    if overflow:
                        raise LM1Trap(TRAP_FIXNUM_OVERFLOW)
                    t.regs[inst.rd] = result
                case 1:  # SUB.FIX
                    result, overflow = sub64(a, b)
                    if overflow:
                        raise LM1Trap(TRAP_FIXNUM_OVERFLOW)
                    t.regs[inst.rd] = result
                case 2:  # MUL.FIX
                    # untag(a) times the still-tagged b is the tagged
                    # product, which is a fixnum only when the exact product
                    # fits a signed 64-bit word (spec/02-isa.md section 3.2).
                    product = untag_fixnum(a) * s64(b)
                    if not -SIGN_BIT <= product < SIGN_BIT:
                        raise LM1Trap(TRAP_FIXNUM_OVERFLOW)
                    t.regs[inst.rd] = product & WORD_MASK
                case 3:  # DIV.FIX
                    if b == 0:
                        raise LM1Trap(TRAP_DIVIDE_BY_ZERO)
                    va = untag_fixnum(a)
                    vb = untag_fixnum(b)
                    # The quotient truncates toward zero and is exact at
                    # every size.  Only the most negative fixnum divided by
                    # -1 leaves the fixnum range, and that traps.
                    quotient = abs(va) // abs(vb)
                    if (va < 0) != (vb < 0):
                        quotient = -quotient
                    if quotient >= FIXNUM_LIMIT:
                        raise LM1Trap(TRAP_FIXNUM_OVERFLOW)
                    t.regs[inst.rd] = tag_fixnum(quotient)

        elif op == Op.ADD_FIX_IMM:
            a = t.regs[inst.rs1]
            imm = inst.imm16 & WORD_MASK  # pre-tagged fixnum
            if not is_fixnum(a):
                raise LM1Trap(TRAP_NOT_FIXNUM)
            result, overflow = add64(a, imm)
            if overflow:
                raise LM1Trap(TRAP_FIXNUM_OVERFLOW)
            t.regs[inst.rd] = result

        elif op == Op.CMP_TAGGED:
            a = t.regs[inst.rs1]
            b = t.regs[inst.rs2]
            match inst.func:
                case 0:  # CMP.TAGGED
                    if is_fixnum(a) and is_fixnum(b):
                        va, vb = s64(a), s64(b)
                        if va < vb:
                            t.regs[inst.rd] = tag_fixnum(-1)
                        elif va == vb:
                            t.regs[inst.rd] = tag_fixnum(0)
                        else:
                            t.regs[inst.rd] = tag_fixnum(1)
                    elif (a & 7) == (b & 7):
                        # Same primary tag — identity comparison
                        t.regs[inst.rd] = tag_fixnum(0) if a == b else tag_fixnum(1)
                    else:
                        raise LM1Trap(TRAP_TYPE_MISMATCH)
                case 1:  # EQ (raw word equality)
                    t.regs[inst.rd] = T if a == b else NIL

        # ---- TST (type test) ----
        elif op == Op.TST:
            val = t.regs[inst.rs1]
            tag_const = inst.imm16 & 0x7
            result = False
            match tag_const:
                case 0: result = is_fixnum(val)            # TAG_FIXNUM
                case 1: result = (val & 3) == 1            # TAG_REF
                case 2: result = (val & 7) == 3            # TAG_CONS
                case 3: result = (val & 7) == 5            # TAG_SPECIAL
                case 4: result = val == NIL                 # TAG_NIL
                case 5: result = (val & 0xFF) == 0x35      # TAG_CHAR
                case 6: result = (val & 0xFF) == 0x3D      # TAG_SFLOAT
                case 7: result = (val & 7) == 7            # TAG_HEADER
            t.regs[inst.rd] = T if result else NIL

        # ---- Prefetch (no-ops) ----
        elif op in (Op.PREFETCH_REF, Op.PREFETCH_FLD, Op.PREFETCH_CDR, Op.GATHER_PRE):
            pass  # no-op in emulator

        # ================================================================
        # Phase 2: Allocation
        # ================================================================

        # ---- ALLOC Rd, #words, #header_template ----
        elif op == Op.ALLOC:
            rd = (inst.raw26 >> 21) & 0x1F
            n_words = (inst.raw26 >> 16) & 0x1F
            tmpl_idx = inst.raw26 & 0xFFFF
            total_bytes = (1 + n_words) * 8  # header + payload
            self._alloc_object(t, rd, total_bytes, tmpl_idx, n_words)

        # ---- ALLOC.CONS Rd ----
        elif op == Op.ALLOC_CONS:
            rd = (inst.raw26 >> 21) & 0x1F
            rs1 = inst.rs1  # car register
            rs2 = inst.rs2  # cdr register
            # Convention: rs1/rs2 == 0 → default to NIL (X-format compat)
            car = t.regs[rs1] if rs1 != 0 else NIL
            cdr = t.regs[rs2] if rs2 != 0 else NIL
            self._alloc_cons(t, rd, car, cdr)

        # ---- ALLOCV Rd, Rs_length, #header_template ----
        elif op == Op.ALLOCV:
            rd = (inst.raw26 >> 21) & 0x1F
            rs_len = (inst.raw26 >> 16) & 0x1F
            tmpl_idx = inst.raw26 & 0xFFFF
            length = t.regs[rs_len]
            if not is_fixnum(length):
                raise LM1Trap(TRAP_NOT_FIXNUM, "ALLOCV: length must be a fixnum")
            n_elems = untag_fixnum(length)
            # A header's 16-bit size holds the length slot and the elements,
            # until the extended header exists (spec/02-isa.md section 4.3).
            if not 0 <= n_elems <= ALLOCV_MAX_LENGTH:
                raise LM1Trap(TRAP_BAD_LENGTH, f"ALLOCV: length {n_elems}")
            # Vector layout: header + length_word + elements
            n_words = 1 + n_elems  # length word + elements
            total_bytes = (1 + n_words) * 8  # header + payload
            self._alloc_object(t, rd, total_bytes, tmpl_idx, n_words,
                               init_fn=lambda addr: self.mem.store_word(addr + 8, length))

        # ---- ALLOC.CLOSURE Rd, Rs_code, #env_size ----
        elif op == Op.ALLOC_CLOSURE:
            rd = (inst.raw26 >> 21) & 0x1F
            rs_code = (inst.raw26 >> 16) & 0x1F
            env_size = (inst.raw26 >> 11) & 0x1F
            code_ptr = t.regs[rs_code]
            n_words = 1 + env_size  # code_entry + env slots
            total_bytes = (1 + n_words) * 8
            self._alloc_object(t, rd, total_bytes, 1, n_words,  # template 1 = closure
                               init_fn=lambda addr: self.mem.store_word(addr + 8, code_ptr))

        # ================================================================
        # Phase 2: Tagged Field Access
        # ================================================================

        # ---- LD Rd, Rs, #field ----
        elif op == Op.LD:
            obj_ref = t.regs[inst.rs1]
            if not is_any_ref(obj_ref):
                raise LM1Trap(TRAP_NOT_REF, "LD: source is not a ref")
            field_idx = inst.imm16 & 0x1F
            addr = ref_address(obj_ref)
            # Skip header: field 0 is at offset +8
            t.regs[inst.rd] = self.mem.load_word(addr + (field_idx + 1) * 8)

        # ---- LD.CAR / LD.CDR ----
        elif op == Op.LD_CAR_CDR:
            obj_ref = t.regs[inst.rs1]
            if not is_any_ref(obj_ref):
                raise LM1Trap(TRAP_NOT_REF, "LD.CAR/CDR: source is not a ref")
            addr = ref_address(obj_ref)
            selector = inst.imm16 & 1  # 0 = car, 1 = cdr
            # car at offset +8, cdr at offset +16
            t.regs[inst.rd] = self.mem.load_word(addr + (selector + 1) * 8)

        # ---- ST Rs, #field, Rt  (no barrier) ----
        elif op == Op.ST:
            obj_ref = t.regs[inst.rd]  # Format S: Rs in rd position
            val = t.regs[inst.rs1]     # Rt in rs1 position
            field_idx = inst.rs2       # field in rs2 position
            if not is_any_ref(obj_ref):
                raise LM1Trap(TRAP_NOT_REF, "ST: target is not a ref")
            addr = ref_address(obj_ref)
            self.mem.store_word(addr + (field_idx + 1) * 8, val)

        # ---- ST.WB Rs, #field, Rt  (with write barrier) ----
        elif op == Op.ST_WB:
            obj_ref = t.regs[inst.rd]
            val = t.regs[inst.rs1]
            field_idx = inst.rs2
            if not is_any_ref(obj_ref):
                raise LM1Trap(TRAP_NOT_REF, "ST.WB: target is not a ref")
            field_addr = ref_address(obj_ref) + (field_idx + 1) * 8
            self.mem.store_word(field_addr, val)
            # Write barrier: storing a nursery ref into an old-gen object
            # marks the card holding the field.
            self._write_barrier(field_addr, val)

        # ---- ST.CAR / ST.CDR (barriered, spec/02-isa.md § 5.5) ----
        elif op == Op.ST_CAR_CDR:
            obj_ref = t.regs[inst.rd]
            val = t.regs[inst.rs1]
            selector = inst.rs2  # field: 0 = car, 1 = cdr
            if not is_any_ref(obj_ref):
                raise LM1Trap(TRAP_NOT_REF, "ST.CAR/CDR: target is not a ref")
            field_addr = ref_address(obj_ref) + (selector + 1) * 8
            self.mem.store_word(field_addr, val)
            # The same barrier as ST.WB on field 0 or 1
            self._write_barrier(field_addr, val)

        # ---- TST.SHAPE Rd, Rs, #shape_id ----
        elif op == Op.TST_SHAPE:
            val = t.regs[inst.rs1]
            shape_test = inst.imm16 & 0xFFFF
            if is_any_ref(val):
                addr = ref_address(val)
                hdr = self.mem.load_word(addr)
                if is_header(hdr) and (header_shape_id(hdr) & 0xFFFF) == shape_test:
                    t.regs[inst.rd] = T
                else:
                    t.regs[inst.rd] = NIL
            else:
                t.regs[inst.rd] = NIL

        # ================================================================
        # Phase 4: Dispatch (Inline Cache)
        # ================================================================

        # ---- CALL.IC: inline-cache dispatch ----
        elif op == Op.CALL_IC:
            # Format I: Rd = receiver register, Rs1 = selector (shape comes
            # from the object header).  imm16 is unused / reserved.
            receiver = t.regs[inst.rd]
            if not is_any_ref(receiver):
                raise LM1Trap(TRAP_NOT_REF, "CALL.IC: receiver is not a ref")
            addr = ref_address(receiver)
            hdr = self.mem.load_word(addr)
            shape = header_shape_id(hdr) if is_header(hdr) else 0
            callsite = t.pc  # use current PC as callsite key
            key = (callsite, shape)
            target = self.ic_table.get(key)
            if target is not None:
                # IC hit — push frame and jump
                self._push_frame(t, next_pc)
                next_pc = target
            else:
                # IC miss — trap.  Handler should IC.INSTALL then ERET.
                raise LM1Trap(TRAP_IC_MISS,
                              f"IC miss: callsite={callsite:#x} shape={shape}")

        # ---- IC.INSTALL: populate IC entry ----
        elif op == Op.IC_INSTALL:
            # Format R: Rs1 = receiver (for shape), Rs2 = code_entry register,
            # Rd = callsite PC register.
            receiver = t.regs[inst.rs1]
            code_entry = t.regs[inst.rs2]
            callsite = t.regs[inst.rd]
            if is_any_ref(receiver):
                addr = ref_address(receiver)
                hdr = self.mem.load_word(addr)
                shape = header_shape_id(hdr) if is_header(hdr) else 0
            else:
                shape = 0
            self.ic_table[(callsite, shape)] = code_entry

        # ---- CALL.DIRECT: direct call (no IC) ----
        elif op == Op.CALL_DIRECT:
            # Format I: Rd = unused, Rs1 = unused, imm16 = relative offset (words)
            # Or: I-format with target address.
            # Convention: imm16 is a signed word offset from current PC.
            self._push_frame(t, next_pc)
            offset = inst.imm16  # signed word offset
            next_pc = t.pc + (offset * 4)

        # ---- CALL.CLOSURE: call through closure object ----
        elif op == Op.CALL_CLOSURE:
            # Format I: Rd = closure register.
            # Closure layout: [header][code_entry][env0][env1]...
            closure_ref = t.regs[inst.rd]
            if not is_any_ref(closure_ref):
                raise LM1Trap(TRAP_NOT_CLOSURE, "CALL.CLOSURE: not a ref")
            caddr = ref_address(closure_ref)
            hdr = self.mem.load_word(caddr)
            if not is_header(hdr) or header_subtype(hdr) != HDR_CLOSURE:
                raise LM1Trap(TRAP_NOT_CLOSURE, "CALL.CLOSURE: not a closure")
            code_entry = self.mem.load_word(caddr + 8)  # field 0 = code entry
            self._push_frame(t, next_pc)
            next_pc = code_entry

        # ---- CALL.REG: call the address in a register ----
        elif op == Op.CALL_REG:
            # Format I: Rd (bits 25:21) holds the target, read before the
            # frame changes LR, FP and SP (spec/02-isa.md section 6.8).
            target = t.regs[inst.rd]
            self._push_frame(t, next_pc)
            next_pc = target

        # ---- RET: return from call ----
        elif op == Op.RET:
            next_pc = self._pop_frame(t)

        # ---- JR: jump register (absolute jump to address in Rs1) ----
        elif op == Op.JR:
            next_pc = t.regs[inst.rd]  # register is in rd position (bits 25:21)

        # ---- TAILCALL.IC: tail-call with inline cache ----
        elif op == Op.TAILCALL_IC:
            receiver = t.regs[inst.rd]
            if not is_any_ref(receiver):
                raise LM1Trap(TRAP_NOT_REF, "TAILCALL.IC: receiver is not a ref")
            addr = ref_address(receiver)
            hdr = self.mem.load_word(addr)
            shape = header_shape_id(hdr) if is_header(hdr) else 0
            callsite = t.pc
            key = (callsite, shape)
            target = self.ic_table.get(key)
            if target is not None:
                # Tail call: DON'T push a new frame, reuse current
                next_pc = target
            else:
                raise LM1Trap(TRAP_IC_MISS,
                              f"IC miss (tail): callsite={callsite:#x} shape={shape}")

        # ---- TAILCALL.DIRECT: direct tail-call ----
        elif op == Op.TAILCALL_DIR:
            offset = inst.imm16
            next_pc = t.pc + (offset * 4)
            # No frame push — reuse current frame

        # ================================================================
        # Phase 5: Messaging & Atomics
        # ================================================================

        # ---- SEND: enqueue value to a hardware queue ----
        elif op == Op.SEND:
            # Format S: rd (25:21) = queue descriptor register,
            #           rs1 (20:16) = value register
            desc = self._descriptor(t.regs[inst.rd])
            value = t.regs[inst.rs1]
            q = self._queue_of(desc)
            if len(q) >= self.QUEUE_DEPTH:
                raise LM1Trap(TRAP_QUEUE_FULL,
                              f"SEND: tile {desc >> 2} queue {desc & 3} full")
            q.append(value)

        # ---- RECV / TRY.RECV: dequeue from a hardware queue ----
        elif op == Op.RECV:
            # When func != 0 → TRY.RECV (non-blocking)
            # When func == 0 → RECV, which waits for a word
            desc = self._descriptor(t.regs[inst.rs1])
            q = self._queue_of(desc)

            if inst.func != 0:
                # TRY.RECV: non-blocking.  Rd = value (or nil), Rd2 = status
                rd2 = inst.func  # Rd2 register index is encoded in func field
                if len(q) > 0:
                    t.regs[inst.rd] = q.popleft()
                    t.regs[rd2] = T
                else:
                    t.regs[inst.rd] = NIL
                    t.regs[rd2] = NIL
            else:
                # RECV: on an empty queue the thread stalls until the queue
                # holds a word, and then runs the RECV again.
                if len(q) > 0:
                    t.regs[inst.rd] = q.popleft()
                else:
                    t.stalled = True
                    t.stall_queue = desc   # the descriptor's value
                    next_pc = t.pc

        # ---- CAS.TAGGED: atomic compare-and-swap ----
        elif op == Op.CAS_TAGGED:
            # Format X: Rd (25:21), Rs_addr (20:16), Rs_expected (15:11),
            #           Rt_new (10:6) — register index
            addr_reg = inst.rs1
            expected_reg = inst.rs2
            new_reg = inst.func  # bits 10:6 = register index for new value
            addr = ref_address(t.regs[addr_reg]) if is_any_ref(t.regs[addr_reg]) else t.regs[addr_reg]
            old = self.mem.load_word(addr)
            expected = t.regs[expected_reg]
            if old == expected:
                new_val = t.regs[new_reg]
                self.mem.store_word(addr, new_val)
                # Write barrier if storing a nursery ref into old-gen
                if is_any_ref(new_val):
                    self._write_barrier(addr, new_val)
                t.regs[inst.rd] = T
            else:
                t.regs[inst.rd] = NIL

        # ---- FAA / FENCE.GC ----
        elif op == Op.FAA_FENCE:
            if inst.func == 0x1F:
                # FENCE.GC: memory fence for GC coordination
                # In sequential emulator, this is a no-op.
                pass
            else:
                # FAA: fetch-and-add on fixnum memory
                addr = ref_address(t.regs[inst.rs1]) if is_any_ref(t.regs[inst.rs1]) else t.regs[inst.rs1]
                old = self.mem.load_word(addr)
                delta = t.regs[inst.rs2]
                if not is_fixnum(old):
                    raise LM1Trap(TRAP_NOT_FIXNUM,
                                  f"FAA: mem[{addr:#x}] = {old:#x} is not a fixnum")
                if not is_fixnum(delta):
                    raise LM1Trap(TRAP_NOT_FIXNUM,
                                  "FAA: delta is not a fixnum")
                # Fixnum add: since tag bit 0 is 0 for both, a + b preserves
                # the fixnum tag.  We mask to 64 bits.
                new_val = (old + delta) & WORD_MASK
                self.mem.store_word(addr, new_val)
                t.regs[inst.rd] = old

        # ---- Not yet implemented ----
        else:
            raise LM1Trap(TRAP_UNIMPLEMENTED, f"Unimplemented opcode {op} ({op:#04x})")

        return next_pc

    # -- Queue descriptors (Phase 5) ---

    @staticmethod
    def _descriptor(word: int) -> int:
        """The value of a queue descriptor, tile * 4 + queue (spec/02-isa.md
        section 8): a descriptor that is not a fixnum traps TRAP_NOT_FIXNUM."""
        if not is_fixnum(word):
            raise LM1Trap(TRAP_NOT_FIXNUM, f"queue descriptor {word:#x} is not a fixnum")
        return untag_fixnum(word)

    def _queue_of(self, desc: int) -> deque:
        """The queue a descriptor's value names: one of this tile's own, or
        of another tile of its cluster.  A tile no route reaches, one the
        cluster lacks, any other tile when there is no cluster, or a negative
        value, traps TRAP_NO_ROUTE."""
        tile = desc >> 2
        if tile == self.tile_id:
            return self.queues[desc & 3]
        if self.cluster is not None and 0 <= tile < len(self.cluster.tiles):
            return self.cluster.tiles[tile].queues[desc & 3]
        raise LM1Trap(TRAP_NO_ROUTE, f"no route reaches tile {tile}")

    def _stall_queue(self, t: ThreadContext) -> deque:
        """The queue a stalled thread's RECV waits on, by the descriptor
        value it recorded, which named a reachable tile."""
        return self._queue_of(t.stall_queue)

    # -- Frame helpers (Phase 4) ---

    def _push_frame(self, t: ThreadContext, return_addr: int) -> None:
        """Push a call frame: save LR and FP on the stack, set FP = SP."""
        # Push LR (return address)
        t.sp = (t.sp - 8) & WORD_MASK
        self.mem.store_word(t.sp, t.lr)
        # Push FP (saved frame pointer)
        t.sp = (t.sp - 8) & WORD_MASK
        self.mem.store_word(t.sp, t.fp)
        # Set up new frame
        t.lr = return_addr
        t.fp = t.sp

    def _pop_frame(self, t: ThreadContext) -> int:
        """Pop a call frame: restore FP and LR from stack, return the saved LR."""
        return_addr = t.lr
        # Restore FP and LR from stack
        t.sp = t.fp
        t.fp = self.mem.load_word(t.sp)
        t.sp = (t.sp + 8) & WORD_MASK
        t.lr = self.mem.load_word(t.sp)
        t.sp = (t.sp + 8) & WORD_MASK
        return return_addr

    # -- Allocation helpers (Phase 2) ---

    def _write_barrier(self, store_addr: int, stored_val: int) -> None:
        """Card-table write barrier of ST.WB, ST.CAR, ST.CDR and CAS.TAGGED.

        Fires when:
          1. stored_val is a ref (any tag with bit0=1 except specials & headers)
          2. store_addr >= gen_boundary  (the word, and so its container,
             is in old-gen)
          3. ref_address(stored_val) < gen_boundary  (value is in young-gen)

        Action: writes 0xFF to the card-table byte of the word stored,
                card_table_base + (store_addr >> card_shift), using a
                byte-store (spec/02-isa.md § 5.4, spec/05-memory-gc.md
                § 4.2).  A field past its header's card is then found by
                the collector's scan of that card.
        """
        if not is_any_ref(stored_val):
            return
        val_addr = ref_address(stored_val)
        # Filter: container in old-gen, value in young-gen
        if store_addr < self.gen_boundary:
            return   # container is in young-gen — no barrier
        if val_addr >= self.gen_boundary:
            return   # value is also in old-gen — no cross-gen store
        # Mark the card as dirty (0xFF)
        card_idx = store_addr >> self.card_shift
        if self.card_table_base != 0:
            # Write 0xFF to physical card-table address in memory
            card_addr = self.card_table_base + card_idx
            self.mem.store_byte(card_addr, 0xFF)
        # Also update internal card table for GC use
        if card_idx < self.card_table_size:
            self.card_table[card_idx] = 0xFF

    def _bump_alloc(self, t: ThreadContext, total_bytes: int) -> int:
        """Bump-allocate `total_bytes` from the nursery.

        Returns the address of the new object (aligned to 8).
        Raises TRAP_NURSERY_OVERFLOW if the nursery is full.
        """
        addr = t.np
        new_np = addr + total_bytes
        if new_np > t.nl:
            trap = LM1Trap(TRAP_NURSERY_OVERFLOW,
                           f"Nursery overflow: need {total_bytes} bytes, "
                           f"have {t.nl - t.np}")
            trap.alloc_bytes = total_bytes
            raise trap
        t.np = new_np
        return addr

    def _alloc_object(self, t: ThreadContext, rd: int, total_bytes: int,
                      tmpl_idx: int, n_words: int,
                      *, init_fn=None) -> None:
        """Generic allocation: bump NP, write header, zero fields, make ref."""
        addr = self._bump_alloc(t, total_bytes)

        # Build header from template, patching in the actual size
        tmpl = t.header_templates[tmpl_idx & 0xFFFF] if tmpl_idx < len(t.header_templates) else 0
        # Patch size into the header (bits 23:8)
        hdr = (tmpl & ~(0xFFFF << 8)) | ((n_words & 0xFFFF) << 8)
        self.mem.store_word(addr, hdr)

        # Zero payload words
        for i in range(1, n_words + 1):
            self.mem.store_word(addr + i * 8, 0)

        # Optional init (e.g., write code pointer for closure, length for vector)
        if init_fn is not None:
            init_fn(addr)

        # Write a ref to the new object into Rd
        t.regs[rd] = make_ref(addr)

    def _alloc_cons(self, t: ThreadContext, rd: int,
                    car: int = NIL, cdr: int = NIL) -> None:
        """Allocate a cons cell (header + car + cdr = 24 bytes)."""
        addr = self._bump_alloc(t, 24)

        # Cons header: template index 0, with the cons's two payload words
        # patched into its size as every allocation patches its own, so the
        # collector copies the car and the cdr whatever the template holds.
        hdr = (t.header_templates[0] & ~(0xFFFF << 8)) | (2 << 8)
        self.mem.store_word(addr, hdr)
        # car and cdr from register operands (default nil for backward compat)
        self.mem.store_word(addr + 8, car)
        self.mem.store_word(addr + 16, cdr)

        # Cons ref (tag = 011)
        t.regs[rd] = make_ref(addr, cons=True)

    # -- Trap handling (Phase 1: emulator-level I/O traps) ---

    # -- GC: Cheney copy collector (Phase 3) ---

    def _is_nursery_addr(self, addr: int) -> bool:
        """Check if an address falls within the nursery."""
        return self.nursery_base <= addr < self.nursery_base + self.nursery_size

    def _forward_ref(self, w: int) -> int:
        """If w is a ref pointing into the nursery, copy the object to old-gen
        (or return the already-forwarded address).  Returns the updated word.
        Non-ref or non-nursery words are returned unchanged."""
        if not is_any_ref(w):
            return w
        addr = ref_address(w)
        if not self._is_nursery_addr(addr):
            return w  # already in old-gen or elsewhere
        tag = w & 7  # preserve original tag (cons vs ref)

        # Check if already forwarded: a forwarded object has its header
        # replaced with a forwarding pointer (a ref with TAG_REF).
        hdr = self.mem.load_word(addr)
        if is_ref(hdr) or is_cons_ref(hdr):
            # Already forwarded — hdr IS the forwarding pointer
            new_addr = ref_address(hdr)
            return new_addr | tag

        # Not yet forwarded — copy the object to old-gen
        if not is_header(hdr):
            # Shouldn't happen — a non-header, non-forwarded word at an obj base
            return w

        n_payload_words = header_size(hdr)
        total_words = 1 + n_payload_words  # header + payload
        total_bytes = total_words * 8

        # Allocate in old-gen
        new_addr = self.oldgen_ptr
        if new_addr + total_bytes > self.oldgen_base + self.oldgen_size:
            raise RuntimeError(
                f"Old-gen overflow during GC: need {total_bytes} bytes, "
                f"have {self.oldgen_base + self.oldgen_size - new_addr}"
            )
        self.oldgen_ptr += total_bytes

        # Copy all words (header + payload)
        for i in range(total_words):
            self.mem.store_word(new_addr + i * 8,
                                self.mem.load_word(addr + i * 8))

        # Install forwarding pointer in the nursery copy
        self.mem.store_word(addr, make_ref(new_addr))

        return new_addr | tag

    def _take_trap(self, trap: LM1Trap) -> int:
        """Answer a trap that the current thread's instruction raised, and
        return the PC the thread goes on at, or let the trap stand.

        A trap below 0x80 vectors through the thread's trap table to the
        runtime's handler (spec/03-core.md section 6.1).  ERET resumes a
        fault at the instruction that faulted, and an explicit TRAP at the
        instruction after it.  TRAP_NURSERY_OVERFLOW with no handler in the
        table is answered here as the runtime answers it, by a minor
        collection after which the allocation is retried (spec/02-isa.md
        section 4.1): the emulator stands in for a runtime that is absent,
        and steps aside for one that installed its handler.  A trap with no
        handler stands, and the run ends with it.
        """
        t = self.thread
        if trap.code == TRAP_NURSERY_OVERFLOW and not self._handler_installed(trap.code):
            self._collect_for_overflow(trap)
            return t.pc
        if t.trap_table_base != 0 and trap.code < 0x80:
            t.trap_pc = t.pc if trap.resume_pc is None else trap.resume_pc
            t.trap_cause = trap.code
            t.in_trap = True
            # trap_table[code] is a 64-bit word holding the handler's PC
            handler_addr = self.mem.load_word(t.trap_table_base + trap.code * 8)
            if handler_addr == 0:
                raise trap
            return handler_addr
        raise trap

    def _handler_installed(self, code: int) -> bool:
        """Whether the current thread's trap table names a handler for CODE."""
        t = self.thread
        return (t.trap_table_base != 0 and code < 0x80
                and self.mem.load_word(t.trap_table_base + code * 8) != 0)

    def _give_fresh_chunks(self) -> None:
        """Every thread's chunk from the nursery's base on, one after
        another, and the rest of the nursery free (spec/04-soc.md section
        4.3)."""
        for tid, tc in enumerate(self.threads):
            base = self.nursery_base + tid * self.nursery_chunk
            self.nursery_slices[tid] = base
            tc.np = base
            tc.nl = base + self.nursery_chunk
        self.nursery_free = self.nursery_base + len(self.threads) * self.nursery_chunk

    def _collect_for_overflow(self, trap: LM1Trap) -> None:
        """Answer TRAP_NURSERY_OVERFLOW as the runtime does (spec/02-isa.md
        § 4.1, spec/04-soc.md § 4.3), after which the caller retries the
        faulting allocation: a fresh chunk for the thread from the nursery's
        free space, the allocation's size at least, while any is left, and a
        minor collection only when none is.

        A collection empties the nursery and can do no more.  When the
        thread's chunk is already empty and no thread has taken free space
        since the last collection, the nursery is as empty as a collection
        leaves it, the allocation is larger than it can hold, and the trap
        stands.
        """
        idx = self._current_thread_idx
        t = self.thread
        need = (getattr(trap, "alloc_bytes", 0) + 7) & ~7
        chunk = max(self.nursery_chunk, need)
        if self.nursery_free + chunk <= self.nursery_base + self.nursery_size:
            t.np = self.nursery_free
            t.nl = self.nursery_free + chunk
            self.nursery_slices[idx] = self.nursery_free
            self.nursery_free += chunk
            return
        fresh = self.nursery_base + len(self.threads) * self.nursery_chunk
        if t.np == self.nursery_slices[idx] and self.nursery_free == fresh:
            raise trap
        self._gc_collect()

    def _gc_collect(self) -> None:
        """Cheney copy collector: evacuate live nursery objects to old-gen.

        Roots: the 32 registers and the stack of each of the tile's threads
        (spec/05-memory-gc.md section 5.1), thread by thread.
        Algorithm:
          1. Scan roots, forward any nursery refs → copies to old-gen
          2. Scan the copied objects in old-gen (Cheney scan pointer),
             forwarding any nursery refs found in their fields
          3. Scan the words of every dirty card, which hold the old-gen
             refs back into the nursery that the barrier recorded, and
             clear the card
          4. Reset the nursery: every thread has a fresh chunk again
        """
        mem = self.mem
        scan_start = self.oldgen_ptr  # where new copies start

        # --- Phase A: forward roots ---

        for t in self.threads:
            # 1. Registers
            for i in range(32):
                t.regs[i] = self._forward_ref(t.regs[i])

            # 2. Stack.  It grows down toward its SP from just below the
            # nursery, the emulator's convention, so the words from SP up to
            # nursery_base (exclusive) are scanned.
            stack_scan_limit = self.nursery_base
            sp = t.sp
            while sp < stack_scan_limit:
                w = mem.load_word(sp)
                mem.store_word(sp, self._forward_ref(w))
                sp += 8

        # 3. Dirty cards.  A barriered store marks the card holding the word
        #    it stored (spec/05-memory-gc.md § 4.2), so the words of the
        #    dirty cards hold every old-to-young ref the barrier saw.  Each
        #    dirty card is scanned and then cleared, in the emulator's card
        #    table and in the one the runtime configured in memory.
        card_shift = self.card_shift
        card_size = 1 << card_shift
        cards = self.card_table
        card_idx = cards.find(0xFF)
        while card_idx != -1:
            a = card_idx << card_shift
            card_end = a + card_size
            while a < card_end:
                w = mem.load_word(a)
                mem.store_word(a, self._forward_ref(w))
                a += 8
            cards[card_idx] = 0
            if self.card_table_base != 0:
                mem.store_byte(self.card_table_base + card_idx, 0)
            card_idx = cards.find(0xFF, card_idx + 1)

        # --- Phase B: Cheney scan — process copied objects ---
        scan_ptr = scan_start
        while scan_ptr < self.oldgen_ptr:
            w = mem.load_word(scan_ptr)
            if is_header(w):
                # Skip the header itself, but forward all payload words
                n_words = header_size(w)
                for i in range(1, n_words + 1):
                    field_addr = scan_ptr + i * 8
                    fv = mem.load_word(field_addr)
                    mem.store_word(field_addr, self._forward_ref(fv))
                scan_ptr += (1 + n_words) * 8
            else:
                # Not a header — shouldn't happen in well-formed heap
                scan_ptr += 8

        # --- Phase C: reset nursery: every thread a fresh chunk ---
        self._give_fresh_chunks()

        self.gc_count += 1

    def _system_trap(self, code: int) -> None:
        """Carry out TRAP #code for a code from 0x80 up, a system trap the
        core carries out itself and which never reaches the trap table
        (spec/02-isa.md section 11.6): the configuration traps 0x90 to 0x95,
        and the emulator's bridge to its host, its console and block device,
        on 0x80 to 0x82, 0x84 and 0x9F (design/emulator.md section 7).  Any
        other code is reserved, and the TRAP traps TRAP_UNIMPLEMENTED, a
        fault that saves the TRAP's own PC.
        """
        t = self.thread

        if code == EMU_TRAP_PUTCHAR:
            # r1 holds the character (as a tagged fixnum or char immediate)
            val = t.regs[1]
            if is_fixnum(val):
                ch = untag_fixnum(val) & 0xFF
            elif (val & 0xFF) == 0x35:
                # Character immediate
                ch = (val >> 8) & 0x1F_FFFF
            else:
                ch = val & 0xFF
            self._stdout.write(chr(ch))
            self._stdout.flush()

        elif code == EMU_TRAP_GETCHAR:
            # Read one character, put in r0 as tagged fixnum
            ch = self._stdin.read(1)
            if ch:
                t.regs[0] = tag_fixnum(ord(ch))
            else:
                t.regs[0] = tag_fixnum(-1)  # EOF

        elif code == EMU_TRAP_BLOCK_IO:
            # Block I/O: r1 = sub-function (0=read, 1=write)
            #            r2 = block number
            #            r3 = memory destination/source address
            func = t.regs[1]
            block_num = t.regs[2]
            mem_addr = t.regs[3]
            if self.block_device_path is None:
                # No block device attached — return error in r0
                t.regs[0] = tag_fixnum(-1)
                return
            import os
            try:
                if func == BLOCK_IO_READ:
                    with open(self.block_device_path, 'rb') as f:
                        f.seek(block_num * BLOCK_SIZE)
                        data = f.read(BLOCK_SIZE)
                    # Pad to BLOCK_SIZE if file is shorter
                    if len(data) < BLOCK_SIZE:
                        data = data + b'\x00' * (BLOCK_SIZE - len(data))
                    # Write into emulator memory byte by byte
                    for i, b in enumerate(data):
                        self.mem.store_byte((mem_addr + i) & WORD_MASK, b)
                    t.regs[0] = tag_fixnum(0)  # success
                elif func == BLOCK_IO_WRITE:
                    # Read BLOCK_SIZE bytes from emulator memory
                    data = bytearray(BLOCK_SIZE)
                    for i in range(BLOCK_SIZE):
                        data[i] = self.mem.load_byte((mem_addr + i) & WORD_MASK)
                    # Ensure file exists and write
                    mode = 'r+b' if os.path.exists(self.block_device_path) else 'wb'
                    with open(self.block_device_path, mode) as f:
                        f.seek(block_num * BLOCK_SIZE)
                        f.write(data)
                    t.regs[0] = tag_fixnum(0)  # success
                else:
                    t.regs[0] = tag_fixnum(-2)  # unknown sub-function
            except (IOError, OSError):
                t.regs[0] = tag_fixnum(-1)  # I/O error

        elif code == EMU_TRAP_SET_TRAP_TABLE:
            # Set the trap table base address for the current thread
            t.trap_table_base = t.regs[1]

        elif code == EMU_TRAP_SET_TEMPLATE:
            # Install a header template: r1 = index, r2 = 64-bit header value
            idx = t.regs[1] & 0xFF
            val = t.regs[2]
            t.header_templates[idx] = val

        elif code == EMU_TRAP_SET_CARD_BASE:
            # Set card table base address (matches RTL SYS_SET_CARD_BASE)
            self.card_table_base = t.regs[1]

        elif code == EMU_TRAP_SET_CARD_SHIFT:
            # Set card shift (matches RTL SYS_SET_CARD_SHIFT)
            self.card_shift = t.regs[1] & 0x3F
            # Rebuild internal card table with new card size
            card_size = 1 << self.card_shift
            self.card_table_size = self.mem._size // card_size
            self.card_table = bytearray(self.card_table_size)

        elif code == EMU_TRAP_SET_GEN_BOUNDARY:
            # Set generation boundary (matches RTL SYS_SET_GEN_BOUNDARY)
            self.gen_boundary = t.regs[1]

        elif code == EMU_TRAP_SET_QUEUE_BASE:
            # Record the queue base (the RTL's SYS_SET_QUEUE_BASE); the
            # modeled queues do not live in memory, so nothing else uses it.
            self.queue_base = t.regs[1]

        elif code == EMU_TRAP_MEM_BYTE:
            # Byte-level memory access:
            #   r1 = sub-function: 0=load_byte, 1=store_byte
            #   r2 = base address (raw)
            #   r3 = byte offset (tagged fixnum)
            #   r4 = value to store (tagged fixnum, for store only)
            sub = untag_fixnum(t.regs[1]) if is_fixnum(t.regs[1]) else int(t.regs[1])
            addr = t.regs[2]
            offset = untag_fixnum(t.regs[3]) if is_fixnum(t.regs[3]) else int(t.regs[3])
            if sub == 0:
                # Load byte → return as tagged fixnum in r1
                byte_val = self.mem.load_byte((addr + offset) & WORD_MASK)
                t.regs[1] = tag_fixnum(byte_val)
            elif sub == 1:
                # Store byte
                val = untag_fixnum(t.regs[4]) if is_fixnum(t.regs[4]) else int(t.regs[4])
                self.mem.store_byte((addr + offset) & WORD_MASK, val & 0xFF)
                t.regs[1] = tag_fixnum(0)  # success

        elif code == EMU_TRAP_DEBUG_PRINT:
            # Debug print: r1 = address of byte string, r2 = length
            addr = t.regs[1]
            length = t.regs[2]
            chars = []
            for i in range(length):
                chars.append(chr(self.mem.load_byte((addr + i) & WORD_MASK)))
            self._stdout.write(''.join(chars))
            self._stdout.flush()

        else:
            raise LM1Trap(TRAP_UNIMPLEMENTED, f"no system trap {code:#04x}")

    # -- Trace output ---

    def _trace_instruction(self, pc: int, inst: Instruction) -> None:
        """Print a trace line for the current instruction."""
        try:
            name = Op(inst.opcode).name
        except ValueError:
            name = f"???({inst.opcode:#04x})"
        self._stdout.write(
            f"[{pc:#010x}] {inst.raw:#010x}  {name:16s} "
            f"rd={inst.rd} rs1={inst.rs1} rs2={inst.rs2} "
            f"func={inst.func} imm16={inst.imm16}\n"
        )

    # -- Multi-thread round-robin scheduling (Phase 5) ---

    def run_round_robin(self, max_instructions: int = 0,
                        quanta: int = 1) -> None:
        """Run multiple threads in round-robin order.

        Each non-halted, non-stalled thread gets *quanta* instructions per
        round.  A stalled thread (blocked on RECV) is un-stalled when its
        queue has data.
        """
        mem = self.mem
        count = 0
        n_threads = len(self.threads)

        while True:
            any_runnable = False

            for tidx in range(n_threads):
                self._current_thread_idx = tidx
                t = self.threads[tidx]

                if t.halted:
                    continue

                # Check if stalled thread can be un-stalled
                if t.stalled:
                    q = self._stall_queue(t)
                    if len(q) > 0:
                        t.stalled = False
                        t.stall_queue = -1
                    else:
                        continue  # still stalled

                any_runnable = True

                # Execute up to *quanta* instructions on this thread
                for _ in range(quanta):
                    if t.halted or t.stalled:
                        break

                    raw = mem.load_u32(t.pc)
                    inst = decode(raw)

                    if self.trace:
                        self._trace_instruction(t.pc, inst)

                    next_pc = t.pc + 4

                    try:
                        next_pc = self._execute(inst, next_pc)
                    except LM1Trap as trap:
                        next_pc = self._take_trap(trap)

                    t.pc = next_pc & WORD_MASK
                    t.cycle_count += 1
                    self.instruction_count += 1
                    count += 1

                    if max_instructions and count >= max_instructions:
                        return

            if not any_runnable:
                break  # all threads halted or deadlocked


# ===========================================================================
# Cluster: multi-tile emulator (Phase 5)
# ===========================================================================

class Cluster:
    """Manages multiple Emulator tiles with cross-tile message routing.

    Queue descriptors are fixnum-encoded: (tile_id << 2) | queue_idx.
    """

    def __init__(
        self,
        n_tiles: int = 2,
        *,
        mem_size: int = 4 * 1024 * 1024,
        trace: bool = False,
        nursery_base: int | None = None,
        nursery_size: int | None = None,
        oldgen_base: int | None = None,
        oldgen_size: int | None = None,
        num_threads_per_tile: int = 1,
    ):
        self.tiles: list[Emulator] = []
        for i in range(n_tiles):
            emu = Emulator(
                mem_size=mem_size,
                trace=trace,
                nursery_base=nursery_base,
                nursery_size=nursery_size,
                oldgen_base=oldgen_base,
                oldgen_size=oldgen_size,
                tile_id=i,
                num_threads=num_threads_per_tile,
            )
            emu.cluster = self
            self.tiles.append(emu)

    def run(self, max_instructions: int = 0, quanta: int = 1) -> None:
        """Round-robin across all tiles.

        Each tile executes up to *quanta* instructions per round on its
        active thread(s).  The outer loop terminates when all tiles' threads
        are halted or the instruction budget is exhausted.
        """
        count = 0

        while True:
            any_runnable = False

            for tile in self.tiles:
                n_threads = len(tile.threads)
                for tidx in range(n_threads):
                    tile._current_thread_idx = tidx
                    t = tile.threads[tidx]

                    if t.halted:
                        continue

                    # Try to un-stall
                    if t.stalled:
                        q = tile._stall_queue(t)
                        if len(q) > 0:
                            t.stalled = False
                            t.stall_queue = -1
                        else:
                            continue

                    any_runnable = True
                    mem = tile.mem

                    for _ in range(quanta):
                        if t.halted or t.stalled:
                            break

                        raw = mem.load_u32(t.pc)
                        inst = decode(raw)

                        if tile.trace:
                            tile._trace_instruction(t.pc, inst)

                        next_pc = t.pc + 4

                        try:
                            next_pc = tile._execute(inst, next_pc)
                        except LM1Trap as trap:
                            next_pc = tile._take_trap(trap)

                        t.pc = next_pc & WORD_MASK
                        t.cycle_count += 1
                        tile.instruction_count += 1
                        count += 1

                        if max_instructions and count >= max_instructions:
                            return

            if not any_runnable:
                break
