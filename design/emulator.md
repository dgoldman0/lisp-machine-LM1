# LM-1 Emulator Design

**Status:** Design sketch  
**Date:** 2026-02-16

---

## 1. Purpose

The emulator is the first thing that runs LM-1 code. It's not a cycle-accurate simulator — it's a **functional emulator** that executes the ISA correctly so we can:

1. Run the compiled Lisp runtime
2. Develop and test the compiler
3. Run Lisp programs and validate semantics
4. Debug the ISA design (find missing instructions, broken invariants)

Performance target: ~10–100 MIPS on a modern host. Fast enough for interactive REPL use, slow enough that we don't waste time optimizing the emulator itself.

## 2. Architecture

```
┌──────────────────────────────────────────────────────────┐
│                     Host Process                          │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │                  Emulator Core                      │  │
│  │                                                    │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │  │
│  │  │ Decoder   │  │ Executor │  │ Tag Logic        │  │  │
│  │  │          │  │          │  │ (pure functions)  │  │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │  │
│  │                                                    │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │             Thread Contexts (N)              │  │  │
│  │  │  [regs, pc, sp, fp, np, nl, state, ...]      │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │                  Memory Subsystem                   │  │
│  │                                                    │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │  │
│  │  │ Tile SRAM│  │ Cluster  │  │ HBM (mmap'd      │  │  │
│  │  │ (arrays) │  │ SRAM     │  │  host memory)    │  │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │  │
│  │                                                    │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │  Card Tables  │  Queues  │  DMA Emulation    │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │                  Support Services                   │  │
│  │                                                    │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │  │
│  │  │ Trap     │  │ IC Table │  │ Movement Engine  │  │  │
│  │  │ Dispatch │  │ Emulation│  │ Emulation        │  │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │  │
│  │                                                    │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │  │
│  │  │ Debugger │  │ Profiler │  │ I/O (terminal,   │  │  │
│  │  │ Interface│  │ Counters │  │  files, network) │  │  │
│  │  └──────────┘  └──────────┘  └──────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

## 3. What We Emulate

### 3.1 Must Emulate (Functional Correctness)

| Component | Emulation Strategy |
|-----------|-------------------|
| **Tagged words** | Native 64-bit host integers. Tag operations are just bitmasks. |
| **Registers** | Array of 32 uint64 per thread context. |
| **Instruction decode + execute** | Giant switch on opcode. Each case implements the pseudocode from the ISA spec. |
| **Nursery allocator** | Per-tile: a base pointer, bump pointer (np), and limit (nl). The tile's threads share its nursery in slices, each bumping its own np up to its nl, and share its header-template table. `ALLOC` bumps; overflow calls the GC slow path, a minor collection that scans every thread's registers and stack and empties every slice. |
| **Write barriers** | `ST.WB`, `ST.CAR`, `ST.CDR` and `CAS.TAGGED` check tags and generations and mark the card holding the stored word in a card-table byte array, the barrier the spec gives the hardware. |
| **IC table** | A hash map per tile: key = (callsite, shape), value = code_entry. `CALL.IC` probes it; miss triggers the trap handler. |
| **Traps** | On trap: save PC, switch to handler address from trap table, set trap-code register. `ERET` restores: a fault retries its instruction, an explicit `TRAP` resumes after itself. With no handler installed for `TRAP_NURSERY_OVERFLOW`, the emulator's own minor collection answers it, standing in for the runtime. |
| **Message queues** | Per-tile array of ring buffers. `SEND` enqueues, `RECV` dequeues. Cross-tile: route through a simulated NoC (just a function call in the emulator). |
| **Movement engines** | `ENQ.SCAN/COPY/FIXUP` run their logic synchronously (in the emulator, they just execute immediately, not concurrently). |
| **Memory map** | One big host-allocated array. Tile SRAMs, cluster SRAMs, and HBM are just offset ranges within it. |

### 3.2 Simplified (Don't Need Full Fidelity)

| Component | Simplification |
|-----------|---------------|
| **Pipeline stages** | No pipeline simulation. Instructions execute atomically. |
| **FGMT** | Emulate as cooperative multitasking: run one thread for N instructions, then switch. |
| **NoC** | No latency modeling. Cross-tile messages are instant. |
| **DMA** | Instant memcpy. No latency, no bandwidth limits. |
| **Power/clock gating** | Ignored. |
| **Caches** | No I-cache or D-cache simulation. All memory accesses are instant. |
| **Capability mode** | Deferred. Implement later if/when we design the safe-mode runtime. |

### 3.3 Not Emulated

| Component | Why |
|-----------|-----|
| **SIMD sidekick tiles** | Removed from design per discussion. |
| **HBM bandwidth limits** | Not useful for functional correctness. |
| **Physical layout** | Not relevant. |

## 4. Configuration

The emulator is parameterized:

```
EmulatorConfig = {
    tile_count:         4       -- start small; 1 is fine for initial bringup
    threads_per_tile:   4
    tile_sram_bytes:    262144  -- 256 KiB
    cluster_size:       4       -- tiles per cluster (so 1 cluster for 4 tiles)
    cluster_sram_bytes: 2097152 -- 2 MiB
    hbm_bytes:          67108864 -- 64 MiB (plenty for emulation)
    nursery_bytes:      65536   -- 64 KiB per tile
    ic_entries:         64      -- per tile
    queue_depth:        512     -- entries per queue
    queues_per_tile:    4
}
```

Start with **1 tile, 1 thread** for initial bringup. Scale up once the Lisp runtime runs.

## 5. Emulator Loop

```python
def run(emu: Emulator) -> None:
    while True:
        tile = emu.schedule_tile()             # round-robin across tiles
        thread = tile.schedule_thread()         # round-robin across threads

        pc = thread.pc
        instruction = tile.fetch(pc)            # read 32 bits from memory at pc
        decoded = decode(instruction)           # extract opcode, fields

        match decoded.family:
            case F1_TAGGED_ARITH: exec_tagged_arith(tile, thread, decoded)
            case F2_ALLOC:        exec_alloc(tile, thread, decoded)
            case F3_FIELD_ACCESS: exec_field_access(tile, thread, decoded)
            case F4_DISPATCH:     exec_dispatch(tile, thread, decoded)
            case F5_PREFETCH:     pass  # no-op in emulator
            case F6_CONCURRENCY:  exec_concurrency(tile, thread, decoded)
            case F7_REGION_OPS:   exec_region_ops(tile, thread, decoded)
            case F8_CAPABILITY:   trap(thread, TRAP_UNIMPLEMENTED)
            case SCALAR:          exec_scalar(tile, thread, decoded)
            case SYSTEM:          exec_system(tile, thread, decoded)

        thread.cycle_count += 1

        # Check for pending interrupts, GC requests, etc.
        if thread.cycle_count % 1024 == 0:
            check_interrupts(tile, thread)
```

The same loop runs in C++ in the chiplet module (`chiplet/`, design/chiplet-module.md), a
separate model of LM-1 that follows this one step for step and calls back into Python only
for the console and block traps.

## 6. Debug Interface

The emulator exposes a debug interface (over a socket or stdio) that lets you:

| Command | Description |
|---------|-------------|
| `step [N]` | Execute N instructions on current thread |
| `continue` | Run until breakpoint or trap |
| `break <addr>` | Set breakpoint |
| `regs` | Dump all registers (pretty-printed as tagged values) |
| `inspect <ref>` | Dereference a ref and print the object (header, fields) |
| `mem <addr> <len>` | Dump raw memory |
| `trace on/off` | Log every instruction executed |
| `tiles` | Show tile status (running/idle, thread states) |
| `queues` | Show queue contents |
| `gc-status` | Show nursery fill levels, card table stats |
| `ic-dump` | Show IC table contents |
| `disas <addr> <len>` | Disassemble instructions |

This is critical during bring-up: until the runtime's REPL runs, the debugger is the only window into the machine.

## 7. Host I/O Bridge

The emulator provides I/O devices as memory-mapped regions or via special TRAP codes:

| Device | Mechanism | Purpose |
|--------|-----------|---------|
| **Console** | TRAP #0x80 with char in r1 | Print a character to the host terminal |
| **Console input** | TRAP #0x81, char returned in r0 | Read a character from the host terminal |
| **Block storage** | TRAP #0x82 with args | Read/write a 4K block from a host file |
| **Timer** | CYCLE instruction | Read host cycle counter (or wall clock) |
| **Shutdown** | HALT instruction | Exit emulator |

These traps are **not** part of the ISA spec — they're emulator-specific stand-ins for I/O, which the chiplet receives from MegaPad over the link. They use system trap codes, 0x80 to 0x82, 0x84 and 0x9F, which the spec leaves reserved (`spec/02-isa.md` § 11.6), so on the RTL, which has no such bridge, each traps `TRAP_UNIMPLEMENTED`.

## 8. Implementation Language

Python, with the chiplet module as its C++ counterpart.

**Core in Python:**
- Clean, readable implementation of the ISA semantics
- Rapid iteration during design — modify-and-run, no compile step
- Tagged words as Python ints masked to 64-bit where needed
- Memory subsystem backed by `array.array('Q', ...)` (unsigned 64-bit)
- Match/case (Python 3.10+) for decode dispatch
- Debug interface and CLI trivially built with Python's standard library

**The chiplet module in C++:**
- `chiplet/` builds one shared library, the model MegaPad attaches as LM-1's chiplet and
  LM-1's own tests drive through `ctypes` (`lm1/chiplet.py`, design/chiplet-module.md)
- Its first stage models what this emulator executes, and the tests in
  `tests/test_chiplet_core.py` run every program of this emulator's tests, the RTL bench's
  programs and random instruction streams on both and compare them state for state
- This emulator stays the reference until the module passes every test it passes; then
  the module becomes the reference and this emulator retires (decision F of that plan)

## 9. Bringup Sequence

1. **Phase 1:** Emulate 1 tile, 1 thread. No GC, no IC, no queues. Just enough to execute scalar ops, branches, loads/stores. Verify with hand-assembled test programs.
2. **Phase 2:** Add allocation (nursery bump pointer). Write `ALLOC` + `LD` + `ST` test programs. No GC yet — just let the nursery fill up and halt.
3. **Phase 3:** Add trap handling. Implement `TRAP_NURSERY_OVERFLOW` → simple stop-and-copy GC in the trap handler (written in LM-1 assembly or cheating with emulator-native code initially).
4. **Phase 4:** Add IC table. Test `CALL.IC` with hand-crafted dispatch. Verify hit/miss/install cycle.
5. **Phase 5:** Add message queues. Test `SEND`/`RECV` between threads, then between tiles.
6. **Phase 6:** Compile Lisp programs with the cross-compiler and run them.
7. **Phase 7:** MegaPad loads and starts the runtime over the link.

## 10. Testing Strategy

- **ISA conformance tests:** One test per instruction, verifying the pseudocode from the spec. These are tiny hand-assembled programs (or assembled from a trivial assembler).
- **GC stress tests:** Tight allocation loops that force nursery overflows. Verify no tagged pointers are lost.
- **IC tests:** Monomorphic, polymorphic, and megamorphic dispatch patterns.
- **Queue tests:** Producer-consumer across tiles.
- **Integration:** Compile and run Lisp programs end to end, such as `(+ 1 2)` and `(fact 10)`.
