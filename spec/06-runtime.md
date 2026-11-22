# LM-1 Runtime Strategy and Programming Model

**Spec ID:** LM1-SPEC-06  
**Revision:** 0.1-draft  
**Date:** 2026-02-16

---

## 1. Scope

This document specifies the runtime strategy that makes the LM-1 hardware "feel like a Lisp machine": how tasks are scheduled across tiles, how dispatch and method lookup work, how the distributed heap is managed, and what the programming model looks like from the language level.

## 2. Design Philosophy

The runtime sits between the Lisp language and the ISA. Its job is to:

1. **Map Lisp's sequential, recursive, allocating semantics onto many tiles** — without requiring the programmer to think about tiles.
2. **Keep the fast paths fast** — the runtime is invoked only on slow paths (IC misses, nursery overflows, etc.).
3. **Present a single-image illusion** — from the programmer's perspective, there is one heap, one namespace, one set of functions.

The runtime is itself written in a mixture of LM-1 assembly (for trap handlers and hot loops) and Lisp (for policy, scheduling, method lookup).

## 3. Task Model: Actors over Shared State

### 3.1 The Task

The fundamental unit of schedulable work is a **task**. A task is:

```
task = {
    closure: ref        ; the function to call (a closure ref)
    args: ref           ; arguments (a vector ref or cons list)
    continuation: ref   ; where to send the result
    priority: fixnum    ; scheduling priority
    affinity: fixnum    ; preferred tile/cluster (or -1 for any)
}
```

Tasks are lightweight (5 tagged words = 40 bytes). They are allocated in cluster work queues and dispatched to tiles via hardware message queues.

### 3.2 Task Lifecycle

```
┌──────────┐   enqueue    ┌──────────┐   dispatch   ┌──────────┐
│ CREATED  │ ──────────►  │ QUEUED   │ ──────────►  │ RUNNING  │
└──────────┘              └──────────┘              └────┬─────┘
                                                        │
                               ┌────────────────────────┤
                               │                        │
                          result/error             suspend (I/O,
                               │                    GC, wait)
                               ▼                        │
                         ┌──────────┐              ┌────▼─────┐
                         │COMPLETED │              │SUSPENDED │
                         └──────────┘              └──────────┘
```

### 3.3 Task Creation

From Lisp, tasks are created implicitly:

```lisp
;; Spawn a concurrent task
(spawn (lambda () (compute-something x y)))

;; Spawn with affinity
(spawn-on tile-id (lambda () (local-work)))

;; Future: spawn + return a promise
(let ((f (future (expensive-computation))))
  ;; ... do other work ...
  (force f))  ; block until result
```

Each `spawn` allocates a task descriptor, enqueues it in the cluster work queue, and returns immediately.

### 3.4 Actor Model Integration

For programs that want explicit actor-style concurrency:

```lisp
(defactor counter ((count 0))
  (:increment ()
    (setf count (+ count 1)))
  (:get-count ()
    count))

(let ((c (make-actor 'counter)))
  (send c :increment)
  (send c :increment)
  (ask c :get-count))  ; => 2
```

An actor is a task with:
- An associated mailbox (hardware message queue)
- A processing loop that dequeues messages and dispatches methods
- Private state (captured in the closure's environment)
- A guarantee of **sequential message processing** (no concurrent access to actor state)

Actors naturally map to hardware threads: one actor per hardware thread, with the thread's message queue as the mailbox.

### 3.5 Tasks of Leviathan's Language

**Status:** proposed on 2026-11-20.

A task of Leviathan's language, Uroko, means the same whichever engine runs it (release plan decisions 31 and 32), so the runtime runs one under four rules, which a machine with one engine keeps as well:

1. **Its inputs are lent.** MegaPad names each input object by a handle, and lends those handles with the request until its reply, so MegaPad neither re-points nor releases them meanwhile. The runtime reads their slots when the task starts.
2. **Its inputs are read-only.** The runtime gives the task each input as a capability without write permission ([05-memory-gc.md](05-memory-gc.md) § 12.8), so the task writes nothing it reaches through them.
3. **Its objects stay its own.** The objects the task makes are private to it while it runs. A store that would put one into a shared object traps ([05-memory-gc.md](05-memory-gc.md) § 4.5), and the runtime refuses it and ends the task with an error, so no object of the task's escapes but its result.
4. **Its result is copied out.** When the task ends, the runtime copies its result, with everything the result reaches, into the shared heap, and stores the result's reference in the handle slot MegaPad lent the task for it, before it replies. MegaPad's handle then keeps the result live, and the task's other objects become garbage.

On a machine with one engine, the task's objects live in a heap that lasts as long as the task, and the same four rules give the same result.

---

## 4. Scheduler

### 4.1 Hierarchical Scheduling

```
┌───────────────────────────────┐
│       Global Scheduler        │  Runs on a dedicated tile (or small set)
│  (load balancing, affinity)   │  Periodically rebalances across clusters
└──────────────┬────────────────┘
               │
    ┌──────────▼──────────┐
    │  Cluster Scheduler  │    One per cluster, runs on the cluster's
    │  (work stealing)    │    shared SRAM work queues
    └──────────┬──────────┘
               │
    ┌──────────▼──────────┐
    │   Tile Scheduler    │    Per-tile, selects which HW thread runs
    │  (thread dispatch)  │    which task, handles preemption
    └─────────────────────┘
```

### 4.2 Work Stealing

When a tile's work queue is empty:

1. **Local steal:** Try to dequeue from the cluster work queue.
2. **Remote steal:** If the cluster queue is empty, send a steal request to a neighboring cluster via the NoC.
3. **Idle:** If no work is available, the tile parks its threads and clock-gates.

Steal granularity: one task at a time (fine-grained) or a batch (coarse-grained, better for locality). The cluster scheduler decides based on queue depth.

### 4.3 Affinity and Locality

The scheduler respects task affinity hints:

- **Tile affinity:** A task that accesses data in a specific tile's SRAM should run on that tile.
- **Cluster affinity:** A task that accesses the cluster's old-gen region should run in that cluster.
- **No affinity:** The task can run anywhere; the scheduler assigns it to the least-loaded tile.

The runtime tracks which objects are in which tile/cluster SRAM via a **location directory** (a distributed hash table in cluster shared SRAMs).

### 4.4 Preemption

Tasks run until they:
1. Complete (return a result)
2. Block (I/O, waiting on a future, waiting on a message)
3. Are preempted (time quantum expired, GC needs to run)

Preemption is cooperative at most safepoints: the scheduler sets a per-thread flag, and the next safepoint (backward branch, call, allocation) checks it and yields.

For truly uncooperative tasks (infinite loops without safepoints), the scheduler can trigger a hardware interrupt to force a context switch.

---

## 5. Method Dispatch

### 5.1 Overview

LM-1 supports **generic function dispatch** (as in CLOS) with hardware-accelerated inline caches.

The dispatch chain:

```
CALL.IC ──► IC hit? ──yes──► direct jump to cached method
            │
            no (TRAP_IC_MISS)
            │
            ▼
    Runtime method lookup
            │
            ▼
    Shape → class → generic function → applicable methods → effective method
            │
            ▼
    IC.INSTALL (cache the result)
            │
            ▼
    Retry CALL.IC (now hits)
```

### 5.2 Shape System

Every object has a **shape** (also called "hidden class," "map," or "structure" in other systems). The shape describes:

```
shape = {
    shape_id: uint32        ; globally unique ID (in header's shape field)
    class: ref              ; pointer to the class object
    slot_count: fixnum      ; number of instance slots
    slot_names: ref         ; vector of slot names (symbols)
    slot_types: ref         ; vector of type constraints (or nil = any)
    parent_shape: ref       ; for shape transitions (adding a slot creates a new shape)
}
```

Shapes are interned: two objects with the same set of slots in the same order share the same shape. Shape descriptors live in cluster shared SRAM (header subtype `01111`).

### 5.3 Class Precedence and Method Lookup

When `CALL.IC` misses:

1. **Extract the receiver's shape** from the object header.
2. **Map shape → class** via the shape descriptor.
3. **Look up the generic function** by the callsite's function name (a symbol).
4. **Compute applicable methods** using the class precedence list (CPL).
5. **Select the effective method** (most specific applicable method, considering method combination if applicable).
6. **Cache the result** via `IC.INSTALL`.

Steps 3–5 are the expensive part. The runtime maintains a **method cache** (per-cluster, in shared SRAM) that maps `(generic-function, shape_id) → effective-method-entry-point`. This avoids repeating the full CPL walk for the same combination.

### 5.4 Dispatch Optimization Tiers

| Tier | Mechanism | Latency | When |
|:----:|-----------|:-------:|------|
| 1 | Hardware IC (CAM hit) | 1–2 cycles | Monomorphic or low-polymorphic sites |
| 2 | Software method cache (shared SRAM) | 10–20 cycles | Moderate polymorphism, cold IC |
| 3 | Full method lookup (CPL walk) | 100–1000 cycles | First call, highly polymorphic, class changes |

The runtime monitors IC miss rates. If a callsite is **megamorphic** (many different shapes), it replaces the IC entry with a **dispatch stub** — a small code sequence that indexes into a hash table. This avoids constant IC thrashing.

### 5.5 Method Combination

Standard CLOS method combination (`:before`, `:after`, `:around`) is supported at the runtime level. The effective method is a closure that calls the appropriate primaries and auxiliaries. Once computed, it's cached like any other method.

---

## 6. Distributed Heap Management

### 6.1 Object Placement Policy

The runtime decides where to allocate objects:

| Object Category | Placement | Rationale |
|----------------|-----------|-----------|
| Short-lived temporaries | Tile nursery (Gen 0) | Fast bump allocation, tile-local |
| Lambda captures / closures | Tile nursery → cluster old-gen | Closures often survive; promoted on first GC |
| Data structure nodes (cons, vectors) | Tile nursery → cluster old-gen | Follow the data they're attached to |
| Long-lived globals (symbols, classes) | Cluster shared SRAM or HBM | Rarely collected, widely shared |
| Large arrays (> 4 KiB) | Direct to cluster SRAM or HBM | Don't fill nursery with big objects |

### 6.2 Object Migration

Objects may be **migrated** between tiles/clusters for locality:

```
1. Tile A detects frequent remote accesses to an object in Tile B's SRAM.
2. Tile A requests migration: SEND migration_queue, {src: tile_b, obj: ref, dst: tile_a}
3. The migration service (runs on a dedicated thread) copies the object to Tile A's hot-data region.
4. Updates the forwarding pointer at the old location.
5. Broadcasts a fixup notification to tiles that may hold refs to the old location.
```

Migration is a **policy decision**, not an ISA feature. The ISA provides the primitives (DMA, forwarding pointers, fixup) that make migration possible.

### 6.3 Remote Object Access

When a tile needs to access an object in another tile or cluster's SRAM:

1. **Cache miss path:** The tile's LD instruction targets an address not in local SRAM. The hardware generates a **remote read request** via the NoC.
2. **The remote tile's SRAM controller** services the read and returns the word.
3. **The requesting tile** receives the word and completes the LD.

This is transparent to the instruction stream but incurs NoC latency (5–40 cycles). The runtime should minimize it via locality-aware placement.

For write-heavy access to remote objects, migration is preferred over repeated remote writes.

---

## 7. Compilation Strategy

### 7.1 Compiler Pipeline

```
Lisp source
    │
    ▼
┌──────────────┐
│    Reader     │  S-expression parsing
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   Macro      │  Macro expansion, syntax transforms
│  Expansion   │
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   Type       │  Type inference (optional, for optimization)
│ Inference    │  Shape prediction for IC hints
└──────┬───────┘
       │
       ▼
┌──────────────┐
│   CPS / ANF  │  Continuation-passing or A-normal form
│  Transform   │  Makes control flow explicit
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  LM-1 Code   │  Maps Lisp operations to ISA families
│  Generation   │  Inserts safepoints, selects ST vs ST.WB
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Register    │  Allocate 32 GPRs, spill to stack
│  Allocation  │
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Instruction │  32-bit instruction encoding
│  Encoding    │  (see 07-encoding.md)
└──────────────┘
```

### 7.2 Key Compilation Decisions

**Calling convention:**

| Register | Usage |
|----------|-------|
| r0 | Return value |
| r1–r6 | Argument registers (first 6 args) |
| r7–r15 | Temporaries (caller-saved) |
| r16–r24 | Callee-saved |
| r25 (nl) | Nursery limit |
| r26 (np) | Nursery pointer |
| r27 (tp) | Thread pointer |
| r28 (lr) | Link register |
| r29 (fp) | Frame pointer |
| r30 (sp) | Stack pointer |
| r31 | Reserved (zero or special) |

**Barrier insertion:**
- The compiler must distinguish initialization stores (unbarriered) from mutation stores (barriered).
- Rule: if the object was allocated in the current basic block and has not yet been stored into a reachable location, `ST` is safe. Otherwise, `ST.WB`.

**Closure representation:**
- Flat closures: each captured variable is a direct slot in the closure object.
- Shared closures: multiple closures from the same scope share an environment vector.
- The compiler chooses based on capture set size and mutability.

### 7.3 Optimization Opportunities

| Optimization | Description | ISA Support |
|-------------|------------|-------------|
| **Fixnum unboxing** | Keep loop counters as untagged integers in registers | Supplementary scalar ops |
| **Escape analysis** | Stack-allocate objects that don't escape | `ALLOC` on stack instead of nursery |
| **Inline dispatch** | Monomorphic callsites become direct calls | Compiler speculates shape, emits `TST.SHAPE` + `CALL.DIRECT` |
| **Loop unrolling** | Reduce safepoint/branch overhead | Standard, with safepoint insertion |
| **Tail call elimination** | `TAILCALL.IC` / `TAILCALL.DIRECT` | ISA-level support |
| **Prefetch insertion** | Insert `PREFETCH.CDR` before list traversal | Compiler pattern match on `(cdr x)` in loops |

---

## 8. Debugging and Introspection

### 8.1 The Lisp Machine Advantage

One of the classic Lisp-machine virtues: the running system is fully introspectable. LM-1 preserves this by:

- **Tagged memory everywhere.** Any word in any register, stack, or heap location can be printed as a Lisp value. No "untyped raw bits" guessing.
- **Stack frames are walkable.** The FP chain gives full backtraces. Each frame's live values are tagged and identifiable.
- **Symbols are first-class.** Function names, variable names, and source locations are interned symbols in the symbol table.
- **No JIT surprises.** Code is compiled ahead of time (or interactively at the REPL). There are no opaque JIT stubs — all code is LM-1 instructions.

### 8.2 REPL and Interactive Development

The runtime provides a **REPL** (Read-Eval-Print Loop) that:

1. Reads an S-expression from input.
2. Macro-expands and compiles it to LM-1 code.
3. Allocates the code in HBM or cluster SRAM.
4. Spawns a task to execute it.
5. Prints the result.

Compilation at the REPL is fast because:
- The LM-1 ISA is simple (no complex optimization needed for interactive use).
- The compiler is itself a Lisp program running on the LM-1.
- Compiled code is immediately usable (no linking step needed for most definitions).

### 8.3 Debugger

The debugger is a Lisp program that:

- Can **break** at any safepoint by setting a per-tile debug flag.
- Can **inspect** any object by dereferencing refs and reading headers.
- Can **modify** live objects (with appropriate barriers).
- Can **single-step** by setting the tile's single-step mode (a trap-on-every-instruction flag).
- Can **trace** function calls by temporarily installing a tracing wrapper via IC manipulation.

### 8.4 Profiler

Hardware performance counters (§ 10 of [05-memory-gc.md](05-memory-gc.md)) provide cycle-accurate profiling data:

- Allocation rate per tile
- IC hit rates per callsite
- GC pause times per tile and cluster
- NoC traffic volume

The profiler aggregates these into Lisp-level reports (e.g., "function FOO allocated 1.2 MiB and caused 3 nursery GCs").

---

## 9. Boot and Image Loading

### 9.1 System Image

The LM-1 runtime is distributed as a **system image**: a serialized snapshot of the Lisp world containing:

- All compiled code
- All symbols and their values
- Class/shape definitions
- Method tables
- The compiler itself
- The runtime's own code (scheduler, GC policy, etc.)

The image is stored in HBM and loaded at boot time. It's analogous to a Smalltalk image or a Lisp `save-lisp-and-die` core file.

### 9.2 Image Format

```
┌─────────────────────────────────────────┐
│              Image Header               │
│  magic: "LM1I"                          │
│  version: 1                             │
│  entry_point: address of init function  │
│  heap_size: total bytes                 │
│  symbol_table_offset: ...               │
│  class_table_offset: ...                │
└─────────────────────────────────────────┘
│              Heap Dump                  │
│  (serialized tagged words, with         │
│   addresses relocated at load time)     │
└─────────────────────────────────────────┘
│           Code Segments                 │
│  (LM-1 machine code, position-           │
│   independent or with relocation table) │
└─────────────────────────────────────────┘
│          Relocation Table               │
│  (list of addresses that need           │
│   adjustment based on load address)     │
└─────────────────────────────────────────┘
```

### 9.3 Incremental Saves

The runtime SHOULD support incremental image saves: saving only the delta from the base image. This enables fast checkpoint/restart and crash recovery.

---

## 10. Foreign Function Interface (FFI)

### 10.1 Calling Convention Bridge

The LM-1 uses tagged words; external code (C libraries, OS interfaces) uses untagged values. The FFI bridge:

1. **Untags arguments:** strips tags from fixnums, extracts addresses from foreign-pointer objects.
2. **Calls the foreign function** via a trampoline that follows the platform's C ABI.
3. **Tags return values:** wraps C integers as fixnums, C pointers as foreign-pointer objects.
4. **Pins objects:** Any Lisp object passed to C is pinned (not movable by GC) for the duration of the foreign call.

### 10.2 Foreign Pointers

Foreign pointers (header subtype `01110`) are opaque: the GC does not trace them. They hold raw addresses into non-Lisp memory (e.g., `mmap`'d regions, device memory, host CPU memory via PCIe).

### 10.3 Callbacks

C code may call back into Lisp via registered callback functions. The callback mechanism:

1. The Lisp runtime allocates a C-callable trampoline stub.
2. The stub tags the C arguments, pushes a Lisp frame, and calls the Lisp function.
3. On return, it untags the result and returns to C.

---

## 11. Multi-Chip Scaling

### 11.1 Chip-to-Chip Communication

For systems larger than one die, multiple LM-1 chips are connected via:

- **High-speed serial links** (e.g., 112G SerDes) on die edges
- **Same NoC protocol** extended across chip boundaries (with higher latency)
- **Remote DMA** for cross-chip object migration

### 11.2 Distributed Scheduler

The global scheduler extends across chips:

- Each chip has a chip-local scheduler.
- A super-scheduler coordinates load balancing across chips.
- Task affinity now includes chip affinity.

### 11.3 Distributed GC

Full-heap GC across chips uses the same algorithmic framework but with higher coordination latency. `FENCE.GC` becomes a cross-chip barrier. The runtime amortizes this cost by making cross-chip collections rare (promote very old objects to per-chip cold heaps that are collected independently).

---

## 12. The Managed-Data Service

**Status:** decided on 2026-11-20. The runtime is not built; until it is, the chiplet module's stand-in for it serves the function natively (design/chiplet-module.md, decision A), recovery's stage 0 included.

The runtime serves managed data for MegaPad ([05-memory-gc.md](05-memory-gc.md) § 12) as the function of class `MANAGED`, whose records and requests are specified in MegaPad's `docs/managed-data.md` § 9 and § 10.

1. **Starting.** Once up, the runtime offers the class, asking for the shared heap's first block. When MegaPad's `ATTACH` arrives, it checks MegaPad's records and writes its own block: the hold word clear, its three time bounds, the shape IDs of cons cells, vectors, bytevectors and strings, and, from stage 0 of recovery, `MOVE` reading `NONE` and the range of shape IDs set aside for the kinds MegaPad registers ([05-memory-gc.md](05-memory-gc.md) § 12.6). It then replies.
2. **Requests.** It serves `REGION`, `SHAPE` and `COLLECT` on each core's queue pair, and answers every `REGION` without a hold, also while it waits for windows to close. It sends `GROW` before its free space in the shared heap runs out, `RETURN` for blocks it has emptied, and `NOTICE` for an overdue window, corrupt data or a broken log.
3. **Holds.** It takes holds as [05-memory-gc.md](05-memory-gc.md) § 12.3 describes, keeps each within `HOLD_LIMIT` by spreading collection work over several, and leaves `HOLD_GAP` between them.
4. **Logs.** It drains MegaPad's store logs between holds, and drains each completely in every hold before it decides anything.
5. **Shapes.** It interns each record kind MegaPad registers (§ 5.2), keeps its field kinds, refuses a store by LM-1 code that breaks one, and shows records to Lisp by them.
6. **Tasks.** It turns the handles that name a task's inputs into capabilities, and copies the task's result into the shared heap, as § 3.5 describes.
7. **Failure.** When it finds corrupt data, it sends a `NOTICE`, sets `FAILED` in the hold word, and stops collecting the shared heap, while its private work goes on. MegaPad classifies every other end of its service from the records ([05-memory-gc.md](05-memory-gc.md) § 12.9).
8. **Resume** (stage 2). When MegaPad's `ATTACH` carries the resume flag, the runtime takes back the shared heap that the control block names, as [05-memory-gc.md](05-memory-gc.md) § 12.11 describes, and replies `NO_RESUME` when its image cannot take that heap back. Until that `ATTACH`, it leaves the heap and its records untouched.
