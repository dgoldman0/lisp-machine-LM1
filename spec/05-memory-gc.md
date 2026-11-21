# LM-1 Memory Model, GC Invariants, and Barrier Protocol

**Spec ID:** LM1-SPEC-05  
**Revision:** 0.1-draft  
**Date:** 2026-02-16

---

## 1. Scope

This document specifies the memory model, garbage collection invariants, write-barrier protocol, and GC algorithmic framework for the LM-1 architecture. These are ISA-level contracts: any conforming implementation MUST uphold these invariants regardless of the specific GC algorithm chosen.

## 2. Memory Hierarchy from the GC Perspective

```
┌─────────────────────────────────────────────────────┐
│                    GC View of Memory                 │
│                                                      │
│  ┌─────────────┐   ┌─────────────┐   ┌───────────┐  │
│  │   Nursery   │   │  Old-Gen    │   │ Cold Heap │  │
│  │  (per-tile) │   │(per-cluster)│   │  (HBM)    │  │
│  │             │   │             │   │           │  │
│  │ Gen 0       │   │ Gen 1       │   │ Gen 2     │  │
│  │ Bump alloc  │   │ Free-list / │   │ Mark-     │  │
│  │ Copy-collect│   │ copy-compact│   │ compact   │  │
│  └──────┬──────┘   └──────┬──────┘   └─────┬─────┘  │
│         │                 │                 │        │
│    tile SRAM        cluster SRAM          HBM       │
│   (256 KiB)          (2 MiB)          (32 GiB)      │
└─────────────────────────────────────────────────────┘
```

### 2.1 Generation Boundaries

| Generation | Location | Typical Size | Collection Strategy |
|:----------:|----------|:------------:|-------------------|
| **Gen 0** (Nursery) | Tile SRAM | 64 KiB per tile | Copying (bump evacuate to Gen 1) |
| **Gen 1** (Old-Gen) | Cluster shared SRAM | 1.5 MiB per cluster | Copy-compact (within cluster) or promote to Gen 2 |
| **Gen 2** (Cold Heap) | HBM | Up to 32 GiB | Mark-compact (rare, incremental) |

The shared heap of § 12, which lies in memory MegaPad lends and holds the objects MegaPad's code uses, is collected as a heap of its own, with young and old objects, beside these generations.

## 3. Fundamental GC Invariants

These are the rules the **hardware** enforces and the **runtime** depends upon.

### Invariant 1: Precise Pointer Identification

> **Every traced word in the heap, and every word on a stack and in a register, can be unambiguously classified as a pointer or non-pointer by inspecting the tag bits.**

This is guaranteed by the tagged word model ([01-object-model.md](01-object-model.md)). The GC never needs conservative scanning.

A word in the heap is traced unless its object's header marks it raw ([01-object-model.md](01-object-model.md) § 3.5): the bytes of a bytevector or string, the limbs of a bignum, the bits of a boxed double-float, the address in a foreign pointer, and a record's raw words. Scanning, copying and fixup walk objects by their headers, and never read a raw word as a pointer, so raw words may hold any bit pattern, a full 64-bit number included.

### Invariant 2: Object Header Integrity

> **Every live heap object starts with a valid header word (tag `111`) or a forwarding pointer (tag `111`, GC bits = `0xFF`). The header encodes the object's size and layout.**

This allows the GC (and movement engines) to walk the heap linearly, determining object boundaries and pointer-containing fields.

### Invariant 3: Write Barrier Coverage

> **Every store of a ref into a heap object field that is already reachable MUST use `ST.WB` (or an equivalent barriered operation).**

Unbarriered stores (`ST`) are only permitted for:
- Initializing fields of a freshly allocated, not-yet-reachable object (between `ALLOC` and the point where the ref to the new object is stored into another reachable object)
- Storing non-ref values (fixnums, specials, characters)
- Stores to stack locations (the stack is scanned as a root, not via barriers)

MegaPad, which stores into shared objects without LM-1's instructions, logs each of its reference stores instead, and the collector replays the log through the barrier (§ 12.4).

### Invariant 4: Nursery Isolation

> **A nursery object is only reachable from: (a) nursery-local refs, (b) stack roots, (c) register roots. No old-gen object may point to a nursery object unless the corresponding card/remembered-set entry exists.**

This allows nursery collection to scan only roots + remembered sets, not the entire old-gen.

The shared heap's young objects (§ 12) obey the same rule. An old shared object may refer to a young one only where a card, a remembered-set entry, or an entry in a MegaPad store log records it, and the collector replays every such entry before it collects.

### Invariant 5: Forwarding Pointer Protocol

> **During GC, when an object is moved, a forwarding pointer MUST be installed at the old location. Any access to the old location MUST follow the forwarding pointer to the new location before the mutation cycle resumes.**

The movement engines handle this automatically. The fixup pass (via `ENQ.FIXUP`) resolves all forwarding pointers in a region.

MegaPad's code never follows a forwarding pointer: the collector moves shared objects only while it holds MegaPad out, and resolves every reference MegaPad can reach before it lets MegaPad back in (§ 12.3).

### Invariant 6: GC-Safe Points

> **All trap entries (except `TRAP_NURSERY_OVERFLOW`) are GC-safe points: the runtime may perform GC before returning from a trap handler.**

This means all live pointers must be in registers or on the stack at trap boundaries. The compiler MUST NOT keep untagged derived pointers across potential trap points.

### Invariant 7: Shared Closure

> **No object in the shared heap refers to an object outside it.**

Every object MegaPad can reach lies in the shared heap (§ 12.1), so MegaPad never meets a reference into memory it cannot read. The stores of § 4.5 keep the invariant.

---

## 4. Write Barrier Protocol

### 4.1 Barrier Semantics

When `ST.WB Rs, #field, Rt` executes:

```
store(untag_ref(Rs) + (field+1)*8, Rt)

if is_ref(Rt):
    src_gen = generation(Rs)    // from ref metadata or region lookup
    dst_gen = generation(Rt)    // from ref metadata or region lookup
    if dst_gen < src_gen:       // younger-to-older pointer: cross-gen ref
        card_mark(effective_addr(Rs), field)
```

**"Cross-generation" means:** The *stored ref* (Rt) points to a *younger* generation than the *container object* (Rs). This is the interesting case because the nursery collector needs to know about old→young pointers.

### 4.2 Card Table Structure

Each tile maintains a **card table** for its local SRAM. Each cluster maintains card tables for the cluster shared SRAM.

| Parameter | Value |
|-----------|-------|
| Card size | 64 bytes (8 words) |
| Card table entry | 1 byte |
| Entry values | `0x00` = clean, `0xFF` = dirty |
| Tile card table size | 256 KiB / 64 = 4096 entries = 4 KiB |
| Cluster card table size | 2 MiB / 64 = 32768 entries = 32 KiB |

**Card-mark operation:**

```
card_index = (store_address - region_base) >> 6    // divide by card size
card_table[card_index] = 0xFF
```

This is computed by the hardware's barrier logic in the MA stage and written as a byte store.

### 4.3 Remembered Sets (Optional Enhancement)

In addition to card tables, an implementation MAY maintain **per-object remembered sets** — precise lists of incoming cross-generation pointers. This trades space for a faster nursery scan (no need to re-scan entire dirty cards).

Remembered sets are stored in tile or cluster SRAM. The `ST.WB` barrier logic appends entries to a remembered-set log buffer:

```
rs_log[rs_log_top++] = { container_ref: Rs, field: field }
```

If the log buffer overflows, `TRAP_BARRIER_OVERFLOW` fires, and the runtime flushes the log to a more compact structure.

### 4.4 Barrier Filtering

The hardware barrier applies **two fast filters** before marking:

1. **Non-ref filter:** If `Rt` is not a ref (bit 0 = 0, or tag ≠ `x01`), no barrier needed. This is the most common filter — most stores are fixnums or specials.
2. **Same-generation filter:** If the generation bits in `Rs` and `Rt` metadata match, no barrier needed. This is the second most common case — most pointers are to same-generation objects.

Only when both filters fail (cross-generation ref store) does the card-mark/RS-log write occur. Empirically, this happens in <5% of all stores in typical Lisp programs.

### 4.5 Stores into Shared Objects

A tagged store (`ST`, `ST.WB`, `ST.CAR`, `ST.CDR`, or `CAS.TAGGED`) whose container lies in the shared heap (§ 12.1) checks its value before it writes:

1. A reference whose address lies outside the shared-heap ranges traps with `TRAP_SHARED_STORE`, and nothing is written. The runtime's handler publishes the value or refuses the store (§ 12.1), so Invariant 7 holds.
2. A reference inside them is written with bits 63:51 cleared, so references in the shared heap carry no metadata.
3. Otherwise the store proceeds as its instruction specifies, the barrier of § 4.1 included; `ST` into a shared object is permitted only where Invariant 3 permits `ST`.

MegaPad's reference stores into shared objects reach the barrier through its store logs instead (§ 12.4).

---

## 5. GC Algorithmic Framework

The ISA and movement engines support multiple GC algorithms. This section describes the **reference algorithm**: a generational, mostly-concurrent, mostly-copying collector.

### 5.1 Nursery Collection (Gen 0 → Gen 1)

**Trigger:** `TRAP_NURSERY_OVERFLOW` on a tile.

**Algorithm:** Stop-the-tile copying collection.

```
1. PAUSE the tile's mutator threads (other tiles continue)
2. SCAN ROOTS:
   - All registers and stack frames of the tile's 4 HW threads
   - The tile's card table (for old→young pointers from cluster SRAM)
   - The tile's remembered-set log (if applicable)
3. For each root pointer into the nursery:
   a. If already forwarded: update root to forwarding address
   b. Else: copy object to cluster old-gen (via DMA or direct write)
          Install forwarding pointer at old location
          Recursively scan the copied object's fields
4. RESET nursery: np = nursery_base
5. CLEAR card table
6. RESUME the tile's mutator threads
```

**Latency:** Proportional to the number of live nursery objects. With a 64 KiB nursery and typical Lisp survival rates (~5–15%), this is tens of microseconds.

**Concurrency:** Only the affected tile pauses. All other tiles continue executing. The cluster crossbar handles the DMA of survivors to shared SRAM concurrently with other tiles' memory traffic.

### 5.2 Old-Gen Collection (Gen 1)

**Trigger:** Cluster old-gen region usage exceeds a threshold (e.g., 75% full).

**Algorithm:** Concurrent mark-copy using movement engines.

```
Phase 1: MARK (concurrent with mutators)
   a. Enqueue ENQ.SCAN for the old-gen region
   b. Scanner engine walks all objects, identifies live refs
   c. Mutator barriers (ST.WB) ensure new cross-gen refs are captured

Phase 2: COPY (mostly concurrent)
   a. FENCE.GC on all cluster tiles (brief pause to drain in-flight barriers)
   b. Enqueue ENQ.COPY for the old-gen region → new old-gen region
   c. Copier engine copies live objects, installs forwarding pointers

Phase 3: FIXUP (concurrent)
   a. Enqueue ENQ.FIXUP with the pointer list from Phase 1 + forwarding table from Phase 2
   b. Fixup engine updates all pointers in tile nurseries, stacks, and the new old-gen region
   c. Tile nursery card tables are rescanned to update any old→old-gen pointers

Phase 4: RECLAIM
   a. Old old-gen region is freed
   b. Long-surviving objects may be promoted to Gen 2 (HBM)
```

**Pause time:** The only synchronous pause is the `FENCE.GC` in Phase 2, which takes a few hundred nanoseconds (drain store buffers). All other work is concurrent.

### 5.3 Full-Heap Collection (Gen 2)

**Trigger:** HBM heap usage exceeds a threshold, or system-wide memory pressure.

**Algorithm:** Incremental mark-compact.

```
1. Incremental marking:
   - Run on dedicated GC tiles (or steal cycles from idle tiles)
   - Walk all objects in HBM, marking live objects via a bitmap
   - Use snapshot-at-the-beginning (SATB) write barrier:
     ST.WB logs the old value before overwriting, so the marker doesn't miss objects

2. Compaction (optional):
   - Slide live objects toward the beginning of the region
   - Update all pointers (full-heap fixup)
   - This is expensive and done rarely

3. Sweeping (alternative to compaction):
   - Build a free list from unmarked objects
   - Faster than compaction but causes fragmentation
```

**Frequency:** Rare (minutes to hours between full collections, depending on live-set size and allocation rate).

### 5.4 Cross-Tile Pointer Handling

When a tile stores a ref to another tile's nursery object, the write barrier fires (cross-generation). The remembered-set entry identifies the cross-tile pointer.

During nursery collection of the target tile:
1. The collecting tile scans its remembered sets.
2. For cross-tile entries, it sends a **pointer-update message** to the owning tile after forwarding.
3. The owning tile processes the update (either immediately via an interrupt, or lazily via a queue).

This is the main coordination cost of the distributed heap model. The system biases against cross-tile nursery pointers by preferring tile-local allocation for short-lived objects.

---

## 6. Memory Ordering

### 6.1 Intra-Tile Ordering

Within a single tile (and a single hardware thread), memory accesses are **sequentially consistent** — loads and stores are seen in program order. This is natural for an in-order core.

Across hardware threads on the **same tile**, memory accesses to shared SRAM are sequentially consistent (single-port SRAM with cycle-level arbitration).

### 6.2 Inter-Tile Ordering

Across tiles, the memory model is **relaxed** with explicit synchronization:

- **No global coherence.** Writes to one tile's SRAM are not automatically visible to other tiles.
- **Message ordering.** Messages sent via `SEND`/`RECV` have FIFO ordering per queue. A `SEND` followed by a `RECV` establishes happens-before.
- **DMA ordering.** DMA transfers are ordered with respect to the initiating tile: a DMA completion signal establishes happens-before between the DMA source and the initiating tile.
- **Atomics.** `CAS.TAGGED` and `FAA` on shared SRAM (cluster shared SRAM or HBM) provide SC-per-location semantics.
- **FENCE.GC.** Ensures all pending stores and barrier metadata updates are globally visible. Used only during GC phase transitions.

### 6.3 Implications for Lisp

For most Lisp code, the memory model is invisible:
- Single-threaded Lisp code runs on a single hardware thread, seeing sequential consistency.
- Actor-style message passing (the recommended concurrency model) uses `SEND`/`RECV`, which provide happens-before.
- Shared mutable state (which is rare in well-written Lisp) requires explicit atomics or locks.

### 6.4 Ordering with MegaPad

LM-1 reaches memory MegaPad lends through the host link's host-memory port, one request at a time, and MegaPad answers each request only after its access has completed at MegaPad's external-memory controller, which serves one access at a time for every client (MegaPad's `docs/chiplet-link.md` § 4 and § 5). LM-1's accesses to that memory and MegaPad's therefore fall in one order that both chips observe, and LM-1 issues a request there only after the one before it has been answered. The hold protocol of § 12.3 depends on this order, so an implementation that pipelines host-memory requests keeps it for the accesses that protocol makes.

---

## 7. Safepoint Protocol

### 7.1 Definition

A **safepoint** is a program point where:
1. All live refs are in registers or on the stack (no derived untagged pointers).
2. No half-completed allocation is in progress (either `ALLOC` completed, or it hasn't started).
3. The runtime is free to perform GC without data loss.

### 7.2 Automatic Safepoints

The following are automatic safepoints (no compiler annotation needed):
- Every trap entry
- Every `CALL.IC`, `CALL.DIRECT`, `CALL.CLOSURE` (before the call)
- Every `RET`
- Every `SEND`, `RECV`
- Every backward branch (loop back-edges)

### 7.3 Safepoint Polling

For long-running scalar loops without calls or backward branches (rare in Lisp), the compiler SHOULD insert explicit safepoint polls:

```asm
    LD.NL r_scratch, [tp, #gc_request]   ; load GC-request flag
    BR.T  r_scratch, gc_safepoint         ; if set, branch to safepoint handler
```

Alternatively, the runtime can trigger a safepoint by setting the tile's `nl` register to zero, causing the next `ALLOC` to trap.

---

## 8. Root Enumeration

At a safepoint, the GC must find all root pointers. These are:

### 8.1 Register Roots

All 32 GPRs per hardware thread. The hardware tags make scanning trivial: check each register's tag bits, and trace if it's a ref.

### 8.2 Stack Roots

The stack is walked frame-by-frame using the frame pointer chain. Each word on the stack is tag-checked. Since LM-1 uses tagged words everywhere, the stack is precisely scannable without stack maps.

**Note:** This is simpler than conventional architectures where stack maps (generated by the compiler) must identify which stack slots contain pointers. The tagged word model eliminates stack maps entirely.

### 8.3 Global Roots

- The symbol table (cluster shared SRAM)
- The method cache (may contain refs to code objects)
- Finalization queues
- I/O buffers containing refs

These are registered with the runtime and scanned during cluster-level or full-heap collections.

### 8.4 MegaPad's Handles

The slots of MegaPad's handle table are roots of the shared heap, and the collector rewrites them when it moves an object (§ 12.2). They are MegaPad's whole contribution to the roots: a MegaPad core holds references elsewhere only inside a window, and no window is open while the collector decides what lives or moves anything.

---

## 9. Finalization and Weak References

### 9.1 Weak References

Objects with header subtype `01010` (weak reference) contain fields that the GC does **not** trace for liveness. During collection:

1. If the referent is otherwise live, the weak ref is unchanged.
2. If the referent is dead, the weak ref field is replaced with `nil`.
3. If the referent was forwarded, the weak ref field is updated to the new address.

### 9.2 Finalizers

Objects may be registered with the runtime for finalization (running cleanup code before reclamation). The GC:

1. Identifies finalizable objects that are otherwise dead.
2. Marks them as live (to prevent premature reclamation).
3. Enqueues them on a finalization queue.
4. A finalizer thread runs the registered cleanup functions.
5. After finalization, the object becomes eligible for reclamation in the next cycle.

This is a **two-phase** reclamation model, similar to Java's finalization protocol.

---

## 10. GC Metrics and Monitoring

Each tile and cluster maintain hardware performance counters:

| Counter | Description |
|---------|-------------|
| `alloc_count` | Number of `ALLOC` instructions executed |
| `alloc_bytes` | Total bytes allocated |
| `nursery_collections` | Number of nursery overflows / minor GCs |
| `nursery_survivors` | Bytes surviving nursery collection |
| `barrier_fires` | Number of `ST.WB` instructions that triggered a card mark |
| `barrier_filtered` | Number of `ST.WB` instructions filtered (no action needed) |
| `ic_hits` | Number of `CALL.IC` hits |
| `ic_misses` | Number of `CALL.IC` misses |
| `engine_scan_words` | Words scanned by movement engine |
| `engine_copy_words` | Words copied by movement engine |
| `engine_fixup_count` | Pointers fixed up by movement engine |
| `gc_pause_cycles` | Total cycles spent in GC pauses (per tile) |

These counters are readable via `TILE.COUNTER Rd, #counter_id` (a supplementary system instruction).

---

## 11. Memory Lent to MegaPad

**Status:** proposed on 2026-11-20; not built.

LM-1 lends MegaPad ranges of its own memory for writing. MegaPad's one writer into them is its tile engines' `TACC.STORE`, which stores 256-byte accumulator images there when LM-1 borrows the tile engines for numeric work ([04-soc.md](04-soc.md) § 8). The stores arrive through the host link and land through a cluster's memory write port. A lent range follows these rules.

1. **Plain data only.** A range holds numbers and never a word LM-1 reads as a reference, so MegaPad's stores can never forge one. The memory where both chips store references is memory MegaPad lends LM-1, the shared heap of § 12, and LM-1 lends none of its own for them (§ 12.7). It lies outside the collected heap: in a region of a cluster's shared SRAM set apart from the old-generation region ([04-soc.md](04-soc.md) § 5.2), or in the bytes of a pinned bytevector. The collector neither scans nor moves it, so it has no place among the roots (§ 8), and MegaPad's stores need no barrier (§ 4). Tile-work results use a region set apart, whose address stays fixed while it is lent with no pin to track.
2. **Placement.** The range lies in the shared SRAM of the cluster whose tiles run the tasks that read the results, 4 to 8 cycles from them, and the scheduler's cluster affinity ([06-runtime.md](06-runtime.md) § 4.3) keeps those tasks there. Until LM-1's top level maps the link's addresses onto its clusters, the link reaches one cluster's memory.
3. **Alignment and size.** Base and limit are aligned to 64 bytes. The range holds as many 256-byte images as the runtime's tile-work requests in flight may return, and its size follows that demand rather than a fixed bound.
4. **Lending and growing.** The runtime installs a range at the link's lend port before it announces the range to MegaPad in a `LEND` entry that names the function the range serves, so MegaPad never stores into a range LM-1 does not yet admit. To grow, it installs a larger range in the link's second range slot and announces it, then withdraws the old one: it sends `LEND` with rights of zero, waits for MegaPad's reply, which comes once MegaPad's writes into the range have ended, and only then withdraws the range at the port.
5. **Allotting.** The runtime gives each request its own images within the range and frees them only after it has read that request's reply, a cancelled request's included, because MegaPad may store into them until it answers.
6. **Answering.** LM-1's end of the link answers every write MegaPad begins, a cancelled one included. A write left unanswered makes MegaPad refuse every later write until the port is reset (MegaPad's `docs/chiplet-link.md` § 4).
7. **Reset.** LM-1's reset clears every lent range at the link, so MegaPad's stores into them fault until the restarted runtime lends again.

---

## 12. Managed Data: Objects MegaPad Holds

**Status:** decided on 2026-11-20; not built. Recovery after an LM-1 failure is decided in stages, and its design stays in MegaPad's `docs/managed-data-recovery.md` until this section absorbs it. It already binds this section's collector: a moved object is copied in full before anything points to it, a hold moves objects only into space that was free when it began, a word records the hold's progress, marks stay out of object headers, and shapes keep fixed IDs.

MegaPad's Forth code uses LM-1 objects as managed data (release plan Part 9d): it creates them, reads them directly, writes them only through its library's words, and keeps them past short windows as handles, and LM-1's collector frees whatever nothing uses. MegaPad's `docs/managed-data.md` specifies MegaPad's side, the records both chips keep, and the requests that cross the link. This section states what LM-1 owes it. Nothing here depends on where LM-1's own main memory is placed (release plan D8).

### 12.1 The Shared Heap

The **shared heap** is the part of the collected heap that lies in memory MegaPad lends LM-1 for managed data: the first block MegaPad grants with the managed-data function, and the blocks LM-1 asks for later with `GROW`. An object in it is a **shared object**. The shared heap is collected as a heap of its own, with young and old objects, beside the generations of § 2.

1. **Shared closure.** Every object MegaPad can reach is a shared object, and no shared object refers to an object outside the shared heap. MegaPad's cores cannot read LM-1's own memory, so every object they reach must lie in MegaPad's memory.
2. **Publishing.** An object becomes shared by being made in the shared heap: by MegaPad in an allocation region (§ 12.5), or by the runtime, which copies a task's result there when it hands the result to MegaPad. A store that would put a reference to an object outside the shared heap into a shared object traps (§ 4.5). For Lisp code, the runtime's handler moves the value, and everything it reaches, into the shared heap, resolving the forwarding pointers left at the old locations as for any move (Invariant 5), and then completes the store. For a task of Leviathan's language, whose objects leave it only through its result, it refuses the store ([06-runtime.md](06-runtime.md) § 3.5).
3. **Headers.** Every shared object has a header, cons cells included. The headerless cons of [01-object-model.md](01-object-model.md) § 4.1 applies only outside the shared heap.
4. **Metadata.** A reference stored in a shared object or a handle slot carries zero in bits 63:51. The stores of § 4.5 clear them, and the collector writes references without them, so two references to one shared object are equal words, as MegaPad compares them.
5. **Fixed headers.** A shared object's subtype, size and shape change only during a hold (§ 12.3). Its GC bits may change at any time, and MegaPad ignores them.
6. **Ranges.** The runtime installs each block of the shared heap as a shared-heap range, against which the hardware classifies addresses for the stores of § 4.5 and the loads of § 12.8. It asks MegaPad for blocks that each at least double the heap, so a few ranges cover any size. The number of ranges a tile holds is a parameter the hardware reports, as the host link's lend port reports `lend_ranges`.

Where LM-1's main memory is placed changes only where its private heap lies. With memory of its own, the private heap lies there. Without it, the private heap lies in other memory MegaPad lends, outside the shared-heap ranges. Either way the private heap is collected without regard to MegaPad.

### 12.2 MegaPad's Handles Are Roots

MegaPad keeps the objects it holds past a window in its **handle table**: 8-byte slots in memory it lends LM-1 for reading and writing, in segments its handle directory lists (MegaPad's `docs/managed-data.md` § 4 and § 9). A slot holds a reference, nil for a handle that names no object yet, or zero while it is free.

1. **Roots.** Every slot that holds a reference is a root of the shared heap, beside the roots of § 8. A collection that decides what is live, or moves shared objects, reads the directory and the slots during a hold. It may read every slot, or only the slots the store logs name (§ 12.4) together with those it already knows to refer to young objects, since MegaPad logs every reference it stores into a slot.
2. **Updates.** When the collector moves an object, it rewrites every slot that refers to it during the same hold, keeping the reference's tag.
3. **Nothing else.** The collector writes a slot only during a hold, and only to rewrite a reference. The runtime writes one slot more: the slot MegaPad lends a task for its result, into which it stores the result's reference when the task ends ([06-runtime.md](06-runtime.md) § 3.5), while MegaPad leaves that slot alone. The collector counts that slot among its roots from the moment of the store, as it counts a slot MegaPad has logged. MegaPad makes and frees slots, inside its windows.

### 12.3 Holds: Waiting for Windows

MegaPad uses references only inside **windows**. A window belongs to one MegaPad core, which announces it in a window word that it alone writes. The collector moves or frees shared objects only during a **hold**, in which it holds every core out (MegaPad's `docs/managed-data.md` § 3).

1. **Moving and freeing.** The collector moves a shared object only during a hold, and finishes each move within it: every handle slot and every reference to the object is rewritten, and no forwarding pointer is reachable, before the hold ends. It frees a shared object only if it found the object unreachable during a hold: from MegaPad's handles, from LM-1's own roots, and through every logged store. Marking may run between holds, and so may sweeping that frees what a hold found unreachable, but the finding itself and every move happen inside a hold.
2. **Taking a hold.** The collector stores `HELD` in the hold word and waits for the store's answer. It then reads every core's window word and waits until none reads `OPEN`, reading a core's word again when that core rings. It sets `ACTIVE`, does its work, stores the hold word with `HELD` and `ACTIVE` clear and the epoch one higher, then reads every window word again and rings each core that reads `WAITING`.
3. **Order.** These accesses go through the host-memory port one at a time, each answered after it completes at MegaPad's external-memory controller, which orders them with MegaPad's own accesses (§ 6.4). That order makes the handshake sound: a core stores its window word before it loads the hold word, the collector stores the hold word before it loads the window words, so one of them always sees the other.
4. **Answering while waiting.** The collector keeps answering `REGION` requests while it waits for windows, and answers each without a hold. A core may wait for one inside its window, so a collector that stopped answering would wait for ever.
5. **Bounds.** A hold stays `ACTIVE` at most `HOLD_LIMIT`, and the next one begins at least `HOLD_GAP` after it ends. The runtime states both, with `WINDOW_LIMIT`, in its block at `ATTACH`. Work that does not fit is spread over several holds: marking runs between them (§ 5.2), and each hold evacuates only the regions it can finish.
6. **Overdue windows.** A window still open `WINDOW_LIMIT` after `HELD` was set is MegaPad's fault. The collector waits on, and reports the core in a `NOTICE`.
7. **LM-1's own threads.** A move of shared objects also stops, at safepoints (§ 7), every LM-1 thread that may hold references into the regions being moved, as any move does.
8. **The engines.** The movement engines of [04-soc.md](04-soc.md) § 5.3 reach only a cluster's shared SRAM, and the shared heap lies across the link. Work on it goes through the host-memory port, done by the runtime or by engines given a path to that port, which the chiplet revision decides; the traffic it adds to MegaPad's memory is part of the release plan's D8 sizing study.

### 12.4 MegaPad's Stores Reach the Collector's Records

MegaPad cannot write LM-1's card tables or remembered sets, which lie in LM-1's own memory. Each MegaPad core instead logs its reference stores: the address of every word into which it stores a reference, a field of a shared object or a handle slot, goes into the core's **store log**, a ring in memory MegaPad lends for reading. The count of entries written, `TAIL`, is in the core's record, and LM-1 writes the count it has taken, `LOG_HEAD`, in its block.

1. **Replay.** The collector takes each entry, reads the word it names, and applies the barrier of § 4 as if LM-1 had stored that value there. When the value is a reference younger than its container, it marks the card or records the slot. While a mark is under way, it marks the value as the algorithm of § 5 requires of a store. The barrier's rules thus exist once, on LM-1.
2. **When.** The collector may drain a log at any time, a hold not needed. During each hold it drains every log to the `TAIL` it reads there before it moves, frees, or concludes a mark. An entry it can read names a store that has landed, because MegaPad writes the word, then the entry, and only then `TAIL`. It reads `TAIL` before the ring's address and capacity, since a core may move to a larger ring.
3. **What is not logged.** MegaPad logs no store of a fixnum, special, raw word or byte, since none makes a reference. It logs no store into an object in its own current allocation region either, because the collector traces those objects whole (§ 12.5).
4. **The invariant.** With replay, Invariant 4 holds for MegaPad's stores: an old shared object refers to a young one only through a store LM-1 made with a barrier or a store MegaPad logged, and the collector replays every logged store before it collects.
5. **A full log.** A core whose log is full rings LM-1 and waits for `LOG_HEAD` to move. The collector drains the log without a hold, and rings the core when it has.

### 12.5 Objects MegaPad Makes

MegaPad makes objects in **allocation regions**, parts of the shared heap that the runtime gives one core at a time, in the `NEXT` fields of the core's entry in LM-1's block. The core places objects in a row from the region's base, writing each one whole, and publishes `FILL`, the end of its last object, at each window close.

1. **Granting.** The runtime grants a region in reply to a `REGION` request, from free space or by asking MegaPad for a block, never by collecting. It keeps free space in reserve, and asks for blocks before it runs out. At the end of every hold it retires every region it gave MegaPad, and writes each core that makes objects a fresh one. It sizes regions from each core's demand, and at least as large as asked.
2. **Walking.** The bytes from a region's base to `FILL`, or to the fill a core published before asking for its next region, are whole objects, which the collector walks by their headers.
3. **Kinds.** MegaPad makes cons cells, vectors, bytevectors, strings, and records of the shapes it has registered (§ 12.6). The collector checks each object it walks there: a sound header of one of these subtypes, a size inside the region, a registered shape, and references only into the shared heap.
4. **Unscanned until retired.** MegaPad stores into the objects of its current region without log entries, so the collector never counts such an object as scanned while its region is still MegaPad's. While a mark is under way, the hold that retires a region traces the fields of every object in it that the mark has reached, even one the marker traced before. A young collection traces the live ones as it reaches them.

### 12.6 Shapes MegaPad Registers

MegaPad registers each record kind once with a `SHAPE` request, naming the number of tagged fields and raw words, each one's kind, and a name. The runtime interns the shape ([06-runtime.md](06-runtime.md) § 5.2), keeps the field kinds with it, and returns its ID; the same description always gets the same ID. A store by LM-1 code that breaks a shared record's declared field kind is refused, and the runtime shows records to Lisp by their kinds. At `ATTACH`, LM-1 states the shape IDs of cons cells, vectors, bytevectors and strings.

### 12.7 The Memory That Holds References

The shared heap's blocks are the one kind of lent memory that holds references. They differ from the ranges LM-1 lends MegaPad (§ 11), which hold plain data only:

1. MegaPad lends them, from its own memory, for reading and writing. LM-1 lends none of its own memory for references.
2. Both chips write references there: MegaPad through its library, LM-1 through its stores and its collector.
3. They are part of the collected heap. The collector scans them, moves objects within them during holds, and frees objects in them.
4. LM-1 trusts no word in them that MegaPad may have written. It checks every reference it loads from them (§ 12.8), every object it walks there (§ 12.5), and every record of MegaPad's it reads, as MegaPad's `docs/managed-data.md` § 9 lists, and a failed check stops it serving managed data (§ 12.9).

### 12.8 Loading References From Shared Memory

MegaPad can store any word into its own memory, so a word in the shared heap with a reference tag may be forged. A tagged load (`LD`, `LD.CAR`, `LD.CDR`, or the old value `CAS.TAGGED` returns) from a shared-heap address that yields a reference whose address lies outside the shared-heap ranges traps with `TRAP_SHARED_REF`, and leaves its destination unchanged. The collector applies the same test to every reference it reads from the shared heap or a handle slot, and treats a failure as corrupt data. A reference loaded from shared memory therefore carries authority over the shared heap and nothing else. In capability mode, a reference loaded through a capability that lacks write permission lacks it too, so a task given its inputs read-only can write nothing it reaches through them.

### 12.9 Failure

When the collector finds corrupt data in the shared heap or in MegaPad's records, it reports it in a `NOTICE`, sets `FAILED` in the hold word, and stops collecting the shared heap, while LM-1's private work goes on. MegaPad then treats every handle as stale and returns the function's memory. Managed data is not kept across a failure or a reset of LM-1.

### 12.10 Conformance

MegaPad's `docs/managed-data.md` § 15 lists the link conformance tests for managed data (release plan Part 8g). LM-1's C++ chiplet module (Part 8d) implements the collector's side of managed data first and passes them from its first day; both RTL sides pass them later.
