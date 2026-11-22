# LM-1 Instruction Set Architecture

**Spec ID:** LM1-SPEC-02  
**Revision:** 0.1-draft  
**Date:** 2026-02-16

---

## 1. Scope

This document specifies the LM-1 instruction set architecture. The ISA is organized into **8 semantic instruction families** that map to dynamic-language operations, plus a **supplementary scalar family** for systems-level code. Each family defines fast-path behavior (hardware) and slow-path behavior (trap to runtime).

## 2. General Conventions

### 2.1 Registers

| Register | Count | Description |
|----------|:-----:|-------------|
| `r0`–`r31` | 32 | General-purpose tagged-word registers |
| `sp` | 1 | Stack pointer (alias `r30`) |
| `fp` | 1 | Frame pointer (alias `r29`) |
| `lr` | 1 | Link register (alias `r28`) |
| `pc` | 1 | Program counter (not directly addressable) |
| `tp` | 1 | Thread/tile pointer (alias `r27`) — points to per-thread/tile state |
| `np` | 1 | Nursery pointer (alias `r26`) — current nursery bump pointer |
| `nl` | 1 | Nursery limit (alias `r25`) — end of nursery region |

All general-purpose registers hold 64-bit tagged words. The hardware thread context includes all 32 GPRs plus the special registers.

### 2.2 Operand Notation

```
Rd        — destination register
Rs, Rs1   — source register(s)
Rt        — target register (for stores: the value being stored)
imm       — immediate constant
field     — field index (unsigned immediate)
offset    — signed offset: in instructions from the instruction's own address for
            branches and calls, in bytes for loads and stores
tag       — tag constant for type testing
```

### 2.3 Traps and Slow Paths

When a fast-path condition is not met, the instruction raises a **synchronous trap** with a defined trap code. The runtime's trap handler receives:

- Trap code (identifying which slow path)
- The PC that `ERET` resumes at: the faulting instruction's for a fault, so that `ERET` retries it, and the next instruction's for an explicit `TRAP`; `ERET.NEXT` resumes at the instruction after the saved PC, for a handler that carried out the faulting instruction itself ([03-core.md](03-core.md) § 6.3)
- All register state

Trap codes are enumerated per instruction family below.

### 2.4 Condition Codes

LM-1 does **not** use a flags register. Comparisons produce results in registers. Conditional branches test a register value directly.

---

## 3. Family 1: Tagged Arithmetic & Type Tests

### 3.1 Type Test Instructions

#### `TST Rd, Rs, #tag`

Test whether `Rs` matches the given tag class. Write `t` to `Rd` on match, `nil` on mismatch.

| Tag Constant | Matches |
|:--:|:--|
| `TAG_FIXNUM` | `(Rs & 1) == 0` |
| `TAG_REF` | `(Rs & 3) == 1` |
| `TAG_CONS` | `(Rs & 7) == 3` |
| `TAG_SPECIAL` | `(Rs & 7) == 5` |
| `TAG_NIL` | `Rs == 0x05` |
| `TAG_CHAR` | `(Rs & 0xFF) == 0x35` |
| `TAG_SFLOAT` | `(Rs & 0xFF) == 0x3D` |
| `TAG_HEADER` | `(Rs & 7) == 7` |

**Cycles:** 1  
**Traps:** None

#### `TST.SHAPE Rd, Rs, #shape_id`

Test whether the object referenced by `Rs` has the given shape ID. This first checks the shape-cache hint in the ref's metadata bits (no memory access). On hint match, write `t`; on hint mismatch, load the object header and compare the shape ID field. Write `t` on match, `nil` on mismatch.

**Precondition:** `Rs` MUST be a ref (tag `001` or `011`). Behavior is undefined if `Rs` is not a ref.

**Cycles:** 1 (hint hit), 1 + memory latency (hint miss, header load)  
**Traps:** None (non-ref operand is undefined behavior, not a trap)

### 3.2 Fixnum Arithmetic

All fixnum arithmetic instructions operate on tagged fixnums directly. They check that both operands are fixnums (bit 0 = 0), perform the operation, and check for overflow. On non-fixnum operand or overflow, they trap.

#### `ADD.FIX Rd, Rs1, Rs2`

```
if (Rs1 & 1) != 0 or (Rs2 & 1) != 0:
    trap TRAP_NOT_FIXNUM
result = Rs1 + Rs2    // tag bits cancel: (a<<1) + (b<<1) = (a+b)<<1
if overflow:
    trap TRAP_FIXNUM_OVERFLOW
Rd = result
```

**Cycles:** 1  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_FIXNUM_OVERFLOW`

#### `SUB.FIX Rd, Rs1, Rs2`

Same structure as `ADD.FIX` with subtraction.

#### `MUL.FIX Rd, Rs1, Rs2`

```
if (Rs1 & 1) != 0 or (Rs2 & 1) != 0:
    trap TRAP_NOT_FIXNUM
a = Rs1 >> 1    // arithmetic shift to get untagged value
result = a * Rs2    // Rs2 is still shifted, so result = a*b << 1
if overflow:
    trap TRAP_FIXNUM_OVERFLOW
Rd = result
```

**Cycles:** 3–5 (depending on multiplier latency)  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_FIXNUM_OVERFLOW`

#### `DIV.FIX Rd, Rs1, Rs2`

```
if (Rs1 & 1) != 0 or (Rs2 & 1) != 0:
    trap TRAP_NOT_FIXNUM
if Rs2 == 0:
    trap TRAP_DIVIDE_BY_ZERO
a = Rs1 >> 1
b = Rs2 >> 1
result = (a / b) << 1    // re-tag
Rd = result
```

**Cycles:** 8–32 (depending on divider)  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_FIXNUM_OVERFLOW`, `TRAP_DIVIDE_BY_ZERO`

#### `ADD.FIX.IMM Rd, Rs, #imm`

Immediate-operand variant. `imm` is a pre-tagged fixnum constant.

#### `CMP.TAGGED Rd, Rs1, Rs2`

Tagged comparison with type-aware rules. The result in `Rd` is a fixnum, pre-tagged:

- fixnum–fixnum: numeric comparison, `-2` when `Rs1` is less, `0` when the two are equal and `+2` when `Rs1` is greater.
- Two words of one other primary tag (bits 2:0) — two general refs, two cons refs, two specials or two headers: identity comparison, `0` when the words are equal and `+2` when they differ. Such words have no order.
- Any other pair: `TRAP_TYPE_MISMATCH`. A general ref (tag `001`) and a cons ref (tag `011`) are different types for `CMP.TAGGED`, so comparing one with the other traps; `EQ` compares any two words.

**Cycles:** 1–2  
**Traps:** `TRAP_TYPE_MISMATCH`

#### `EQ Rd, Rs1, Rs2`

Raw word equality (identity / `eq`). No type checking. `Rd` = `t` if `Rs1 == Rs2`, else `nil`.

**Cycles:** 1  
**Traps:** None

---

## 4. Family 2: Allocation

### 4.1 `ALLOC Rd, #words, #header_template`

Allocate a fixed-size object in the tile's nursery.

```
size = words             // payload words, the header excluded
if np + (size + 1) * 8 > nl:
    trap TRAP_NURSERY_OVERFLOW
*np = header_template with its size field set to size
the size payload words = 0
Rd = np | TAG_REF        // tag as ref, add metadata hints
np = np + (size + 1) * 8
```

The `header_template` encodes the header subtype and shape ID, and the header written from it carries the object's size. GC bits in the header are initialized to the current generation's defaults. A header's size counts the payload words after it, as every header's does ([01-object-model.md](01-object-model.md) § 3.5), so the object takes one word more than its size. The allocation zeroes the payload, so every traced word of the new object holds a valid tag, fixnum 0, until the program stores into it ([01-object-model.md](01-object-model.md) § 6).

**Cycles:** 1 (fast path)  
**Traps:** `TRAP_NURSERY_OVERFLOW` (runtime performs minor GC, then retries)

### 4.2 `ALLOC.CONS Rd, Rs_car, Rs_cdr`

Allocate a cons cell holding two registers' values. Specialized for the most common allocation.

```
car = Rs_car, or nil when the field names r0
cdr = Rs_cdr, or nil when the field names r0
if np + CONS_SIZE > nl:
    trap TRAP_NURSERY_OVERFLOW
// With a cons header:
*np = header_template[0] with its size field set to 2
*(np + 8) = car
*(np + 16) = cdr
Rd = np | TAG_CONS_REF
np = np + 24
// With headerless cons: *np = car, *(np + 8) = cdr, Rd = np | TAG_CONS_REF, np = np + 16
```

The instruction reads both registers before it moves `np`, so naming `np` gives the address the new cons receives. A field of 0 gives nil rather than `r0`'s value, so `ALLOC.CONS Rd, r0, r0` makes a cons of two nils. Every implementation so far writes the cons header, from entry 0 of the header-template table ([07-encoding.md](07-encoding.md) § 6); the headerless cons of [01-object-model.md](01-object-model.md) § 4.1 is an option an implementation MAY take.

**Cycles:** 1 (fast path)  
**Traps:** `TRAP_NURSERY_OVERFLOW`

### 4.3 `ALLOCV Rd, Rs_length, #header_template`

Allocate a variable-length object (vector, string, bytevector).

```
if (Rs_length & 1) != 0:
    trap TRAP_NOT_FIXNUM
len = Rs_length >> 1    // untag fixnum length
size = len + 1          // payload words: the length slot and the elements
if np + (size + 1) * 8 > nl:
    trap TRAP_NURSERY_OVERFLOW
*np = header_template with its size field set to size
the size payload words = 0
*(np + 8) = Rs_length    // store length as tagged fixnum
Rd = np | TAG_REF
np = np + (size + 1) * 8
```

The header's size counts the payload, the length slot and the elements, as the vector layout of [01-object-model.md](01-object-model.md) § 4.3 has it. Every element starts as fixnum 0.

**Cycles:** 2 (fast path)  
**Traps:** `TRAP_NURSERY_OVERFLOW`, `TRAP_NOT_FIXNUM` (if length is not a fixnum)

### 4.4 `ALLOC.CLOSURE Rd, Rs_code, #env_size`

Allocate a closure with a code entry point and `env_size` environment slots.

```
size = 1 + env_size         // payload words: the code entry and the env slots
if np + (size + 1) * 8 > nl:
    trap TRAP_NURSERY_OVERFLOW
*np = header_template[1] with its size field set to size
the size payload words = 0
*(np + 8) = Rs_code
Rd = np | TAG_REF
np = np + (size + 1) * 8
```

The header comes from entry 1 of the header-template table, the closure template ([07-encoding.md](07-encoding.md) § 6), and its size counts the code entry and the environment slots, as the closure layout of [01-object-model.md](01-object-model.md) § 4.4 has it. The environment slots start as fixnum 0.

**Cycles:** 1–2 (fast path)  
**Traps:** `TRAP_NURSERY_OVERFLOW`

---

## 5. Family 3: Field Access with Barriers

### 5.1 `LD Rd, Rs, #field`

Load tagged word from object field.

```
addr = untag_ref(Rs) + (field + 1) * 8    // +1 to skip header
Rd = *(addr)
```

**Precondition:** `Rs` is a ref.

**Cycles:** 1 + memory latency  
**Traps:** `TRAP_NOT_REF` (if tag check is enabled)

### 5.2 `LD.CAR Rd, Rs` / `LD.CDR Rd, Rs`

Specialized loads for cons cells.

```
// LD.CAR
addr = untag_ref(Rs)
Rd = *(addr + 0)   // car is first word (headerless) or *(addr + 8) (with header)

// LD.CDR
Rd = *(addr + 8)   // cdr is second word (headerless) or *(addr + 16)
```

**Cycles:** 1 + memory latency  
**Traps:** `TRAP_NOT_CONS` (optional, if tag check is enabled)

### 5.3 `ST Rs, #field, Rt`

Store tagged word to object field. **No write barrier.**

```
addr = untag_ref(Rs) + (field + 1) * 8
*(addr) = Rt
```

Use only when the GC can prove no barrier is needed (e.g., initializing a newly allocated object before it becomes reachable).

**Cycles:** 1  
**Traps:** None

### 5.4 `ST.WB Rs, #field, Rt`

Store with **write barrier**. This is the primary store instruction for mutating live objects.

```
addr = untag_ref(Rs) + (field + 1) * 8
*(addr) = Rt

// Hardware barrier logic:
if Rt is a ref AND cross_generation(Rs, Rt):
    card_mark(addr)    // mark the card containing addr in the card table
    // and/or: add (Rs, field) to the remembered set
```

The barrier logic is performed in the store pipeline. The hardware checks:

1. Is `Rt` a ref? (bit 0 = 1, bits 2:1 indicate ref class)
2. Is this a cross-generation pointer? (compare GC generation bits in Rs and Rt metadata, or compare region boundaries)
3. If yes, update the card table and/or remembered set.

**Cycles:** 1–2 (fast path, card-mark hits store buffer)  
**Traps:** `TRAP_BARRIER_OVERFLOW` (if remembered set or card table overflows local capacity), `TRAP_SHARED_STORE`

A store whose container lies in the shared heap that LM-1 keeps with MegaPad also checks its value, as do `ST`, `ST.CAR`, `ST.CDR` and `CAS.TAGGED` ([05-memory-gc.md](05-memory-gc.md) § 4.5).

### 5.5 `ST.CAR Rs, Rt` / `ST.CDR Rs, Rt`

Barriered stores to cons cell fields. Equivalent to `ST.WB` with field 0/1 and cons-specific addressing.

---

## 6. Family 4: Dynamic Dispatch

Every call — `CALL.DIRECT`, `CALL.CLOSURE`, `CALL.REG`, and `CALL.IC` on a hit — starts a frame on the thread's stack, and `RET` ends it:

```
push_frame(return_addr):
    *(sp - 8)  = lr       // the caller's link register
    *(sp - 16) = fp       // the caller's frame pointer
    sp = sp - 16
    lr = return_addr      // pc + 4, the instruction after the call
    fp = sp               // fp points at the saved fp, with the saved lr above it
```

While the callee runs, `lr` holds its return address and `fp` its frame. The tail calls jump without a frame, so their callee returns through the frame in use, to its caller's caller.

### 6.1 `CALL.IC Rs_receiver`

Inline-cached dynamic dispatch. This is the primary instruction for method calls, generic function dispatch, and message sends. Each `CALL.IC` is a call site of its own: the inline cache is keyed by the instruction's address and the receiver's shape, so the instruction names only the receiver, and the arguments travel by the calling convention.

**Hardware behavior:**

```
if Rs_receiver is not a ref (tag 001 or 011):
    trap TRAP_NOT_REF
hdr = *(untag_ref(Rs_receiver))
shape = shape_id(hdr) if hdr is a header, else 0
entry = ic_lookup(pc, shape)        // pc: this CALL.IC's address

if entry.valid:
    // IC hit: direct call
    push_frame(pc + 4)
    pc = entry.code_entry
else:
    // IC miss: trap to runtime
    trap TRAP_IC_MISS
```

The hardware IC table is a small associative structure (per-core or per-tile, see [03-core.md](03-core.md)). Each entry holds:

| Field | Bits | Description |
|-------|:----:|-------------|
| call site | 48 | Address of the `CALL.IC` or `TAILCALL.IC` |
| shape | 32 | Expected receiver shape ID |
| code_entry | 48 | Destination code address |
| valid | 1 | Entry is valid |

**IC table size:** Implementation-defined; RECOMMENDED minimum 64 entries per hardware thread.

**IC installation:** The runtime's `TRAP_IC_MISS` handler finds the call site in the trap PC, which is the `CALL.IC`'s own address (§ 2.3), and the receiver in the register that instruction names. It performs method lookup, resolves the target, executes `IC.INSTALL` to populate the cache, and returns with `ERET`, which runs the call again, now a hit.

**Cycles:** 1 + memory latency (IC hit), full trap latency (IC miss)  
**Traps:** `TRAP_NOT_REF`, `TRAP_IC_MISS`

### 6.2 `IC.INSTALL Rs_site, Rs_receiver, Rs_entry`

Install or update an inline-cache entry. Privileged to the runtime.

```
shape = the shape of Rs_receiver, read as CALL.IC reads it; 0 when Rs_receiver is not a ref
ic_table[(Rs_site, shape)] = { Rs_entry, valid=1 }
```

`Rs_site` holds the address of the call site, and `Rs_receiver` an object of the shape the entry serves, such as the receiver whose miss the handler answers.

**Cycles:** 1–2 (+ memory latency for the header)  
**Traps:** None (but may evict existing entries)

### 6.3 `CALL.DIRECT #offset`

Direct (non-dispatched) call of the code at a PC-relative address.

```
push_frame(pc + 4)
pc = pc + offset * 4    // offset in instructions, from this CALL.DIRECT
```

The 16-bit offset reaches 32,768 instructions (128 KiB) back and 32,767 forward.

**Cycles:** 1  
**Traps:** None

### 6.4 `CALL.CLOSURE Rs_closure`

Call a closure. Loads the code entry from the closure object.

```
if Rs_closure is not a ref, or its header is not a closure header (hdr-sub 00100):
    trap TRAP_NOT_CLOSURE
code = *(untag_ref(Rs_closure) + 8)    // code entry is first field
push_frame(pc + 4)
pc = code
```

The callee finds the closure in `Rs_closure`, a register the calling convention fixes, and reads its captured variables through it: environment slot *i* is field *i* + 1 of the closure ([01-object-model.md](01-object-model.md) § 4.4).

**Cycles:** 1 + memory latency (for the header and code loads)  
**Traps:** `TRAP_NOT_CLOSURE` (if tag/header check fails)

### 6.5 `RET`

Return from a call through the link register, restoring the caller's frame.

```
target = lr
sp = fp
fp = *(sp)        // the caller's frame pointer
lr = *(sp + 8)    // the caller's link register
sp = sp + 16
pc = target
```

**Cycles:** 1  
**Traps:** None

### 6.6 `TAILCALL.IC Rs_receiver`

Tail-call variant of `CALL.IC`. It looks the receiver's shape up as `CALL.IC` does, keyed by its own address; a hit jumps to the entry without a frame, and a miss traps `TRAP_IC_MISS`.

### 6.7 `TAILCALL.DIRECT #offset`

Tail-call variant of `CALL.DIRECT`: `pc = pc + offset * 4`, without a frame.

### 6.8 `CALL.REG Rs`

Call the code at the address in a register.

```
target = Rs
push_frame(pc + 4)
pc = target
```

The call reads its target before it pushes the frame, so `Rs` may be `lr`, `fp` or `sp` and gives the value it held before the call. `CALL.REG` reaches any address, where `CALL.DIRECT` reaches 128 KiB either way, and calls an address a program computed, such as a far function's or one read from a table, with no closure object to hold it. `JR` jumps to a register's address without a frame (§ 11.4).

**Cycles:** 1  
**Traps:** None

---

## 7. Family 5: Pointer Prefetch

### 7.1 `PREFETCH.REF Rs`

Prefetch the object header and first cache line of the object referenced by `Rs`.

```
if Rs is a ref:
    prefetch(untag_ref(Rs), hint=READ, lines=1)
// else: no-op (prefetching a non-ref is silently ignored)
```

**Cycles:** 1 (non-blocking)  
**Traps:** None

### 7.2 `PREFETCH.FIELD Rs, #field`

Prefetch a specific field of an object.

```
if Rs is a ref:
    addr = untag_ref(Rs) + (field + 1) * 8
    prefetch(addr, hint=READ)
```

**Cycles:** 1 (non-blocking)  
**Traps:** None

### 7.3 `PREFETCH.CDR Rs`

Specialized prefetch for list traversal. Loads the cdr field and prefetches the object it points to.

```
if Rs is a cons ref:
    cdr_addr = untag_ref(Rs) + CDR_OFFSET
    cdr_val = *(cdr_addr)    // this is a load, not a prefetch
    if cdr_val is a ref:
        prefetch(untag_ref(cdr_val), hint=READ, lines=1)
```

This is a **compound instruction**: it both loads the cdr and prefetches the next cons cell. It helps the pipeline stay ahead during list traversal.

**Cycles:** 1 (issues prefetch, cdr load is pipelined)  
**Traps:** None

### 7.4 `GATHER.PREFETCH Rs_base, Rs_index_vec, #count` *(Optional)*

Prefetch a batch of objects from an index vector. For each index `i` in `Rs_index_vec[0..count-1]`, prefetches the object at `Rs_base[i]`.

**Cycles:** `count` (pipelined)  
**Traps:** None  
**Conformance:** OPTIONAL (LM-1 Full)

---

## 8. Family 6: Concurrency & Messaging

### 8.1 `SEND Rs_queue, Rt_value`

Enqueue a tagged word onto a hardware message queue.

```
if Rs_queue is not a fixnum:
    trap TRAP_NOT_FIXNUM
if no route reaches the tile Rs_queue names:
    trap TRAP_NO_ROUTE
if queue_full(Rs_queue):
    trap TRAP_QUEUE_FULL
enqueue(Rs_queue, Rt_value)
```

Queues are identified by a queue descriptor in a register: a fixnum whose value is *tile* × 4 + *queue*, where *queue* picks one of the tile's four queues and *tile* names the tile. A descriptor naming the core's own tile reaches its own queues, and one naming another tile reaches that tile's, so a message can go to any tile a route reaches. No route reaches a negative value, a tile beyond the machine, or, while the network between clusters is a stub, a tile outside the sender's cluster; an emulator of one tile, or of a cluster of tiles, reaches only those it models. Such a descriptor is the program's error rather than a busy queue, so it traps `TRAP_NO_ROUTE` where a full queue traps `TRAP_QUEUE_FULL`. `RECV` and `TRY.RECV` read the same descriptors and trap on them the same way. See [04-soc.md](04-soc.md) § 4.4 for queue topology.

**Cycles:** 1 (fast path, if queue not full)  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_NO_ROUTE`, `TRAP_QUEUE_FULL`

### 8.2 `RECV Rd, Rs_queue`

Dequeue a tagged word from a hardware message queue, waiting for one when the queue is empty.

```
if Rs_queue is not a fixnum:
    trap TRAP_NOT_FIXNUM
if no route reaches the tile Rs_queue names:
    trap TRAP_NO_ROUTE
while queue_empty(Rs_queue):
    stall      // the core runs its other hardware threads meanwhile
Rd = dequeue(Rs_queue)
```

A `RECV` on an empty queue stalls its thread until the queue holds a word, and the core's scheduler gives the thread's turns to the tile's other threads meanwhile ([03-core.md](03-core.md) § 4.1). The thread then runs the `RECV` again, and it takes the word. An empty queue raises no trap; a thread that must not wait polls with `TRY.RECV`.

**Cycles:** 1 (fast path, if message available); a stalled thread waits until a word arrives  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_NO_ROUTE`

### 8.3 `TRY.RECV Rd, Rd2, Rs_queue`

Non-blocking receive. `Rd` gets the value (or `nil`), `Rd2` gets `t` (success) or `nil` (empty). Its descriptor traps as `RECV`'s does, and an empty queue raises no trap.

**Cycles:** 1  
**Traps:** `TRAP_NOT_FIXNUM`, `TRAP_NO_ROUTE`

### 8.4 `CAS.TAGGED Rd, Rs_addr, Rs_expected, Rt_new`

Compare-and-swap on a tagged word in memory.

```
old = *Rs_addr
if old == Rs_expected:
    *Rs_addr = Rt_new
    Rd = t
else:
    Rd = nil
```

If `Rt_new` is a ref and the store crosses generations, the write barrier is also triggered.

**Cycles:** Variable (depends on memory system)  
**Traps:** None (failure is reported via `Rd`)

### 8.5 `FAA Rd, Rs_addr, Rs_delta`

Fetch-and-add on a tagged fixnum in memory.

```
old = *Rs_addr
*Rs_addr = old + Rs_delta    // must both be fixnums; tag bits cancel
Rd = old
```

**Cycles:** Variable  
**Traps:** `TRAP_NOT_FIXNUM` (if old or delta is not a fixnum)

### 8.6 `FENCE.GC`

Memory fence for GC coordination. Ensures all pending stores (including barrier metadata updates) are visible before the GC phase transition.

**Cycles:** Variable (drain store buffer)  
**Traps:** None

---

## 9. Family 7: Region & Bulk Operations (Movement Engine)

These instructions do not execute on the DOP core's pipeline. They enqueue work items to the cluster's **movement engines** (see [04-soc.md](04-soc.md) § 4.2).

### 9.1 `ENQ.SCAN Rs_region, Rd_result`

Enqueue a region for pointer scanning. The movement engine scans all objects in the region, producing a list of pointer fields (for GC tracing).

```
enqueue_to_movement_engine(OP_SCAN, Rs_region)
// Rd_result receives a descriptor for the pending operation
// Completion is signaled via a queue or interrupt
```

**Cycles:** 1 (enqueue), actual work is asynchronous  
**Traps:** `TRAP_ENGINE_BUSY`

### 9.2 `ENQ.COPY Rs_src_region, Rd_dst_region`

Enqueue a region-to-region object copy. The movement engine copies all live objects from `Rs_src_region` to a new region, updating forwarding pointers.

**Cycles:** 1 (enqueue)  
**Traps:** `TRAP_ENGINE_BUSY`

### 9.3 `ENQ.FIXUP Rs_pointer_list, Rs_forwarding_table`

Enqueue a pointer fixup pass. The movement engine walks the pointer list and updates each pointer according to the forwarding table.

**Cycles:** 1 (enqueue)  
**Traps:** `TRAP_ENGINE_BUSY`

### 9.4 `ENQ.COMPACT Rs_region`

Enqueue compaction of a region (slide objects to eliminate fragmentation, update pointers).

**Cycles:** 1 (enqueue)  
**Traps:** `TRAP_ENGINE_BUSY`

### 9.5 `POLL.ENGINE Rd, Rs_descriptor`

Poll the status of an enqueued movement-engine operation.

```
Rd = status of operation Rs_descriptor
// t = complete, nil = still running, fixnum = error code
```

**Cycles:** 1  
**Traps:** None

### 9.6 `AWAIT.ENGINE Rs_descriptor`

Block the current hardware thread until the movement-engine operation completes.

**Cycles:** Variable (yields to other hardware threads)  
**Traps:** None

---

## 10. Family 8: Capability / Safety Mode *(Optional)*

These instructions are available only in **capability mode** (a per-tile or per-thread configuration). They extend the ref model with bounds, permissions, and compartmentalization, inspired by CHERI.

### 10.1 `MKCAP Rd, Rs_ref, Rs_bounds, Rs_perms`

Create a capability from a ref, bounds descriptor, and permission set.

### 10.2 `CHKCAP Rs_cap, #perm`

Check that a capability has the required permission. Trap on failure.

**Traps:** `TRAP_CAPABILITY_VIOLATION`

### 10.3 `NARROW Rd, Rs_cap, Rs_new_bounds`

Narrow a capability's bounds (can only shrink, never expand).

### 10.4 `SEAL Rd, Rs_cap, Rs_type`

Seal a capability, making it opaque and usable only via `CALL` (entry capability).

### 10.5 `UNSEAL Rd, Rs_cap, Rs_type`

Unseal a capability (requires matching type authority).

**Conformance:** All Family 8 instructions are OPTIONAL. Required for LM-1 Full conformance.

**Encoding:** the five instructions share opcode `010101` and are told apart by a func field ([07-encoding.md](07-encoding.md) § 5.9). No implementation has capability mode yet, so each raises `TRAP_UNIMPLEMENTED` for the opcode.

---

## 11. Supplementary: Scalar & Control

These are conventional instructions for systems code, boot, and interfacing.

### 11.1 Arithmetic (Untagged)

`ADD`, `SUB`, `MUL`, `DIV`, `MOD` — operate on raw 64-bit integers, ignoring tags. Used in the runtime, GC, and bootloader where tagged semantics are not wanted.

### 11.2 Bitwise

`AND`, `OR`, `XOR`, `SHL`, `SHR`, `ASR`, `NOT`

### 11.3 Load/Store (Raw)

`LDR Rd, [Rs, #offset]` — load raw 64-bit word  
`STR Rt, [Rs, #offset]` — store raw 64-bit word  
`LDB`, `STB` — byte load/store  
`LDH`, `STH` — halfword (16-bit) load/store  
`LDW`, `STW` — word (32-bit) load/store

The offset is a signed byte offset. Each access clears the address bits below its size, so a doubleword access is 8-byte aligned, and the narrow loads zero-extend what they read. Each width has an opcode of its own ([07-encoding.md](07-encoding.md) § 5.8).

### 11.4 Control Flow

`BR #offset` — unconditional branch  
`BR.T Rs, #offset` — branch if `Rs` is not `nil` and not fixnum `0`  
`BR.NIL Rs, #offset` — branch if `Rs` is `nil`  
`BR.FIX.LT Rs, #offset` — branch if `Rs` is a fixnum below zero  
`BR.FIX.EQ Rs, #offset` — branch if `Rs` is fixnum zero, the word 0  
`BR.FIX.GT Rs, #offset` — branch if `Rs` is a fixnum above zero  
`BR.EQ Rs, #offset` — branch if `Rs` is the word 0  
`JR Rs` — jump to the address in `Rs`, without a frame

Every conditional branch tests one register, and every branch counts its offset in instructions from its own address. A program compares two values first and branches on the result: `CMP.TAGGED` gives `-2`, `0` or `+2` for `BR.FIX.LT`, `BR.FIX.EQ` and `BR.FIX.GT` to test, and `EQ` gives `t` or `nil` for `BR.T` and `BR.NIL`.

### 11.5 Stack Operations

`PUSH Rs` — push to stack  
`POP Rd` — pop from stack  
`PUSH.MULTI #bank, #bitmask` — push multiple registers  
`POP.MULTI #bank, #bitmask` — pop multiple registers

`PUSH` decrements `sp` by 8 and stores `Rs` at the new `sp`; `POP` loads `Rd` from `sp` and increments `sp` by 8. The multiple forms name a bank of 16 registers, bank 0 for `r0`–`r15` and bank 1 for `r16`–`r31`, other banks being reserved, and a 16-bit mask in which bit *i* selects the bank's register *i*. `PUSH.MULTI` pushes the selected registers from the lowest-numbered up, and `POP.MULTI` pops them from the highest-numbered down, so a `POP.MULTI` with the same bank and mask restores what a `PUSH.MULTI` saved.

### 11.6 System

`TRAP #code` — explicit trap (syscall-like)  
`ERET` — return from trap handler to the saved PC  
`ERET.NEXT` — return from trap handler to the saved PC + 4, after the instruction that faulted  
`HALT` — halt the current hardware thread  
`NOP` — no operation  
`TILE.ID Rd` — read current tile ID  
`THREAD.ID Rd` — read current hardware thread ID  
`CYCLE Rd` — read cycle counter

`TRAP #code` with a code below 0x80 raises that code: the thread enters the handler the trap table names for it, and `ERET` resumes after the `TRAP`, as a return from a system call does (§ 12). A code from 0x80 up is a system trap, which the core carries out itself and which no handler sees:

| Code | Name | Effect |
|:----:|------|--------|
| 0x90 | `SET_TRAP_TABLE` | The trap table's base becomes `r1` |
| 0x91 | `SET_TEMPLATE` | Entry `r1` bits 7:0 of the header-template table becomes `r2` ([07-encoding.md](07-encoding.md) § 6) |
| 0x92 | `SET_CARD_BASE` | The card table's base becomes `r1` |
| 0x93 | `SET_CARD_SHIFT` | The card size's log₂ becomes `r1` bits 5:0; it is 6, 64-byte cards, from reset |
| 0x94 | `SET_GEN_BOUNDARY` | The generation boundary becomes `r1`; the barrier counts addresses at or above it as old ([05-memory-gc.md](05-memory-gc.md) § 4) |
| 0x95 | `SET_QUEUE_BASE` | The message queues' base becomes `r1`, which `SYS_INFO` 8 reads back; nothing else uses it yet |

Every other code from 0x80 up is reserved, and a `TRAP` naming one raises `TRAP_UNIMPLEMENTED`, a fault that saves the `TRAP`'s own PC. The emulators give 0x80 to 0x82, 0x84 and 0x9F to their host's console and block device (`design/emulator.md` § 7), a bridge the chip lacks: LM-1's own I/O travels the link to MegaPad.

`TILE.ID`, `THREAD.ID` and `CYCLE` are three of the reads of `SYS_INFO Rd, #sub`, which writes `Rd` from the source its sub-code names:

| Sub | Mnemonic | Reads |
|:---:|----------|-------|
| 0 | `TILE.ID` | The tile's ID |
| 1 | `THREAD.ID` | The hardware thread's ID |
| 2 | `CYCLE` | The cycle counter |
| 3 | `TRAP.CAUSE` | The code of the last trap taken |
| 4 | `TRAP.PC` | The PC the last trap saved |
| 5 | `CARD.BASE` | The card table's base |
| 6 | `CARD.SHIFT` | The card shift |
| 7 | `GEN.BOUNDARY` | The generation boundary |
| 8 | `QUEUE.BASE` | The queue base |
| 9 | `GC.STATUS` | 1 while a movement engine works, otherwise 0 |
| 10 | — | The performance counter that the immediate's bits 4:0 name |
| 11 | — | The number of entries in the scanner's result FIFO |
| 12 | — | The object address of the FIFO's head entry |
| 13 | — | The field index of the head entry |
| 14 | — | The reference of the head entry, popping the entry |

Sub-codes 15 to 31 are reserved. The emulators model no movement engines and no counters, so they read 0 for sub-codes 9 to 14.

### 11.7 Immediate Loads

`LI Rd, #imm16` — `Rd` = `imm16` sign-extended to 64 bits  
`LUI Rd, #imm16` — `Rd` = `imm16` shifted left 16 bits, the other bits zero  
`LI32 Rd, #imm32` — `Rd` = the 32-bit word that follows the instruction, zero-extended; the instruction takes two words, and the PC advances past both

None of them tags its value, so a program writes a fixnum constant pre-tagged.

---

## 12. Trap Code Summary

A trap's code picks its entry in the trap table, whose base `TRAP 0x90` sets: entry *code* is the 64-bit word at the base plus *code* × 8, and it holds the handler's address. The codes below 0x80 vector this way, the faults the table lists and every code an explicit `TRAP` raises, and a handler returns with `ERET` or `ERET.NEXT` ([03-core.md](03-core.md) § 6.3). A thread whose trap finds no table, or a zero entry, stops with the trap standing. The codes from 0x80 up belong to the system traps of § 11.6, which no handler sees.

| Code | Mnemonic | Source |
|:----:|----------|--------|
| 0x01 | `TRAP_NOT_FIXNUM` | Fixnum arithmetic on non-fixnum |
| 0x02 | `TRAP_FIXNUM_OVERFLOW` | Fixnum arithmetic overflow |
| 0x03 | `TRAP_DIVIDE_BY_ZERO` | Division by zero |
| 0x04 | `TRAP_TYPE_MISMATCH` | `CMP.TAGGED` on incompatible types |
| 0x05 | `TRAP_NOT_REF` | Ref-expecting op on non-ref |
| 0x06 | `TRAP_NOT_CONS` | Cons-expecting op on non-cons |
| 0x07 | `TRAP_NOT_CLOSURE` | Closure call on non-closure |
| 0x10 | `TRAP_NURSERY_OVERFLOW` | Nursery allocation failure |
| 0x20 | `TRAP_IC_MISS` | Inline cache miss |
| 0x30 | `TRAP_QUEUE_FULL` | Send to full queue |
| 0x32 | `TRAP_NO_ROUTE` | A queue descriptor naming a tile no route reaches (§ 8.1) |
| 0x40 | `TRAP_ENGINE_BUSY` | Movement engine queue full |
| 0x50 | `TRAP_BARRIER_OVERFLOW` | Write barrier metadata overflow |
| 0x51 | `TRAP_SHARED_STORE` | A store into a shared object of a reference outside the shared heap ([05-memory-gc.md](05-memory-gc.md) § 4.5) |
| 0x60 | `TRAP_CAPABILITY_VIOLATION` | Capability check failure |
| 0x61 | `TRAP_SHARED_REF` | A tagged load from the shared heap that yields a reference outside it ([05-memory-gc.md](05-memory-gc.md) § 12.8) |
| 0x70 | `TRAP_STACK_OVERFLOW` | A stack check finds the stack past its region; no instruction checks a stack's bounds, so the check's explicit `TRAP` raises it |
| 0x7E | `TRAP_UNIMPLEMENTED` | A reserved encoding ([07-encoding.md](07-encoding.md) § 2), an opcode no instruction has, a reserved system trap code, or `ERET` outside a handler |
| 0x7F | `TRAP_USER` | A user program's call into the runtime, by an explicit `TRAP` |

---

## 13. Instruction Encoding Overview

All instructions are fixed-width **32 bits**. The encoding format is detailed in [07-encoding.md](07-encoding.md). A summary:

```
 31    26 25                                  0
┌────────┬────────────────────────────────────┐
│ family │        family-specific fields      │
│ (6 bit)│                                    │
└────────┴────────────────────────────────────┘
```

The 6-bit family field gives 64 top-level opcodes, more than enough for the 8 semantic families + scalar/control ops. Each family uses its 26-bit payload differently (register fields, immediates, etc.).

---

## 14. Pseudocode for Key Operations

### 14.1 `(cons x y)` → `ALLOC.CONS`

```asm
    ALLOC.CONS r3, r1, r2      ; r3 = (cons x y), with x in r1 and y in r2
```

The new cons holds its car and cdr from the start, so no store follows it, and none needs a barrier.

### 14.2 `(car x)` → `LD.CAR`

```asm
    LD.CAR r1, r0              ; r1 = (car r0)
```

### 14.3 `(+ a b)` where a, b might not be fixnums

```asm
    ADD.FIX r3, r1, r2        ; fast path: both fixnums, no overflow
    ; if either is not fixnum: traps to TRAP_NOT_FIXNUM → runtime does generic +
    ; if overflow: traps to TRAP_FIXNUM_OVERFLOW → runtime promotes to bignum
```

The handler finds the `ADD.FIX` at the trap PC, computes the sum, writes it into the instruction's `Rd`, here `r3`, and returns with `ERET.NEXT`, which resumes after the `ADD.FIX` ([03-core.md](03-core.md) § 6.3). A plain `ERET` would run the `ADD.FIX` again, and it would trap again.

### 14.4 Generic function call `(foo obj arg1 arg2)`

```asm
    ; r1 = obj (receiver), r2 = arg1, r3 = arg2, pushed or in regs per ABI
    CALL.IC r1                ; receiver = r1; this instruction's address is the call site
    ; on IC hit: direct call to the cached method
    ; on IC miss: TRAP_IC_MISS → runtime looks up method for foo on obj's class,
    ;             installs with IC.INSTALL, retries
```

### 14.5 Write-barriered mutation `(set-car! pair new-val)`

```asm
    ST.WB r1, #0, r2          ; r1 = pair, field 0 = car, r2 = new-val
    ; hardware handles barrier: if r2 is a young ref and r1 is old, card-mark
```
