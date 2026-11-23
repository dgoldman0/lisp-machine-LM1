# LM-1 Test Gap Analysis & Ideas

_Generated 2026-02-17 — based on RTL review of all modules, FSM states, opcodes, and GC engine logic._

---

## Priority Legend

| Tag | Meaning |
|-----|---------|
| **P0** | Potential correctness bug found during review — fix or prove safe |
| **P1** | Entire instruction or major feature has zero test coverage |
| **P2** | Feature is partially tested but important edge cases are missing |
| **P3** | Requires infrastructure beyond the current single-core testbench |

---

## P0 — Potential Bugs to Investigate

### P0-3  I-Cache has no invalidation mechanism

`lm1_icache.sv` stores lines in a direct-mapped cache with tag+valid
bits, but there is no `FENCE.I`, `IC.INVAL`, or any other way to flush
a cache line.  If code is patched via `STR` to an address that's already
cached, the CPU executes the stale cached copy.

**Where:** `lm1_icache.sv` — no state/port for invalidation.
**Risk:** JIT compilation, dynamic code loading, and self-modifying code
are all silently broken.
**Action:** Either (a) add an I-cache flush instruction, or (b) document
that code regions are immutable and enforce it architecturally.

### P0-12  Cluster accesses wait for neither the grant nor the response

A core raises `xbar_req_valid` for the one cycle its LSU issues an access
to a cluster address and never looks at `xbar_req_ready`, which its own
TODO beside the assignment notes. The LSU treats every access as a
one-cycle SRAM access: it finishes a store at once and takes a load's data
the next cycle, and nothing reads `xbar_resp_valid`. The crossbar grants
one port a cycle in round-robin order, and none in the cycle after it
grants a read, while that read's answer is pending. A request it does not
grant is lost whether it writes or reads: an ungranted store writes
nothing, and an ungranted load returns 0, the value of a response line that
carries no answer for its port, with no trap in either case. One tile
alone never meets a busy crossbar, since its accesses are at least two
cycles apart; two tiles using cluster memory at once do.

**Where:** `lm1_cpu.sv` (the crossbar request), `lm1_lsu.sv`
(`LSU_WAIT_RD`), `lm1_crossbar.sv`.
**Risk:** Lost stores and loads of 0 under any contention for a
cluster's shared memory.
**Action:** release plan Part 8e: cores wait for the grant, and each
response returns to the request it answers, under the cluster memory model
Part 8b states first.

### P0-13  Stores narrower than a word to cluster memory write the whole word

The LSU makes `STB`, `STH` and `STW` by copying the stored byte, halfword
or 32-bit word into every lane of the 64-bit word and setting the byte
enables of its own lanes, and the tile's SRAM honours them. The crossbar's
ports carry no byte enables, and `lm1_cluster.sv` ties the shared SRAM's
enables to all ones, so such a store to a cluster address writes the whole
word: a byte store leaves the byte in all eight lanes, a halfword store its
halfword in all four, and a 32-bit store its word in both halves. The
write barrier's card mark is a byte store of `0xFF`, so a card table in
cluster memory would get eight cards marked at once.

**Where:** `lm1_cpu.sv` (crossbar request signals), `lm1_crossbar.sv`,
`lm1_cluster.sv` (`a_be`).
**Risk:** Strings, bytevectors and narrow fields in cluster memory lose
their neighbouring bytes.
**Action:** release plan Part 8e: the crossbar carries byte enables, as
Part 8b's memory model states for narrow stores.

### P0-14  CAS.TAGGED and FAA are atomic only within a tile

Each runs as a load and then a store (`S_MEM`, then `S_ATOMIC_STORE`), as
`rtl/doc/04-memory.md` records. In a tile's own SRAM nothing comes between
them, since one instruction holds the core from its load to its store.
Through the crossbar they are two requests, and another tile's request
can be granted between them: two tiles' `FAA`s on one word of cluster
memory can read the same old value and lose an increment, and a
`CAS.TAGGED` can store over a value another tile wrote after the
comparison.

**Where:** `lm1_control.sv` (`OP_CAS_TAGGED`, `OP_FAA_FENCE`),
`lm1_crossbar.sv`.
**Risk:** Locks and counters shared between tiles are unsound.
**Action:** release plan Part 8e: the atomics run inside the crossbar as
single requests, as Part 8b's memory model states them for tiles and for
a later network between clusters.

### P0-15  The core switches threads only between instructions

The scheduler in `lm1_control.sv` hands the core to the next active
thread when an instruction ends, on the move to `S_FETCH`. An instruction
that waits holds the core for its whole wait, on the LSU, the crossbar, a
queue's port, the GC engines or the 64-cycle divider, so one thread's
memory wait stalls all four, where spec/03-core.md section 4.2 promises
that the other threads hide it. A `RECV` on an empty queue is the one wait
that ends its instruction's turn, unretired, so the scheduler moves on and
the thread runs the `RECV` again at its next turn. No bench can show
either case yet:
`thread_active` comes out of reset with thread 0 alone, and nothing starts
threads 1 to 3 (P3-1).

**Where:** `lm1_control.sv` (the thread switch in the `always_ff` block).
**Risk:** Memory latency, small in tile SRAM, long across a cluster or
the link, is never hidden.
**Action:** release plan Part 8e: a thread waiting on memory gives the
core to another ready thread, shown by a four-thread bench at realistic
memory latency.

### P0-16  CALL.IC and TAILCALL.IC never hit

`S_HDR_WAIT` presents the call site and the receiver's shape to the IC
table, whose lookup is combinational, and moves on to `S_IC_DISPATCH`,
which reads `ic_hit` a clock later, when the lookup's inputs are back at
their defaults and `lu_valid` is low.  Every `CALL.IC` therefore misses,
and a handler that installs the entry and returns with `ERET` runs the
call again into the same miss, for ever.  Bench program 55 takes one miss
and passes the call with `ERET.NEXT`; the models hit, as
`emu/tests/test_timing.py` shows.

**Where:** `lm1_control.sv` (`S_HDR_WAIT`, `S_IC_DISPATCH`).
**Risk:** Every dynamic dispatch on the RTL traps.
**Action:** look the entry up in the state that reads `ic_hit`, or
register the hit and target in `S_HDR_WAIT`, with a bench program whose
handler installs the entry and whose retry hits.

### P0-17  CAS.TAGGED runs no write barrier

spec/02-isa.md § 8.4 has `CAS.TAGGED` run the write barrier when it
stores a reference across generations, and both models do.  The RTL's
`CAS.TAGGED` stores through `S_ATOMIC_STORE` and never reaches
`S_BARRIER_CHECK`, so a young reference it stores into an old object marks
no card, and counter 3 counts no filtered store for it (bench program 36
shows the counts differ).

**Where:** `lm1_control.sv` (`OP_CAS_TAGGED`, `S_ATOMIC_STORE_W`).
**Risk:** A minor collection misses an old-to-young reference that a
`CAS.TAGGED` stored, and frees a live object.
**Action:** send a successful `CAS.TAGGED` of a reference through
`S_BARRIER_CHECK`, with a bench program that checks the card.

---

## P1 — Entirely Untested Instructions

### P1-2  SEND to a full queue

Bench test 35 covers SEND and RECV round trips, two of the four queues
kept apart, FIFO order, TRY.RECV on empty and holding queues, and the
descriptor traps, and tb/cluster test 02 covers RECVs that wait for a
word. No bench fills a queue, so the path by which a SEND to a full queue
traps `TRAP_QUEUE_FULL` has no test.

**Test idea:** 512 SENDs into one queue, a 513th that traps and adds
nothing, then 512 RECVs that return the words in order.

---

## P2 — Partially Tested Features, Missing Edge Cases

### P2-2  CALL.IC / TAILCALL.IC / IC.INSTALL (inline cache dispatch)

Bench program 55 takes one miss; nothing else of the inline-cache dispatch
system is tested on the RTL, and hits wait on P0-16.

**Test ideas:**
- IC.INSTALL shape + target, then CALL.IC on matching shape → hits
- CALL.IC on non-matching shape → `TRAP_IC_MISS`
- IC miss handler installs, retry hits
- IC table fill + FIFO eviction (install 65 entries, verify oldest evicted)
- TAILCALL.IC (no frame push variant)

### P2-4  Nested traps

A trap inside a trap handler overwrites `trap_pc` and `trap_cause` —
no trap context stack.

**Test ideas:**
- Allocate inside a trap handler → trigger nursery overflow → nested trap
- Verify ERET returns to the correct location (it won't — this is a
  known architectural limitation, but should be documented as such)
- If nested traps are "undefined behavior," add a trap-inside-trap
  detection that halts or panics

### P2-5  ERET when not in trap

Executing ERET with `in_trap == 0` should trigger `TRAP_UNIMPLEMENTED`,
which is 0x7E now, below 0x80, so a handler can count it and the program
go on.

**Test ideas:**
- Execute ERET and ERET.NEXT without a preceding trap → verify each traps
  at its own PC

### P2-6  Trap table entry = 0 (halt-on-unhandled-trap)

When the loaded trap handler address is 0, the CPU halts.

**Test ideas:**
- Trigger a trap for a vector that was never configured → verify HALT
- Set a vector to 0 explicitly, trigger it → verify HALT

### P2-12 CMP.TAGGED cross-type comparison

spec/02-isa.md section 3.2 settles the rules: a cons ref (tag 011) and a
general ref (tag 001) are different types, so comparing them traps
`TRAP_TYPE_MISMATCH`, and two words of one other primary tag compare by
identity, giving fixnum 0 when equal and +2 when not. The RTL checks
`a[2:0] == b[2:0]` as the models do, and no bench test covers the rules.

**Test ideas:**
- cons-ref vs general-ref → verify trap fires
- NIL vs NIL → same tag, identity match → returns fixnum 0
- Same ref vs same ref → returns fixnum 0
- Different refs, same tag → returns fixnum +2

---

## P3 — Needs Infrastructure Work

### P3-1  Multi-thread (FGMT)

Every test runs single-threaded (only thread 0 active).  Thread
activation, context switching, per-thread NP/NL, interleaved execution,
and thread halting with remaining active siblings are entirely untested.

**Known concerns:**
- CSRs (trap table, card base, etc.) are shared, not per-thread
- `in_trap` flag is shared — nested trap from a different thread corrupts it
- Thread activation mechanism needs to be identified (is there a
  `THREAD.START` instruction or a CSR write?)

**Infrastructure needed:** Testbench support for activating threads
(probably via a system trap or init sequence).

### P3-2  GC engine integration (scanner, copier, fixup)

The scanner, copier, and fixup engines in `rtl/gc/` are never
instantiated in any test.  They need shared SRAM and the memory arbiter.

**Known concerns in scanner:**
- If a region starts mid-object, the scanner interprets payload words as headers
- If the arbiter delays a grant, `mem_rd_valid` timing may be off

**Known concerns in copier:**
- Already-forwarded objects (gc_bits == 0xFF) are skipped with 8-byte
  advance, but a multi-word object's payload words are then individually
  scanned — a payload word that looks like a header is misinterpreted

**Known concerns in fixup:**
- `update_ref()` zeroes the top 8 bits (gc_bits) — fine for < 48-bit addrs
- Self-fixup (fixing up the same region being copied) is an edge case

**Infrastructure needed:** Cluster-level testbench with `lm1_gc_engine_top`,
SRAM, and the memory arbiter.  Alternatively, a standalone GC-engine-only
testbench.

### P3-3  FENCE.GC stall behavior

`gc_engine_busy` is hardwired to 0 in the testbench, so `FENCE.GC`
completes in one cycle always.

**Infrastructure needed:** A toggle-able `gc_engine_busy` signal, or
integration with the real GC engine top.

### P3-4  Scanner result FIFO (SYS_SCAN_COUNT, SYS_SCAN_POP_REF, etc.)

All scanner FIFO inputs are hardwired to 0 in the testbench.

**Infrastructure needed:** Either scanner engine integration or a separate
FIFO stimulus mechanism in the testbench.

### P3-5  Cluster / crossbar / tile modules

`tb/cluster` runs programs on a two-tile `lm1_cluster.sv`, and the
MegaPad link bench joins a cluster to MegaPad's RTL through its memory
write port, so the tiles, the message router and the write port are
tested. The crossbar arbiter, multi-tile memory routing and clock gating
are not: no program uses a cluster's shared memory, and P0-12 to P0-14
record what such a program would meet.

**Infrastructure needed:** cluster programs that use the shared memory,
once Part 8e's crossbar keeps the memory model.

### P3-6  External message queue injection

Within a cluster the message router drives the external side of the
message queues, and `tb/cluster` tests it.  Messages from beyond the
cluster wait on the NoC, a stub, so a request naming a tile outside the
cluster is answered as one no route reaches.

---

## Suggested Test Ordering

If implementing these tests, a natural order based on dependency and
value:

1. **P0-12 to P0-15** (the crossbar and thread switching) — release plan
   Part 8e builds them under the memory model Part 8b states, with their
   tests
2. **P1-2** (SEND to a full queue) — one bench program
3. **P2-5, P2-6, P2-12** — short programs on the trap paths and
   `CMP.TAGGED`
4. **P0-16, P0-17, P2-2** (CALL.IC's hits and CAS.TAGGED's barrier) —
   the dispatch system
5. **P2-4** — nested traps
6. **P0-3, P3-*** — need a design decision or new infrastructure
