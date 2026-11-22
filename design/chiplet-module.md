# LM-1 Chiplet Module: Plan for Part 8d

**Status:** Proposed, 2026-11-22. Items marked Decided cite the decision;
section 5 lists the decisions still to take.

---

## 1. What the module is

The chiplet module is LM-1's model as MegaPad's emulator and native
simulator load it: a C++ library, built from this repository, that
implements MegaPad's chiplet module interface (`shared/accel/chiplet_module.h`
in MegaPad, ABI 2). The interface carries what the link carries
(MegaPad's `docs/chiplet-link.md`): LM-1's memory requests, which MegaPad's
port bridge checks against the ranges MegaPad lends exactly as the hardware
bridge does; MegaPad's writes into memory LM-1 lends; doorbells in both
directions; reset, release and stop; the sideband's ready and stopped
signals; the identity record; and save and restore of the model's state. A
module calls MegaPad only from inside its `step`, starts no thread and reads
no clock, so its results depend only on the machine's state and the order of
steps. MegaPad loads a module library with `MegapadSystem.attach_chiplet` in
the emulator and `HostedChipletPorts.attach_module` in the simulator.

Three rules govern the work (Decided, release plan Part 8d):

- The module is written in accelerated C++ from the start.
- The Python emulator in `emu/` stays the reference model until the module
  is complete. It may be wrong too, so the spec is the referee: when the
  two disagree, the spec settles it, the wrong one is fixed, and a test is
  added.
- Managed data's collector side, which counts MegaPad's handles as roots,
  replays MegaPad's logged writes and waits for open windows, is built
  first in this module, and the Python emulator does not implement it
  (Decided 2026-11-20). Its collector keeps the write order recovery relies
  on from the start: a moved object is copied in full before anything
  points to it, it moves only into space free when the hold began, a word
  records the hold's progress, marks stay out of object headers, and shapes
  keep fixed IDs.

## 2. What the module models

The module models two things, and the plan treats them differently.

**LM-1's hardware.** Cores with their four hardware threads, tiles with
their SRAM regions, nursery and message queues, the cluster's shared SRAM
with its card tables, the instruction set as `spec/02-isa.md` defines it,
the traps and trap table, and LM-1's end of the link as `rtl/doc/08-host-link.md`
describes the RTL's endpoint: the boot fetch, the boot argument, doorbells,
the sideband, and the write port through which MegaPad's tile engines store
results into memory LM-1 lends (decision 38 of the release plan).

**Stand-ins for LM-1's runtime.** On the chip, LM-1's runtime is LM-1
software, written in Lisp, and the model stands in for it in two places.
The first is the reference's own: the Python emulator answers
`TRAP_NURSERY_OVERFLOW` with a native minor collection, which
`spec/02-isa.md` § 4.1 gives the runtime, and the module keeps that
collection as the reference runs it. The second is new. LM-1's spec gives
the shared heap's collector to the runtime as well (`spec/05-memory-gc.md`
§ 12): the runtime grants allocation regions, interns shapes and states the
bounds of holds, and the movement engines of `spec/04-soc.md` § 5.3 reach
only a cluster's shared SRAM, so work on the shared heap, which lies across
the link in MegaPad's memory, goes through the host-memory port. Until
LM-1's runtime runs on the model and serves the shared heap itself, the
module serves it natively in C++: managed data's function (class 1 of
`docs/chiplet-link.md` § 8) and the queue software at LM-1's end of the
link. Decision A in section 5 is whether this reading of the 2026-11-20
decision holds.

## 3. Stages

### Stage 1. The core model

The core model executes LM-1 code as the Python reference does, in C++:
tagged words, memory, decoding, every instruction family the reference
implements, traps, allocation, the write barrier, the nursery collector,
the four hardware threads in round-robin, and a cluster of tiles with
message routing. The emulator-level console and block traps are built for
LM-1's own tests; inside MegaPad they have no device and trap, until the
link carries the chiplet's I/O (release plan Part 1c). The reference's
executable model is about 2,350 lines of Python (`execute.py`, `decode.py`,
`word.py`, `memory.py`, `core.py`, `traps.py`); the compiler and assembler
stay host tools that build the images the model runs.

The model's sizes, among them tiles, threads and SRAM regions, come from the
module's configuration string, with the spec's present numbers as defaults
(decision E).

Tests run the same programs on both models and compare them: the programs
of the reference's 177 tests, the RTL testbench's 43 programs against their
expected registers, so that the RTL, the reference and the module agree on
each, and generated instruction streams compared state for state.

*Estimate: 8 to 14 agent hours.*

### Stage 2. The module and LM-1's end of the link

The core model becomes a chiplet module: `mp64_chiplet_create`, the identity
record, reset, release with the boot image and the boot argument, stop,
doorbells both ways, the sideband, `step` with a budget of the model's own
units, `lent_write` into the memory LM-1 lends, and save and restore of the
whole state. The boot fetch reads LM-1's image from MegaPad's memory
through the host's memory requests, eight words to a request as the RTL's
`lm1_link_boot` reads it. Until the `LEND` entries of `docs/chiplet-link.md`
§ 8 are built, the ranges LM-1 lends come from the configuration string.

Tests attach the module to MegaPad's emulator and simulator, boot a small
LM-1 image from MegaPad's memory, exchange doorbells, take a `TACC.STORE`
into LM-1's lent memory, and round-trip save and restore. The link's
conformance tests are shared with the RTL's link testbenches, `lm1_link_tb`
and `lm1_megapad_link_tb`, as Part 8g asks, so the module and both RTL sides
pass the same cases.

*Estimate: 5 to 10 agent hours.*

### Stage 3. Managed data at both ends

The module's runtime stand-in serves managed data, and MegaPad's side is
built beside it (decision B):

- **LM-1's side,** in the module: offer `MANAGED` and serve `ATTACH`; count
  handles as roots and rewrite their slots on every move; drain each MegaPad
  core's store log and replay its entries through the barrier; collect only
  inside holds that wait for open windows, within `HOLD_LIMIT` and
  `HOLD_GAP`, with `NOTICE` for an overdue window; grant, retire and walk
  allocation regions, with a reserve and `GROW` and `RETURN`; rescan
  MegaPad's region objects when their region retires; skip raw words in
  scan, copy and fixup; refuse or publish references through the two
  shared-heap traps; intern shapes and enforce their field kinds; serve
  `COLLECT` in its three modes with the two test hooks; validate what
  MegaPad wrote; keep one host-memory request in flight at a time; and keep
  recovery's stage 0 write order throughout.
- **MegaPad's side,** in MegaPad's chiplet system module (release plan Part
  9a): the general words for the control block and queue pairs
  (`docs/chiplet-interconnect-plan.md` § 9), and managed data's `OBJ-` words
  with their records in lent memory, windows, store logs, regions and
  failure handling (`docs/managed-data.md`).

Tests are the 27 conformance tests of `docs/managed-data.md` § 15, run
through the real code at both ends in the emulator and the simulator.

*Estimate: 20 to 35 agent hours for both ends.*

## 4. Testing

The module's own tests run through LM-1's test runner as batches that load
the module through `ctypes` (decision D), so the release's test selection
runs them like LM-1's other batches. The core model is one C++ library
compiled into two products: the chiplet module, and a Python extension the
differential tests drive. Tests that need MegaPad, from stage 2 on, run as
MegaPad or Akashic test files that attach the built module.

## 5. Decisions to take

**A. The runtime stand-in.** The 2026-11-20 decision builds managed data's
collector side first in this module. Because the spec gives that collector
to LM-1's runtime, which on the chip is Lisp, the module's version is a
native stand-in for runtime software.
- *Options:* (a) build the stand-in, and retire it once LM-1's Lisp runtime
  serves the shared heap on the model and passes the same 27 tests; (b)
  build only the hardware model now, and write the collector side as LM-1
  runtime code, which waits for LM-1's runtime and compiler (Parts 8f and
  9e).
- *Recommendation:* (a). It lets MegaPad's side of managed data, Study's
  concept map and the D13 recovery work proceed now, and the stand-in is a
  bridge with a stated end, which the workspace rules allow.
- *Cost:* the stand-in is stage 3's LM-1 side; under (b) that work moves
  into the runtime instead and stage 3 waits on Parts 8f and 9e.

**B. MegaPad's side of stage 3 is the real code.**
- *Options:* (a) build MegaPad's managed-data words (part of Part 9a) with
  stage 3; (b) drive the module from a test harness that plays MegaPad's
  part.
- *Recommendation:* (a). The conformance tests are written against
  MegaPad's words, and a harness would be a second implementation of
  MegaPad's side that the release does not keep.
- *Cost:* stage 3 spans MegaPad and this repository, on a branch in each.

**C. Where the module finds `chiplet_module.h`.**
- *Options:* (a) build against the workspace's MegaPad checkout, named as
  the link testbench names it (`MEGAPAD_ROOT`), with a compile-time check
  of `MP64_CHIPLET_ABI`; (b) keep a copy of the header here.
- *Recommendation:* (a). The interface belongs to MegaPad, and a copy could
  drift; the release's lock file (Part 1b) records which MegaPad commit a
  build used.
- *Cost:* none beyond the build script.

**D. How the module's tests reach the test selection.**
- *Options:* (a) LM-1 runner batches that load the module through `ctypes`;
  (b) pytest files, which the selection's LM-1 driver does not run today.
- *Recommendation:* (a), which needs no change to the selection.
- *Cost:* none.

**E. Sizes before D8.** The model's sizes come from the configuration
string, with the spec's present numbers as defaults, so stage 1 does not
wait for the sizing study of D8. The study can measure Leviathan's load on
LM-1 with the module once stage 3 serves managed data.

**F. One model afterwards** (for later). Once the module passes every test
the Python emulator passes, the module becomes the reference, its Python
extension serves the toolchain's needs, and the Python emulator retires by
archive tag, so that one model remains.

## 6. After this plan

- Sandbox jobs and borrowed tile work need their part of LM-1's layer
  specified before they are built (`docs/chiplet-link.md` § 10, release
  plan Part 8c), and `LEND` comes with borrowed tile work.
- LM-1's Lisp runtime, running on the module, takes the shared heap over
  from the stand-in (Parts 8f, 9e).
- A restarted LM-1 taking the heap back joins the release if stage 3 lands
  early, and is otherwise the first work after it (release plan Part 9d).

| Stage | What it builds | Estimate | What it unblocks |
|-------|----------------|----------|------------------|
| 1 | The core model | 8–14 h | Stages 2 and 3; the module as differential oracle |
| 2 | The module and the link | 5–10 h | MegaPad attaches LM-1; Part 8g's shared link tests |
| 3 | Managed data, both ends | 20–35 h | Parts 9a, 9d and Study's concept map |

The estimates are agent hours, by judgment from the sizes of the reference,
the interface and the managed-data specification; none is measured.
