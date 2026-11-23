# LM-1 Chiplet Module: Plan for Part 8d

**Status:** Accepted, 2026-11-22. Items marked Decided cite the decision;
section 5 records the decisions this plan rests on. All three stages are
built (2026-11-22); section 3 says what each left for later.

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
collection as the reference runs it. The second is new: managed data's
function (class 1 of `docs/chiplet-link.md` § 8) and the queue software at
LM-1's end of the link, which the module serves natively in C++ until
LM-1's runtime runs on the model and serves them itself (decision A).

The shared heap divides between the two. Its policy belongs to the runtime
(`spec/05-memory-gc.md` § 12): the runtime grants allocation regions,
interns shapes, states the bounds of holds, and sequences each collection,
as it sequences the movement engines with `ENQ.SCAN`, `ENQ.COPY` and
`ENQ.FIXUP` on LM-1's own heap (`spec/04-soc.md` § 5.3). Its moving work,
the scanning, copying and fixing up, is done "by the runtime or by engines
given a path to that port, which the chiplet revision decides"
(`spec/05-memory-gc.md` § 12.3, item 8), since the engines reach only a
cluster's shared SRAM and the shared heap lies across the link in
MegaPad's memory. The module's stand-in carries the policy, and models the
moving work as the movement engines given a path to the link (decision G).

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

- **LM-1's side,** in the module, with the moving work done by the model of
  the movement engines that decision G describes: offer `MANAGED` and
  serve `ATTACH`; count handles as roots and rewrite their slots on every
  move; drain each MegaPad
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

The stand-in's minor collection of LM-1's own nursery takes the division
that `spec/04-soc.md` § 4.3 gives the runtime in this stage as well: a
thread whose chunk of the nursery runs out gets another from the nursery's
free space, and the stand-in collects only when none is left. The Python
reference divides its nursery the same way, so the two models still agree
step for step.

Tests are the 27 conformance tests of `docs/managed-data.md` § 15, run
through the real code at both ends in the emulator and the simulator.

*Built.* With `runtime=1` in its configuration the module's stand-in
(`chiplet/src/managed.cpp`) reads the control block, offers managed data
and serves it as this stage lists, and MegaPad's `chiplet.f` is its other
end. MegaPad's `tests/test_lm1_managed_data.py` runs the 27 tests through both:
21 in the emulator and the simulator alike, and the six that need a second
MegaPad core in the emulator, until the simulator has several cores. The
same file checks a moving hold's writes against recovery's stage 0 order,
through a hook that records every word the stand-in writes, and saves and
restores the port while the function serves. The runtime stays off by
default, since the link's shared cases also run on the RTL, where nothing
reads the control block. A step of the module stands for `unit_ns` times
its budget of LM-1's time, set so that LM-1's clock keeps within a small
factor of MegaPad's in both of MegaPad's models.

Recovery's stage 1 is built (2026-11-23), with MegaPad's never-free engine
on MegaPad's side. The stand-in answers a stop only between moves
(`spec/05-memory-gc.md` § 12.9): a hold does its whole `ACTIVE` work within
a step, so a stop MegaPad makes between steps finds `MOVE` reading `NONE`
or `FIXED`, and a test hook, `lm1c_md_stop_at_flip`, makes the request
arrive as `MOVE` comes to read `FLIPPING`; the stand-in then finishes the
hold's moves and rewrites, writes `FIXED`, and writes nothing more, leaving
the hold `ACTIVE`, and LM-1's threads run no further in that step. The
record of the stand-in's writes outlives a reset, so that a recovery test
reads it after MegaPad has stopped LM-1. MegaPad's
`tests/test_lm1_managed_data.py` runs the recovery tests of stage 1 with
the module on both of MegaPad's models. Left for later: recovery's stage 2
and beyond; `KIND`, which the stand-in never needs, since it puts no kinds
of its own in the shared heap; the field kinds of LM-1 code's stores, which
no LM-1 code makes yet; and the stand-in's tasks, which wait for the link's
layer for jobs.

*Estimate: 20 to 35 agent hours for both ends.*

## 4. Testing

The module's own tests run through LM-1's test runner as batches that load
the module through `ctypes` (decision D), so the release's test selection
runs them like LM-1's other batches. The module is one C++ library: it
exports `mp64_chiplet_create` for MegaPad and a small C interface through
which LM-1's own tests drive the core model, and MegaPad attaches the same
library that LM-1's tests load. No test runs against a library older than
its sources. Tests that need MegaPad, from stage 2 on, run as MegaPad or
Akashic test files that attach the built module.

## 4a. Timing and the per-function profile

The module and the Python emulator count cycles by one architectural timing
model, whose table `chiplet/src/timing.h` holds for both, and the
performance counters of `spec/02-isa.md` § 11.6 count by it. The module also
profiles the LM-1 code it runs, as MegaPad's simulator profiles guest words:
with `profile=N` in its configuration it samples each core's current thread
every N cycles of the timing model, names the function holding the PC and
the callers the frame chain names from the symbol table an image carries at
its end, and appends a record of each core's samples to `profile_file`
every `profile_interval` cycles, in the shape of MegaPad's
`guest-words.jsonl`. The profile reads the model and never writes it, so a
profiled run computes the same as an unprofiled one, and with the default
`profile=0` it costs one comparison an instruction. `docs/profile.md`
describes the sampling, the configuration, the symbol table that
`lm1.image` builds and LM-1's compiler names, and the records. The Python
emulator has no profile.

## 5. Decisions

All decided on 2026-11-22.

**A. The runtime stand-in.** The module serves managed data natively, as a
stand-in for LM-1's runtime, which on the chip is Lisp. The stand-in
retires once LM-1's Lisp runtime serves the shared heap on the model and
passes the same 27 conformance tests. Writing the function as runtime code
from the start would have made stage 3 wait for LM-1's runtime and
compiler (Parts 8f and 9e); the stand-in lets MegaPad's side of managed
data, Study's concept map and the recovery work of D13 proceed now, as a
bridge with a stated end.

**B. MegaPad's side of stage 3 is the real code.** Stage 3 builds
MegaPad's managed-data words, part of release plan Part 9a, beside the
module. The conformance tests are written against those words, and a
harness that played MegaPad's part would be a second implementation of
MegaPad's side that the release does not keep. Stage 3 therefore spans
MegaPad and this repository, on a branch in each.

**C. The interface header comes from MegaPad.** The module builds against
`shared/accel/chiplet_module.h` in the workspace's MegaPad checkout, named
as the link testbench names it (`MEGAPAD_ROOT`), and checks
`MP64_CHIPLET_ABI` when it compiles. The interface belongs to MegaPad, a
copy here could drift, and the release's lock file (Part 1b) records which
MegaPad commit a build used.

**D. The module's tests are LM-1 runner batches.** They load the module
through `ctypes`, so the release's test selection runs them as it runs
LM-1's other batches.

**E. Sizes come from the configuration.** The model's sizes come from the
module's configuration string, with the spec's present numbers as
defaults, so no stage waits for the sizing study of D8. The study can
measure Leviathan's load on LM-1 with the module once stage 3 serves
managed data.

**F. One model afterwards.** Once the module passes every test the Python
emulator passes, the module becomes the reference, its C interface serves
the toolchain's needs, and the Python emulator retires by archive tag, so
that one model remains.

**G. The shared heap's moving work is modeled as the movement engines.**
Stage 3 models the scanning, copying and fixing up on the shared heap as
the movement engines of `spec/04-soc.md` § 5.3 given a path to the link,
sequenced by the stand-in's policy. The chiplet revision (release plan
Part 8b) is expected to give the engines that path, and confirms it once
the sizing study of D8 has measured the traffic the moves add to
MegaPad's memory. Under D8's option of keeping LM-1's main memory in
MegaPad's memory, LM-1's own old generation lies across the link too, and
the engines need the path for LM-1's own heap. Should the revision choose
the runtime instead, the stand-in was temporary in any case. The engines'
model follows four rules the spec already sets, which the engines' own
description in `spec/04-soc.md` § 5.3 predates and the chiplet revision
brings into it:

1. Marks stay outside shared objects' headers (`spec/05-memory-gc.md`
   § 12.3, item 12), so the copier takes liveness from a mark table
   rather than a header's mark bit.
2. The scanner reads an object's layout from its header and skips raw
   words, which since 2026-11-20 follow a record's tagged fields
   (`spec/01-object-model.md`); a tag check on every word would read raw
   words as references, on LM-1's own heap as well.
3. References written into shared objects carry zero in bits 63:51
   (`spec/05-memory-gc.md` § 12.1, item 4), using the shared-heap range
   check that tagged stores already make.
4. An engine reports completion only after its last write over the link
   is answered, so the `MOVE` word's `FIXED` follows every rewrite
   (`spec/05-memory-gc.md` § 12.3, items 10 and 11).

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
