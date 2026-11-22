# LM-1 Host Link

LM-1 is a chiplet beside MegaPad. MegaPad lends LM-1 ranges of its external
memory, and LM-1 reads and writes them across a link at MegaPad's own
addresses, so both chips work on one copy of the data. In the other
direction LM-1 lends MegaPad ranges of its own memory, into which MegaPad's
tile engines store their accumulator images. The link also carries
doorbells in both directions and a sideband for reset, identity and boot.

The link is specified on MegaPad's side, in `docs/chiplet-link.md` of the
MegaPad repository, as contract version 1. The two modules here are LM-1's
end of it.

## Modules

| File | Module | Role |
|------|--------|------|
| `link/lm1_link.sv` | `lm1_link` | The link endpoint: memory requests, MegaPad's writes into memory LM-1 lends, doorbells, sideband |
| `link/lm1_link_boot.sv` | `lm1_link_boot` | Fetches the image MegaPad names into LM-1's memory, then hands the memory ports to the runtime and the link |

## The endpoint: `lm1_link`

### Link side

Every channel is a valid/ready handshake, and a message transfers in the
cycle in which both are high.

| Channel | Direction | Payload |
|---------|-----------|---------|
| Memory request | LM-1 to MegaPad | `addr`, `wdata`, `wen`, `size`, `len`; one request outstanding at a time |
| Memory response | MegaPad to LM-1 | `rdata`, `code`: 0 OK, 1 REFUSED, 2 MEM_ERROR, 3 CANCELLED; `last` |
| Write | MegaPad to LM-1 | `addr`, `data` (one 64-byte unit), `len`, `zeros`; one write outstanding at a time |
| Write answer | LM-1 to MegaPad | `code`, with the same four values |
| Doorbell in | MegaPad to LM-1 | `src`, the ID of the participant that signalled |
| Doorbell out | LM-1 to MegaPad | `dst`, the ID of the participant to signal |

`lk_rq_cancel` abandons the outstanding request. MegaPad still answers it,
with CANCELLED unless its response was already waiting.

A request with `len` = n asks for a burst of n + 1 consecutive 64-bit words.
A read burst is one request and n + 1 responses, the final one marked
`last`. A write burst is n + 1 request messages, each with its word, and one
response. MegaPad admits or refuses a burst whole, and a response other than
OK ends it.

An address is MegaPad's address of the access. MegaPad's port bridge checks
it against the ranges lent to LM-1 and answers REFUSED for anything outside
them, so LM-1 can reach nothing else of MegaPad's.

### MegaPad's writes into memory LM-1 lends

MegaPad's one writer is the tile engines' `TACC.STORE`, which sends a
256-byte image as one write. A write covers `len` + 1 units of 64 bytes
from `addr`, LM-1's own address, followed by `zeros` units of zeros. It is
`len` + 1 messages, one for each unit, and LM-1 takes `addr`, `len` and
`zeros` from the first.

LM-1 admits a write when its address is aligned to 64 bytes and one range
lent with the write right contains every byte the write covers, its zero
units included; a write across two ranges is refused even where they
adjoin. The check runs once, in the cycle after LM-1 takes the first
message, so the whole write is admitted or refused, and a refused write
leaves LM-1's memory untouched.

An admitted write passes through a buffer of one unit. The held unit's
eight words go to the memory write port, one in each cycle the port takes
one. LM-1 takes the next unit from the link in the cycle the port takes the
held unit's last word, so units pass back to back, and the zero units
follow as words of zeros. With a memory that takes a word every cycle, a
four-unit write takes 35 cycles from its first message to its answer: one
to take it, one to check it, 32 for the words and one for the answer.
Joined to MegaPad's RTL, a U8 `TACC.STORE` takes 39 cycles from the image
stage's request to its response, with both ends on one clock.

| Answer | When |
|--------|------|
| OK | Every word is written. The memory write port makes a word it takes visible to LM-1's own accesses from the next cycle, and the answer leaves in the cycle after the last word, so every byte can be read by then |
| REFUSED | The write is misaligned or lies outside every lent range. LM-1 answers before it takes a second unit and writes nothing |
| MEM_ERROR | The memory failed a word. The words before it are written |
| CANCELLED | MegaPad cancelled the write. The words the memory took before the cancel are written |

An answer before the last unit ends the write, and MegaPad sends no more of
it. `lk_wq_cancel` abandons the outstanding write: LM-1 lets the memory
take the word it is offering, stops, and answers CANCELLED. A cancel that
comes before LM-1 has taken the write's first message is answered
CANCELLED too, because MegaPad awaits one answer for every write it has
begun and holds `cancel` until that answer comes. LM-1 returned to reset
answers nothing, and MegaPad then ends the write with a fault for its
store.

MegaPad bounds each phase of a write to 255 cycles: a unit waiting to be
taken, and the answer waiting to come. LM-1 takes each unit within the
eight cycles its predecessor's words need, and the answer phase of a
`TACC.STORE` is at most 32 words. Behind the cluster's memory write port,
where a word waits at most one cycle behind the GC engines, both phases
stay well inside the bound. A write whose last data unit was followed by
more than about 30 zero units could not be answered within it at one word
a cycle; no writer sends one.

### The ranges LM-1 lends

LM-1's runtime lends MegaPad ranges of LM-1's memory through the lend port
and announces each one to MegaPad in a `LEND` entry on its control queue
(MegaPad's `docs/chiplet-link.md` §8). The link keeps the ranges and checks
every write against them; MegaPad keeps no copy.

A range is a base, an exclusive limit and the write right. A lend operation
installs the range at `lend_index` in one step from `lend_base`,
`lend_limit` and `lend_write`, and a write right of zero withdraws it. An
index of `RANGES` or beyond is ignored, and `lend_ranges` reports `RANGES`,
so the runtime sizes its records from the hardware it runs on.

The port takes an operation only while no write is held. A write in
progress therefore keeps the ranges it was admitted under, and a
withdrawal completes only after MegaPad's writes into the range have ended:
once the port has taken it, nothing more lands in the range. LM-1's own
reset clears every range.

The runtime installs a range before it sends the `LEND` that announces it,
so MegaPad never stores into a range LM-1 does not yet admit. To take a
range back it sends `LEND` with rights of zero, waits for MegaPad's reply,
which comes once MegaPad's own writes into the range have ended, and then
withdraws the range at the lend port. A withdrawal at the port alone is
safe for LM-1, because every later write into the range is refused
unwritten, but MegaPad's stores into it then fault.

`RANGES` is a parameter, two by default. A range serves one function's
writes, and one function writes into LM-1 today, the tile engines'
`TACC.STORE`. The second range lets the runtime lend a replacement before it
withdraws the range it replaces, so that function is never without one.
Each range costs two 64-bit registers and two comparators.

A range may hold plain data only, such as the bytes of a pinned bytevector
or a region outside the collected heap, so that MegaPad can store numbers
and never a word LM-1 would read as a reference
([LM1-SPEC-05](../../spec/05-memory-gc.md) §11).

### Sideband

| Signal | Direction | Meaning |
|--------|-----------|---------|
| `sb_present` | to MegaPad | Tied high: a chiplet is attached |
| `sb_rst_n` | to LM-1 | Low holds LM-1's own logic in reset |
| `sb_stop_req` | to LM-1 | Stop: issue no new request and no new doorbell |
| `sb_stopped` | to MegaPad | The runtime has stopped, and nothing is in flight: no request of LM-1's and no write of MegaPad's |
| `sb_ready` | to MegaPad | LM-1's runtime is up |
| `sb_id_req`, `sb_id_index`, `sb_id_ack`, `sb_id_data` | both | Read of one identity word |
| `sb_boot_addr`, `sb_boot_len` | to LM-1 | The image MegaPad names |
| `sb_boot_arg` | to LM-1 | The address of the control block MegaPad lends, where LM-1's runtime finds its queues |

The identity record answers while LM-1 is held in reset, so it has a reset
of its own, `por_n`. Word 0 is `0x4348_4950_0000_0001`: the ASCII tag `CHIP`
above contract version 1. Word 1 is the device name, `LM1`, first character
in the low byte. Every further word reads zero.

A stop leaves MegaPad's writes flowing: they are MegaPad's requests, not
LM-1's, and land in ranges that are still lent. `sb_stopped` falls while one
is held and rises again once it has been answered.

### Local side

| Port | Purpose |
|------|---------|
| `core_rst_n` | Reset for LM-1's own logic: `por_n` and `sb_rst_n` |
| `stop_req`, `runtime_stopped`, `runtime_ready` | The stop handshake and the ready flag, to and from the runtime |
| `boot_addr`, `boot_len`, `boot_arg` | The sideband's boot words |
| `hm_*` | The host-memory port: a request with its burst length, `hm_cancel`, and responses with their code, a `last` mark and a ready input |
| `lend_*` | The lend port: install or withdraw one range; `lend_ranges` reports how many the table holds |
| `mw_*` | The memory write port: MegaPad's writes as 64-bit words at LM-1's addresses, each held until taken; `mw_error` with `mw_ready` means the memory failed the word |
| `dbi_*` | A doorbell from MegaPad, offered with its source ID |
| `dbo_*` | A doorbell to MegaPad, taken with its destination ID |

A request on the host-memory port reaches the link in the cycle it is
raised, and its response returns in the cycle MegaPad gives it, so the
endpoint adds no cycle to an access. When the link is busy for a cycle the
endpoint holds the request and presents it until MegaPad takes it. The
further words of a write burst pass straight through as MegaPad takes them,
and a response that ends the burst early returns the port to idle.

## The boot fetch: `lm1_link_boot`

MegaPad loads LM-1's runtime image into a range it lends read-only and names
it by address and length. When LM-1 leaves reset, `lm1_link_boot` reads the
image through the host-memory port in bursts of up to `BURST_WORDS` 64-bit
words, eight by default, and writes each word into LM-1's memory through
the memory write port, from `LOAD_BASE` on. After the last word it raises
`boot_done` and hands both ports over: the host-memory port to the runtime,
and the memory write port to the link, which writes MegaPad's stores
through it.

A length of zero means no image, and `boot_done` rises at once. A response
other than OK, or a word the memory fails, stops the fetch and raises
`boot_failed`; a burst still under way is cancelled and its remaining
responses are taken, so the link returns to idle. The runtime's port and
the link's writes then stay closed until MegaPad resets LM-1. A length that
is no multiple of 8 is rounded up to whole words.

The link has no write to pass while the fetch runs. The ranges it admits
are lent by the runtime, which starts after the fetch, and LM-1's reset
clears them, so every write MegaPad makes before then is refused unwritten.

## The memory LM-1 writes

The memory write port reaches LM-1's memory through a cluster's memory
write port ([SoC Integration](06-soc-integration.md)), whose addresses are
byte addresses in the cluster's shared SRAM; a word beyond the SRAM fails.
The benches join the link to one cluster. A top level that maps the link's
addresses onto LM-1's clusters waits on where LM-1's main memory is placed.

## Clocking

The link is specified at the level of messages. These modules place both
ends on one clock. A link layer between the dies carries the same handshakes
across the boundary; its choice waits on where LM-1's main memory is placed.
It has to keep each handshake end to end, or mark the first message of each
write and burst: an answer or a cancel can end a write before its last unit,
and LM-1 takes the next message it receives as the start of a new write.

## Tests

`tb/link/link_cases.txt` lists the link's conformance cases once, for
every implementation of LM-1's end: the identity record, the boot fetch,
its bursts and its failures, the boot argument, a fetch that waits for a
withdrawn stop, the ranges LM-1 lends and MegaPad's images written into
them with their zero units, the refusals, a withdrawal, a reset that clears
the ranges, doorbells both ways and the stop handshake. Each harness reads
the list when it runs and performs each operation its own way: both benches
through `tb/link/link_cases.svh`, and the chiplet module, the C++ model that
MegaPad's emulator and simulator attach, through LM-1's test batch
`chiplet_link` (`emu/tests/test_chiplet_link.py`).

`tb/link/run_link_tests.sh` builds and runs two benches with Verilator.
Each runs the shared cases first, then those only it can run:

- `lm1_link_tb` drives the two modules from a behavioural MegaPad, with a
  behavioural LM-1 memory behind the memory write port: the local side's
  signals in reset, the runtime's requests with their latency and codes,
  cancellation, the doorbells' handshakes, MegaPad's writes into lent
  memory (the cycle a refusal comes in, back-to-back units, a failing and a
  slow memory, a slow writer, cancels before, during and between units, and
  a withdrawal while a write is held), the stop handshake with a write in
  flight, a return to reset during one, and a word the memory fails during
  the fetch.
- `lm1_megapad_link_tb` joins the two modules, with a cluster of LM-1's as
  their memory, to MegaPad's own RTL: its port bridge, mailbox,
  external-memory arbiter and controller, and the tile engines' image
  stage. It runs when `MEGAPAD_ROOT` names a MegaPad checkout. Beside the
  shared cases it shows MegaPad's records of refused requests, the
  runtime's requests and bursts through the bridge, the units each format
  sends and the bytes in place at LM-1's answer, a doorbell rung after a
  store reaching LM-1 only once the image is in its memory, a tile's own
  loads reading the image back, and the stores MegaPad refuses or cancels
  itself.

## What remains

- **Doorbells and the message queues.** `dbi_*` and `dbo_*` are meant for
  the external ports of a tile's hardware message queue, so a doorbell
  arrives as a message. The cluster's message router drives those ports
  now, so a doorbell joins its requesters. That wiring is still to do. The
  chiplet module wires them so: a doorbell arrives as the fixnum of its
  source on queue 0 of the first tile, and the link rings each participant
  whose ID LM-1 code sends as a fixnum to queue 1 of that tile.
- **The runtime's use of the ports.** Nothing in the cluster issues
  host-memory requests or lend operations yet, and the runtime has no
  `LEND` entries to send. LM-1's main memory and its collector's traffic
  reach lent memory through the host-memory port once the chiplet revision
  of the spec places them.
- **LM-1's top level.** Wiring the link into LM-1's clusters, and choosing
  which cluster's memory a link address reaches, wait on where LM-1's main
  memory is placed.
- **Contention at the cluster's port B.** No bench runs the GC engines
  while the link writes; that a word waits at most one cycle behind them
  follows from the engines' state machines and is not measured.
- **The stop handshake.** `runtime_stopped` and `runtime_ready` come from
  the runtime, which has no such signals yet.
