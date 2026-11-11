# LM-1 Host Link

LM-1 is a chiplet beside MegaPad. MegaPad lends LM-1 ranges of its external
memory, and LM-1 reads and writes them across a link at MegaPad's own
addresses, so both chips work on one copy of the data. The link also carries
doorbells in both directions and a sideband for reset, identity and boot.

The link is specified on MegaPad's side, in `docs/chiplet-link.md` of the
MegaPad repository, as contract version 1. The two modules here are LM-1's
end of it.

## Modules

| File | Module | Role |
|------|--------|------|
| `link/lm1_link.sv` | `lm1_link` | The link endpoint: memory requests, doorbells, sideband |
| `link/lm1_link_boot.sv` | `lm1_link_boot` | Fetches the image MegaPad names, then hands the memory port to the runtime |

## The endpoint: `lm1_link`

### Link side

Every channel is a valid/ready handshake, and a message transfers in the
cycle in which both are high.

| Channel | Direction | Payload |
|---------|-----------|---------|
| Memory request | LM-1 to MegaPad | `addr`, `wdata`, `wen`, `size`, `len`; one request outstanding at a time |
| Memory response | MegaPad to LM-1 | `rdata`, `code`: 0 OK, 1 REFUSED, 2 MEM_ERROR, 3 CANCELLED; `last` |
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

### Sideband

| Signal | Direction | Meaning |
|--------|-----------|---------|
| `sb_present` | to MegaPad | Tied high: a chiplet is attached |
| `sb_rst_n` | to LM-1 | Low holds LM-1's own logic in reset |
| `sb_stop_req` | to LM-1 | Stop: issue no new request and no new doorbell |
| `sb_stopped` | to MegaPad | The runtime has stopped and no request is in flight |
| `sb_ready` | to MegaPad | LM-1's runtime is up |
| `sb_id_req`, `sb_id_index`, `sb_id_ack`, `sb_id_data` | both | Read of one identity word |
| `sb_boot_addr`, `sb_boot_len` | to LM-1 | The image MegaPad names |
| `sb_boot_arg` | to LM-1 | The address of the control block MegaPad lends, where LM-1's runtime finds its queues |

The identity record answers while LM-1 is held in reset, so it has a reset
of its own, `por_n`. Word 0 is `0x4348_4950_0000_0001`: the ASCII tag `CHIP`
above contract version 1. Word 1 is the device name, `LM1`, first character
in the low byte. Every further word reads zero.

### Local side

| Port | Purpose |
|------|---------|
| `core_rst_n` | Reset for LM-1's own logic: `por_n` and `sb_rst_n` |
| `stop_req`, `runtime_stopped`, `runtime_ready` | The stop handshake and the ready flag, to and from the runtime |
| `boot_addr`, `boot_len`, `boot_arg` | The sideband's boot words |
| `hm_*` | The host-memory port: a request with its burst length, `hm_cancel`, and responses with their code, a `last` mark and a ready input |
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
words, eight by default, and hands each word with its offset to the load
port, which is to write LM-1's own memory. After the last word it raises
`boot_done` and passes the host-memory port through to the runtime.

A length of zero means no image, and `boot_done` rises at once. A response
other than OK stops the fetch and raises `boot_failed`; the runtime's port
then stays closed until MegaPad resets LM-1. A length that is no multiple of
8 is rounded up to whole words.

## Clocking

The link is specified at the level of messages. These modules place both
ends on one clock. A link layer between the dies carries the same handshakes
across the boundary; its choice waits on where LM-1's main memory is placed.

## Tests

`tb/link/run_link_tests.sh` builds and runs two benches with Verilator:

- `lm1_link_tb` drives the two modules from a behavioural MegaPad: the
  identity record in reset, the boot fetch, a refused image, the runtime's
  requests with their latency and codes, cancellation, doorbells, the stop
  handshake and a return to reset.
- `lm1_megapad_link_tb` joins the two modules to MegaPad's own RTL: its port
  bridge, mailbox, external-memory arbiter and controller. It runs when
  `MEGAPAD_ROOT` names a MegaPad checkout, and follows the start order of
  MegaPad's specification from the identity read to the fetched image.

## What remains

- **The load port.** It matches the cluster's DMA request port, which is
  still a stub (see [SoC Integration](06-soc-integration.md)). Until that
  port writes cluster memory, the fetched image goes nowhere.
- **Doorbells and the message queues.** `dbi_*` and `dbo_*` are meant for
  the external ports of a tile's hardware message queue, so a doorbell
  arrives as a message. That wiring is still to do.
- **The runtime's use of the port.** Nothing in the cluster issues
  host-memory requests yet. LM-1's main memory and its collector's traffic
  reach lent memory through this port once the chiplet revision of the spec
  places them.
- **The stop handshake.** `runtime_stopped` and `runtime_ready` come from
  the runtime, which has no such signals yet.
