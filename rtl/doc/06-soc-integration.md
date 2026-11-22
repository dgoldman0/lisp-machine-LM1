# LM-1 SoC Integration

## Hierarchy

The LM-1 has a three-level physical hierarchy:

```
SoC (full chip)
 └── Cluster × 8
      ├── Tile × 8 (per cluster)
      │    ├── CPU Core
      │    │    ├── Decoder
      │    │    ├── Regfile (128×64)
      │    │    ├── ALU + Divider
      │    │    ├── Branch Evaluator
      │    │    ├── LSU
      │    │    ├── Control FSM
      │    │    ├── I-Cache (8 KiB)
      │    │    ├── IC Table (64-entry)
      │    │    ├── Template Table (256-entry)
      │    │    ├── Message Queue (4×512)
      │    │    └── Perf Counters (8×64-bit)
      │    ├── Tile SRAM (256 KiB, single-port)
      │    └── Clock Gate
      ├── Crossbar (8-port round-robin)
      ├── Shared SRAM (2 MiB, dual-port)
      ├── GC Engine Top
      │    ├── Scanner
      │    ├── Copier
      │    └── Fixup
      └── Memory write port (the host link's writes)
```

**Totals (full chip):**
- 64 tiles, 256 hardware threads
- 16 MiB tile-local SRAM (64 × 256 KiB)
- 16 MiB cluster-shared SRAM (8 × 2 MiB)
- 8 GC engine sets (scanner + copier + fixup per cluster)

---

## Tile Wrapper

**File:** `lm1_tile.sv`

The tile wraps a CPU core, its local SRAM, and a clock gate. It provides
a uniform interface for the cluster to instantiate.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `XLEN` | 64 | Word width |
| `MEM_DEPTH_LOG2` | 15 | Local SRAM depth (32K words = 256 KiB) |

### Ports

| Group | Signals | Direction | Purpose |
|-------|---------|-----------|---------|
| Clock/Reset | `clk`, `rst_n` | in | System clock and active-low reset |
| Config | `cfg_tile_id[7:0]` | in | Unique tile identifier |
| External Memory | `ext_mem_en/we/addr/wdata` | in | Program loading / debug access |
| Debug | `dbg_reg_addr`, `dbg_reg_data` | in/out | Register read while halted |
| Status | `halted` | out | All threads stopped |
| Crossbar | `xbar_req_*`, `xbar_resp_*` | out/in | Cluster SRAM access |
| GC Command | `gc_cmd_*` | out/in | GC engine command interface |
| Queues | `noc_mq_*` | in/out | The tile's queues, for the message router |
| Remote Queue | `rmq_*` | out/in | The core's SEND and RECV on other tiles' queues |
| Clock Gate | `cg_enable` | in | Clock gating control |

### Clock Gating

The tile instantiates `lm1_clock_gate`:

```systemverilog
lm1_clock_gate u_cg (
    .clk_in  (clk),
    .enable  (cg_enable),
    .clk_out (gated_clk)
);
```

When `cg_enable = 0`, the gated clock stops and the CPU core freezes.
This allows power management — idle tiles can be clock-gated without
affecting other tiles in the cluster.

The clock gate implementation uses an AND-latch topology:

```systemverilog
always_latch begin
    if (~clk_in) en_latched = enable;  // Latch on low phase
end
assign clk_out = clk_in & en_latched;  // Gate on high phase
```

This is the standard glitch-free clock gating cell used in ASIC design.

### Tile ID Propagation

The cluster assigns each tile a unique 8-bit ID:

```
tile_id = {cluster_id[4:0], tile_index[2:0]}
```

This is readable by software via `SYS_INFO` with `rs1 = SYS_TILE_ID`.
The core compares it with a queue descriptor's tile to tell its own
queues from another tile's.

---

## Cluster

**File:** `lm1_cluster.sv`

The cluster is the primary unit of scaling. It contains 8 tiles, a crossbar,
shared SRAM, and the GC engine complex.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLUSTER_ID` | 0 | Cluster identifier (0-7) |
| `TILES_PER_CLUSTER` | 8 | Number of tiles |
| `TILE_MEM_LOG2` | 15 | Per-tile SRAM (256 KiB) |
| `CLUSTER_MEM_LOG2` | 18 | Shared SRAM (2 MiB) |

### Tile Instantiation

Tiles are instantiated in a generate loop:

```systemverilog
for (genvar t = 0; t < TILES_PER_CLUSTER; t++) begin : gen_tiles
    lm1_tile #(
        .MEM_DEPTH_LOG2(TILE_MEM_LOG2)
    ) u_tile (
        .cfg_tile_id({CLUSTER_ID[4:0], t[2:0]}),
        .cg_enable(1'b1),    // always enabled (software-controlled)
        ...
    );
end
```

Each tile gets a unique `cfg_tile_id` composed of the cluster ID and tile
index within the cluster.

### Crossbar Connection

The crossbar connects all 8 tiles' cluster-bound memory requests to the
shared SRAM port A:

```
Tile[0..7].xbar_req_* ──▶ Crossbar ──▶ Shared SRAM Port A
Tile[0..7].xbar_resp_* ◀── Crossbar ◀── Shared SRAM Port A
```

### GC Engine Connection

The GC engine top connects to shared SRAM port B, and the memory write port
uses the cycles the engines leave free:

```
GC Engine Top     ──▶ Shared SRAM Port B (read/write, first)
Memory write port ──▶ Shared SRAM Port B (write, when the engines leave it free)
```

This dual-port arrangement (port A for CPU, port B for GC) is central to
the concurrent GC design. The memory write port is described below.

### GC Command Arbitration

The cluster contains a priority-encoded arbiter for GC commands from tiles:

```systemverilog
for (int i = 0; i < TILES_PER_CLUSTER; i++) begin
    if (tile_gc_cmd_valid[i] && !gc_cmd_valid_out) begin
        gc_cmd_valid_out     = 1'b1;
        gc_cmd_op_out        = tile_gc_cmd_op[i];
        gc_cmd_arg0_out      = tile_gc_cmd_arg0[i];
        gc_cmd_arg1_out      = tile_gc_cmd_arg1[i];
        gc_cmd_arg2_out      = tile_gc_cmd_arg2[i];
        tile_gc_cmd_ready[i] = gc_cmd_ready_in;
    end
end
```

Tile 0 has highest priority; tile 7 has lowest. In practice, GC is
coordinated by a runtime scheduler that ensures only one tile issues
GC commands at a time, making priority order irrelevant.

### Scanner Result FIFO

The cluster contains a 128-entry FIFO that buffers scanner results:

```
Scanner ──▶ scan_res_valid/obj/field/ref ──▶ FIFO ──▶ (runtime reads)
                                              ▲
                                     scan_res_ready
                                   (backpressure when full)
```

| Parameter | Value |
|-----------|-------|
| Depth | 128 entries |
| Width | 144 bits (64-bit obj + 16-bit field + 64-bit ref) |
| Pointers | 7-bit write/read pointers (wrap at 128) |

### GC Read Valid Tracking

The cluster tracks SRAM port B read latency:

```systemverilog
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        gc_mem_rd_valid_r <= 1'b0;
    else
        gc_mem_rd_valid_r <= gc_mem_rd_en && !gc_mem_wr_en;
end
```

Reads are valid one cycle after issue, but only if no write preempted the
port on the same cycle.

### Message Router

Each tile's core reaches another tile's queues through its remote queue
port: `rmq_req` with `rmq_write`, `rmq_tile`, `rmq_queue` and
`rmq_wdata`, driven from the core's registers while SEND or RECV waits,
and answered by `rmq_ack` with `rmq_ok` and `rmq_rdata`. The router grants
each tile's external write port and external read port to one request a
cycle, starting after the requester it granted last, and answers in the
cycle it grants: `rmq_ok` says that a SEND's word was taken or that a RECV
took one. A request naming a tile outside the cluster is answered at once
without `rmq_ok`, since the NoC is a stub. The requests come from
registers, and the answers from the grants and the queues' registers, so
no combinational path runs from one core's decisions to another's. The
tile's clock gate opens for the router's accesses (`noc_mq_wr_en`,
`noc_mq_rd_en`), so a halted tile's queues still take and give words.

`tb/cluster/run_cluster_tests.sh` builds a two-tile cluster bench,
`tb/cluster/lm1_cluster_tb.sv`, with Verilator and runs the programs that
`tb/cluster/gen_cluster_tests.py` writes for its two tiles: 64 words from
each tile into one queue of tile 1, the two tiles emptying it together,
a word tile 0 takes from tile 1's queue, a remote queue found empty, and a
tile outside the cluster. The bench counts the cycles in which a tile's
core waited on its own queue because the router used it, and the test
requires some of each kind.

---

## Crossbar

**File:** `lm1_crossbar.sv`

### Architecture

An 8-to-1 round-robin arbiter that multiplexes tile memory requests onto
a single SRAM port.

```
       req[0]  req[1]  req[2]  ...  req[7]
         │       │       │           │
         ▼       ▼       ▼           ▼
    ┌────────────────────────────────────┐
    │         Round-Robin Arbiter        │
    │                                    │
    │  rr_ptr ──▶ scan from ptr ──▶ grant│
    │             wrap around            │
    └──────────────┬─────────────────────┘
                   │
                   ▼
           ┌──────────────┐
           │  SRAM Port A │
           │  (shared)    │
           └──────┬───────┘
                  │
                  ▼
           resp_data ──▶ resp_valid[granted]
```

### Parameters

| Parameter | Default |
|-----------|---------|
| `N_TILES` | 8 |
| `XLEN` | 64 |
| `MEM_DEPTH_LOG2` | 18 |

### Round-Robin Arbitration

```systemverilog
always_comb begin
    grant   = '0;
    granted = 1'b0;
    for (int k = 0; k < N_TILES; k++) begin
        int idx = (rr_ptr + k) % N_TILES;
        if (req_valid[idx] && !granted) begin
            grant   = idx[$clog2(N_TILES)-1:0];
            granted = 1'b1;
        end
    end
end
```

The pointer advances to `(grant + 1) % N_TILES` on each grant, ensuring
fairness over time.

### Timing

| Phase | Cycle | Action |
|-------|-------|--------|
| Request | 0 | Tile asserts `req_valid`, `req_addr`, `req_we`, `req_wdata` |
| Grant | 0 | Arbiter selects winner, drives SRAM port A |
| SRAM access | 0-1 | SRAM reads in 1 cycle (writes immediate) |
| Response | 1 | `resp_valid[grant]` asserts with `resp_data` |

Minimum latency: 2 cycles (request + SRAM read latency).
Maximum latency: 9 cycles (8 tiles ahead in round-robin + SRAM).

### Response Routing

Responses are broadcast to all tiles, but `resp_valid` is only asserted
for the granted tile. Other tiles ignore the data.

```systemverilog
always_ff @(posedge clk) begin
    resp_valid <= '0;
    if (granted) begin
        resp_valid[grant] <= 1'b1;
        resp_data         <= sram_rdata;
    end
end
```

---

## Message Queues

**File:** `lm1_msg_queue.sv`

Each CPU core has 4 hardware message queues, primarily used for inter-tile
communication.

### Parameters

| Parameter | Value |
|-----------|-------|
| Queues per core | 4 |
| Depth per queue | 512 entries |
| Entry width | 64 bits (tagged word) |

### Interface

```
Core write (SEND):   wr_en, wr_id[1:0], wr_data[63:0] → wr_ready, wr_full
Core read  (RECV):   rd_en, rd_id[1:0] → rd_data[63:0], rd_valid, rd_empty
External write:      ext_wr_en, ext_wr_id[1:0], ext_wr_data[63:0] → ext_wr_ready
External read:       ext_rd_en, ext_rd_id[1:0] → ext_rd_data[63:0], ext_rd_valid
Status:              q_empty[3:0], q_full[3:0]
```

`wr_ready` and `rd_valid` say that the core's word is taken this cycle, and
`wr_full` and `rd_empty` that the selected queue is full or empty.

### Priority

When the core and an external port use the same queue in one cycle, the
external port goes first: `ext_wr_ready` is "not full" and `ext_rd_valid`
"not empty", and the core's access waits a cycle, with `wr_ready` or
`rd_valid` low while `wr_full` or `rd_empty` is low too. The cluster's
message router can therefore answer another tile in the cycle it grants.

### ISA Integration

| Instruction | Queue Op | Behavior |
|-------------|----------|----------|
| `SEND Rs, Rt` | Write | Push `Rt` to the queue the descriptor in `Rs` names. Trap `QUEUE_FULL` if it is full, or if no route reaches its tile. |
| `RECV Rd, Rs` | Read (blocking) | Pop a word of the queue `Rs` names into `Rd`. Trap `QUEUE_EMPTY` if it is empty, or if no route reaches its tile. |
| `TRY.RECV Rd, Rd2, Rs` | Read (non-blocking) | If a word: `Rd = value`, `Rd2 = VAL_T`. Else: `Rd = VAL_NIL`, `Rd2 = VAL_NIL`. |

A queue descriptor is a fixnum whose value is tile × 4 + queue
(spec/02-isa.md § 8). The control FSM takes a word that is not a fixnum
as its own value, as both models do. A descriptor naming the core's own
tile reaches its queues in `S_EXECUTE`, and waits in `S_SEND_WAIT` or
`S_RECV_WAIT` while an external port holds the queue. Any other
descriptor goes out through the core's remote queue port from
`S_SEND_WAIT` or `S_RECV_WAIT`, and a negative value names no tile, so
SEND traps `QUEUE_FULL` and RECV finds the queue empty. The single-core
bench answers every remote request as one no route reaches.

### Usage Model

Queues carry messages between tiles. A typical pattern:

```
; Producer, tile 0: send r1 to queue 0 of tile 1
LI   r9, 8              ; descriptor: fixnum 4 = tile 1, queue 0
SEND r9, r1

; Consumer, tile 1: poll its own queue 0
LI   r9, 8
poll:
TRY.RECV r2, r3, r9
BR.NIL r3, poll         ; r3 == nil: no message yet
; r2 has the message
```

Within a cluster the message router joins the cores' remote queue ports
to the tiles' external queue ports; messages to other clusters wait on
the NoC, which is a stub.

---

## Template Table

**File:** `lm1_tmpl_table.sv`

A 256-entry × 64-bit lookup table of pre-formed header words, used by
allocation instructions to avoid runtime header construction.

### Interface

| Signal | Direction | Width | Purpose |
|--------|-----------|-------|---------|
| `rd_idx` | in | 8 | Template index to read |
| `rd_data` | out | 64 | Template header word |
| `wr_en` | in | 1 | Write enable |
| `wr_idx` | in | 8 | Template index to write |
| `wr_data` | in | 64 | Template header word to store |

### Usage

The runtime populates the template table at startup using `TRAP 0x91`
(SET_TEMPLATE). Each entry contains a complete header word with:

- `gc_bits` = 0 (newly allocated object)
- `shape_id` = the class/type identifier
- `size` = default object size
- `hdr_sub` = header subtype
- `tag` = `111` (header)

During allocation, the CPU reads the template in a single cycle, patches
the size field if needed (for variable-length objects), and writes it to
the new object's first word.

---

## IC Table

**File:** `lm1_ic_table.sv`

A 64-entry fully-associative inline cache for method dispatch.

### Parameters

| Parameter | Value |
|-----------|-------|
| Entries | 64 |
| Entry | `(callsite_pc, shape_id) → target_address` |
| Lookup | Combinational (parallel match) |
| Replacement | FIFO (circular pointer `next_wr`) |

### Interface

| Signal | Direction | Width | Purpose |
|--------|-----------|-------|---------|
| `lookup_en` | in | 1 | Lookup request |
| `lookup_pc` | in | 64 | Callsite PC |
| `lookup_shape` | in | 32 | Object shape ID |
| `hit` | out | 1 | Cache hit |
| `target` | out | 64 | Cached target address |
| `install_en` | in | 1 | Install new entry |
| `install_pc` | in | 64 | Callsite PC |
| `install_shape` | in | 32 | Shape ID |
| `install_target` | in | 64 | Target address |

### Lookup Logic

```systemverilog
hit = 0;
for (int i = 0; i < N; i++) begin
    if (valid[i] && entries[i].pc == lookup_pc 
                 && entries[i].shape == lookup_shape) begin
        hit    = 1;
        target = entries[i].target;
    end
end
```

All 64 entries are checked in parallel (fully associative). This is
area-expensive but provides O(1) lookup latency.

### Replacement Policy

New entries overwrite the slot at `next_wr`, which advances circularly:

```systemverilog
always_ff @(posedge clk) begin
    if (install_en) begin
        entries[next_wr].pc     <= install_pc;
        entries[next_wr].shape  <= install_shape;
        entries[next_wr].target <= install_target;
        valid[next_wr]          <= 1'b1;
        next_wr                 <= (next_wr + 1) % N;
    end
end
```

This is a FIFO policy — oldest entry is evicted first. Not optimal for
skewed access patterns, but simple and predictable.

---

## Performance Counters

**File:** `lm1_perf_counters.sv`

8 × 64-bit hardware performance counters.

| Index | Name | What It Counts |
|-------|------|---------------|
| 0 | Allocations | Number of successful ALLOC/ALLOC_CONS/ALLOCV/ALLOC_CLOSURE |
| 1 | Bytes allocated | Cumulative bytes allocated (from template size) |
| 2 | Barrier fires | Write barriers that detected cross-gen stores |
| 3 | Barrier filtered | Write barriers that were filtered (no cross-gen) |
| 4 | IC hits | Inline cache lookup hits |
| 5 | IC misses | Inline cache lookup misses |
| 6 | Nursery overflows | Times NP exceeded NL (triggering GC) |
| 7 | GC busy cycles | Clock cycles with any GC engine active |

Read via `SYS_INFO` with `rs1 = SYS_PERF_CTR` and `imm16[4:0]` = counter index.

---

## Clock Gating

**File:** `lm1_clock_gate.sv`

Standard AND-latch clock gate cell:

```systemverilog
module lm1_clock_gate (
    input  logic clk_in,
    input  logic enable,
    output logic clk_out
);
    logic en_latched;
    always_latch begin
        if (~clk_in) en_latched = enable;
    end
    assign clk_out = clk_in & en_latched;
endmodule
```

The latch captures `enable` on the **falling edge** of the clock (when
`clk_in` is low). The AND gate then gates the clock output. This ensures
no glitches on the gated clock:

```
clk_in:    ‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾\____/‾‾‾‾
enable:    ‾‾‾‾‾‾‾‾‾‾\____________________________
en_latched:‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\________________________  (latched on falling edge)
clk_out:   ‾‾‾‾\____/‾‾‾‾\____________________________  (clean cutoff)
```

Each tile has one clock gate instance. The `cg_enable` input is currently
hardwired to `1'b1` in the cluster, but can be controlled by software or
a power management unit.

---

## NoC Stub and Memory Write Port

### NoC Interface

The cluster exposes a stub port for future inter-cluster communication:

```systemverilog
input  logic         noc_in_valid,
input  logic [63:0]  noc_in_data,
output logic         noc_in_ready,    // = 1'b1 (always accept)
output logic         noc_out_valid,   // = 1'b0 (never sends)
output logic [63:0]  noc_out_data,    // = '0
input  logic         noc_out_ready
```

Currently: accepts and discards incoming NoC packets; never sends. The NoC
would connect clusters into a mesh or ring network.

### Memory Write Port

```systemverilog
input  logic         dma_wr_valid,
output logic         dma_wr_ready,
input  logic [63:0]  dma_wr_addr,     // byte address in the shared SRAM
input  logic [63:0]  dma_wr_data,
output logic         dma_wr_error     // with ready: nothing was written
```

The host link writes the shared SRAM through this port, one 64-bit word
per accepted cycle: first the runtime image that the boot fetch reads from
MegaPad, then the stores MegaPad makes into memory LM-1 lends it
([Host Link](08-host-link.md)). A word is written at the clock edge that
takes it, so every access a tile or an engine makes from the next cycle
reads it. An address that is not a multiple of 8, or that lies beyond the
shared SRAM, is taken with `dma_wr_error` and writes nothing, so a write
never wraps onto another word.

The port shares SRAM port B with the GC movement engines, which go first:
an engine neither waits for the port nor repeats an access it drops. Each
engine follows every access with a cycle in which it awaits its answer, so
the engines leave port B free at least every other cycle, and a word waits
at most one cycle behind them. The tiles' port A is untouched.

---

## RTL File Map

| Layer | File | Module | Lines |
|-------|------|--------|-------|
| **Core** | `core/lm1_pkg.sv` | Package (types, constants) | ~505 |
| | `core/lm1_decoder.sv` | `lm1_decoder` | ~285 |
| | `core/lm1_regfile.sv` | `lm1_regfile` | ~61 |
| | `core/lm1_alu.sv` | `lm1_alu` | ~409 |
| | `core/lm1_branch.sv` | `lm1_branch` | ~89 |
| | `core/lm1_lsu.sv` | `lm1_lsu` | ~241 |
| | `core/lm1_control.sv` | `lm1_control` | ~2,216 |
| | `core/lm1_tmpl_table.sv` | `lm1_tmpl_table` | ~46 |
| | `core/lm1_ic_table.sv` | `lm1_ic_table` | ~76 |
| | `core/lm1_msg_queue.sv` | `lm1_msg_queue` | ~133 |
| | `core/lm1_perf_counters.sv` | `lm1_perf_counters` | ~65 |
| | `core/lm1_icache.sv` | `lm1_icache` | ~181 |
| | `core/lm1_cpu.sv` | `lm1_cpu` | ~630 |
| **Tile** | `tile/lm1_tile.sv` | `lm1_tile` | ~154 |
| **Cluster** | `cluster/lm1_cluster.sv` | `lm1_cluster` | ~325 |
| | `cluster/lm1_crossbar.sv` | `lm1_crossbar` | ~112 |
| **GC** | `gc/lm1_gc_engine_top.sv` | `lm1_gc_engine_top` | ~216 |
| | `gc/lm1_gc_scanner.sv` | `lm1_gc_scanner` | ~169 |
| | `gc/lm1_gc_copier.sv` | `lm1_gc_copier` | ~214 |
| | `gc/lm1_gc_fixup.sv` | `lm1_gc_fixup` | ~178 |
| **Tech** | `tech/lm1_sram_sp.sv` | `lm1_sram_sp` | ~45 |
| | `tech/lm1_sram_dp.sv` | `lm1_sram_dp` | ~65 |
| | `tech/lm1_clock_gate.sv` | `lm1_clock_gate` | ~35 |
| | | **Total** | **~6,500** |
