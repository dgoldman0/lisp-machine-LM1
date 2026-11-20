// ============================================================================
// LM-1 Cluster Top Module
//
// Instantiates:
//   - 8 tiles (each with DOP core, tile SRAM, queues, perf counters)
//   - 8-port non-blocking crossbar
//   - Cluster shared SRAM (2 MiB = 256K × 64-bit words)
//   - GC movement engine top (scanner + copier + fixup)
//   - a memory write port into the shared SRAM, through which the host
//     link writes the boot image and MegaPad's stores into lent memory
//   - NoC uplink stub
//
// Parameters:
//   CLUSTER_ID       — unique cluster ID within the SoC
//   TILES_PER_CLUSTER — tiles per cluster (default 8)
//   TILE_MEM_LOG2    — per-tile SRAM size (log2 words, default 15)
//   CLUSTER_MEM_LOG2 — cluster shared SRAM (log2 words, default 18 = 256K words = 2 MiB)
// ============================================================================
module lm1_cluster
    import lm1_pkg::*;
#(
    parameter int CLUSTER_ID        = 0,
    parameter int TILES_PER_CLUSTER = 8,
    parameter int TILE_MEM_LOG2     = 15,   // 32K words = 256 KiB per tile
    parameter int CLUSTER_MEM_LOG2  = 18    // 256K words = 2 MiB shared
)
(
    input  logic               clk,
    input  logic               rst_n,

    // --- Per-tile status ---
    output logic [TILES_PER_CLUSTER-1:0] tile_halted,

    // --- Per-tile external memory port (initial program load) ---
    input  logic [TILES_PER_CLUSTER-1:0] ext_mem_en,
    input  logic [TILES_PER_CLUSTER-1:0] ext_mem_we,
    input  logic [TILE_MEM_LOG2-1:0]     ext_mem_addr  [0:TILES_PER_CLUSTER-1],
    input  logic [XLEN-1:0]             ext_mem_wdata [0:TILES_PER_CLUSTER-1],
    output logic [XLEN-1:0]             ext_mem_rdata [0:TILES_PER_CLUSTER-1],

    // --- Per-tile debug ---
    input  logic [REG_IDX_W-1:0]        dbg_reg_addr  [0:TILES_PER_CLUSTER-1],
    output logic [XLEN-1:0]             dbg_reg_data  [0:TILES_PER_CLUSTER-1],

    // --- NoC uplink (stub — one port for the whole cluster) ---
    output logic               noc_tx_valid,
    output logic [XLEN-1:0]   noc_tx_data,
    /* verilator lint_off UNUSEDSIGNAL */  // stub: NoC not implemented
    input  logic               noc_tx_ready,
    input  logic               noc_rx_valid,
    input  logic [XLEN-1:0]   noc_rx_data,
    /* verilator lint_on UNUSEDSIGNAL */
    output logic               noc_rx_ready,

    // --- Memory write port into the shared SRAM ---
    // One 64-bit word per accepted cycle, at its byte address in the shared
    // SRAM.  A word taken (valid and ready) is written at that clock edge,
    // so every access the tiles and engines make from the next cycle reads
    // it.  An address that is unaligned or beyond the SRAM is taken with
    // dma_wr_error and writes nothing.
    input  logic               dma_wr_valid,
    output logic               dma_wr_ready,
    input  logic [XLEN-1:0]   dma_wr_addr,
    input  logic [XLEN-1:0]   dma_wr_data,
    output logic               dma_wr_error
);

    // ---------------------------------------------------------------
    // Per-tile wires to crossbar
    // ---------------------------------------------------------------
    logic [TILES_PER_CLUSTER-1:0]     xbar_req_valid;
    logic [TILES_PER_CLUSTER-1:0]     xbar_req_we;
    logic [XLEN-1:0]                 xbar_req_addr  [0:TILES_PER_CLUSTER-1];
    logic [XLEN-1:0]                 xbar_req_wdata [0:TILES_PER_CLUSTER-1];
    logic [TILES_PER_CLUSTER-1:0]     xbar_req_ready;
    logic [XLEN-1:0]                 xbar_resp_data [0:TILES_PER_CLUSTER-1];
    logic [TILES_PER_CLUSTER-1:0]     xbar_resp_valid;

    // Per-tile GC engine command — arbitrated into single gc_cmd
    logic [TILES_PER_CLUSTER-1:0]     tile_gc_cmd_valid;
    logic [3:0]                       tile_gc_cmd_op    [0:TILES_PER_CLUSTER-1];
    logic [XLEN-1:0]                 tile_gc_cmd_arg0  [0:TILES_PER_CLUSTER-1];
    logic [XLEN-1:0]                 tile_gc_cmd_arg1  [0:TILES_PER_CLUSTER-1];
    logic [XLEN-1:0]                 tile_gc_cmd_arg2  [0:TILES_PER_CLUSTER-1];
    logic [TILES_PER_CLUSTER-1:0]     tile_gc_cmd_ready;
    logic                             gc_busy;

    // Scanner result FIFO — shared outputs to all tiles, pop from any tile
    logic [TILES_PER_CLUSTER-1:0]     tile_scan_fifo_pop;
    logic                             scan_fifo_pop_any;

    // ---------------------------------------------------------------
    // Tile instances
    // ---------------------------------------------------------------
    genvar t;
    generate
        for (t = 0; t < TILES_PER_CLUSTER; t++) begin : gen_tiles
            lm1_tile #(
                .TILE_MEM_LOG2 (TILE_MEM_LOG2),
                .TILE_ID       (CLUSTER_ID * TILES_PER_CLUSTER + t)
            ) u_tile (
                .clk            (clk),
                .rst_n          (rst_n),
                .halted         (tile_halted[t]),
                .pc_out         (),                // unused at cluster level
                .cycle_count    (),
                // External memory
                .ext_mem_en     (ext_mem_en[t]),
                .ext_mem_we     (ext_mem_we[t]),
                .ext_mem_addr   (ext_mem_addr[t]),
                .ext_mem_wdata  (ext_mem_wdata[t]),
                .ext_mem_rdata  (ext_mem_rdata[t]),
                // Debug
                .dbg_reg_addr   (dbg_reg_addr[t]),
                .dbg_reg_data   (dbg_reg_data[t]),
                // Crossbar
                .xbar_req_valid (xbar_req_valid[t]),
                .xbar_req_we    (xbar_req_we[t]),
                .xbar_req_addr  (xbar_req_addr[t]),
                .xbar_req_wdata (xbar_req_wdata[t]),
                .xbar_req_ready (xbar_req_ready[t]),
                .xbar_resp_data (xbar_resp_data[t]),
                .xbar_resp_valid(xbar_resp_valid[t]),
                // GC engine command
                .gc_cmd_valid   (tile_gc_cmd_valid[t]),
                .gc_cmd_op      (tile_gc_cmd_op[t]),
                .gc_cmd_arg0    (tile_gc_cmd_arg0[t]),
                .gc_cmd_arg1    (tile_gc_cmd_arg1[t]),
                .gc_cmd_arg2    (tile_gc_cmd_arg2[t]),
                .gc_cmd_ready   (tile_gc_cmd_ready[t]),
                .gc_engine_busy (gc_busy),
                // Scanner result FIFO (shared, broadcast to all tiles)
                .scan_fifo_count     (scan_fifo_count),
                .scan_fifo_head_obj  (scan_fifo_head_obj),
                .scan_fifo_head_field(scan_fifo_head_field),
                .scan_fifo_head_ref  (scan_fifo_head_ref),
                .scan_fifo_pop       (tile_scan_fifo_pop[t]),
                // NoC message port — tied off for now (NoC stub)
                .noc_mq_wr_en   (1'b0),
                .noc_mq_wr_id   (2'b0),
                .noc_mq_wr_data ({XLEN{1'b0}}),
                .noc_mq_wr_ready(),
                .noc_mq_rd_en   (1'b0),
                .noc_mq_rd_id   (2'b0),
                .noc_mq_rd_data (),
                .noc_mq_rd_valid(),
                // Queue status
                .mq_empty       (),
                .mq_full        ()
            );
        end
    endgenerate

    // ---------------------------------------------------------------
    // Cluster Crossbar
    // ---------------------------------------------------------------
    logic               csram_en;
    logic               csram_we;
    logic [XLEN-1:0]   csram_addr;
    logic [XLEN-1:0]   csram_wdata;
    logic [XLEN-1:0]   csram_rdata;

    lm1_crossbar #(
        .NUM_PORTS (TILES_PER_CLUSTER)
    ) u_crossbar (
        .clk        (clk),
        .rst_n      (rst_n),
        .req_valid  (xbar_req_valid),
        .req_we     (xbar_req_we),
        .req_addr   (xbar_req_addr),
        .req_wdata  (xbar_req_wdata),
        .req_ready  (xbar_req_ready),
        .resp_data  (xbar_resp_data),
        .resp_valid (xbar_resp_valid),
        .sram_en    (csram_en),
        .sram_we    (csram_we),
        .sram_addr  (csram_addr),
        .sram_wdata (csram_wdata),
        .sram_rdata (csram_rdata)
    );

    // ---------------------------------------------------------------
    // Cluster Shared SRAM (2 MiB) — Dual-Port
    //
    // Port A: crossbar (tile access)
    // Port B: GC movement engines (scanner/copier/fixup), and the memory
    //         write port in the cycles the engines leave free
    // ---------------------------------------------------------------
    localparam int CLUSTER_MEM_DEPTH = 1 << CLUSTER_MEM_LOG2;

    logic [XLEN-1:0]   csram_rdata_b;  // GC engine read data from port B
    logic               gc_mem_rd_valid_r;  // 1-cycle latency for SRAM reads

    // The memory write port.  The engines go first on port B, since they
    // neither wait for it nor repeat an access it drops.  Each of their
    // accesses is followed by at least one cycle in which the engine awaits
    // its answer and leaves the port free, so a word waits at most one
    // cycle behind them.
    localparam logic [XLEN-1:0] CLUSTER_MEM_BYTES = XLEN'(CLUSTER_MEM_DEPTH) << 3;

    logic dma_wr_in_mem;        // an aligned word inside the shared SRAM
    logic dma_wr_go;            // the word is written at this edge
    assign dma_wr_in_mem = (dma_wr_addr[2:0] == 3'b000) &&
                           (dma_wr_addr < CLUSTER_MEM_BYTES);
    assign dma_wr_ready  = !(gc_mem_rd_en || gc_mem_wr_en);
    assign dma_wr_error  = !dma_wr_in_mem;
    assign dma_wr_go     = dma_wr_valid && dma_wr_ready && dma_wr_in_mem;

    lm1_sram_dp #(
        .DATA_WIDTH (XLEN),
        .DEPTH      (CLUSTER_MEM_DEPTH)
    ) u_cluster_sram (
        .clk   (clk),
        // Port A — crossbar
        .a_en    (csram_en),
        .a_we    (csram_we),
        .a_be    ({(XLEN/8){1'b1}}),
        .a_addr  (csram_addr[CLUSTER_MEM_LOG2-1:0]),
        .a_wdata (csram_wdata),
        .a_rdata (csram_rdata),
        // Port B — GC engines, then the memory write port
        .b_en    (gc_mem_rd_en | gc_mem_wr_en | dma_wr_go),
        .b_we    (gc_mem_wr_en | dma_wr_go),
        .b_be    ({(XLEN/8){1'b1}}),
        .b_addr  (gc_mem_wr_en ? gc_mem_wr_addr[CLUSTER_MEM_LOG2+2:3]
                : gc_mem_rd_en ? gc_mem_rd_addr[CLUSTER_MEM_LOG2+2:3]
                               : dma_wr_addr[CLUSTER_MEM_LOG2+2:3]),
        .b_wdata (gc_mem_wr_en ? gc_mem_wr_data : dma_wr_data),
        .b_rdata (csram_rdata_b)
    );

    // Track GC read valid — 1-cycle SRAM latency
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            gc_mem_rd_valid_r <= 1'b0;
        else
            gc_mem_rd_valid_r <= gc_mem_rd_en && !gc_mem_wr_en;
    end

    // ---------------------------------------------------------------
    // GC Command Arbiter — first valid tile wins
    // ---------------------------------------------------------------
    logic               gc_cmd_valid_out;
    logic [3:0]         gc_cmd_op_out;
    logic [XLEN-1:0]   gc_cmd_arg0_out;
    logic [XLEN-1:0]   gc_cmd_arg1_out;
    logic [XLEN-1:0]   gc_cmd_arg2_out;
    logic               gc_cmd_ready_in;

    always_comb begin
        gc_cmd_valid_out = 1'b0;
        gc_cmd_op_out    = '0;
        gc_cmd_arg0_out  = '0;
        gc_cmd_arg1_out  = '0;
        gc_cmd_arg2_out  = '0;
        tile_gc_cmd_ready = '0;

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
    end

    // ---------------------------------------------------------------
    // GC Movement Engine Top
    //
    // Engines access cluster SRAM via port B of dual-port SRAM.
    // ---------------------------------------------------------------
    logic               gc_mem_rd_en;
    logic [XLEN-1:0]   gc_mem_rd_addr;
    logic               gc_mem_wr_en;
    logic [XLEN-1:0]   gc_mem_wr_addr;
    logic [XLEN-1:0]   gc_mem_wr_data;

    // Scanner result wires
    logic               scan_res_valid;
    logic [XLEN-1:0]   scan_res_obj;
    logic [15:0]        scan_res_field;
    logic [XLEN-1:0]   scan_res_ref;
    logic               scan_res_ready;

    // Scanner result FIFO (128-deep) — buffered for runtime to drain
    localparam int SCAN_FIFO_DEPTH = 128;
    localparam int SCAN_ENTRY_W    = XLEN + 16 + XLEN;  // obj + field + ref

    logic [SCAN_ENTRY_W-1:0] scan_fifo [0:SCAN_FIFO_DEPTH-1];
    logic [6:0] scan_fifo_wr, scan_fifo_rd;
    logic [7:0] scan_fifo_count;

    // FIFO head outputs (broadcast to all tiles)
    logic [XLEN-1:0] scan_fifo_head_obj;
    logic [15:0]     scan_fifo_head_field;
    logic [XLEN-1:0] scan_fifo_head_ref;

    wire [SCAN_ENTRY_W-1:0] scan_fifo_head = scan_fifo[scan_fifo_rd];
    assign scan_fifo_head_obj   = scan_fifo_head[SCAN_ENTRY_W-1 -: XLEN];
    assign scan_fifo_head_field = scan_fifo_head[XLEN+15 : XLEN];
    assign scan_fifo_head_ref   = scan_fifo_head[XLEN-1 : 0];

    // Pop if any tile requests it
    assign scan_fifo_pop_any = |tile_scan_fifo_pop;

    assign scan_res_ready = (scan_fifo_count < SCAN_FIFO_DEPTH[7:0]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_fifo_wr    <= '0;
            scan_fifo_rd    <= '0;
            scan_fifo_count <= '0;
        end else begin
            // Simultaneous push and pop
            if (scan_res_valid && scan_res_ready && scan_fifo_pop_any && scan_fifo_count != 8'd0) begin
                scan_fifo[scan_fifo_wr] <= {scan_res_obj, scan_res_field, scan_res_ref};
                scan_fifo_wr <= scan_fifo_wr + 7'd1;
                scan_fifo_rd <= scan_fifo_rd + 7'd1;
                // count stays the same (push + pop)
            end else if (scan_res_valid && scan_res_ready) begin
                scan_fifo[scan_fifo_wr] <= {scan_res_obj, scan_res_field, scan_res_ref};
                scan_fifo_wr <= scan_fifo_wr + 7'd1;
                scan_fifo_count <= scan_fifo_count + 8'd1;
            end else if (scan_fifo_pop_any && scan_fifo_count != 8'd0) begin
                scan_fifo_rd <= scan_fifo_rd + 7'd1;
                scan_fifo_count <= scan_fifo_count - 8'd1;
            end
        end
    end

    lm1_gc_engine_top u_gc_engines (
        .clk            (clk),
        .rst_n          (rst_n),
        // Command
        .cmd_valid      (gc_cmd_valid_out),
        .cmd_op         (gc_cmd_op_out),
        .cmd_arg0       (gc_cmd_arg0_out),
        .cmd_arg1       (gc_cmd_arg1_out),
        .cmd_arg2       (gc_cmd_arg2_out),
        .cmd_ready      (gc_cmd_ready_in),
        .busy           (gc_busy),
        // Memory — port B of dual-port cluster SRAM
        .mem_rd_en      (gc_mem_rd_en),
        .mem_rd_addr    (gc_mem_rd_addr),
        .mem_rd_data    (csram_rdata_b),
        .mem_rd_valid   (gc_mem_rd_valid_r),
        .mem_wr_en      (gc_mem_wr_en),
        .mem_wr_addr    (gc_mem_wr_addr),
        .mem_wr_data    (gc_mem_wr_data),
        .mem_wr_ready   (1'b1),             // SRAM writes always succeed
        // Scanner results
        .scan_res_valid (scan_res_valid),
        .scan_res_obj   (scan_res_obj),
        .scan_res_field (scan_res_field),
        .scan_res_ref   (scan_res_ref),
        .scan_res_ready (scan_res_ready),
        .copy_dst_ptr   ()
    );

    // ---------------------------------------------------------------
    // NoC uplink — stub
    // ---------------------------------------------------------------
    assign noc_tx_valid = 1'b0;
    assign noc_tx_data  = '0;
    assign noc_rx_ready = 1'b1;   // always accept (discard)

endmodule
