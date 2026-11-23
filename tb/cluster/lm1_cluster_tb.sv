// ============================================================================
// LM-1 Two-Tile Cluster Testbench (SystemVerilog wrapper for Verilator)
//
// Loads a program into each of a cluster's two tiles through their external
// memory ports (+HEX0= and +HEX1=, one 64-bit word per line), releases
// reset, and runs until both tiles halt or MAX_CYCLES pass.  It then prints
// both tiles' registers and how often a tile's core waited on one of its
// own queues, to write or to read, because the message router used it.
// ============================================================================
module lm1_cluster_tb;
    import lm1_pkg::*;

    parameter int TILE_MEM_LOG2    = 12;   // 4096 words, 32 KiB per tile
    parameter int CLUSTER_MEM_LOG2 = 12;
    parameter int MAX_CYCLES       = 200000;

    logic clk;
    logic rst_n;

    logic [1:0]                 tile_halted;
    logic [1:0]                 ext_en, ext_we;
    logic [TILE_MEM_LOG2-1:0]   ext_addr  [0:1];
    logic [XLEN-1:0]            ext_wdata [0:1];
    logic [XLEN-1:0]            ext_rdata [0:1];
    logic [REG_IDX_W-1:0]       dbg_addr  [0:1];
    logic [XLEN-1:0]            dbg_data  [0:1];

    lm1_cluster #(
        .CLUSTER_ID(0), .TILES_PER_CLUSTER(2),
        .TILE_MEM_LOG2(TILE_MEM_LOG2), .CLUSTER_MEM_LOG2(CLUSTER_MEM_LOG2)
    ) u_cluster (
        .clk(clk), .rst_n(rst_n),
        .tile_halted(tile_halted),
        .ext_mem_en(ext_en), .ext_mem_we(ext_we),
        .ext_mem_addr(ext_addr), .ext_mem_wdata(ext_wdata),
        .ext_mem_rdata(ext_rdata),
        .dbg_reg_addr(dbg_addr), .dbg_reg_data(dbg_data),
        .noc_tx_valid(), .noc_tx_data(), .noc_tx_ready(1'b1),
        .noc_rx_valid(1'b0), .noc_rx_data({XLEN{1'b0}}), .noc_rx_ready(),
        .dma_wr_valid(1'b0), .dma_wr_ready(), .dma_wr_addr({XLEN{1'b0}}),
        .dma_wr_data({XLEN{1'b0}}), .dma_wr_error()
    );

    initial clk = 0;
    always #5 clk = ~clk;

    logic [XLEN-1:0] img0 [0:(1<<TILE_MEM_LOG2)-1];
    logic [XLEN-1:0] img1 [0:(1<<TILE_MEM_LOG2)-1];

    // Cycles in which a tile's core waits on one of its own queues because
    // the message router uses it: SEND or RECV in S_SEND_WAIT or
    // S_RECV_WAIT with no remote request, which only a queue the router
    // took that cycle sends there
    int wr_collisions [0:1];
    int rd_collisions [0:1];
`define LM1_CTRL(t) u_cluster.gen_tiles[t].u_tile.u_core.u_ctrl
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (`LM1_CTRL(0).state.name() == "S_SEND_WAIT" && !`LM1_CTRL(0).rmq_req)
                wr_collisions[0] <= wr_collisions[0] + 1;
            if (`LM1_CTRL(1).state.name() == "S_SEND_WAIT" && !`LM1_CTRL(1).rmq_req)
                wr_collisions[1] <= wr_collisions[1] + 1;
            if (`LM1_CTRL(0).state.name() == "S_RECV_WAIT" && !`LM1_CTRL(0).rmq_req)
                rd_collisions[0] <= rd_collisions[0] + 1;
            if (`LM1_CTRL(1).state.name() == "S_RECV_WAIT" && !`LM1_CTRL(1).rmq_req)
                rd_collisions[1] <= rd_collisions[1] + 1;
        end
    end
`undef LM1_CTRL

    initial begin
        int unsigned ncycles;
        string hex0, hex1;

        rst_n = 0;
        ext_en = '0;
        ext_we = '0;
        for (int t = 0; t < 2; t++) begin
            ext_addr[t]  = '0;
            ext_wdata[t] = '0;
            dbg_addr[t]  = '0;
            wr_collisions[t] = 0;
            rd_collisions[t] = 0;
        end
        if (!$value$plusargs("HEX0=%s", hex0)) hex0 = "t0.hex";
        if (!$value$plusargs("HEX1=%s", hex1)) hex1 = "t1.hex";
        $readmemh(hex0, img0);
        $readmemh(hex1, img1);

        repeat (2) @(negedge clk);
        ext_en = 2'b11;
        ext_we = 2'b11;
        for (int i = 0; i < (1 << TILE_MEM_LOG2); i++) begin
            ext_addr[0]  = i[TILE_MEM_LOG2-1:0];
            ext_addr[1]  = i[TILE_MEM_LOG2-1:0];
            ext_wdata[0] = img0[i];
            ext_wdata[1] = img1[i];
            @(negedge clk);
        end
        ext_en = '0;
        ext_we = '0;

        repeat (2) @(negedge clk);
        rst_n = 1;

        ncycles = 0;
        while (tile_halted != 2'b11 && ncycles < MAX_CYCLES) begin
            @(posedge clk);
            ncycles++;
        end
        @(negedge clk);

        if (tile_halted == 2'b11)
            $display("CLUSTER halted after %0d cycles", ncycles);
        else
            $display("TIMEOUT after %0d cycles (halted %b)", ncycles, tile_halted);
        $display("TILE 0 COLLISIONS write %0d read %0d", wr_collisions[0], rd_collisions[0]);
        $display("TILE 1 COLLISIONS write %0d read %0d", wr_collisions[1], rd_collisions[1]);
        // Each core's cycle counter, which stops while its tile gates the
        // clock of a halted core, and the instructions thread 0 retired.
        $display("TILE 0 CYCLE = %0d INSTRET = %0d",
                 u_cluster.gen_tiles[0].u_tile.u_core.u_ctrl.cyc,
                 u_cluster.gen_tiles[0].u_tile.u_core.u_ctrl.instret[0]);
        $display("TILE 1 CYCLE = %0d INSTRET = %0d",
                 u_cluster.gen_tiles[1].u_tile.u_core.u_ctrl.cyc,
                 u_cluster.gen_tiles[1].u_tile.u_core.u_ctrl.instret[0]);

        for (int i = 0; i < 32; i++)
            $display("TILE 0 REG r%0d = %016h", i,
                     u_cluster.gen_tiles[0].u_tile.u_core.u_regfile.regs[i]);
        for (int i = 0; i < 32; i++)
            $display("TILE 1 REG r%0d = %016h", i,
                     u_cluster.gen_tiles[1].u_tile.u_core.u_regfile.regs[i]);
        $finish;
    end

endmodule
