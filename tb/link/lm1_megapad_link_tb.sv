// ============================================================================
// lm1_megapad_link_tb.sv — LM-1's link against MegaPad's RTL
//
// Joins the two real ends of the link: LM-1's lm1_link and lm1_link_boot,
// with a cluster of LM-1's as the memory they write, on one side; MegaPad's
// port bridge, mailbox, external-memory arbiter and controller, and the
// tile engines' image stage on the other, over a behavioural
// external-memory PHY.  The bench plays MegaPad's software, one of its
// cores and the tile engine whose stores the stage carries, and LM-1's
// runtime and boot control.
//
// It first runs the shared link conformance cases of tb/link/link_cases.txt
// (link_cases.svh), the list the chiplet module and lm1_link_tb run too, each
// in the start order of MegaPad's link specification: read the identity
// record, lend the image, name it, release the chiplet, and let it fetch the
// image into its cluster's memory; then the ranges LM-1 lends and the
// TACC.STORE images MegaPad writes into them, doorbells both ways and the
// stop.  Then it runs what only the two RTL ends together show: MegaPad's
// records of the runtime's refused requests, the runtime's requests and
// bursts through the bridge, the mailbox's state around a doorbell, the
// units each format sends and the bytes in place at LM-1's answer, a
// doorbell rung after a store reaching LM-1 only after the image has landed,
// a tile reading the image back, stores MegaPad refuses or cancels itself,
// and the local side of the stop and the reset.
//
// Built by tb/link/run_link_tests.sh when MEGAPAD_ROOT is set.
// ============================================================================
module lm1_megapad_link_tb;
    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;                 // MegaPad's reset and LM-1's power-on

    int pass_count = 0;
    int fail_count = 0;
    task automatic check(input string label, input logic cond);
        if (cond) begin
            pass_count++;
            $display("  PASS: %s", label);
        end else begin
            fail_count++;
            $display("  FAIL: %s", label);
        end
    endtask

    // ------------------------------------------------------------------
    // The link
    // ------------------------------------------------------------------
    logic        lk_rq_valid, lk_rq_ready, lk_rq_wen, lk_rq_cancel;
    logic [63:0] lk_rq_addr, lk_rq_wdata;
    logic [1:0]  lk_rq_size;
    logic [7:0]  lk_rq_len;
    logic        lk_rs_valid, lk_rs_ready, lk_rs_last;
    logic [63:0] lk_rs_rdata;
    logic [1:0]  lk_rs_code;
    logic         lk_wq_valid, lk_wq_ready, lk_wq_cancel;
    logic [63:0]  lk_wq_addr;
    logic [511:0] lk_wq_data;
    logic [7:0]   lk_wq_len, lk_wq_zeros;
    logic         lk_wa_valid, lk_wa_ready;
    logic [1:0]   lk_wa_code;
    logic        lk_dbt_valid, lk_dbt_ready, lk_dbr_valid, lk_dbr_ready;
    logic [7:0]  lk_dbt_src, lk_dbr_dst;
    logic        sb_present, sb_rst_n, sb_stop_req, sb_stopped, sb_ready;
    logic        sb_id_req, sb_id_ack;
    logic [7:0]  sb_id_index;
    logic [63:0] sb_id_data, sb_boot_addr, sb_boot_len, sb_boot_arg;

    // ------------------------------------------------------------------
    // MegaPad's side
    // ------------------------------------------------------------------
    localparam logic [7:0]  PORT_ID = 8'd16;
    localparam logic [63:0] WINDOW  = 64'h0100_0000_0000_0000;   // port 0's

    logic        reg_sel = 1'b0, reg_wen = 1'b0;
    logic [3:0]  reg_idx = 4'd0;
    logic [63:0] reg_wdata = 64'd0;
    logic [63:0] reg_rdata;

    logic        m_req, m_wen, m_ack, m_error;
    logic [63:0] m_addr, m_wdata, m_rdata;
    logic [1:0]  m_size;
    logic [7:0]  m_weight;
    logic [15:0] m_bwlimit;

    logic        mb_db_valid, mb_take_valid, mb_send_valid;
    logic [7:0]  mb_db_src, mb_take_src, mb_send_dst;

    // The image stage's write to the port bridge.
    logic         tl_valid, tl_ready, tl_cancel;
    logic         tl_done, tl_error, tl_cancel_done;
    logic [7:0]   tl_port, tl_len, tl_zeros;
    logic [63:0]  tl_addr;
    logic [511:0] tl_data;

    mp64_chiplet_port #(
        .RANGES     (8),
        .RANGE_BITS (3),
        .PORT_ID    (PORT_ID),
        .EXT_BASE   (64'h0000_0000_0010_0000),
        .EXT_LIMIT  (64'h0000_0000_0020_0000),
        .WINDOW_BASE(WINDOW)
    ) u_port (
        .clk(clk), .rst_n(rst_n),
        .reg_sel(reg_sel), .reg_wen(reg_wen), .reg_idx(reg_idx),
        .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .lk_rq_valid(lk_rq_valid), .lk_rq_ready(lk_rq_ready),
        .lk_rq_addr(lk_rq_addr), .lk_rq_wdata(lk_rq_wdata),
        .lk_rq_wen(lk_rq_wen), .lk_rq_size(lk_rq_size),
        .lk_rq_len(lk_rq_len), .lk_rq_cancel(lk_rq_cancel),
        .lk_rs_valid(lk_rs_valid), .lk_rs_ready(lk_rs_ready),
        .lk_rs_rdata(lk_rs_rdata), .lk_rs_code(lk_rs_code),
        .lk_rs_last(lk_rs_last),
        .lk_dbt_valid(lk_dbt_valid), .lk_dbt_ready(lk_dbt_ready),
        .lk_dbt_src(lk_dbt_src),
        .lk_dbr_valid(lk_dbr_valid), .lk_dbr_ready(lk_dbr_ready),
        .lk_dbr_dst(lk_dbr_dst),
        .lk_wq_valid(lk_wq_valid), .lk_wq_ready(lk_wq_ready),
        .lk_wq_addr(lk_wq_addr), .lk_wq_data(lk_wq_data),
        .lk_wq_len(lk_wq_len), .lk_wq_zeros(lk_wq_zeros),
        .lk_wq_cancel(lk_wq_cancel),
        .lk_wa_valid(lk_wa_valid), .lk_wa_ready(lk_wa_ready),
        .lk_wa_code(lk_wa_code),
        .bw_valid(tl_valid && tl_port == 8'd0), .bw_ready(tl_ready),
        .bw_addr(tl_addr), .bw_data(tl_data),
        .bw_len(tl_len), .bw_zeros(tl_zeros),
        .bw_cancel(tl_cancel && tl_port == 8'd0),
        .bw_done(tl_done), .bw_error(tl_error),
        .bw_cancel_done(tl_cancel_done),
        .sb_present(sb_present), .sb_rst_n(sb_rst_n),
        .sb_stop_req(sb_stop_req), .sb_stopped(sb_stopped),
        .sb_ready(sb_ready), .sb_id_req(sb_id_req),
        .sb_id_index(sb_id_index), .sb_id_ack(sb_id_ack),
        .sb_id_data(sb_id_data), .sb_boot_addr(sb_boot_addr),
        .sb_boot_len(sb_boot_len), .sb_boot_arg(sb_boot_arg),
        .m_req(m_req), .m_addr(m_addr), .m_wdata(m_wdata), .m_wen(m_wen),
        .m_size(m_size), .m_ack(m_ack), .m_error(m_error),
        .m_rdata(m_rdata), .m_weight(m_weight), .m_bwlimit(m_bwlimit),
        .mb_db_valid(mb_db_valid), .mb_db_src(mb_db_src),
        .mb_take_valid(mb_take_valid), .mb_take_src(mb_take_src),
        .mb_send_valid(mb_send_valid), .mb_send_dst(mb_send_dst)
    );

    // The chip-wide TACC image stage.  Source 0 plays a tile engine whose
    // TACC.STORE names port 0's window; source 1 stays idle.
    localparam logic [3:0] EW_U8 = 4'd0, EW_FP32 = 4'd6, EW_FP64 = 4'd7;
    localparam logic [2:0] FAULT_NONE = 3'd0, FAULT_BUS = 3'd3;

    logic [1:0]    st_req = 2'b00, st_cancel = 2'b00, st_finish = 2'b00;
    logic [63:0]   st_base = 64'd0;
    logic [3:0]    st_ew = 4'd0;
    logic [2047:0] st_image = '0;
    logic [1:0]    st_done;
    logic [5:0]    st_fault;
    logic [127:0]  st_fault_addr;

    mp64_tacc_transfer #(
        .SOURCE_COUNT(2), .OWNER_BITS(1), .TOKEN_BITS(8)
    ) u_stage (
        .clk(clk), .rst(!rst_n),
        .req(st_req), .req_store(2'b11), .req_ext(2'b00), .req_link(2'b11),
        .req_port(16'd0), .req_base({64'd0, st_base}),
        .req_format_ew({4'd0, st_ew}), .req_token(16'h005A),
        .req_store_image({2048'd0, st_image}), .req_cancel(st_cancel),
        .finish(st_finish),
        .port_ack(2'b00), .port_error(2'b00), .port_fault_addr(128'd0),
        .tile_rdata(512'd0), .ext_rdata(512'd0), .port_cancel_done(2'b00),
        .beat_req(), .beat_ext(), .beat_addr(), .beat_wen(), .beat_wdata(),
        .port_cancel(), .stall_cycle(),
        .done(st_done), .response_token(), .response_fault(st_fault),
        .response_fault_addr(st_fault_addr), .result_image(),
        .link_valid(tl_valid), .link_ready(tl_ready), .link_port(tl_port),
        .link_addr(tl_addr), .link_data(tl_data), .link_len(tl_len),
        .link_zeros(tl_zeros), .link_cancel(tl_cancel),
        .link_done(tl_done), .link_error(tl_error),
        .link_cancel_done(tl_cancel_done)
    );

    // Mailbox: four full cores and the port.
    logic        mb_req = 1'b0, mb_wen = 1'b0;
    logic [11:0] mb_addr = 12'd0;
    logic [7:0]  mb_wdata = 8'd0, mb_rdata;
    logic [7:0]  mb_requester = 8'd0;
    // MegaPad's mailbox has a line for each of its sixteen cores.
    logic [15:0] ipi_out, ef_out;

    mp64_mailbox #(
        .N_CORES(4), .N_GLOBAL_CORES(16), .N_PORTS(1), .PORT_ID_BASE(PORT_ID)
    ) u_mailbox (
        .clk(clk), .rst_n(rst_n),
        .req(mb_req), .addr(mb_addr), .wdata(mb_wdata), .wen(mb_wen),
        .rdata(mb_rdata), .ack(),
        .requester_valid(1'b1), .requester_id(mb_requester),
        .ipi_out(ipi_out),
        .csr_ipi_wen(4'd0), .csr_ipi_addr(32'd0), .csr_ipi_wdata(256'd0),
        .csr_ipi_rdata(),
        .port_send_valid(mb_send_valid), .port_send_dst(mb_send_dst),
        .port_take_valid(mb_take_valid), .port_take_src(mb_take_src),
        .port_db_valid(mb_db_valid), .port_db_src(mb_db_src),
        .ef_out(ef_out)
    );

    // External memory: the arbiter with an idle core path, the controller
    // and a PHY of 1 MiB at the external base.
    logic        xm_req, xm_wen, xm_ack, xm_error;
    logic [63:0] xm_addr, xm_wdata, xm_rdata;
    logic [1:0]  xm_size;
    logic [1:0]  xc_ack, xc_error;

    mp64_extmem_arb #(.N_CLIENTS(2), .CLIENT_BITS(1)) u_arb (
        .clk(clk), .rst_n(rst_n),
        .c_req({m_req, 1'b0}), .c_addr({m_addr, 64'd0}),
        .c_wdata({m_wdata, 64'd0}), .c_wen({m_wen, 1'b0}),
        .c_size({m_size, 2'd0}),
        .c_ack(xc_ack), .c_error(xc_error), .c_rdata(m_rdata),
        .c_weight({m_weight, 8'd1}), .c_bwlimit({m_bwlimit, 16'd0}),
        .ext_req(xm_req), .ext_addr(xm_addr), .ext_wdata(xm_wdata),
        .ext_wen(xm_wen), .ext_size(xm_size), .ext_rdata(xm_rdata),
        .ext_ack(xm_ack), .ext_error(xm_error)
    );
    assign m_ack   = xc_ack[1];
    assign m_error = xc_error[1];

    logic        phy_req, phy_wen, phy_cancel;
    logic [31:0] phy_addr;
    logic [63:0] phy_wdata;
    logic [63:0] phy_rdata = 64'd0;
    logic        phy_ack = 1'b0;

    mp64_extmem u_extmem (
        .clk(clk), .rst_n(rst_n),
        .cpu_req(xm_req), .cpu_addr(xm_addr[31:0]), .cpu_wdata(xm_wdata),
        .cpu_wen(xm_wen), .cpu_size(xm_size), .cpu_rdata(xm_rdata),
        .cpu_ack(xm_ack), .cpu_error(xm_error),
        .tile_req(1'b0), .tile_addr(32'd0), .tile_wdata(512'd0),
        .tile_wen(1'b0), .tile_cancel(1'b0), .tile_accept(), .tile_rdata(),
        .tile_ack(), .tile_error(), .tile_fault_addr(), .tile_word_done(),
        .phy_req(phy_req), .phy_addr(phy_addr), .phy_wdata(phy_wdata),
        .phy_wen(phy_wen), .phy_ready(1'b1), .phy_rdata(phy_rdata),
        .phy_ack(phy_ack), .phy_error(1'b0), .phy_cancel(phy_cancel),
        .phy_cancel_done(phy_cancel), .phy_burst_len()
    );

    logic [63:0] xmem [0:131071];
    logic        ppend = 1'b0;
    logic [3:0]  pcnt;
    logic [31:0] p_addr;
    logic [63:0] p_wdata;
    logic        p_wen;
    int          phy_launches = 0;
    always_ff @(posedge clk) begin
        phy_ack <= 1'b0;
        if (!rst_n || phy_cancel) begin
            ppend <= 1'b0;
        end else if (phy_req && !ppend) begin
            ppend   <= 1'b1;
            pcnt    <= 4'd2;
            p_addr  <= phy_addr;
            p_wdata <= phy_wdata;
            p_wen   <= phy_wen;
            phy_launches <= phy_launches + 1;
        end else if (ppend) begin
            if (pcnt == 4'd0) begin
                ppend   <= 1'b0;
                phy_ack <= 1'b1;
                if (p_wen)
                    xmem[p_addr[19:3]] <= p_wdata;
                else
                    phy_rdata <= xmem[p_addr[19:3]];
            end else begin
                pcnt <= pcnt - 4'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // LM-1's side
    // ------------------------------------------------------------------
    logic        core_rst_n, stop_req;
    logic        runtime_stopped = 1'b0, runtime_ready = 1'b0;
    logic [63:0] boot_addr, boot_len, boot_arg;

    logic        hm_req_valid, hm_req_ready, hm_req_we, hm_cancel;
    logic [63:0] hm_req_addr, hm_req_wdata;
    logic [1:0]  hm_req_size;
    logic [7:0]  hm_req_len;
    logic        hm_resp_valid, hm_resp_ready, hm_resp_last;
    logic [63:0] hm_resp_data;
    logic [1:0]  hm_resp_code;

    logic        rt_req_valid = 1'b0, rt_req_ready, rt_req_we = 1'b0;
    logic        rt_cancel = 1'b0;
    logic [63:0] rt_req_addr = 64'd0, rt_req_wdata = 64'd0;
    logic [1:0]  rt_req_size = 2'd3;
    logic [7:0]  rt_req_len = 8'd0;
    logic        rt_resp_valid, rt_resp_last;
    logic [63:0] rt_resp_data;
    logic [1:0]  rt_resp_code;

    logic        lend_valid = 1'b0, lend_ready, lend_write = 1'b0;
    logic [7:0]  lend_index = 8'd0, lend_ranges;
    logic [63:0] lend_base = 64'd0, lend_limit = 64'd0;

    logic        lw_valid, lw_ready, lw_error;
    logic [63:0] lw_addr, lw_data;
    logic        mw_valid, mw_ready, mw_error;
    logic [63:0] mw_addr, mw_data;
    logic        boot_done, boot_failed;

    logic        dbi_valid, dbo_ready;
    logic        dbi_ready = 1'b0, dbo_valid = 1'b0;
    logic [7:0]  dbi_src;
    logic [7:0]  dbo_dst = 8'd0;

    lm1_link u_link (
        .clk(clk), .por_n(rst_n),
        .lk_rq_valid(lk_rq_valid), .lk_rq_ready(lk_rq_ready),
        .lk_rq_addr(lk_rq_addr), .lk_rq_wdata(lk_rq_wdata),
        .lk_rq_wen(lk_rq_wen), .lk_rq_size(lk_rq_size),
        .lk_rq_len(lk_rq_len), .lk_rq_cancel(lk_rq_cancel),
        .lk_rs_valid(lk_rs_valid), .lk_rs_ready(lk_rs_ready),
        .lk_rs_rdata(lk_rs_rdata), .lk_rs_code(lk_rs_code),
        .lk_rs_last(lk_rs_last),
        .lk_wq_valid(lk_wq_valid), .lk_wq_ready(lk_wq_ready),
        .lk_wq_addr(lk_wq_addr), .lk_wq_data(lk_wq_data),
        .lk_wq_len(lk_wq_len), .lk_wq_zeros(lk_wq_zeros),
        .lk_wq_cancel(lk_wq_cancel),
        .lk_wa_valid(lk_wa_valid), .lk_wa_ready(lk_wa_ready),
        .lk_wa_code(lk_wa_code),
        .lk_dbt_valid(lk_dbt_valid), .lk_dbt_ready(lk_dbt_ready),
        .lk_dbt_src(lk_dbt_src),
        .lk_dbr_valid(lk_dbr_valid), .lk_dbr_ready(lk_dbr_ready),
        .lk_dbr_dst(lk_dbr_dst),
        .sb_present(sb_present), .sb_rst_n(sb_rst_n),
        .sb_stop_req(sb_stop_req), .sb_stopped(sb_stopped),
        .sb_ready(sb_ready), .sb_id_req(sb_id_req),
        .sb_id_index(sb_id_index), .sb_id_ack(sb_id_ack),
        .sb_id_data(sb_id_data), .sb_boot_addr(sb_boot_addr),
        .sb_boot_len(sb_boot_len), .sb_boot_arg(sb_boot_arg),
        .core_rst_n(core_rst_n), .stop_req(stop_req),
        .runtime_stopped(runtime_stopped), .runtime_ready(runtime_ready),
        .boot_addr(boot_addr), .boot_len(boot_len), .boot_arg(boot_arg),
        .hm_req_valid(hm_req_valid), .hm_req_ready(hm_req_ready),
        .hm_req_addr(hm_req_addr), .hm_req_wdata(hm_req_wdata),
        .hm_req_we(hm_req_we), .hm_req_size(hm_req_size),
        .hm_req_len(hm_req_len), .hm_cancel(hm_cancel),
        .hm_resp_valid(hm_resp_valid), .hm_resp_ready(hm_resp_ready),
        .hm_resp_data(hm_resp_data), .hm_resp_code(hm_resp_code),
        .hm_resp_last(hm_resp_last),
        .lend_valid(lend_valid), .lend_ready(lend_ready),
        .lend_index(lend_index), .lend_base(lend_base),
        .lend_limit(lend_limit), .lend_write(lend_write),
        .lend_ranges(lend_ranges),
        .mw_valid(lw_valid), .mw_ready(lw_ready), .mw_addr(lw_addr),
        .mw_data(lw_data), .mw_error(lw_error),
        .dbi_valid(dbi_valid), .dbi_ready(dbi_ready), .dbi_src(dbi_src),
        .dbo_valid(dbo_valid), .dbo_ready(dbo_ready), .dbo_dst(dbo_dst)
    );

    // LM-1 loads its image at 0x2000 of the cluster's shared memory.
    localparam logic [63:0] LOAD_BASE = 64'h2000;

    lm1_link_boot #(.LOAD_BASE(LOAD_BASE)) u_boot (
        .clk(clk), .rst_n(core_rst_n),
        .boot_addr(boot_addr), .boot_len(boot_len),
        .hm_req_valid(hm_req_valid), .hm_req_ready(hm_req_ready),
        .hm_req_addr(hm_req_addr), .hm_req_wdata(hm_req_wdata),
        .hm_req_we(hm_req_we), .hm_req_size(hm_req_size),
        .hm_req_len(hm_req_len), .hm_cancel(hm_cancel),
        .hm_resp_valid(hm_resp_valid), .hm_resp_ready(hm_resp_ready),
        .hm_resp_data(hm_resp_data), .hm_resp_code(hm_resp_code),
        .hm_resp_last(hm_resp_last),
        .rt_req_valid(rt_req_valid), .rt_req_ready(rt_req_ready),
        .rt_req_addr(rt_req_addr), .rt_req_wdata(rt_req_wdata),
        .rt_req_we(rt_req_we), .rt_req_size(rt_req_size),
        .rt_req_len(rt_req_len), .rt_cancel(rt_cancel),
        .rt_resp_valid(rt_resp_valid), .rt_resp_ready(1'b1),
        .rt_resp_data(rt_resp_data), .rt_resp_code(rt_resp_code),
        .rt_resp_last(rt_resp_last),
        .lw_valid(lw_valid), .lw_ready(lw_ready), .lw_addr(lw_addr),
        .lw_data(lw_data), .lw_error(lw_error),
        .mw_valid(mw_valid), .mw_ready(mw_ready), .mw_addr(mw_addr),
        .mw_data(mw_data), .mw_error(mw_error),
        .boot_done(boot_done), .boot_failed(boot_failed)
    );

    // A cluster of two tiles, each with 8 KiB of its own, and 32 KiB of
    // shared memory, written through its memory write port.  Its tiles
    // wait in reset until the bench, as LM-1's boot control, starts them.
    localparam int TILE_MEM_LOG2    = 10;
    localparam int CLUSTER_MEM_LOG2 = 12;
    logic       tiles_go = 1'b0;
    logic [1:0] tile_halted;
    logic [1:0] cl_ext_en = 2'b00, cl_ext_we = 2'b00;
    logic [TILE_MEM_LOG2-1:0] cl_ext_addr  [0:1];
    logic [63:0]              cl_ext_wdata [0:1];
    logic [63:0]              cl_ext_rdata [0:1];
    logic [lm1_pkg::REG_IDX_W-1:0] cl_dbg_addr [0:1];
    logic [63:0]                   cl_dbg_data [0:1];
    initial begin
        for (int t = 0; t < 2; t++) begin
            cl_ext_addr[t] = '0; cl_ext_wdata[t] = '0; cl_dbg_addr[t] = '0;
        end
    end

    lm1_cluster #(
        .CLUSTER_ID(0), .TILES_PER_CLUSTER(2),
        .TILE_MEM_LOG2(TILE_MEM_LOG2), .CLUSTER_MEM_LOG2(CLUSTER_MEM_LOG2)
    ) u_cluster (
        .clk(clk), .rst_n(core_rst_n && tiles_go),
        .tile_halted(tile_halted),
        .ext_mem_en(cl_ext_en), .ext_mem_we(cl_ext_we),
        .ext_mem_addr(cl_ext_addr), .ext_mem_wdata(cl_ext_wdata),
        .ext_mem_rdata(cl_ext_rdata),
        .dbg_reg_addr(cl_dbg_addr), .dbg_reg_data(cl_dbg_data),
        .noc_tx_valid(), .noc_tx_data(), .noc_tx_ready(1'b1),
        .noc_rx_valid(1'b0), .noc_rx_data(64'd0), .noc_rx_ready(),
        .dma_wr_valid(mw_valid), .dma_wr_ready(mw_ready),
        .dma_wr_addr(mw_addr), .dma_wr_data(mw_data),
        .dma_wr_error(mw_error)
    );

    // A word of the cluster's shared memory, by its byte address.
    function automatic logic [63:0] lm1_word(input logic [63:0] addr);
        return u_cluster.u_cluster_sram.mem[addr[CLUSTER_MEM_LOG2+2:3]];
    endfunction

    // ------------------------------------------------------------------
    // What crosses and lands, counted at each rising edge
    // ------------------------------------------------------------------
    int         cyc = 0;
    int         link_requests = 0;      // request messages the bridge took
    int         wq_units = 0;           // units of MegaPad's writes LM-1 took
    int         wa_count = 0;           // answers LM-1 gave
    logic [1:0] wa_code_last = 2'd0;
    int         wa_cyc = 0;             // the edge that took the last answer
    int         lm1_writes = 0;         // words written into LM-1's memory
    int         cancels_done = 0;       // cancels the bridge confirmed
    always_ff @(posedge clk) begin
        cyc <= cyc + 1;
        if (lk_rq_valid && lk_rq_ready)
            link_requests <= link_requests + 1;
        if (lk_wq_valid && lk_wq_ready)
            wq_units <= wq_units + 1;
        if (lk_wa_valid && lk_wa_ready) begin
            wa_count     <= wa_count + 1;
            wa_code_last <= lk_wa_code;
            wa_cyc       <= cyc;
        end
        if (mw_valid && mw_ready && !mw_error)
            lm1_writes <= lm1_writes + 1;
        if (tl_cancel_done)
            cancels_done <= cancels_done + 1;
    end

    // The image a store sends, its active bytes set by its format as the
    // stage sets them, and whether LM-1's memory holds it at addr.
    function automatic logic [2047:0] image_of(input logic [7:0] seed);
        logic [2047:0] v;
        for (int w = 0; w < 32; w++)
            v[w*64 +: 64] = {seed, 8'(w), 16'h7ACC, 32'(w * 7 + 1)};
        return v;
    endfunction
    function automatic logic [2047:0] active(input logic [2047:0] img,
                                             input logic [3:0] ew);
        if (ew == EW_FP64)
            return {1536'd0, img[511:0]};
        if (ew == EW_U8 || ew == 4'd1)
            return img;
        return {1024'd0, img[1023:0]};
    endfunction
    function automatic logic lm1_holds(input logic [63:0] addr,
                                       input logic [2047:0] img);
        logic ok = 1'b1;
        for (int w = 0; w < 32; w++)
            if (lm1_word(addr + 64'(w * 8)) != img[w*64 +: 64])
                ok = 1'b0;
        return ok;
    endfunction

    // At the edge that takes LM-1's OK for the first image, and at the
    // doorbell that reaches LM-1 after it, whether the image is all there.
    logic [63:0]   watch_addr  = 64'd0;
    logic [2047:0] watch_image = '0;
    logic          watch_on = 1'b0;
    logic          img_at_answer = 1'b0, img_at_doorbell = 1'b0;
    int            answer_cyc = 0, doorbell_cyc = 0;
    always @(posedge clk) begin
        if (watch_on && lk_wa_valid && lk_wa_ready && lk_wa_code == 2'd0 &&
            answer_cyc == 0) begin
            answer_cyc    = cyc;
            img_at_answer = lm1_holds(watch_addr, watch_image);
        end
        if (watch_on && dbi_valid && doorbell_cyc == 0) begin
            doorbell_cyc    = cyc;
            img_at_doorbell = lm1_holds(watch_addr, watch_image);
        end
    end

    // ------------------------------------------------------------------
    // Tasks.  Inputs change on the falling edge and outputs are read just
    // after it, so no input changes on the edge that samples it.
    // ------------------------------------------------------------------
    localparam logic [3:0] R_CTRL = 4'h0, R_STATUS = 4'h1, R_FAULT_ADDR = 4'h2;
    localparam logic [3:0] R_FAULT_INFO = 4'h3, R_RANGE_SEL = 4'h4;
    localparam logic [3:0] R_RANGE_BASE = 4'h5, R_RANGE_LIM = 4'h6;
    localparam logic [3:0] R_RANGE_RTS = 4'h7, R_ID_INDEX = 4'h9;
    localparam logic [3:0] R_ID_DATA = 4'hA, R_BOOT_ADDR = 4'hB;
    localparam logic [3:0] R_BOOT_LEN = 4'hC, R_BOOT_ARG = 4'hD;
    localparam logic [3:0] R_WINDOW = 4'hE;

    logic [63:0] rd;
    task automatic reg_write(input logic [3:0] idx, input logic [63:0] data);
        @(negedge clk);
        reg_sel = 1'b1; reg_wen = 1'b1; reg_idx = idx; reg_wdata = data;
        @(negedge clk);
        reg_sel = 1'b0; reg_wen = 1'b0;
    endtask

    task automatic reg_read(input logic [3:0] idx);
        @(negedge clk);
        reg_sel = 1'b1; reg_wen = 1'b0; reg_idx = idx;
        #1;
        rd = reg_rdata;
        reg_sel = 1'b0;
    endtask

    task automatic lend(input logic [2:0] idx, input logic [63:0] base,
                        input logic [63:0] limit, input logic [1:0] rights);
        reg_write(R_RANGE_SEL, {61'd0, idx});
        reg_write(R_RANGE_BASE, base);
        reg_write(R_RANGE_LIM, limit);
        reg_write(R_RANGE_RTS, {62'd0, rights});
    endtask

    // LM-1's runtime lends MegaPad a range of its own memory, or withdraws it.
    task automatic lm1_lend(input logic [7:0] idx, input logic [63:0] base,
                            input logic [63:0] limit, input logic wr);
        int guard = 0;
        @(negedge clk);
        lend_valid = 1'b1; lend_index = idx; lend_base = base;
        lend_limit = limit; lend_write = wr;
        #1;
        while (!lend_ready && guard < 2000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        @(negedge clk);
        lend_valid = 1'b0;
    endtask

    logic [63:0] idw;
    task automatic read_identity(input logic [7:0] index);
        int guard = 0;
        reg_write(R_ID_INDEX, {56'd0, index});
        reg_read(R_STATUS);
        while (!rd[5] && guard < 100) begin
            reg_read(R_STATUS);
            guard++;
        end
        reg_read(R_ID_DATA);
        idw = rd;
    endtask

    task automatic mbox_write(input logic [7:0] core, input logic [3:0] off,
                              input logic [7:0] data);
        @(negedge clk);
        mb_req = 1'b1; mb_wen = 1'b1; mb_requester = core;
        mb_addr = {8'h50, off}; mb_wdata = data;
        @(negedge clk);
        mb_req = 1'b0; mb_wen = 1'b0;
    endtask

    logic [63:0] r_data;
    logic [1:0]  r_code;
    task automatic rt_access(input logic [63:0] addr, input logic [63:0] wdata,
                             input logic we);
        int guard = 0;
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = addr; rt_req_wdata = wdata;
        rt_req_we = we; rt_req_size = 2'd3;
        #1;
        while (!rt_req_ready && guard < 2000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        @(negedge clk);
        rt_req_valid = 1'b0;
        #1;
        while (!rt_resp_valid && guard < 4000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        r_data = rt_resp_data;
        r_code = rt_resp_code;
        @(negedge clk);
    endtask

    // A read burst on the runtime's port.
    logic [63:0] b_data [0:15];
    logic        b_last [0:15];
    logic [1:0]  b_code [0:15];
    int          b_n;
    task automatic rt_burst_read(input logic [63:0] addr, input logic [7:0] len);
        logic done;
        int   guard = 0;
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = addr; rt_req_we = 1'b0;
        rt_req_size = 2'd3; rt_req_len = len;
        #1;
        while (!rt_req_ready && guard < 2000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        @(negedge clk);
        rt_req_valid = 1'b0; rt_req_len = 8'd0;
        b_n = 0;
        done = 1'b0;
        #1;
        while (!done && guard < 8000) begin
            if (rt_resp_valid) begin
                b_data[b_n] = rt_resp_data;
                b_code[b_n] = rt_resp_code;
                b_last[b_n] = rt_resp_last;
                done = rt_resp_last;
                b_n++;
            end
            @(negedge clk);
            #1;
            guard++;
        end
    endtask

    // A TACC.STORE of img in format ew to MegaPad's address dst, as the
    // tile engine hands it to the stage: the request held until the
    // terminal response, which the engine then retires.  With cancel_at
    // positive, the engine cancels that many cycles after LM-1 takes the
    // first unit, and waits for the bridge to confirm instead.
    logic        s_done;
    logic [2:0]  s_fault;
    logic [63:0] s_fault_addr;
    int          s_cycles;          // from the request to the response
    task automatic tacc_store(input logic [63:0] dst, input logic [3:0] ew,
                              input logic [2047:0] img, input int cancel_at);
        int guard = 0, units0, cancels0, after = -1;
        units0 = wq_units; cancels0 = cancels_done;
        s_done = 1'b0;
        @(negedge clk);
        st_base = dst; st_ew = ew; st_image = img; st_req[0] = 1'b1;
        #1;
        while (guard < 2000) begin
            if (st_done[0]) begin
                s_done       = 1'b1;
                s_fault      = st_fault[2:0];
                s_fault_addr = st_fault_addr[63:0];
                break;
            end
            if (cancels_done != cancels0)
                break;
            if (cancel_at > 0 && after < 0 && wq_units != units0)
                after = 0;
            if (after >= 0) begin
                if (after == cancel_at)
                    st_cancel[0] = 1'b1;
                after++;
            end
            @(negedge clk);
            #1;
            guard++;
        end
        s_cycles = guard;
        @(negedge clk);
        st_finish[0] = s_done;
        st_req[0]    = 1'b0;
        st_cancel[0] = 1'b0;
        @(negedge clk);
        st_finish[0] = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // The shared link cases (link_cases.svh), done this bench's way: MegaPad
    // plays its part through its port registers and its mailbox, and its
    // tile engine through the image stage; the bench plays LM-1's runtime.
    // A case's image lies at 0x0010_4000, lent read-only, and 0x0010_6000
    // is lent to nobody.  LM-1's memory is the cluster's shared memory.
    // ------------------------------------------------------------------
    localparam logic [63:0] LC_IMAGE = 64'h0010_4000, LC_UNLENT = 64'h0010_6000;
    localparam logic [63:0] LC_HELD = 64'h02;       // RESET_HOLD
    localparam logic [63:0] LC_OPEN = 64'h29;       // ENABLE, DOORBELL_EN, BORROW_EN
    localparam logic [63:0] LC_STOP = 64'h04;       // STOP_REQ
    int          lc_mark;               // link_requests at the release
    logic [63:0] lc_before [0:63];      // LM-1's memory from the load base then
    logic [63:0] lc_near [0:33];        // LM-1's memory around a store, before it
    logic        lc_stop_req = 1'b0, lc_released = 1'b0;
    string       lc_path;

    task automatic lc_reset();
        lc_stop_req = 1'b0;
        lc_released = 1'b0;
        reg_write(R_CTRL, LC_HELD);
        runtime_ready = 1'b0; runtime_stopped = 1'b0;
        dbo_valid = 1'b0; dbi_ready = 1'b0;
        repeat (3) @(negedge clk);
    endtask

    task automatic lc_identity(input logic [7:0] index, input logic [63:0] value);
        read_identity(index);
        lc_check($sformatf("identity word %0d reads %h", index, value), idw == value);
    endtask

    task automatic lc_image(input int n, input logic [63:0] value);
        for (int k = 0; k < n; k++)
            xmem[17'(LC_IMAGE >> 3) + 17'(k)] = value + 64'(k);
    endtask

    task automatic lc_release(input logic [63:0] len, input logic [63:0] arg,
                              input logic lent);
        for (int k = 0; k < 64; k++)
            lc_before[k] = lm1_word(LOAD_BASE + 64'(8 * k));
        lc_mark = link_requests;
        lend(3'd0, LC_IMAGE, LC_IMAGE + 64'h1000, 2'b01);
        reg_write(R_BOOT_ADDR, lent ? LC_IMAGE : LC_UNLENT);
        reg_write(R_BOOT_LEN, len);
        reg_write(R_BOOT_ARG, arg);
        lc_released = 1'b1;
        reg_write(R_CTRL, LC_OPEN | (lc_stop_req ? LC_STOP : 64'd0));
    endtask

    task automatic lc_boot(input logic failed);
        int guard = 0;
        while (!boot_done && !boot_failed && guard < 20000) begin
            @(negedge clk);
            guard++;
        end
        repeat (40) @(negedge clk);     // a cancelled burst's responses drain
        lc_check(failed ? "the fetch fails" : "the fetch completes",
                 failed ? (boot_failed && !boot_done) : (boot_done && !boot_failed));
    endtask

    task automatic lc_boot_arg(input logic [63:0] value);
        lc_check("the boot argument reaches the runtime", boot_arg == value);
    endtask

    task automatic lc_loaded(input int n, input logic [63:0] value);
        logic ok = 1'b1;
        for (int k = 0; k < n; k++)
            if (lm1_word(LOAD_BASE + 64'(8 * k)) != value + 64'(k))
                ok = 1'b0;
        lc_check($sformatf("%0d words land in order from the load base", n), ok);
        lc_check("the word after them is untouched",
                 lm1_word(LOAD_BASE + 64'(8 * n)) == lc_before[n]);
    endtask

    task automatic lc_requests(input int n);
        lc_check($sformatf("the fetch made %0d requests", n), link_requests - lc_mark == n);
    endtask

    task automatic lc_wait();
        repeat (64) @(negedge clk);
    endtask

    task automatic lc_runtime_ready();
        @(negedge clk);
        runtime_ready = 1'b1;
    endtask

    task automatic lc_runtime_stop(input logic b);
        @(negedge clk);
        runtime_stopped = b;
    endtask

    task automatic lc_ready(input logic b);
        repeat (2) @(negedge clk);
        reg_read(R_STATUS);
        lc_check($sformatf("MegaPad reads READY as %0d", b), rd[2] == b);
    endtask

    task automatic lc_stopped(input logic b);
        repeat (2) @(negedge clk);
        reg_read(R_STATUS);
        lc_check($sformatf("MegaPad reads STOPPED as %0d", b), rd[1] == b);
    endtask

    task automatic lc_stop(input logic b);
        lc_stop_req = b;
        reg_write(R_CTRL, (lc_released ? LC_OPEN : LC_HELD) | (b ? LC_STOP : 64'd0));
    endtask

    task automatic lc_lend(input logic [7:0] index, input logic [63:0] base,
                           input logic [63:0] limit, input logic w);
        lm1_lend(index, base, limit, w);
    endtask

    task automatic lc_store(input int units, input int zeros, input logic [63:0] addr,
                            input logic [7:0] seed, input logic [1:0] code);
        logic          ok = 1'b1;
        logic [3:0]    ew;
        logic [2047:0] img;
        int            answers;
        ew = (units == 4) ? EW_U8 : (units == 2) ? EW_FP32 : EW_FP64;
        for (int w = 0; w < 32; w++)
            img[w*64 +: 64] = lc_image_word(seed, w);
        for (int k = 0; k < 34; k++)
            lc_near[k] = lm1_word(addr - 64'd8 + 64'(8 * k));
        answers = wa_count;
        tacc_store(WINDOW + addr, ew, img, 0);
        lc_check($sformatf("LM-1 answers %0d", code),
                 wa_count == answers + 1 && wa_code_last == code);
        lc_check("the store ends as LM-1 answered: done, or a bus fault at its base",
                 s_done && (code == 2'd0 ? s_fault == FAULT_NONE
                                         : (s_fault == FAULT_BUS &&
                                            s_fault_addr == WINDOW + addr)));
        lc_check("the words beside the image are untouched",
                 lm1_word(addr - 64'd8) == lc_near[0] &&
                 lm1_word(addr + 64'd256) == lc_near[33]);
        for (int w = 0; w < 32; w++)
            if (code == 2'd0 ? lm1_word(addr + 64'(8 * w)) !=
                                   ((w < 8 * units) ? lc_image_word(seed, w) : 64'd0)
                             : lm1_word(addr + 64'(8 * w)) != lc_near[w + 1])
                ok = 1'b0;
        lc_check(code == 2'd0 ? "LM-1's memory holds the image and its zeros"
                              : "LM-1's memory is untouched", ok);
    endtask

    task automatic lc_ring_in(input logic [7:0] src);
        mbox_write(src, 4'h8, PORT_ID);     // MBOX_SEND from core src
    endtask

    task automatic lc_rung_in(input logic [7:0] src);
        int guard = 0;
        #1;
        while (!dbi_valid && guard < 1000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        lc_check($sformatf("LM-1 takes a doorbell from %0d", src),
                 dbi_valid && dbi_src == src);
        dbi_ready = 1'b1;
        @(negedge clk);
        dbi_ready = 1'b0;
        repeat (2) @(negedge clk);
    endtask

    // LM-1's doorbell is offered and held until the port takes it.
    task automatic lc_ring_out(input logic [7:0] dst);
        @(negedge clk);
        dbo_valid = 1'b1;
        dbo_dst   = dst;
    endtask

    task automatic lc_rung_out(input logic [7:0] dst);
        int guard = 0;
        #1;
        while (!dbo_ready && guard < 1000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        @(negedge clk);                 // the rising edge between took it
        dbo_valid = 1'b0;
        repeat (2) @(negedge clk);
        #1;
        lc_check($sformatf("LM-1's doorbell raises core %0d's IPI line", dst),
                 ipi_out[dst[3:0]] == 1'b1);
        mbox_write(dst, 4'hA, PORT_ID);     // MBOX_ACK from that core
    endtask

    task automatic lc_no_ring_out();
        #1;
        lc_check("no doorbell leaves LM-1", ipi_out == 16'h0 && lk_dbr_valid == 1'b0);
    endtask

    `include "link_cases.svh"

    int prev_n, prev_w, prev_a, prev_u;
    logic [2047:0] img_a, img_b, img_c;

    initial begin
        for (int i = 0; i < 131072; i++) xmem[i] = 64'd0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (6) @(posedge clk);

        $display("\nIn reset, through MegaPad's port registers");
        reg_read(R_STATUS);
        check("MegaPad sees a chiplet on the port", rd[0] == 1'b1);
        check("LM-1 is held in reset", core_rst_n == 1'b0);

        $display("\nThe shared link cases");
        if (!$value$plusargs("cases=%s", lc_path))
            lc_path = "tb/link/link_cases.txt";
        run_link_cases(lc_path);

        // The cases this bench alone runs follow, from a start of its own:
        // MegaPad's records of refusals, the runtime's requests through the
        // bridge, what the stores carry across and when, a tile reading an
        // image back, and the stores MegaPad itself refuses or cancels.
        lc_reset();
        reg_write(R_STATUS, 64'h10);        // clear the refusals of the cases
        $display("\nStart: lend, name the image, release");
        // A four-word image at 0x0010_4000 and a data page at 0x0010_8000.
        xmem[17'h00800] = 64'hC0DE_0000_0000_0001;
        xmem[17'h00801] = 64'hC0DE_0000_0000_0002;
        xmem[17'h00802] = 64'hC0DE_0000_0000_0003;
        xmem[17'h00803] = 64'hC0DE_0000_0000_0004;
        lend(3'd0, 64'h0010_4000, 64'h0010_4020, 2'b01);
        lend(3'd1, 64'h0010_8000, 64'h0010_9000, 2'b11);
        reg_write(R_BOOT_ADDR, 64'h0010_4000);
        reg_write(R_BOOT_LEN, 64'd32);
        reg_write(R_BOOT_ARG, 64'h0010_5000);
        reg_write(R_CTRL, 64'h09);          // ENABLE, DOORBELL_EN, released
        #1;
        check("LM-1 leaves reset", core_rst_n == 1'b1);
        while (!boot_done && !boot_failed) @(negedge clk);
        @(negedge clk);
        runtime_ready = 1'b1;
        reg_read(R_STATUS);
        check("no request was refused during the fetch", boot_done && rd[4] == 1'b0);

        $display("\nLent memory");
        rt_access(64'h0010_8010, 64'h0BAD_C0FF_EE00_0042, 1'b1);
        check("LM-1 writes a word of lent memory", r_code == 2'd0);
        check("the word is in MegaPad's external memory",
              xmem[17'h01002] == 64'h0BAD_C0FF_EE00_0042);
        rt_access(64'h0010_8010, 64'd0, 1'b0);
        check("LM-1 reads it back",
              r_code == 2'd0 && r_data == 64'h0BAD_C0FF_EE00_0042);
        prev_n = phy_launches;
        rt_access(64'h0010_4008, 64'hFFFF, 1'b1);
        check("a write to the read-only image is refused", r_code == 2'd1);
        check("the image is unchanged",
              xmem[17'h00801] == 64'hC0DE_0000_0000_0002 &&
              phy_launches == prev_n);
        reg_read(R_FAULT_ADDR);
        check("MegaPad records the refused address",
              rd == 64'h0010_4008);
        reg_read(R_FAULT_INFO);
        check("with the reason and the write flag",
              rd[7:4] == 4'd4 && rd[2] == 1'b1);
        reg_write(R_STATUS, 64'h10);
        rt_access(64'h0001_0000, 64'd0, 1'b0);
        check("a read of MegaPad's Bank 0 is refused", r_code == 2'd1);
        reg_read(R_FAULT_INFO);
        check("as outside external memory", rd[7:4] == 4'd3);
        reg_write(R_STATUS, 64'h10);

        $display("\nBursts");
        xmem[17'h01010] = 64'hE000_0000_0000_0000;
        xmem[17'h01011] = 64'hE000_0000_0000_0001;
        xmem[17'h01012] = 64'hE000_0000_0000_0002;
        prev_n = phy_launches;
        rt_burst_read(64'h0010_8080, 8'd2);
        check("a read burst of three words arrives in order",
              b_n == 3 && b_code[2] == 2'd0 &&
              b_data[0] == 64'hE000_0000_0000_0000 &&
              b_data[2] == 64'hE000_0000_0000_0002);
        check("only the third is marked last",
              !b_last[0] && !b_last[1] && b_last[2]);
        check("each word is its own access at external memory",
              phy_launches == prev_n + 3);
        prev_n = phy_launches;
        rt_burst_read(64'h0010_8FF0, 8'd3);
        check("a burst that runs past the lent page is refused whole",
              b_n == 1 && b_code[0] == 2'd1 && b_last[0] &&
              phy_launches == prev_n);
        reg_write(R_STATUS, 64'h10);

        $display("\nDoorbells");
        mbox_write(8'd0, 4'h8, PORT_ID);    // MBOX_SEND from core 0
        repeat (2) @(negedge clk);
        #1;
        check("a core's signal waits in the mailbox until LM-1 takes it",
              dbi_valid == 1'b1 && mb_db_valid == 1'b1);
        dbi_ready = 1'b1;
        @(negedge clk);
        dbi_ready = 1'b0;
        repeat (2) @(negedge clk);
        #1;
        check("once LM-1 takes it the mailbox holds it no more",
              dbi_valid == 1'b0 && mb_db_valid == 1'b0);

        @(negedge clk);
        dbo_valid = 1'b1; dbo_dst = 8'd2;
        #1;
        while (!dbo_ready) begin
            @(negedge clk);
            #1;
        end
        @(negedge clk);
        dbo_valid = 1'b0;
        repeat (2) @(negedge clk);
        #1;
        check("LM-1's signal raises core 2's IPI line and external flag",
              ipi_out == 16'h0004 && ef_out == 16'h0004);
        mbox_write(8'd2, 4'hA, PORT_ID);    // MBOX_ACK from core 2
        #1;
        check("the core's acknowledgement by port ID clears both",
              ipi_out == 16'h0000 && ef_out == 16'h0000);

        // --------------------------------------------------------------
        $display("\nTACC.STORE into memory LM-1 lends");
        reg_read(R_WINDOW);
        check("MegaPad's WINDOW register names port 0's window",
              rd == WINDOW);
        reg_write(R_CTRL, 64'h29);          // ENABLE, DOORBELL_EN, BORROW_EN
        img_a = image_of(8'hA1);
        prev_u = wq_units; prev_w = lm1_writes;
        tacc_store(WINDOW + 64'h4040, EW_U8, img_a, 0);
        check("LM-1 refuses a store at its first unit, before it lends a range",
              wa_code_last == 2'd1 && wq_units == prev_u + 1 &&
              lm1_writes == prev_w);

        lm1_lend(8'd0, 64'h4000, 64'h4800, 1'b1);     // 2 KiB of shared memory
        watch_addr = 64'h4040; watch_image = img_a; watch_on = 1'b1;
        prev_n = phy_launches; prev_u = wq_units; prev_w = lm1_writes;
        tacc_store(WINDOW + 64'h4040, EW_U8, img_a, 0);
        $display("    (request to retirement: %0d cycles)", s_cycles);
        check("a U8 store sends four units, and LM-1 writes 32 words",
              s_done && s_fault == FAULT_NONE && wq_units == prev_u + 4 &&
              lm1_writes == prev_w + 32);
        check("every byte was there when LM-1 answered",
              img_at_answer && answer_cyc != 0);
        check("the image never reached MegaPad's external memory",
              phy_launches == prev_n);
        reg_read(R_STATUS);
        check("BORROW_BUSY is clear once the store retired", rd[6] == 1'b0);

        mbox_write(8'd0, 4'h8, PORT_ID);    // the core rings after its store
        repeat (3) @(negedge clk);
        #1;
        check("a doorbell rung after the store reaches LM-1",
              dbi_valid == 1'b1 && doorbell_cyc != 0);
        check("after LM-1's answer, with the image already in its memory",
              doorbell_cyc > answer_cyc && img_at_doorbell);
        dbi_ready = 1'b1;
        @(negedge clk);
        dbi_ready = 1'b0;
        watch_on = 1'b0;

        // Tile 0 loads the image's first and last words; tile 1 halts.
        @(negedge clk);
        cl_ext_en = 2'b11; cl_ext_we = 2'b11;
        cl_ext_addr[0] = '0; cl_ext_wdata[0] = 64'hC862_0000_DC40_4040;
        cl_ext_addr[1] = '0; cl_ext_wdata[1] = 64'hFC00_0000_FC00_0000;
        @(negedge clk);
        cl_ext_addr[0] = 10'd1; cl_ext_wdata[0] = 64'hFC00_0000_C882_00F8;
        cl_ext_en = 2'b01;
        @(negedge clk);
        cl_ext_en = 2'b00; cl_ext_we = 2'b00;
        // LI r2, 0x4040; LDR r3, [r2]; LDR r4, [r2 + 0xF8]; HALT
        tiles_go = 1'b1;
        prev_n = 0;
        while (tile_halted != 2'b11 && prev_n < 2000) begin
            @(negedge clk);
            prev_n++;
        end
        #1;
        check("a tile's own loads read the image's first and last words",
              tile_halted == 2'b11 &&
              u_cluster.gen_tiles[0].u_tile.u_core.u_regfile.regs[3] ==
                  img_a[63:0] &&
              u_cluster.gen_tiles[0].u_tile.u_core.u_regfile.regs[4] ==
                  img_a[2047:1984]);

        img_b = image_of(8'hB2);
        prev_u = wq_units; prev_w = lm1_writes;
        tacc_store(WINDOW + 64'h4140, EW_FP64, img_b, 0);
        check("an FP64 store sends one unit, and LM-1 writes 32 words",
              s_done && s_fault == FAULT_NONE && wq_units == prev_u + 1 &&
              lm1_writes == prev_w + 32);
        img_c = image_of(8'hC3);
        prev_u = wq_units;
        tacc_store(WINDOW + 64'h4140, EW_FP32, img_c, 0);
        check("an FP32 store sends two units",
              s_done && s_fault == FAULT_NONE && wq_units == prev_u + 2);

        $display("\nStores MegaPad refuses or cancels");
        reg_write(R_CTRL, 64'h09);          // BORROW_EN clear
        prev_u = wq_units; prev_a = wa_count;
        tacc_store(WINDOW + 64'h4240, EW_U8, img_a, 0);
        check("with BORROW_EN clear the store faults and nothing crosses",
              s_done && s_fault == FAULT_BUS && wq_units == prev_u &&
              wa_count == prev_a);
        reg_write(R_CTRL, 64'h29);
        prev_n = cancels_done; prev_w = lm1_writes;
        tacc_store(WINDOW + 64'h4240, EW_U8, image_of(8'hD4), 3);
        check("a store cancelled mid-write gets no response",
              !s_done && cancels_done == prev_n + 1);
        check("LM-1 answered the cancel CANCELLED, before its last word",
              wa_code_last == 2'd3 && lm1_writes > prev_w &&
              lm1_writes < prev_w + 32);
        reg_read(R_STATUS);
        check("the bridge holds no write afterwards", rd[6] == 1'b0);
        tacc_store(WINDOW + 64'h4240, EW_U8, img_a, 0);
        check("the next store into the range completes",
              s_done && s_fault == FAULT_NONE && lm1_holds(64'h4240, img_a));

        $display("\nStop and reset");
        reg_write(R_CTRL, 64'h0D);          // + STOP_REQ
        #1;
        check("LM-1's runtime is asked to stop", stop_req == 1'b1);
        check("LM-1 takes no new request", rt_req_ready == 1'b0);
        runtime_stopped = 1'b0; runtime_ready = 1'b0;
        reg_write(R_CTRL, 64'h0B);          // RESET_HOLD
        #1;
        check("LM-1 returns to reset", core_rst_n == 1'b0);

        $display("\n=== lm1_megapad_link_tb: %0d passed, %0d failed ===",
                 pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
