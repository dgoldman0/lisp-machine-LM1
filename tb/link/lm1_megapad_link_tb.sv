// ============================================================================
// lm1_megapad_link_tb.sv — LM-1's link against MegaPad's RTL
//
// Joins the two real ends of the link: LM-1's lm1_link and lm1_link_boot on
// one side, MegaPad's port bridge, mailbox, external-memory arbiter and
// controller on the other, over a behavioural external-memory PHY.  The
// bench plays MegaPad's software and one of its cores, and LM-1's runtime.
//
// It follows the start order of MegaPad's link specification: read the
// identity record, lend the image and the data, name the image, release the
// chiplet, and let it fetch.  Then it runs requests, refusals, doorbells in
// both directions, the stop handshake and a return to reset.
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
    logic        lk_dbt_valid, lk_dbt_ready, lk_dbr_valid, lk_dbr_ready;
    logic [7:0]  lk_dbt_src, lk_dbr_dst;
    logic        sb_present, sb_rst_n, sb_stop_req, sb_stopped, sb_ready;
    logic        sb_id_req, sb_id_ack;
    logic [7:0]  sb_id_index;
    logic [63:0] sb_id_data, sb_boot_addr, sb_boot_len, sb_boot_arg;

    // ------------------------------------------------------------------
    // MegaPad's side
    // ------------------------------------------------------------------
    localparam logic [7:0] PORT_ID = 8'd16;

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

    mp64_chiplet_port #(
        .RANGES    (8),
        .RANGE_BITS(3),
        .PORT_ID   (PORT_ID),
        .EXT_BASE  (64'h0000_0000_0010_0000),
        .EXT_LIMIT (64'h0000_0000_0020_0000)
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

    logic        ld_valid;
    logic [63:0] ld_offset, ld_data;
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
        .dbi_valid(dbi_valid), .dbi_ready(dbi_ready), .dbi_src(dbi_src),
        .dbo_valid(dbo_valid), .dbo_ready(dbo_ready), .dbo_dst(dbo_dst)
    );

    lm1_link_boot u_boot (
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
        .ld_valid(ld_valid), .ld_ready(1'b1),
        .ld_offset(ld_offset), .ld_data(ld_data),
        .boot_done(boot_done), .boot_failed(boot_failed)
    );

    logic [63:0] loaded [0:15];
    int          loaded_n = 0;
    int          link_requests = 0;     // request messages the bridge took
    always_ff @(posedge clk) begin
        if (ld_valid && loaded_n < 16) begin
            loaded[loaded_n] <= ld_data;
            loaded_n <= loaded_n + 1;
        end
        if (lk_rq_valid && lk_rq_ready)
            link_requests <= link_requests + 1;
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

    int prev_n;

    initial begin
        for (int i = 0; i < 131072; i++) xmem[i] = 64'd0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (6) @(posedge clk);

        $display("\nIdentity, read through MegaPad's port registers");
        reg_read(R_STATUS);
        check("MegaPad sees a chiplet on the port", rd[0] == 1'b1);
        check("LM-1 is held in reset", core_rst_n == 1'b0);
        read_identity(8'd0);
        check("word 0 is CHIP and contract version 1",
              idw == 64'h4348_4950_0000_0001);
        read_identity(8'd1);
        check("word 1 names the device LM1", idw == 64'h0000_0000_0031_4D4C);

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
        check("the control block's address reaches LM-1's runtime",
              boot_arg == 64'h0010_5000);
        while (!boot_done && !boot_failed) @(negedge clk);
        @(negedge clk);
        check("LM-1 fetches its image through the bridge",
              boot_done && !boot_failed);
        check("the four words arrive in order",
              loaded_n == 4 &&
              loaded[0] == 64'hC0DE_0000_0000_0001 &&
              loaded[1] == 64'hC0DE_0000_0000_0002 &&
              loaded[2] == 64'hC0DE_0000_0000_0003 &&
              loaded[3] == 64'hC0DE_0000_0000_0004);
        check("the four-word image crosses the link as one burst",
              link_requests == 1);
        runtime_ready = 1'b1;
        reg_read(R_STATUS);
        check("MegaPad reads READY", rd[2] == 1'b1);
        check("no request was refused during the fetch", rd[4] == 1'b0);

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
        check("a core's signal reaches LM-1 with the core's ID",
              dbi_valid == 1'b1 && dbi_src == 8'd0);
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

        $display("\nStop and reset");
        reg_write(R_CTRL, 64'h0D);          // + STOP_REQ
        #1;
        check("LM-1's runtime is asked to stop", stop_req == 1'b1);
        check("LM-1 takes no new request", rt_req_ready == 1'b0);
        runtime_stopped = 1'b1;
        reg_read(R_STATUS);
        check("MegaPad reads STOPPED", rd[1] == 1'b1);
        runtime_stopped = 1'b0; runtime_ready = 1'b0;
        reg_write(R_CTRL, 64'h0B);          // RESET_HOLD
        #1;
        check("LM-1 returns to reset", core_rst_n == 1'b0);
        prev_n = loaded_n;
        reg_write(R_CTRL, 64'h09);
        @(negedge clk);
        while (!boot_done && !boot_failed) @(negedge clk);
        @(negedge clk);
        check("released again, LM-1 fetches its image once more",
              boot_done && loaded_n == prev_n + 4);

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
