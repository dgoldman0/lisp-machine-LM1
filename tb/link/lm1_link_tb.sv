// ============================================================================
// lm1_link_tb.sv — LM-1's host link against a behavioural MegaPad
//
// Drives lm1_link and lm1_link_boot from a model of MegaPad's side of the
// link.  Covers the identity record in reset, the boot fetch, a failed
// fetch, the runtime's requests with their latency and response codes,
// cancellation, doorbells in both directions, the stop handshake and a
// return to reset.
// ============================================================================

module lm1_link_tb;
    logic clk = 1'b0;
    always #5 clk = ~clk;

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
    // Signals
    // ------------------------------------------------------------------
    logic        por_n;
    logic        lk_rq_valid, lk_rq_ready, lk_rq_wen, lk_rq_cancel;
    logic [63:0] lk_rq_addr, lk_rq_wdata;
    logic [1:0]  lk_rq_size;
    logic        lk_rs_valid, lk_rs_ready;
    logic [63:0] lk_rs_rdata;
    logic [1:0]  lk_rs_code;
    logic        lk_dbt_valid, lk_dbt_ready, lk_dbr_valid, lk_dbr_ready;
    logic [7:0]  lk_dbt_src, lk_dbr_dst;
    logic        sb_present, sb_rst_n, sb_stop_req, sb_stopped, sb_ready;
    logic        sb_id_req, sb_id_ack;
    logic [7:0]  sb_id_index;
    logic [63:0] sb_id_data, sb_boot_addr, sb_boot_len;

    logic        core_rst_n, stop_req, runtime_stopped, runtime_ready;
    logic [63:0] boot_addr, boot_len;

    logic        hm_req_valid, hm_req_ready, hm_req_we, hm_cancel;
    logic [63:0] hm_req_addr, hm_req_wdata;
    logic [1:0]  hm_req_size;
    logic        hm_resp_valid;
    logic [63:0] hm_resp_data;
    logic [1:0]  hm_resp_code;

    logic        rt_req_valid, rt_req_ready, rt_req_we, rt_cancel;
    logic [63:0] rt_req_addr, rt_req_wdata;
    logic [1:0]  rt_req_size;
    logic        rt_resp_valid;
    logic [63:0] rt_resp_data;
    logic [1:0]  rt_resp_code;

    logic        ld_valid, ld_ready;
    logic [63:0] ld_offset, ld_data;
    logic        boot_done, boot_failed;

    logic        dbi_valid, dbi_ready, dbo_valid, dbo_ready;
    logic [7:0]  dbi_src, dbo_dst;

    lm1_link u_link (
        .clk(clk), .por_n(por_n),
        .lk_rq_valid(lk_rq_valid), .lk_rq_ready(lk_rq_ready),
        .lk_rq_addr(lk_rq_addr), .lk_rq_wdata(lk_rq_wdata),
        .lk_rq_wen(lk_rq_wen), .lk_rq_size(lk_rq_size),
        .lk_rq_cancel(lk_rq_cancel),
        .lk_rs_valid(lk_rs_valid), .lk_rs_ready(lk_rs_ready),
        .lk_rs_rdata(lk_rs_rdata), .lk_rs_code(lk_rs_code),
        .lk_dbt_valid(lk_dbt_valid), .lk_dbt_ready(lk_dbt_ready),
        .lk_dbt_src(lk_dbt_src),
        .lk_dbr_valid(lk_dbr_valid), .lk_dbr_ready(lk_dbr_ready),
        .lk_dbr_dst(lk_dbr_dst),
        .sb_present(sb_present), .sb_rst_n(sb_rst_n),
        .sb_stop_req(sb_stop_req), .sb_stopped(sb_stopped),
        .sb_ready(sb_ready), .sb_id_req(sb_id_req),
        .sb_id_index(sb_id_index), .sb_id_ack(sb_id_ack),
        .sb_id_data(sb_id_data), .sb_boot_addr(sb_boot_addr),
        .sb_boot_len(sb_boot_len),
        .core_rst_n(core_rst_n), .stop_req(stop_req),
        .runtime_stopped(runtime_stopped), .runtime_ready(runtime_ready),
        .boot_addr(boot_addr), .boot_len(boot_len),
        .hm_req_valid(hm_req_valid), .hm_req_ready(hm_req_ready),
        .hm_req_addr(hm_req_addr), .hm_req_wdata(hm_req_wdata),
        .hm_req_we(hm_req_we), .hm_req_size(hm_req_size),
        .hm_cancel(hm_cancel),
        .hm_resp_valid(hm_resp_valid), .hm_resp_data(hm_resp_data),
        .hm_resp_code(hm_resp_code),
        .dbi_valid(dbi_valid), .dbi_ready(dbi_ready), .dbi_src(dbi_src),
        .dbo_valid(dbo_valid), .dbo_ready(dbo_ready), .dbo_dst(dbo_dst)
    );

    lm1_link_boot u_boot (
        .clk(clk), .rst_n(core_rst_n),
        .boot_addr(boot_addr), .boot_len(boot_len),
        .hm_req_valid(hm_req_valid), .hm_req_ready(hm_req_ready),
        .hm_req_addr(hm_req_addr), .hm_req_wdata(hm_req_wdata),
        .hm_req_we(hm_req_we), .hm_req_size(hm_req_size),
        .hm_cancel(hm_cancel),
        .hm_resp_valid(hm_resp_valid), .hm_resp_data(hm_resp_data),
        .hm_resp_code(hm_resp_code),
        .rt_req_valid(rt_req_valid), .rt_req_ready(rt_req_ready),
        .rt_req_addr(rt_req_addr), .rt_req_wdata(rt_req_wdata),
        .rt_req_we(rt_req_we), .rt_req_size(rt_req_size),
        .rt_cancel(rt_cancel),
        .rt_resp_valid(rt_resp_valid), .rt_resp_data(rt_resp_data),
        .rt_resp_code(rt_resp_code),
        .ld_valid(ld_valid), .ld_ready(ld_ready),
        .ld_offset(ld_offset), .ld_data(ld_data),
        .boot_done(boot_done), .boot_failed(boot_failed)
    );

    // ------------------------------------------------------------------
    // Behavioural MegaPad: lends [0x1000, 0x3000), answers after a delay
    // ------------------------------------------------------------------
    logic [63:0] hostmem [0:1023];
    int          host_delay = 2;
    int          host_requests = 0;

    typedef enum int { H_IDLE, H_BUSY, H_RESP } hstate_t;
    hstate_t     hstate = H_IDLE;
    int          hcount;
    logic [63:0] h_addr, h_wdata;
    logic        h_wen, h_cancelled;

    assign lk_rq_ready = (hstate == H_IDLE) && sb_rst_n;
    assign lk_rs_valid = (hstate == H_RESP);

    always_ff @(posedge clk) begin
        case (hstate)
            H_IDLE: begin
                if (lk_rq_valid && lk_rq_ready) begin
                    h_addr      <= lk_rq_addr;
                    h_wdata     <= lk_rq_wdata;
                    h_wen       <= lk_rq_wen;
                    h_cancelled <= 1'b0;
                    hcount      <= host_delay;
                    host_requests <= host_requests + 1;
                    hstate      <= H_BUSY;
                end
            end
            H_BUSY: begin
                if (lk_rq_cancel)
                    h_cancelled <= 1'b1;
                if (hcount == 0) begin
                    if (h_cancelled || lk_rq_cancel) begin
                        lk_rs_code  <= 2'd3;
                        lk_rs_rdata <= 64'd0;
                    end else if (h_addr < 64'h1000 || h_addr >= 64'h3000) begin
                        lk_rs_code  <= 2'd1;
                        lk_rs_rdata <= 64'd0;
                    end else if (h_wen) begin
                        hostmem[h_addr[12:3]] <= h_wdata;
                        lk_rs_code  <= 2'd0;
                        lk_rs_rdata <= 64'd0;
                    end else begin
                        lk_rs_code  <= 2'd0;
                        lk_rs_rdata <= hostmem[h_addr[12:3]];
                    end
                    hstate <= H_RESP;
                end else begin
                    hcount <= hcount - 1;
                end
            end
            H_RESP: begin
                if (lk_rs_ready)
                    hstate <= H_IDLE;
            end
            default: hstate <= H_IDLE;
        endcase
        if (!sb_rst_n)
            hstate <= H_IDLE;
    end

    // ------------------------------------------------------------------
    // Image words taken by the load port
    // ------------------------------------------------------------------
    logic [63:0] loaded      [0:15];
    logic [63:0] loaded_offs [0:15];
    int          loaded_n = 0;
    always_ff @(posedge clk) begin
        if (ld_valid && ld_ready && loaded_n < 16) begin
            loaded[loaded_n]      <= ld_data;
            loaded_offs[loaded_n] <= ld_offset;
            loaded_n <= loaded_n + 1;
        end
    end

    // ------------------------------------------------------------------
    // Tasks
    // ------------------------------------------------------------------
    logic [63:0] idw;
    // The handshake tasks change their inputs on the falling edge and read
    // outputs just after it, so no input changes on the edge that samples it.
    task automatic read_identity(input logic [7:0] index);
        @(negedge clk);
        sb_id_index = index;
        sb_id_req   = 1'b1;
        #1;
        while (!sb_id_ack) begin
            @(negedge clk);
            #1;
        end
        idw = sb_id_data;
        sb_id_req = 1'b0;
        @(negedge clk);
    endtask

    logic [63:0] r_data;
    logic [1:0]  r_code;
    logic        r_prompt;     // the response reached the runtime at once
    // One runtime access; cancel_after n raises rt_cancel n cycles in.
    task automatic rt_access(input logic [63:0] addr, input logic [63:0] wdata,
                             input logic we, input int cancel_after);
        int waited;
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = addr; rt_req_wdata = wdata;
        rt_req_we = we; rt_req_size = 2'd3;
        #1;
        while (!rt_req_ready) begin
            @(negedge clk);
            #1;
        end
        @(negedge clk);                 // the rising edge between took it
        rt_req_valid = 1'b0;
        waited = 0;
        r_prompt = 1'b1;
        #1;
        while (!rt_resp_valid) begin
            if (lk_rs_valid)
                r_prompt = 1'b0;        // the host answered, the runtime waits
            waited++;
            rt_cancel = (cancel_after != 0 && waited >= cancel_after);
            @(negedge clk);
            #1;
        end
        r_data = rt_resp_data;
        r_code = rt_resp_code;
        @(negedge clk);
        rt_cancel = 1'b0;
    endtask

    int prev_n;

    initial begin
        por_n = 1'b0;
        sb_rst_n = 1'b0; sb_stop_req = 1'b0;
        sb_id_req = 1'b0; sb_id_index = 8'd0;
        sb_boot_addr = 64'd0; sb_boot_len = 64'd0;
        lk_dbt_valid = 1'b0; lk_dbt_src = 8'd0; lk_dbr_ready = 1'b0;
        runtime_stopped = 1'b0; runtime_ready = 1'b0;
        rt_req_valid = 1'b0; rt_req_addr = 64'd0; rt_req_wdata = 64'd0;
        rt_req_we = 1'b0; rt_req_size = 2'd3; rt_cancel = 1'b0;
        ld_ready = 1'b1;
        dbi_ready = 1'b0; dbo_valid = 1'b0; dbo_dst = 8'd0;
        lk_rs_code = 2'd0; lk_rs_rdata = 64'd0;
        for (int i = 0; i < 1024; i++) hostmem[i] = 64'd0;
        repeat (4) @(posedge clk);
        por_n = 1'b1;
        repeat (2) @(posedge clk);

        $display("\nIdentity, read while LM-1 is held in reset");
        check("a chiplet reports itself present", sb_present == 1'b1);
        check("LM-1's own logic is in reset", core_rst_n == 1'b0);
        read_identity(8'd0);
        check("word 0 is CHIP and contract version 1",
              idw == 64'h4348_4950_0000_0001);
        read_identity(8'd1);
        check("word 1 names the device LM1", idw == 64'h0000_0000_0031_4D4C);
        read_identity(8'd7);
        check("a word beyond the record reads zero", idw == 64'd0);
        check("the link asks for nothing in reset",
              lk_rq_valid == 1'b0 && sb_ready == 1'b0);

        $display("\nBoot fetch");
        hostmem[10'h200] = 64'h1111_1111_0000_0001;
        hostmem[10'h201] = 64'h2222_2222_0000_0002;
        hostmem[10'h202] = 64'h3333_3333_0000_0003;
        sb_boot_addr = 64'h1000;
        sb_boot_len  = 64'd20;          // two and a half words
        @(posedge clk);
        sb_rst_n = 1'b1;
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        check("the fetch completes", boot_done && !boot_failed);
        check("a length of 20 bytes loads three words", loaded_n == 3);
        check("the words arrive in order with their offsets",
              loaded[0] == 64'h1111_1111_0000_0001 && loaded_offs[0] == 0 &&
              loaded[1] == 64'h2222_2222_0000_0002 && loaded_offs[1] == 8 &&
              loaded[2] == 64'h3333_3333_0000_0003 && loaded_offs[2] == 16);
        runtime_ready = 1'b1;
        @(posedge clk);
        check("READY follows the runtime", sb_ready == 1'b1);

        $display("\nThe runtime's requests");
        rt_access(64'h2000, 64'hABCD_EF01_2345_6789, 1'b1, 0);
        check("a write to lent memory completes", r_code == 2'd0);
        rt_access(64'h2000, 64'd0, 1'b0, 0);
        check("the word reads back",
              r_code == 2'd0 && r_data == 64'hABCD_EF01_2345_6789);
        check("the response reaches the runtime in the cycle the host gives it",
              r_prompt == 1'b1);
        rt_access(64'h8000, 64'd0, 1'b0, 0);
        check("a refusal reaches the runtime as REFUSED", r_code == 2'd1);
        host_delay = 8;
        rt_access(64'h2008, 64'h5, 1'b1, 3);
        check("a cancelled request is answered CANCELLED", r_code == 2'd3);
        host_delay = 2;
        rt_access(64'h2000, 64'd0, 1'b0, 0);
        check("the port serves requests after a cancellation",
              r_code == 2'd0 && r_data == 64'hABCD_EF01_2345_6789);

        // The request reaches the link in the cycle the runtime raises it.
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = 64'h2000; rt_req_we = 1'b0;
        #1;
        check("a request is on the link in the cycle it is raised",
              lk_rq_valid == 1'b1 && lk_rq_addr == 64'h2000);
        @(negedge clk);
        rt_req_valid = 1'b0;
        #1;
        while (!rt_resp_valid) begin
            @(negedge clk);
            #1;
        end
        @(negedge clk);

        $display("\nDoorbells");
        @(posedge clk);
        lk_dbt_valid <= 1'b1; lk_dbt_src <= 8'd2;
        @(posedge clk);
        #1;
        check("a doorbell from MegaPad is offered locally with its source",
              dbi_valid == 1'b1 && dbi_src == 8'd2);
        check("it waits until the local side takes it",
              lk_dbt_ready == 1'b0);
        dbi_ready <= 1'b1;
        @(posedge clk);
        #1;
        check("taking it locally takes it from the link",
              lk_dbt_ready == 1'b1);
        @(posedge clk);
        lk_dbt_valid <= 1'b0; dbi_ready <= 1'b0;
        dbo_valid <= 1'b1; dbo_dst <= 8'd0;
        @(posedge clk);
        #1;
        check("a local doorbell goes out with its destination",
              lk_dbr_valid == 1'b1 && lk_dbr_dst == 8'd0);
        lk_dbr_ready <= 1'b1;
        @(posedge clk);
        #1;
        check("the link's ready reaches the local side", dbo_ready == 1'b1);
        @(posedge clk);
        dbo_valid <= 1'b0; lk_dbr_ready <= 1'b0;

        $display("\nStop");
        @(posedge clk);
        sb_stop_req <= 1'b1;
        @(posedge clk);
        #1;
        check("the runtime is asked to stop", stop_req == 1'b1);
        check("no new request is taken", rt_req_ready == 1'b0);
        dbo_valid <= 1'b1;
        @(posedge clk);
        #1;
        check("no new doorbell goes out", lk_dbr_valid == 1'b0);
        dbo_valid <= 1'b0;
        check("LM-1 reports stopped only once the runtime has stopped",
              sb_stopped == 1'b0);
        runtime_stopped <= 1'b1;
        @(posedge clk);
        #1;
        check("with the runtime stopped and nothing in flight it reports so",
              sb_stopped == 1'b1);
        sb_stop_req <= 1'b0; runtime_stopped <= 1'b0;
        @(posedge clk);
        rt_access(64'h2000, 64'd0, 1'b0, 0);
        check("released from the stop, the port serves requests",
              r_code == 2'd0);

        $display("\nA failed fetch");
        sb_rst_n <= 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check("reset closes the port", core_rst_n == 1'b0 &&
              rt_req_ready == 1'b0 && sb_ready == 1'b0);
        sb_boot_addr <= 64'h9000;       // outside what the host lends
        sb_boot_len  <= 64'd16;
        prev_n = loaded_n;
        @(posedge clk);
        sb_rst_n <= 1'b1;
        @(posedge clk);
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        check("a refused image ends the fetch as failed",
              boot_failed && !boot_done);
        check("nothing is loaded from it", loaded_n == prev_n);
        check("the runtime's port stays closed", rt_req_ready == 1'b0);

        $display("\nNo image");
        sb_rst_n <= 1'b0;
        repeat (2) @(posedge clk);
        sb_boot_len <= 64'd0;
        @(posedge clk);
        sb_rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        #1;
        check("a length of zero finishes at once", boot_done == 1'b1);

        $display("\n=== lm1_link_tb: %0d passed, %0d failed ===",
                 pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
