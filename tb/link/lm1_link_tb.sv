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

    logic        core_rst_n, stop_req, runtime_stopped, runtime_ready;
    logic [63:0] boot_addr, boot_len, boot_arg;

    logic        hm_req_valid, hm_req_ready, hm_req_we, hm_cancel;
    logic [63:0] hm_req_addr, hm_req_wdata;
    logic [1:0]  hm_req_size;
    logic [7:0]  hm_req_len;
    logic        hm_resp_valid, hm_resp_ready, hm_resp_last;
    logic [63:0] hm_resp_data;
    logic [1:0]  hm_resp_code;

    logic        rt_req_valid, rt_req_ready, rt_req_we, rt_cancel;
    logic [63:0] rt_req_addr, rt_req_wdata;
    logic [1:0]  rt_req_size;
    logic [7:0]  rt_req_len;
    logic        rt_resp_valid, rt_resp_ready, rt_resp_last;
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
        .rt_resp_valid(rt_resp_valid), .rt_resp_ready(rt_resp_ready),
        .rt_resp_data(rt_resp_data), .rt_resp_code(rt_resp_code),
        .rt_resp_last(rt_resp_last),
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

    typedef enum int { H_IDLE, H_BUSY, H_RESP, H_WDATA } hstate_t;
    hstate_t     hstate = H_IDLE;
    int          hcount;
    logic [63:0] h_addr, h_wdata;
    logic [7:0]  h_left;        // words of the burst after the current one
    logic        h_wen, h_cancelled, h_refuse;

    assign lk_rq_ready = (hstate == H_IDLE || hstate == H_WDATA) && sb_rst_n;
    assign lk_rs_valid = (hstate == H_RESP);

    always_ff @(posedge clk) begin
        case (hstate)
            H_IDLE: begin
                if (lk_rq_valid && lk_rq_ready) begin
                    h_addr      <= lk_rq_addr;
                    h_wdata     <= lk_rq_wdata;
                    h_wen       <= lk_rq_wen;
                    h_left      <= lk_rq_len;
                    h_cancelled <= 1'b0;
                    // The whole burst is admitted or refused at its start.
                    h_refuse    <= (lk_rq_addr < 64'h1000) ||
                                   (lk_rq_addr + 64'd8 * (64'(lk_rq_len) + 64'd1)
                                    > 64'h3000);
                    hcount      <= host_delay;
                    host_requests <= host_requests + 1;
                    hstate      <= H_BUSY;
                end
            end
            H_BUSY: begin
                if (lk_rq_cancel)
                    h_cancelled <= 1'b1;
                if (hcount == 0) begin
                    lk_rs_rdata <= 64'd0;
                    lk_rs_last  <= 1'b1;
                    hstate      <= H_RESP;
                    if (h_cancelled || lk_rq_cancel) begin
                        lk_rs_code <= 2'd3;
                    end else if (h_refuse) begin
                        lk_rs_code <= 2'd1;
                    end else if (h_wen) begin
                        hostmem[h_addr[12:3]] <= h_wdata;
                        lk_rs_code <= 2'd0;
                        if (h_left != 8'd0) begin
                            h_addr <= h_addr + 64'd8;
                            h_left <= h_left - 8'd1;
                            hstate <= H_WDATA;
                        end
                    end else begin
                        lk_rs_code  <= 2'd0;
                        lk_rs_rdata <= hostmem[h_addr[12:3]];
                        lk_rs_last  <= (h_left == 8'd0);
                    end
                end else begin
                    hcount <= hcount - 1;
                end
            end
            H_WDATA: begin
                if (lk_rq_cancel) begin
                    lk_rs_code <= 2'd3; lk_rs_rdata <= 64'd0;
                    lk_rs_last <= 1'b1;
                    hstate     <= H_RESP;
                end else if (lk_rq_valid) begin
                    h_wdata <= lk_rq_wdata;
                    hcount  <= host_delay;
                    hstate  <= H_BUSY;
                end
            end
            H_RESP: begin
                if (lk_rs_ready) begin
                    if (lk_rs_last) begin
                        hstate <= H_IDLE;
                    end else begin
                        h_addr <= h_addr + 64'd8;
                        h_left <= h_left - 8'd1;
                        hcount <= host_delay;
                        hstate <= H_BUSY;
                    end
                end
            end
            default: hstate <= H_IDLE;
        endcase
        if (!sb_rst_n)
            hstate <= H_IDLE;
    end

    // ------------------------------------------------------------------
    // Image words taken by the load port
    // ------------------------------------------------------------------
    logic [63:0] loaded      [0:63];
    logic [63:0] loaded_offs [0:63];
    int          loaded_n = 0;
    always_ff @(posedge clk) begin
        if (ld_valid && ld_ready && loaded_n < 64) begin
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
        rt_req_we = we; rt_req_size = 2'd3; rt_req_len = 8'd0;
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

    // Bursts on the runtime's port.
    logic [63:0] b_data [0:15];
    logic [1:0]  b_code [0:15];
    logic        b_last [0:15];
    int          b_n;

    task automatic rt_burst_read(input logic [63:0] addr, input logic [7:0] len);
        logic done;
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = addr; rt_req_we = 1'b0;
        rt_req_size = 2'd3; rt_req_len = len;
        #1;
        while (!rt_req_ready) begin
            @(negedge clk);
            #1;
        end
        @(negedge clk);
        rt_req_valid = 1'b0; rt_req_len = 8'd0;
        b_n = 0;
        done = 1'b0;
        #1;
        while (!done) begin
            if (rt_resp_valid) begin
                b_data[b_n] = rt_resp_data;
                b_code[b_n] = rt_resp_code;
                b_last[b_n] = rt_resp_last;
                done = rt_resp_last;
                b_n++;
            end
            @(negedge clk);
            #1;
        end
    endtask

    // Sends len + 1 words, base, base + 1, ...; stops when LM-1's end
    // answers early.  Leaves the one response in b_code[0] and b_last[0].
    task automatic rt_burst_write(input logic [63:0] addr, input logic [7:0] len,
                                  input logic [63:0] base);
        logic ended;
        @(negedge clk);
        rt_req_valid = 1'b1; rt_req_addr = addr; rt_req_wdata = base;
        rt_req_we = 1'b1; rt_req_size = 2'd3; rt_req_len = len;
        #1;
        while (!rt_req_ready) begin
            @(negedge clk);
            #1;
        end
        ended = 1'b0;
        for (int k = 1; k <= int'(len) && !ended; k++) begin
            @(negedge clk);
            rt_req_wdata = base + 64'(k); rt_req_len = 8'd0;
            #1;
            while (!rt_req_ready && !rt_resp_valid) begin
                @(negedge clk);
                #1;
            end
            if (rt_resp_valid)
                ended = 1'b1;
        end
        if (!ended)
            @(negedge clk);
        rt_req_valid = 1'b0; rt_req_we = 1'b0; rt_req_len = 8'd0;
        #1;
        while (!rt_resp_valid) begin
            @(negedge clk);
            #1;
        end
        b_code[0] = rt_resp_code;
        b_last[0] = rt_resp_last;
        @(negedge clk);
    endtask

    int prev_n;

    initial begin
        por_n = 1'b0;
        sb_rst_n = 1'b0; sb_stop_req = 1'b0;
        sb_id_req = 1'b0; sb_id_index = 8'd0;
        sb_boot_addr = 64'd0; sb_boot_len = 64'd0; sb_boot_arg = 64'd0;
        lk_dbt_valid = 1'b0; lk_dbt_src = 8'd0; lk_dbr_ready = 1'b0;
        runtime_stopped = 1'b0; runtime_ready = 1'b0;
        rt_req_valid = 1'b0; rt_req_addr = 64'd0; rt_req_wdata = 64'd0;
        rt_req_we = 1'b0; rt_req_size = 2'd3; rt_cancel = 1'b0;
        rt_req_len = 8'd0; rt_resp_ready = 1'b1;
        lk_rs_last = 1'b1;
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
        sb_boot_arg  = 64'h2040;
        @(posedge clk);
        sb_rst_n = 1'b1;
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        check("the fetch completes", boot_done && !boot_failed);
        check("the boot argument reaches the runtime", boot_arg == 64'h2040);
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

        $display("\nBursts on the runtime's port");
        hostmem[10'h280] = 64'hB0; hostmem[10'h281] = 64'hB1;
        hostmem[10'h282] = 64'hB2; hostmem[10'h283] = 64'hB3;
        prev_n = host_requests;
        rt_burst_read(64'h1400, 8'd3);
        check("a read burst of four words is one request on the link",
              host_requests == prev_n + 1);
        check("its four words arrive in order, the fourth marked last",
              b_n == 4 && b_data[0] == 64'hB0 && b_data[3] == 64'hB3 &&
              !b_last[0] && !b_last[2] && b_last[3]);
        rt_burst_write(64'h1800, 8'd2, 64'hAA00);
        check("a write burst of three words gets one response, OK",
              b_code[0] == 2'd0 && b_last[0]);
        check("its words are in the host's memory",
              hostmem[10'h300] == 64'hAA00 && hostmem[10'h301] == 64'hAA01 &&
              hostmem[10'h302] == 64'hAA02);
        rt_burst_write(64'h2FF0, 8'd3, 64'hCC00);
        check("a write burst the host refuses ends at once as REFUSED",
              b_code[0] == 2'd1 && b_last[0]);
        rt_access(64'h2000, 64'd0, 1'b0, 0);
        check("a single access follows the refused burst",
              r_code == 2'd0 && r_data == 64'hABCD_EF01_2345_6789);

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

        $display("\nA long image, fetched in bursts");
        sb_rst_n <= 1'b0;
        repeat (2) @(posedge clk);
        for (int i = 0; i < 20; i++)
            hostmem[10'h240 + i] = 64'hF00D_0000 + 64'(i);
        sb_boot_addr <= 64'h1200;
        sb_boot_len  <= 64'd160;        // twenty words
        prev_n = loaded_n;
        @(posedge clk);
        host_requests = 0;
        sb_rst_n <= 1'b1;
        @(posedge clk);
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        check("twenty words load", boot_done && loaded_n == prev_n + 20);
        check("the first and the last are right",
              loaded[prev_n] == 64'hF00D_0000 &&
              loaded[prev_n + 19] == 64'hF00D_0013 &&
              loaded_offs[prev_n + 19] == 64'd152);
        check("in three requests of eight, eight and four words",
              host_requests == 3);

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
