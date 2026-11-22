// ============================================================================
// lm1_link_tb.sv — LM-1's host link against a behavioural MegaPad
//
// Drives lm1_link and lm1_link_boot from a model of MegaPad's side of the
// link, with a behavioural LM-1 memory behind the memory write port.  It
// first runs the shared link conformance cases of tb/link/link_cases.txt
// (link_cases.svh), the list the chiplet module and lm1_megapad_link_tb run
// too: the identity record, the boot fetch and its failures, the ranges
// LM-1 lends and MegaPad's writes into them, doorbells and the stop.  Then
// it covers what only this bench can show: the local side's signals, the
// runtime's requests with their latency and response codes, cancellation,
// the handshakes of the doorbells, cycle counts, memory errors, a slow
// memory and a slow writer, cancels and a withdrawal during a write, the
// stop handshake with a write in flight, a return to reset during one, and
// a fetch that a failing memory word ends.
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

    logic        lend_valid, lend_ready, lend_write;
    logic [7:0]  lend_index, lend_ranges;
    logic [63:0] lend_base, lend_limit;

    logic        lw_valid, lw_ready, lw_error;   // the link's writes
    logic [63:0] lw_addr, lw_data;
    logic        mw_valid, mw_ready, mw_error;   // LM-1's memory write port
    logic [63:0] mw_addr, mw_data;
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

    localparam logic [63:0] LOAD_BASE = 64'h100;

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
        .rt_resp_valid(rt_resp_valid), .rt_resp_ready(rt_resp_ready),
        .rt_resp_data(rt_resp_data), .rt_resp_code(rt_resp_code),
        .rt_resp_last(rt_resp_last),
        .lw_valid(lw_valid), .lw_ready(lw_ready), .lw_addr(lw_addr),
        .lw_data(lw_data), .lw_error(lw_error),
        .mw_valid(mw_valid), .mw_ready(mw_ready), .mw_addr(mw_addr),
        .mw_data(mw_data), .mw_error(mw_error),
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
    // LM-1's memory: 4 KiB at LM-1's addresses 0x000-0xFFF behind the boot
    // module's memory write port.  It takes a word in every cycle, or in
    // every third one while mem_slow is set, and fails a word outside it,
    // an unaligned one and the one at mem_fail.
    // ------------------------------------------------------------------
    localparam int          LMEM_WORDS = 512;
    localparam logic [63:0] LMEM_BYTES = 64'(LMEM_WORDS * 8);
    localparam logic [63:0] FILL       = 64'hEEEE_EEEE_EEEE_EEEE;
    logic [63:0] lmem [0:LMEM_WORDS-1];
    logic        mem_slow;
    logic [63:0] mem_fail;
    int          ncyc = 0;              // falling edges so far
    int          mem_writes = 0;        // words written
    int          last_write_cyc = 0;    // ncyc before the edge of the last
    always @(negedge clk) ncyc = ncyc + 1;

    assign mw_ready = !mem_slow || (ncyc % 3 == 0);
    assign mw_error = (mw_addr >= LMEM_BYTES) || (mw_addr[2:0] != 3'd0) ||
                      (mw_addr == mem_fail);
    always_ff @(posedge clk) begin
        if (mw_valid && mw_ready && !mw_error) begin
            lmem[mw_addr[11:3]] <= mw_data;
            mem_writes          <= mem_writes + 1;
            last_write_cyc      <= ncyc;
        end
    end

    // The word a write tagged tag puts at word i of its unit u.
    function automatic logic [63:0] word_of(input logic [15:0] tag, input int u,
                                            input int i);
        return {tag, 16'(u), 16'h00C0, 16'(i)};
    endfunction
    function automatic logic [511:0] unit_of(input logic [15:0] tag, input int u);
        logic [511:0] v;
        for (int i = 0; i < 8; i++)
            v[i*64 +: 64] = word_of(tag, u, i);
        return v;
    endfunction
    // Whether LM-1's memory holds, from addr, n units of tag and then z
    // units of zeros.
    function automatic logic holds(input logic [63:0] addr, input int n,
                                   input int z, input logic [15:0] tag);
        logic ok = 1'b1;
        for (int u = 0; u < n + z; u++)
            for (int i = 0; i < 8; i++)
                if (lmem[(addr[11:0] >> 3) + 12'(u * 8 + i)] !=
                    ((u < n) ? word_of(tag, u, i) : 64'd0))
                    ok = 1'b0;
        return ok;
    endfunction
    // Whether n words from addr still hold the fill.
    function automatic logic untouched(input logic [63:0] addr, input int n);
        logic ok = 1'b1;
        for (int i = 0; i < n; i++)
            if (lmem[(addr[11:0] >> 3) + 12'(i)] != FILL)
                ok = 1'b0;
        return ok;
    endfunction
    task automatic fill_lmem();
        for (int i = 0; i < LMEM_WORDS; i++) lmem[i] = FILL;
    endtask

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

    // MegaPad's writer, driving the channel as its port bridge does: one
    // unit offered at a time with the header on each, nothing more offered
    // once the write is answered, and cancel held from cancel_at cycles in
    // (never when negative) until the answer.  gap holds each further unit
    // back that many cycles.  The write ends unanswered if LM-1 returns to
    // reset.
    logic [1:0] w_code;
    logic       w_answered;
    int         w_taken;        // units LM-1 took
    int         w_cycles;       // falling edges from the first offer to the end
    int         w_answer_cyc;   // ncyc before the edge that took the answer
    // With w_from_src set, the units sent are w_src's instead of unit_of's.
    logic [511:0] w_src [0:3];
    logic         w_from_src = 1'b0;
    task automatic mp_write(input logic [63:0] addr, input int len,
                            input int zeros, input logic [15:0] tag,
                            input int cancel_at, input int gap);
        int u, cyc, hold;
        u = 0; cyc = 0; hold = 0;
        w_answered = 1'b0; w_taken = 0;
        @(negedge clk);
        lk_wq_addr = addr; lk_wq_len = 8'(len); lk_wq_zeros = 8'(zeros);
        lk_wa_ready = 1'b1;
        while (!w_answered && sb_rst_n && cyc < 5000) begin
            if (cancel_at >= 0 && cyc >= cancel_at)
                lk_wq_cancel = 1'b1;
            lk_wq_valid = (u <= len) && !lk_wq_cancel && (hold == 0);
            lk_wq_data  = (w_from_src && u < 4) ? w_src[u] : unit_of(tag, u);
            #1;
            if (lk_wa_valid) begin
                w_answered   = 1'b1;
                w_code       = lk_wa_code;
                w_answer_cyc = ncyc;
            end
            if (lk_wq_valid && lk_wq_ready) begin
                u++;
                w_taken++;
                hold = gap;
            end else if (hold > 0) begin
                hold--;
            end
            @(negedge clk);
            cyc++;
        end
        w_cycles = cyc;
        lk_wq_valid = 1'b0; lk_wq_cancel = 1'b0; lk_wa_ready = 1'b0;
    endtask

    // LM-1's runtime installs or withdraws a range.
    int lend_cyc;               // ncyc before the edge that took it
    task automatic lend(input logic [7:0] idx, input logic [63:0] base,
                        input logic [63:0] limit, input logic wr);
        int guard = 0;
        @(negedge clk);
        lend_valid = 1'b1; lend_index = idx; lend_base = base;
        lend_limit = limit; lend_write = wr;
        #1;
        while (!lend_ready && guard < 5000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        lend_cyc = ncyc;
        @(negedge clk);                 // the rising edge between took it
        lend_valid = 1'b0;
    endtask

    // ------------------------------------------------------------------
    // The shared link cases (link_cases.svh), done this bench's way.  The
    // behavioural MegaPad lends [0x1000, 0x3000) for reading and writing;
    // a case's image lies at 0x1200, and 0x9000 is lent to nobody.  LM-1's
    // memory is the 4 KiB above, from LOAD_BASE on for the image.
    // ------------------------------------------------------------------
    localparam logic [63:0] LC_IMAGE = 64'h1200, LC_UNLENT = 64'h9000;
    int          lc_mark;               // host_requests at the release
    logic [63:0] lc_before [0:63];      // LM-1's memory from the load base then
    logic [63:0] lc_near [0:33];        // LM-1's memory around a store, before it
    string       lc_path;

    task automatic lc_reset();
        @(negedge clk);
        sb_rst_n = 1'b0; sb_stop_req = 1'b0;
        runtime_ready = 1'b0; runtime_stopped = 1'b0;
        lk_dbt_valid = 1'b0; dbi_ready = 1'b0;
        dbo_valid = 1'b0; lk_dbr_ready = 1'b0;
        repeat (3) @(negedge clk);
    endtask

    task automatic lc_identity(input logic [7:0] index, input logic [63:0] value);
        read_identity(index);
        lc_check($sformatf("identity word %0d reads %h", index, value), idw == value);
    endtask

    task automatic lc_image(input int n, input logic [63:0] value);
        for (int k = 0; k < n; k++)
            hostmem[LC_IMAGE[12:3] + 10'(k)] = value + 64'(k);
    endtask

    task automatic lc_release(input logic [63:0] len, input logic [63:0] arg,
                              input logic lent);
        for (int k = 0; k < 64; k++)
            lc_before[k] = lmem[LOAD_BASE[11:3] + 9'(k)];
        lc_mark = host_requests;
        @(negedge clk);
        sb_boot_addr = lent ? LC_IMAGE : LC_UNLENT;
        sb_boot_len  = len;
        sb_boot_arg  = arg;
        sb_rst_n     = 1'b1;
    endtask

    task automatic lc_boot(input logic failed);
        int guard = 0;
        while (!boot_done && !boot_failed && guard < 20000) begin
            @(posedge clk);
            guard++;
        end
        repeat (40) @(posedge clk);     // a cancelled burst's responses drain
        lc_check(failed ? "the fetch fails" : "the fetch completes",
                 failed ? (boot_failed && !boot_done) : (boot_done && !boot_failed));
    endtask

    task automatic lc_boot_arg(input logic [63:0] value);
        lc_check("the boot argument reaches the runtime", boot_arg == value);
    endtask

    task automatic lc_loaded(input int n, input logic [63:0] value);
        logic ok = 1'b1;
        for (int k = 0; k < n; k++)
            if (lmem[LOAD_BASE[11:3] + 9'(k)] != value + 64'(k))
                ok = 1'b0;
        lc_check($sformatf("%0d words land in order from the load base", n), ok);
        lc_check("the word after them is untouched",
                 lmem[LOAD_BASE[11:3] + 9'(n)] == lc_before[n]);
    endtask

    task automatic lc_requests(input int n);
        lc_check($sformatf("the fetch made %0d requests", n), host_requests - lc_mark == n);
    endtask

    task automatic lc_wait();
        repeat (64) @(posedge clk);
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
        repeat (2) @(posedge clk);
        #1;
        lc_check($sformatf("READY reads %0d", b), sb_ready == b);
    endtask

    task automatic lc_stopped(input logic b);
        repeat (2) @(posedge clk);
        #1;
        lc_check($sformatf("STOPPED reads %0d", b), sb_stopped == b);
    endtask

    task automatic lc_stop(input logic b);
        @(negedge clk);
        sb_stop_req = b;
    endtask

    task automatic lc_lend(input logic [7:0] index, input logic [63:0] base,
                           input logic [63:0] limit, input logic w);
        lend(index, base, limit, w);
    endtask

    task automatic lc_store(input int units, input int zeros, input logic [63:0] addr,
                            input logic [7:0] seed, input logic [1:0] code);
        logic ok = 1'b1;
        int   at = int'(addr[11:3]);
        for (int k = 0; k < 34; k++)
            lc_near[k] = lmem[9'(at - 1 + k)];
        for (int u = 0; u < units; u++)
            for (int i = 0; i < 8; i++)
                w_src[u][i*64 +: 64] = lc_image_word(seed, u * 8 + i);
        w_from_src = 1'b1;
        mp_write(addr, units - 1, zeros, 16'h0, -1, 0);
        w_from_src = 1'b0;
        lc_check($sformatf("LM-1 answers %0d", code), w_answered && w_code == code);
        lc_check("the words beside the image are untouched",
                 lmem[9'(at - 1)] == lc_near[0] && lmem[9'(at + 32)] == lc_near[33]);
        for (int w = 0; w < 32; w++)
            if (code == 2'd0 ? lmem[9'(at + w)] != ((w < 8 * units) ? lc_image_word(seed, w)
                                                                    : 64'd0)
                             : lmem[9'(at + w)] != lc_near[w + 1])
                ok = 1'b0;
        lc_check(code == 2'd0 ? "LM-1's memory holds the image and its zeros"
                              : "LM-1's memory is untouched", ok);
    endtask

    // A doorbell is offered and held until taken, as the handshake has it.
    task automatic lc_ring_in(input logic [7:0] src);
        @(negedge clk);
        lk_dbt_valid = 1'b1;
        lk_dbt_src   = src;
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
        @(negedge clk);
        dbi_ready = 1'b1;               // taken at the next rising edge
        @(negedge clk);
        dbi_ready = 1'b0; lk_dbt_valid = 1'b0;
    endtask

    task automatic lc_ring_out(input logic [7:0] dst);
        @(negedge clk);
        dbo_valid = 1'b1;
        dbo_dst   = dst;
    endtask

    task automatic lc_rung_out(input logic [7:0] dst);
        int guard = 0;
        #1;
        while (!lk_dbr_valid && guard < 1000) begin
            @(negedge clk);
            #1;
            guard++;
        end
        lc_check($sformatf("MegaPad takes a doorbell for %0d", dst),
                 lk_dbr_valid && lk_dbr_dst == dst);
        @(negedge clk);
        lk_dbr_ready = 1'b1;            // taken at the next rising edge
        @(negedge clk);
        lk_dbr_ready = 1'b0; dbo_valid = 1'b0;
    endtask

    task automatic lc_no_ring_out();
        #1;
        lc_check("no doorbell leaves LM-1", lk_dbr_valid == 1'b0);
    endtask

    `include "link_cases.svh"

    int prev_n, prev_w;
    logic seen_stopped;

    initial begin
        por_n = 1'b0;
        sb_rst_n = 1'b0; sb_stop_req = 1'b0;
        sb_id_req = 1'b0; sb_id_index = 8'd0;
        sb_boot_addr = 64'd0; sb_boot_len = 64'd0; sb_boot_arg = 64'd0;
        lk_dbt_valid = 1'b0; lk_dbt_src = 8'd0; lk_dbr_ready = 1'b0;
        lk_wq_valid = 1'b0; lk_wq_cancel = 1'b0; lk_wq_addr = 64'd0;
        lk_wq_data = 512'd0; lk_wq_len = 8'd0; lk_wq_zeros = 8'd0;
        lk_wa_ready = 1'b0;
        lend_valid = 1'b0; lend_index = 8'd0; lend_base = 64'd0;
        lend_limit = 64'd0; lend_write = 1'b0;
        runtime_stopped = 1'b0; runtime_ready = 1'b0;
        rt_req_valid = 1'b0; rt_req_addr = 64'd0; rt_req_wdata = 64'd0;
        rt_req_we = 1'b0; rt_req_size = 2'd3; rt_cancel = 1'b0;
        rt_req_len = 8'd0; rt_resp_ready = 1'b1;
        lk_rs_last = 1'b1;
        mem_slow = 1'b0; mem_fail = '1;
        dbi_ready = 1'b0; dbo_valid = 1'b0; dbo_dst = 8'd0;
        lk_rs_code = 2'd0; lk_rs_rdata = 64'd0;
        for (int i = 0; i < 1024; i++) hostmem[i] = 64'd0;
        fill_lmem();
        repeat (4) @(posedge clk);
        por_n = 1'b1;
        repeat (2) @(posedge clk);

        $display("\nThe shared link cases");
        if (!$value$plusargs("cases=%s", lc_path))
            lc_path = "tb/link/link_cases.txt";
        run_link_cases(lc_path);

        // The cases this bench alone runs follow: the signals of LM-1's
        // local side, cycle counts, cancels, and failing and slow memories.
        lc_reset();
        fill_lmem();
        $display("\nIn reset");
        check("a chiplet reports itself present", sb_present == 1'b1);
        check("LM-1's own logic is in reset", core_rst_n == 1'b0);
        check("the link asks for nothing in reset",
              lk_rq_valid == 1'b0 && sb_ready == 1'b0);
        check("in reset LM-1 takes no write and lends nothing",
              lk_wq_ready == 1'b0 && lk_wa_valid == 1'b0 &&
              lend_ready == 1'b0 && mw_valid == 1'b0);

        // The runtime starts from a short image, as the shared cases check.
        hostmem[10'h200] = 64'h1111_1111_0000_0001;
        sb_boot_addr = 64'h1000;
        sb_boot_len  = 64'd8;
        sb_boot_arg  = 64'h2040;
        @(posedge clk);
        sb_rst_n = 1'b1;
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        runtime_ready = 1'b1;
        @(posedge clk);

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
        check("a doorbell waits until the local side takes it",
              dbi_valid == 1'b1 && lk_dbt_ready == 1'b0);
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
        check("a local doorbell is offered at once, before the link takes it",
              lk_dbr_valid == 1'b1 && dbo_ready == 1'b0);
        lk_dbr_ready <= 1'b1;
        @(posedge clk);
        #1;
        check("the link's ready reaches the local side", dbo_ready == 1'b1);
        @(posedge clk);
        dbo_valid <= 1'b0; lk_dbr_ready <= 1'b0;

        // --------------------------------------------------------------
        $display("\nMegaPad's writes into memory LM-1 lends");
        check("the table reports two ranges", lend_ranges == 8'd2);
        prev_w = mem_writes;
        mp_write(64'h400, 0, 0, 16'hA000, -1, 0);
        check("a refusal comes after the write's first message, writing nothing",
              w_answered && w_code == 2'd1 && w_taken == 1 && mem_writes == prev_w);

        lend(8'd0, 64'h400, 64'h800, 1'b1);     // sixteen units
        prev_w = mem_writes;
        mp_write(64'h400, 3, 0, 16'hA100, -1, 0);
        check("an admitted write of four units gives the memory 32 words",
              w_code == 2'd0 && w_taken == 4 && mem_writes == prev_w + 32);
        check("the answer leaves after the last word is written",
              w_answer_cyc > last_write_cyc);
        check("the units pass back to back: 35 cycles from the first offer",
              w_cycles == 35);
        mp_write(64'h7C0, 1, 0, 16'hB100, -1, 0);
        check("LM-1 refuses before it takes a write's second unit",
              w_code == 2'd1 && w_taken == 1);

        $display("\nA failing memory, a slow memory, a slow writer");
        fill_lmem();
        mem_fail = 64'h448;
        prev_w = mem_writes;
        mp_write(64'h400, 1, 0, 16'hC000, -1, 0);
        check("a word the memory fails ends the write with MEM_ERROR",
              w_code == 2'd2 && w_taken == 2);
        check("the words before it are written, none after",
              mem_writes == prev_w + 9 && holds(64'h400, 1, 0, 16'hC000) &&
              lmem[9'h88] == word_of(16'hC000, 1, 0) && untouched(64'h448, 8));
        mem_fail = '1;
        mem_slow = 1'b1;
        mp_write(64'h400, 3, 1, 16'hC100, -1, 0);
        check("a memory that takes a word every third cycle gets every word",
              w_code == 2'd0 && holds(64'h400, 4, 1, 16'hC100));
        check("and the write takes longer", w_cycles > 2 * 40);
        mem_slow = 1'b0;
        mp_write(64'h600, 2, 0, 16'hC200, -1, 12);
        check("units offered twelve cycles apart are taken as they come",
              w_code == 2'd0 && w_taken == 3 && holds(64'h600, 3, 0, 16'hC200));

        $display("\nCancels");
        fill_lmem();
        prev_w = mem_writes;
        mp_write(64'h400, 3, 0, 16'hD000, 5, 0);
        check("a write cancelled during its first unit is answered CANCELLED",
              w_answered && w_code == 2'd3 && w_taken == 1);
        check("the words taken before the cancel are written, none after",
              mem_writes > prev_w && mem_writes < prev_w + 8 &&
              lmem[9'h80] == word_of(16'hD000, 0, 0) &&
              untouched(64'h400 + 64'((mem_writes - prev_w) * 8),
                        32 - (mem_writes - prev_w)));
        fill_lmem();
        prev_w = mem_writes;
        mp_write(64'h400, 1, 0, 16'hD100, 15, 20);
        check("a write cancelled while LM-1 awaits its next unit ends there",
              w_code == 2'd3 && w_taken == 1 && mem_writes == prev_w + 8 &&
              holds(64'h400, 1, 0, 16'hD100) && untouched(64'h440, 8));
        prev_w = mem_writes;
        mp_write(64'h400, 1, 0, 16'hD200, 0, 0);
        check("a cancel before LM-1 takes the first message is answered too",
              w_answered && w_code == 2'd3 && w_taken == 0 &&
              mem_writes == prev_w);
        mp_write(64'h400, 1, 0, 16'hD300, -1, 0);
        check("after a cancel the next write is served",
              w_code == 2'd0 && holds(64'h400, 2, 0, 16'hD300));

        $display("\nWithdrawing a range");
        fill_lmem();
        mem_slow = 1'b1;
        fork
            mp_write(64'h400, 3, 0, 16'hE000, -1, 0);
            begin
                repeat (6) @(negedge clk);
                lend(8'd0, 64'h400, 64'h800, 1'b0);
            end
        join
        mem_slow = 1'b0;
        check("a write admitted before the withdrawal completes OK",
              w_code == 2'd0 && holds(64'h400, 4, 0, 16'hE000));
        check("the withdrawal is taken only once that write has ended",
              lend_cyc > w_answer_cyc);
        lend(8'd0, 64'h400, 64'h800, 1'b1);

        $display("\nStop");
        @(posedge clk);
        sb_stop_req <= 1'b1;
        @(posedge clk);
        #1;
        check("the runtime is asked to stop", stop_req == 1'b1);
        check("no new request is taken", rt_req_ready == 1'b0);
        check("LM-1 reports stopped only once the runtime has stopped",
              sb_stopped == 1'b0);
        runtime_stopped <= 1'b1;
        @(posedge clk);
        #1;
        seen_stopped = 1'b0;
        mem_slow = 1'b1;
        fork
            mp_write(64'h400, 1, 0, 16'hF000, -1, 0);
            begin
                repeat (4) @(negedge clk);
                #1;
                seen_stopped = sb_stopped;
            end
        join
        mem_slow = 1'b0;
        check("while one of MegaPad's writes is held it is not stopped",
              seen_stopped == 1'b0 && w_code == 2'd0);
        #1;
        check("and stopped again once that write has ended",
              sb_stopped == 1'b1);
        sb_stop_req <= 1'b0; runtime_stopped <= 1'b0;
        @(posedge clk);
        rt_access(64'h2000, 64'd0, 1'b0, 0);
        check("released from the stop, the port serves requests",
              r_code == 2'd0);

        $display("\nReset during a write");
        mem_slow = 1'b1;
        fork
            mp_write(64'h400, 3, 0, 16'hF100, -1, 0);
            begin
                repeat (10) @(negedge clk);
                sb_rst_n = 1'b0;
            end
        join
        mem_slow = 1'b0;
        #1;
        check("a write under way when LM-1 returns to reset gets no answer",
              !w_answered && lk_wa_valid == 1'b0 && mw_valid == 1'b0);
        sb_boot_len = 64'd0;
        @(negedge clk);
        sb_rst_n = 1'b1;
        repeat (3) @(negedge clk);

        $display("\nA failed fetch");
        sb_rst_n <= 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check("reset closes the port", core_rst_n == 1'b0 &&
              rt_req_ready == 1'b0 && sb_ready == 1'b0);
        sb_boot_addr <= 64'h9000;       // outside what the host lends
        sb_boot_len  <= 64'd16;
        @(posedge clk);
        sb_rst_n <= 1'b1;
        @(posedge clk);
        while (!boot_done && !boot_failed) @(posedge clk);
        @(posedge clk);
        check("after a failed fetch the runtime's port stays closed",
              boot_failed && rt_req_ready == 1'b0);

        $display("\nA word the memory fails ends the fetch");
        sb_rst_n <= 1'b0;
        repeat (2) @(posedge clk);
        for (int i = 0; i < 20; i++)
            hostmem[10'h240 + i] = 64'hFEED_0000 + 64'(i);
        sb_boot_addr <= 64'h1200;
        sb_boot_len  <= 64'd160;        // twenty words
        mem_fail = LOAD_BASE + 64'd16;  // the third
        prev_w = mem_writes;
        @(posedge clk);
        sb_rst_n <= 1'b1;
        @(posedge clk);
        while (!boot_done && !boot_failed) @(posedge clk);
        repeat (40) @(posedge clk);
        check("the fetch fails at the word the memory refused",
              boot_failed && mem_writes == prev_w + 2);
        check("the burst under way is cancelled and the link is idle again",
              hstate == H_IDLE && lk_rq_valid == 1'b0);
        mem_fail = '1;

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
