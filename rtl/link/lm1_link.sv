// ============================================================================
// LM-1 Host Link — the chiplet's side of the link to MegaPad
//
// LM-1 is a chiplet beside MegaPad.  MegaPad lends it ranges of external
// memory, and LM-1 reads and writes them across the link at MegaPad's own
// addresses.  LM-1 in turn lends MegaPad ranges of its own memory, into
// which MegaPad's tile engines store their accumulator images.  This module
// is LM-1's end of that link, built to MegaPad's chiplet link
// specification, contract version 1 (docs/chiplet-link.md in the MegaPad
// repository).
//
// Link channels are valid/ready handshakes:
//   memory request   (to MegaPad):   addr, wdata, wen, size, len; one
//                                    outstanding
//   memory response  (from MegaPad): rdata, code, last
// A request with len = n asks for a burst of n + 1 consecutive 64-bit words.
// A read burst is one request and n + 1 responses.  A write burst is n + 1
// request messages, each with its word of data, and one response.  A
// response other than OK ends a burst.
//   write            (from MegaPad): addr, data, len, zeros; one outstanding
//   write answer     (to MegaPad):   code
// MegaPad's write covers len + 1 units of 64 bytes from addr, LM-1's own
// address, followed by zeros units of zeros.  It is len + 1 messages, each
// with its unit, and LM-1 takes addr, len and zeros from the first.
//   doorbell in      (from MegaPad): src, the ID of the participant
//   doorbell out     (to MegaPad):   dst, the ID to signal
//
// The sideband answers while LM-1 is held in reset: it reports the identity
// record, takes the reset line and the stop request, and carries the address
// and length of the image LM-1 starts from.
//
// Local side:
//   - a host-memory port, through which the boot loader and the runtime
//     reach lent memory;
//   - a lend port, through which LM-1's runtime lends MegaPad ranges of
//     LM-1's memory and withdraws them;
//   - a memory write port, through which MegaPad's writes reach LM-1's
//     memory one 64-bit word at a time;
//   - doorbell ports, meant for a tile's hardware message queue;
//   - the reset for LM-1's own logic and the stop handshake with the runtime.
//
// An uncontended request reaches the link in the cycle it is raised, and its
// response returns in the cycle it arrives, so the endpoint adds no cycle to
// an access.
//
// MegaPad's writes.  LM-1 admits a write when its address is aligned to 64
// bytes and one range lent with the write right contains every byte the
// write covers, its zero units included.  The check runs once, on the
// write's first message, so the whole write is admitted or refused, and a
// refused write leaves LM-1's memory untouched.  An admitted write passes
// through a buffer of one unit: the held unit's eight words go to the memory
// write port, the next unit is taken from the link in the cycle the held
// unit's last word is taken, and the zero units follow as words of zeros.
// The answer is
//   OK          every word is written.  The memory write port makes a word
//               it has taken visible to LM-1's own accesses from the next
//               cycle, and the answer leaves in the cycle after the last
//               word, so every byte can be read by then.
//   REFUSED     the write is misaligned or lies outside every lent range.
//   MEM_ERROR   the memory failed a word; the words before it are written.
//   CANCELLED   MegaPad cancelled the write; the words taken before the
//               cancel are written.
// A refusal, an error or a cancel ends the write at once, before its last
// unit when one remains, and MegaPad sends no more of it.  A cancel that
// comes before LM-1 has taken the write's first message is answered
// CANCELLED too: MegaPad awaits one answer for every write it has begun.
// A word offered to the memory write port stays offered until the port
// takes it, so a cancel takes effect at a word's boundary.
//
// The lend port installs a range in one step, from its base, exclusive limit
// and write right; a write right of zero withdraws it.  The port takes an
// operation only while no write is held, so the ranges never change under a
// write, and a withdrawal completes only after MegaPad's writes into the
// range have ended: once the port has taken it, nothing more lands there.
// LM-1's own reset clears every range.
//
// Clocking: the link is specified at the level of messages.  This module
// places both ends on one clock; a link layer between them carries the same
// handshakes across the die boundary.
// ============================================================================
module lm1_link #(
    // Identity word 1: the device name, first character in the low byte.
    parameter logic [63:0] DEVICE_NAME = 64'h0000_0000_0031_4D4C,  // "LM1"
    // Ranges LM-1 lends MegaPad at once, 1 to 255.  A range serves one
    // function's writes, and the one function that writes into LM-1 today
    // is the tile engines' TACC.STORE.  A second range lets the runtime lend
    // a replacement before it withdraws the range it replaces, so that
    // function never goes without one.  Each range is two 64-bit registers
    // and two comparators; lend_ranges reports the count to the runtime.
    parameter int          RANGES      = 2
) (
    input  logic        clk,
    input  logic        por_n,          // power-on reset; the sideband's only reset

    // ---- Link: memory request and response ----
    output logic        lk_rq_valid,
    input  logic        lk_rq_ready,
    output logic [63:0] lk_rq_addr,
    output logic [63:0] lk_rq_wdata,
    output logic        lk_rq_wen,
    output logic [1:0]  lk_rq_size,
    output logic [7:0]  lk_rq_len,
    output logic        lk_rq_cancel,
    input  logic        lk_rs_valid,
    output logic        lk_rs_ready,
    input  logic [63:0] lk_rs_rdata,
    input  logic [1:0]  lk_rs_code,
    input  logic        lk_rs_last,

    // ---- Link: MegaPad's writes into memory LM-1 lends, and the answer ----
    input  logic         lk_wq_valid,
    output logic         lk_wq_ready,
    input  logic [63:0]  lk_wq_addr,    // LM-1's address, aligned to 64 bytes
    input  logic [511:0] lk_wq_data,    // one unit, its first byte in the low bits
    input  logic [7:0]   lk_wq_len,     // units beyond the first
    input  logic [7:0]   lk_wq_zeros,   // units of zeros after the data
    input  logic         lk_wq_cancel,  // abandon the outstanding write
    output logic         lk_wa_valid,
    input  logic         lk_wa_ready,
    output logic [1:0]   lk_wa_code,

    // ---- Link: doorbells ----
    input  logic        lk_dbt_valid,   // MegaPad -> LM-1
    output logic        lk_dbt_ready,
    input  logic [7:0]  lk_dbt_src,
    output logic        lk_dbr_valid,   // LM-1 -> MegaPad
    input  logic        lk_dbr_ready,
    output logic [7:0]  lk_dbr_dst,

    // ---- Sideband ----
    output logic        sb_present,
    input  logic        sb_rst_n,
    input  logic        sb_stop_req,
    output logic        sb_stopped,
    output logic        sb_ready,
    input  logic        sb_id_req,
    input  logic [7:0]  sb_id_index,
    output logic        sb_id_ack,
    output logic [63:0] sb_id_data,
    input  logic [63:0] sb_boot_addr,
    input  logic [63:0] sb_boot_len,
    input  logic [63:0] sb_boot_arg,

    // ---- Local: reset and lifecycle ----
    output logic        core_rst_n,     // reset for LM-1's own logic
    output logic        stop_req,       // the runtime is asked to stop
    input  logic        runtime_stopped,
    input  logic        runtime_ready,
    output logic [63:0] boot_addr,
    output logic [63:0] boot_len,
    output logic [63:0] boot_arg,       // address of the control block

    // ---- Local: host-memory port ----
    input  logic        hm_req_valid,
    output logic        hm_req_ready,
    input  logic [63:0] hm_req_addr,
    input  logic [63:0] hm_req_wdata,
    input  logic        hm_req_we,
    input  logic [1:0]  hm_req_size,
    input  logic [7:0]  hm_req_len,     // words beyond the first
    input  logic        hm_cancel,      // abandon the outstanding request
    output logic        hm_resp_valid,
    input  logic        hm_resp_ready,
    output logic [63:0] hm_resp_data,
    output logic [1:0]  hm_resp_code,
    output logic        hm_resp_last,

    // ---- Local: the ranges LM-1 lends MegaPad ----
    input  logic        lend_valid,     // install or withdraw one range
    output logic        lend_ready,     // taken only while no write is held
    input  logic [7:0]  lend_index,     // an index of RANGES or beyond is ignored
    input  logic [63:0] lend_base,
    input  logic [63:0] lend_limit,     // exclusive
    input  logic        lend_write,     // admit MegaPad's writes; 0 withdraws
    output logic [7:0]  lend_ranges,    // the ranges the table holds

    // ---- Local: MegaPad's writes into LM-1's memory ----
    output logic        mw_valid,       // held until mw_ready
    input  logic        mw_ready,
    output logic [63:0] mw_addr,        // LM-1's address of a 64-bit word
    output logic [63:0] mw_data,
    input  logic        mw_error,       // with mw_ready: the memory failed the word

    // ---- Local: doorbells ----
    output logic        dbi_valid,      // a participant signalled LM-1
    input  logic        dbi_ready,
    output logic [7:0]  dbi_src,
    input  logic        dbo_valid,      // signal a participant
    output logic        dbo_ready,
    input  logic [7:0]  dbo_dst
);

    // Contract version 1 below the ASCII tag "CHIP".
    localparam logic [63:0] ID_WORD0 = 64'h4348_4950_0000_0001;

    // Codes on the link's responses and answers.
    localparam logic [1:0] LK_OK        = 2'd0;
    localparam logic [1:0] LK_REFUSED   = 2'd1;
    localparam logic [1:0] LK_MEM_ERROR = 2'd2;
    localparam logic [1:0] LK_CANCELLED = 2'd3;

    if (RANGES < 1 || RANGES > 255) begin : g_bad_ranges
        $error("lm1_link: RANGES must lie from 1 to 255");
    end

    // ---------------------------------------------------------------
    // Sideband
    // ---------------------------------------------------------------
    assign sb_present = 1'b1;
    assign core_rst_n = por_n && sb_rst_n;
    assign stop_req   = sb_stop_req;
    assign boot_addr  = sb_boot_addr;
    assign boot_len   = sb_boot_len;
    assign boot_arg   = sb_boot_arg;
    assign sb_ready   = core_rst_n && runtime_ready;

    // The identity record is readable in reset, so it resets on por_n alone.
    always_ff @(posedge clk) begin
        if (!por_n) begin
            sb_id_ack  <= 1'b0;
            sb_id_data <= 64'd0;
        end else begin
            sb_id_ack <= sb_id_req && !sb_id_ack;
            if (sb_id_req && !sb_id_ack) begin
                case (sb_id_index)
                    8'd0:    sb_id_data <= ID_WORD0;
                    8'd1:    sb_id_data <= DEVICE_NAME;
                    default: sb_id_data <= 64'd0;
                endcase
            end
        end
    end

    // ---------------------------------------------------------------
    // Memory request and response
    // ---------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE  = 2'd0,    // no request outstanding
        ST_SEND  = 2'd1,    // a latched first message waits for the link
        ST_WDATA = 2'd2,    // a write burst sends its further words
        ST_WAIT  = 2'd3     // await the response, or the rest of them
    } state_t;

    state_t      state;
    logic [63:0] held_addr;
    logic [63:0] held_wdata;
    logic        held_we;
    logic [1:0]  held_size;
    logic [7:0]  held_len;
    logic [7:0]  left;          // words of a write burst still to send
    logic        cancel_lat;

    // New work is taken while LM-1 runs and no stop has been asked.
    logic open;
    assign open = core_rst_n && !sb_stop_req;

    // The first message of a request is taken when idle, even if the link
    // is busy that cycle.  A write burst's further words pass straight
    // through as the link takes them.
    logic take_first, take_data;
    assign take_first = hm_req_valid && open && (state == ST_IDLE);
    assign take_data  = hm_req_valid && (state == ST_WDATA) && lk_rq_ready;

    assign hm_req_ready = (open && state == ST_IDLE) ||
                          (state == ST_WDATA && lk_rq_ready);

    assign lk_rq_valid = (state == ST_SEND) || take_first ||
                         (state == ST_WDATA && hm_req_valid);
    assign lk_rq_addr  = (state == ST_SEND) ? held_addr  : hm_req_addr;
    assign lk_rq_wdata = (state == ST_SEND) ? held_wdata : hm_req_wdata;
    assign lk_rq_wen   = (state == ST_SEND) ? held_we    : hm_req_we;
    assign lk_rq_size  = (state == ST_SEND) ? held_size  : hm_req_size;
    assign lk_rq_len   = (state == ST_SEND) ? held_len
                       : (state == ST_IDLE) ? hm_req_len : 8'd0;

    // A response can arrive while a write burst still has words to send:
    // MegaPad ends a burst at its first failure.
    logic answering;
    assign answering = (state == ST_WAIT) || (state == ST_WDATA);

    assign lk_rs_ready  = answering && hm_resp_ready;
    assign lk_rq_cancel = answering && (hm_cancel || cancel_lat);

    assign hm_resp_valid = answering && lk_rs_valid;
    assign hm_resp_data  = lk_rs_rdata;
    assign hm_resp_code  = lk_rs_code;
    assign hm_resp_last  = lk_rs_last;

    logic resp_end;
    assign resp_end = hm_resp_valid && hm_resp_ready && lk_rs_last;

    always_ff @(posedge clk) begin
        if (!core_rst_n) begin
            state      <= ST_IDLE;
            held_addr  <= 64'd0;
            held_wdata <= 64'd0;
            held_we    <= 1'b0;
            held_size  <= 2'd3;
            held_len   <= 8'd0;
            left       <= 8'd0;
            cancel_lat <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    cancel_lat <= 1'b0;
                    if (take_first) begin
                        held_addr  <= hm_req_addr;
                        held_wdata <= hm_req_wdata;
                        held_we    <= hm_req_we;
                        held_size  <= hm_req_size;
                        held_len   <= hm_req_len;
                        left       <= hm_req_len;
                        if (!lk_rq_ready)
                            state <= ST_SEND;
                        else if (hm_req_we && hm_req_len != 8'd0)
                            state <= ST_WDATA;
                        else
                            state <= ST_WAIT;
                    end
                end
                ST_SEND: begin
                    if (hm_cancel)
                        cancel_lat <= 1'b1;
                    if (lk_rq_ready)
                        state <= (held_we && held_len != 8'd0) ? ST_WDATA
                                                               : ST_WAIT;
                end
                ST_WDATA: begin
                    if (hm_cancel)
                        cancel_lat <= 1'b1;
                    if (resp_end) begin
                        state <= ST_IDLE;
                    end else if (take_data) begin
                        left <= left - 8'd1;
                        if (left == 8'd1)
                            state <= ST_WAIT;
                    end
                end
                ST_WAIT: begin
                    if (hm_cancel)
                        cancel_lat <= 1'b1;
                    if (resp_end)
                        state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // MegaPad's writes into memory LM-1 lends
    // ---------------------------------------------------------------
    typedef enum logic [2:0] {
        W_IDLE   = 3'd0,    // no write held; its first message is taken here
        W_CHECK  = 3'd1,    // the first unit is held; the write is checked
        W_UNIT   = 3'd2,    // the held unit's words go to memory
        W_NEXT   = 3'd3,    // the held unit is written; the next is awaited
        W_ZERO   = 3'd4,    // the zero units' words go to memory
        W_ANSWER = 3'd5     // the answer waits for MegaPad
    } wstate_t;

    wstate_t      wstate;
    logic [511:0] w_unit;       // the buffer of one unit
    logic [2:0]   w_word;       // the word of the unit now offered
    logic [63:0]  w_addr;       // LM-1's address of that word
    logic [7:0]   w_units;      // data units still to take from the link
    logic [7:0]   w_zeros;      // zero units still to write
    logic [1:0]   w_code;
    logic         w_cancel;     // MegaPad has cancelled the held write

    logic [63:0]       rng_base  [RANGES];
    logic [63:0]       rng_limit [RANGES];
    logic [RANGES-1:0] rng_write;

    // The admission check, on the header the first message brought: while
    // the state is W_CHECK, w_units still holds len and w_zeros zeros.  A
    // write covers at most 511 units, so its end needs 65 bits only where
    // the address runs past the top of the 64-bit space.
    logic [9:0]  w_span;        // units the write covers
    logic [64:0] w_end;         // the address after its last byte
    assign w_span = {2'b00, w_units} + 10'd1 + {2'b00, w_zeros};
    assign w_end  = {1'b0, w_addr} + {49'd0, w_span, 6'd0};

    logic w_lent;
    always_comb begin
        w_lent = 1'b0;
        for (int i = 0; i < RANGES; i++)
            if (rng_write[i] && w_addr >= rng_base[i] &&
                w_end <= {1'b0, rng_limit[i]})
                w_lent = 1'b1;
    end

    logic w_admit;
    assign w_admit = (w_addr[5:0] == 6'd0) && w_lent;

    logic w_cancel_now;
    assign w_cancel_now = w_cancel || lk_wq_cancel;

    logic mw_go;
    assign mw_go = mw_valid && mw_ready;

    // The held unit's last word is taken without fault, so the buffer frees
    // in this cycle and can take the next unit at the same edge.
    logic w_unit_ends;
    assign w_unit_ends = (wstate == W_UNIT) && mw_go && !mw_error &&
                         (w_word == 3'd7);

    assign lk_wq_ready = core_rst_n && !w_cancel_now &&
                         ((wstate == W_IDLE) || (wstate == W_NEXT) ||
                          (w_unit_ends && w_units != 8'd0));

    logic wq_go;
    assign wq_go = lk_wq_valid && lk_wq_ready;

    assign lk_wa_valid = core_rst_n && (wstate == W_ANSWER);
    assign lk_wa_code  = w_code;

    assign mw_valid = core_rst_n &&
                      ((wstate == W_UNIT) || (wstate == W_ZERO));
    assign mw_addr  = w_addr;
    assign mw_data  = (wstate == W_UNIT) ? w_unit[w_word*64 +: 64] : 64'd0;

    assign lend_ready  = core_rst_n && (wstate == W_IDLE);
    assign lend_ranges = 8'(RANGES);

    logic lend_go;
    assign lend_go = lend_valid && lend_ready;

    always_ff @(posedge clk) begin
        if (!core_rst_n) begin
            wstate   <= W_IDLE;
            w_unit   <= '0;
            w_word   <= 3'd0;
            w_addr   <= 64'd0;
            w_units  <= 8'd0;
            w_zeros  <= 8'd0;
            w_code   <= LK_OK;
            w_cancel <= 1'b0;
            for (int i = 0; i < RANGES; i++) begin
                rng_base[i]  <= 64'd0;
                rng_limit[i] <= 64'd0;
            end
            rng_write <= '0;
        end else begin
            // ---------------- The ranges ----------------
            for (int i = 0; i < RANGES; i++) begin
                if (lend_go && lend_index == 8'(i)) begin
                    rng_base[i]  <= lend_base;
                    rng_limit[i] <= lend_limit;
                    rng_write[i] <= lend_write;
                end
            end

            // A cancel holds for the rest of the write it reaches.
            if (wstate == W_IDLE)
                w_cancel <= 1'b0;
            else if (lk_wq_cancel)
                w_cancel <= 1'b1;

            // ---------------- The write ----------------
            case (wstate)
                W_IDLE: begin
                    if (wq_go) begin
                        w_unit  <= lk_wq_data;
                        w_addr  <= lk_wq_addr;
                        w_units <= lk_wq_len;
                        w_zeros <= lk_wq_zeros;
                        w_word  <= 3'd0;
                        wstate  <= W_CHECK;
                    end else if (lk_wq_cancel) begin
                        // MegaPad cancelled a write before its first
                        // message reached LM-1.
                        w_code <= LK_CANCELLED;
                        wstate <= W_ANSWER;
                    end
                end

                W_CHECK: begin
                    if (w_cancel_now) begin
                        w_code <= LK_CANCELLED;
                        wstate <= W_ANSWER;
                    end else if (w_admit) begin
                        wstate <= W_UNIT;
                    end else begin
                        w_code <= LK_REFUSED;
                        wstate <= W_ANSWER;
                    end
                end

                W_UNIT: begin
                    if (mw_go) begin
                        if (mw_error) begin
                            w_code <= LK_MEM_ERROR;
                            wstate <= W_ANSWER;
                        end else begin
                            w_addr <= w_addr + 64'd8;
                            w_word <= w_word + 3'd1;
                            if (w_cancel_now) begin
                                w_code <= LK_CANCELLED;
                                wstate <= W_ANSWER;
                            end else if (w_word == 3'd7) begin
                                if (w_units != 8'd0) begin
                                    // The next unit, taken at this edge when
                                    // MegaPad offers it; w_word wraps to 0.
                                    if (wq_go) begin
                                        w_unit  <= lk_wq_data;
                                        w_units <= w_units - 8'd1;
                                    end else begin
                                        wstate <= W_NEXT;
                                    end
                                end else if (w_zeros != 8'd0) begin
                                    wstate <= W_ZERO;
                                end else begin
                                    w_code <= LK_OK;
                                    wstate <= W_ANSWER;
                                end
                            end
                        end
                    end
                end

                W_NEXT: begin
                    if (w_cancel_now) begin
                        w_code <= LK_CANCELLED;
                        wstate <= W_ANSWER;
                    end else if (wq_go) begin
                        w_unit  <= lk_wq_data;
                        w_units <= w_units - 8'd1;
                        wstate  <= W_UNIT;
                    end
                end

                W_ZERO: begin
                    if (mw_go) begin
                        if (mw_error) begin
                            w_code <= LK_MEM_ERROR;
                            wstate <= W_ANSWER;
                        end else begin
                            w_addr <= w_addr + 64'd8;
                            w_word <= w_word + 3'd1;
                            if (w_word == 3'd7)
                                w_zeros <= w_zeros - 8'd1;
                            if (w_cancel_now) begin
                                w_code <= LK_CANCELLED;
                                wstate <= W_ANSWER;
                            end else if (w_word == 3'd7 && w_zeros == 8'd1) begin
                                w_code <= LK_OK;
                                wstate <= W_ANSWER;
                            end
                        end
                    end
                end

                W_ANSWER: begin
                    if (lk_wa_valid && lk_wa_ready)
                        wstate <= W_IDLE;
                end

                default: wstate <= W_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Doorbells
    // ---------------------------------------------------------------
    // Incoming doorbells are delivered while LM-1 runs.  Outgoing ones stop
    // with new memory requests when MegaPad asks LM-1 to stop.
    assign dbi_valid    = core_rst_n && lk_dbt_valid;
    assign dbi_src      = lk_dbt_src;
    assign lk_dbt_ready = core_rst_n && dbi_ready;

    assign lk_dbr_valid = open && dbo_valid;
    assign lk_dbr_dst   = dbo_dst;
    assign dbo_ready    = open && lk_dbr_ready;

    // LM-1 has stopped when the runtime says so and nothing is in flight in
    // either direction: no request of its own and no write of MegaPad's.
    assign sb_stopped = sb_stop_req && runtime_stopped &&
                        (state == ST_IDLE) && (wstate == W_IDLE);

endmodule
