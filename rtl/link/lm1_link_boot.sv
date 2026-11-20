// ============================================================================
// LM-1 Link Boot — fetch the image MegaPad names, then hand the ports over
//
// MegaPad loads LM-1's runtime image into a range it lends read-only and
// names it on the sideband by address and length.  When LM-1 leaves reset
// this module reads the image through the host-memory port in bursts of up
// to BURST_WORDS 64-bit words and writes each word into LM-1's memory, from
// LOAD_BASE on, through the memory write port.  When the last word is
// written it raises boot_done and hands both ports over: the host-memory
// port to the runtime, and the memory write port to the link, which writes
// MegaPad's stores into the memory LM-1 lends through it.
//
// A length of zero means no image: boot_done rises at once.  A response
// other than OK, or a word the memory fails, stops the fetch and raises
// boot_failed; a burst still under way is cancelled and its remaining
// responses taken and dropped.  LM-1 then keeps both ports from the runtime
// and the link until MegaPad resets it.
//
// The image is read in whole words, so a length that is no multiple of 8 is
// rounded up, and the range MegaPad lends covers the rounded length.
//
// The link can have no write to pass while the fetch runs: its ranges are
// lent by the runtime, which starts after the fetch, and LM-1's reset clears
// them, so every write MegaPad makes before then is refused unwritten.
// ============================================================================
module lm1_link_boot #(
    parameter int          BURST_WORDS = 8,     // words per request, 1 to 256
    // The address in LM-1's memory of the image's first byte, a multiple of 8.
    parameter logic [63:0] LOAD_BASE   = 64'd0
) (
    input  logic        clk,
    input  logic        rst_n,          // LM-1's own reset (lm1_link.core_rst_n)

    input  logic [63:0] boot_addr,
    input  logic [63:0] boot_len,

    // ---- Host-memory port toward lm1_link ----
    output logic        hm_req_valid,
    input  logic        hm_req_ready,
    output logic [63:0] hm_req_addr,
    output logic [63:0] hm_req_wdata,
    output logic        hm_req_we,
    output logic [1:0]  hm_req_size,
    output logic [7:0]  hm_req_len,
    output logic        hm_cancel,
    input  logic        hm_resp_valid,
    output logic        hm_resp_ready,
    input  logic [63:0] hm_resp_data,
    input  logic [1:0]  hm_resp_code,
    input  logic        hm_resp_last,

    // ---- The runtime's host-memory port, passed through after boot ----
    input  logic        rt_req_valid,
    output logic        rt_req_ready,
    input  logic [63:0] rt_req_addr,
    input  logic [63:0] rt_req_wdata,
    input  logic        rt_req_we,
    input  logic [1:0]  rt_req_size,
    input  logic [7:0]  rt_req_len,
    input  logic        rt_cancel,
    output logic        rt_resp_valid,
    input  logic        rt_resp_ready,
    output logic [63:0] rt_resp_data,
    output logic [1:0]  rt_resp_code,
    output logic        rt_resp_last,

    // ---- The link's writes (lm1_link's mw_* port), passed through after boot ----
    input  logic        lw_valid,
    output logic        lw_ready,
    input  logic [63:0] lw_addr,
    input  logic [63:0] lw_data,
    output logic        lw_error,

    // ---- LM-1's memory write port ----
    // A word taken (mw_valid and mw_ready) is visible to LM-1's own accesses
    // from the next cycle; mw_error with mw_ready means it was not written.
    output logic        mw_valid,
    input  logic        mw_ready,
    output logic [63:0] mw_addr,
    output logic [63:0] mw_data,
    input  logic        mw_error,

    // ---- Status ----
    output logic        boot_done,      // the whole image is loaded
    output logic        boot_failed     // a fetch or a write of it failed
);

    localparam logic [1:0]  LK_OK    = 2'd0;
    localparam logic [1:0]  SZ_DWORD = 2'd3;
    localparam logic [63:0] BURST    = 64'(BURST_WORDS);
    localparam logic [7:0]  BURST_M1 = 8'(BURST_WORDS - 1);

    typedef enum logic [2:0] {
        B_START = 3'd0,     // first cycle out of reset: sample the length
        B_REQ   = 3'd1,     // ask for the next burst
        B_BEAT  = 3'd2,     // await one of its words
        B_LOAD  = 3'd3,     // write the word into LM-1's memory
        B_DONE  = 3'd4,
        B_FAIL  = 3'd5
    } bstate_t;

    bstate_t     bstate;
    logic [63:0] offset;        // bytes loaded so far
    logic [63:0] remain;        // words still to load
    logic [63:0] word;
    logic        word_last;     // the held word ended its burst

    assign boot_done   = (bstate == B_DONE);
    assign boot_failed = (bstate == B_FAIL);

    // The fetch owns both ports until the image is in; the runtime owns the
    // host-memory port afterwards, and the link the memory write port.
    logic fetching;
    assign fetching = !boot_done;

    // Words beyond the first in the next burst: a full burst, or what is
    // left of the image when that is less.
    logic [7:0] burst_len;
    assign burst_len = (remain > BURST) ? BURST_M1 : (remain[7:0] - 8'd1);

    assign hm_req_valid  = fetching ? (bstate == B_REQ) : rt_req_valid;
    assign hm_req_addr   = fetching ? (boot_addr + offset) : rt_req_addr;
    assign hm_req_wdata  = fetching ? 64'd0 : rt_req_wdata;
    assign hm_req_we     = fetching ? 1'b0 : rt_req_we;
    assign hm_req_size   = fetching ? SZ_DWORD : rt_req_size;
    assign hm_req_len    = fetching ? burst_len : rt_req_len;
    // A fetch that fails while its burst still has words to come cancels the
    // burst and takes what remains, so the link returns to idle.
    assign hm_cancel     = fetching ? (bstate == B_FAIL) : rt_cancel;
    assign hm_resp_ready = fetching ? (bstate == B_BEAT || bstate == B_FAIL)
                                    : rt_resp_ready;

    assign rt_req_ready  = !fetching && hm_req_ready;
    assign rt_resp_valid = !fetching && hm_resp_valid;
    assign rt_resp_data  = hm_resp_data;
    assign rt_resp_code  = hm_resp_code;
    assign rt_resp_last  = hm_resp_last;

    assign mw_valid = fetching ? (bstate == B_LOAD) : lw_valid;
    assign mw_addr  = fetching ? (LOAD_BASE + offset) : lw_addr;
    assign mw_data  = fetching ? word : lw_data;
    assign lw_ready = !fetching && mw_ready;
    assign lw_error = mw_error;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            bstate    <= B_START;
            offset    <= 64'd0;
            remain    <= 64'd0;
            word      <= 64'd0;
            word_last <= 1'b0;
        end else begin
            case (bstate)
                B_START: begin
                    offset <= 64'd0;
                    remain <= (boot_len + 64'd7) >> 3;
                    bstate <= (boot_len == 64'd0) ? B_DONE : B_REQ;
                end
                B_REQ: begin
                    if (hm_req_ready)
                        bstate <= B_BEAT;
                end
                B_BEAT: begin
                    if (hm_resp_valid) begin
                        word      <= hm_resp_data;
                        word_last <= hm_resp_last;
                        bstate    <= (hm_resp_code == LK_OK) ? B_LOAD : B_FAIL;
                    end
                end
                B_LOAD: begin
                    if (mw_ready) begin
                        if (mw_error) begin
                            bstate <= B_FAIL;
                        end else begin
                            offset <= offset + 64'd8;
                            remain <= remain - 64'd1;
                            if (remain == 64'd1)
                                bstate <= B_DONE;
                            else if (word_last)
                                bstate <= B_REQ;
                            else
                                bstate <= B_BEAT;
                        end
                    end
                end
                B_DONE: ;
                B_FAIL: ;
                default: bstate <= B_FAIL;
            endcase
        end
    end

endmodule
