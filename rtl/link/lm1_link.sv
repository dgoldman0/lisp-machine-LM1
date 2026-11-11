// ============================================================================
// LM-1 Host Link — the chiplet's side of the link to MegaPad
//
// LM-1 is a chiplet beside MegaPad.  MegaPad lends it ranges of external
// memory, and LM-1 reads and writes them across the link at MegaPad's own
// addresses.  This module is LM-1's end of that link, built to MegaPad's
// chiplet link specification, contract version 1 (docs/chiplet-link.md in
// the MegaPad repository).
//
// Link channels are valid/ready handshakes:
//   memory request   (to MegaPad):   addr, wdata, wen, size; one outstanding
//   memory response  (from MegaPad): rdata, code
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
//   - doorbell ports, meant for a tile's hardware message queue;
//   - the reset for LM-1's own logic and the stop handshake with the runtime.
//
// An uncontended request reaches the link in the cycle it is raised, and its
// response returns in the cycle it arrives, so the endpoint adds no cycle to
// an access.
//
// Clocking: the link is specified at the level of messages.  This module
// places both ends on one clock; a link layer between them carries the same
// handshakes across the die boundary.
// ============================================================================
module lm1_link #(
    // Identity word 1: the device name, first character in the low byte.
    parameter logic [63:0] DEVICE_NAME = 64'h0000_0000_0031_4D4C   // "LM1"
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
    output logic        lk_rq_cancel,
    input  logic        lk_rs_valid,
    output logic        lk_rs_ready,
    input  logic [63:0] lk_rs_rdata,
    input  logic [1:0]  lk_rs_code,

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

    // ---- Local: reset and lifecycle ----
    output logic        core_rst_n,     // reset for LM-1's own logic
    output logic        stop_req,       // the runtime is asked to stop
    input  logic        runtime_stopped,
    input  logic        runtime_ready,
    output logic [63:0] boot_addr,
    output logic [63:0] boot_len,

    // ---- Local: host-memory port ----
    input  logic        hm_req_valid,
    output logic        hm_req_ready,
    input  logic [63:0] hm_req_addr,
    input  logic [63:0] hm_req_wdata,
    input  logic        hm_req_we,
    input  logic [1:0]  hm_req_size,
    input  logic        hm_cancel,      // abandon the outstanding request
    output logic        hm_resp_valid,
    output logic [63:0] hm_resp_data,
    output logic [1:0]  hm_resp_code,

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

    // ---------------------------------------------------------------
    // Sideband
    // ---------------------------------------------------------------
    assign sb_present = 1'b1;
    assign core_rst_n = por_n && sb_rst_n;
    assign stop_req   = sb_stop_req;
    assign boot_addr  = sb_boot_addr;
    assign boot_len   = sb_boot_len;
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
        ST_IDLE = 2'd0,     // no request outstanding
        ST_SEND = 2'd1,     // a latched request waits for the link
        ST_WAIT = 2'd2      // the link holds the request; await its response
    } state_t;

    state_t      state;
    logic [63:0] held_addr;
    logic [63:0] held_wdata;
    logic        held_we;
    logic [1:0]  held_size;
    logic        cancel_lat;

    // New work is taken while LM-1 runs and no stop has been asked.
    logic open;
    assign open = core_rst_n && !sb_stop_req;

    assign hm_req_ready = open && (state == ST_IDLE);

    logic take;
    assign take = hm_req_valid && hm_req_ready;

    assign lk_rq_valid = (state == ST_SEND) || take;
    assign lk_rq_addr  = (state == ST_SEND) ? held_addr  : hm_req_addr;
    assign lk_rq_wdata = (state == ST_SEND) ? held_wdata : hm_req_wdata;
    assign lk_rq_wen   = (state == ST_SEND) ? held_we    : hm_req_we;
    assign lk_rq_size  = (state == ST_SEND) ? held_size  : hm_req_size;

    assign lk_rs_ready  = (state == ST_WAIT);
    assign lk_rq_cancel = (state == ST_WAIT) && (hm_cancel || cancel_lat);

    assign hm_resp_valid = (state == ST_WAIT) && lk_rs_valid;
    assign hm_resp_data  = lk_rs_rdata;
    assign hm_resp_code  = lk_rs_code;

    always_ff @(posedge clk) begin
        if (!core_rst_n) begin
            state      <= ST_IDLE;
            held_addr  <= 64'd0;
            held_wdata <= 64'd0;
            held_we    <= 1'b0;
            held_size  <= 2'd3;
            cancel_lat <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    cancel_lat <= 1'b0;
                    if (take) begin
                        held_addr  <= hm_req_addr;
                        held_wdata <= hm_req_wdata;
                        held_we    <= hm_req_we;
                        held_size  <= hm_req_size;
                        state      <= lk_rq_ready ? ST_WAIT : ST_SEND;
                    end
                end
                ST_SEND: begin
                    if (hm_cancel)
                        cancel_lat <= 1'b1;
                    if (lk_rq_ready)
                        state <= ST_WAIT;
                end
                ST_WAIT: begin
                    if (hm_cancel)
                        cancel_lat <= 1'b1;
                    if (lk_rs_valid)
                        state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
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

    // LM-1 has stopped when the runtime says so and nothing is in flight.
    assign sb_stopped = sb_stop_req && runtime_stopped && (state == ST_IDLE);

endmodule
