// ============================================================================
// LM-1 Hardware Message Queue — 4 independent FIFOs
//
// Each queue holds 64-bit tagged words.
// Used by SEND/RECV instructions for inter-tile messaging.
//
// Ports:
//   Core write: wr_en + wr_id + wr_data; wr_ready says the word is taken
//               this cycle, wr_full that the selected queue is full
//   Core read:  rd_en + rd_id; rd_valid says rd_data is taken this cycle,
//               rd_empty that the selected queue is empty
//   External push/pop: for the cluster's message router, the NoC or DMA
//               injecting/draining messages
//
// The external ports go first on a queue both sides use in one cycle, so
// they never wait: ext_wr_ready is "not full" and ext_rd_valid "not
// empty".  The core's access to that queue waits a cycle, and the core
// tells a queue that is busy from one that is full or empty by wr_full and
// rd_empty.
// ============================================================================
module lm1_msg_queue
    import lm1_pkg::*;
#(
    parameter int DEPTH     = 512,            // entries per queue
    parameter int NUM_QUEUES = 4
)
(
    input  logic               clk,
    input  logic               rst_n,

    // --- Core write port (SEND) ---
    input  logic               wr_en,
    input  logic [1:0]         wr_id,
    input  logic [XLEN-1:0]   wr_data,
    output logic               wr_ready,       // a word written now is taken
    output logic               wr_full,        // selected queue is full

    // --- Core read port (RECV) ---
    input  logic               rd_en,
    input  logic [1:0]         rd_id,
    output logic [XLEN-1:0]   rd_data,
    output logic               rd_valid,       // a word read now is taken
    output logic               rd_empty,       // selected queue is empty

    // --- External write port (NoC → queue) ---
    input  logic               ext_wr_en,
    input  logic [1:0]         ext_wr_id,
    input  logic [XLEN-1:0]   ext_wr_data,
    output logic               ext_wr_ready,

    // --- External read port (queue → NoC) ---
    input  logic               ext_rd_en,
    input  logic [1:0]         ext_rd_id,
    output logic [XLEN-1:0]   ext_rd_data,
    output logic               ext_rd_valid,

    // --- Status ---
    output logic [NUM_QUEUES-1:0] q_empty,
    output logic [NUM_QUEUES-1:0] q_full
);

    localparam int ADDR_W = $clog2(DEPTH);

    // Per-queue storage and pointers
    logic [XLEN-1:0] mem [0:NUM_QUEUES-1][0:DEPTH-1];
    logic [ADDR_W:0] wr_ptr [0:NUM_QUEUES-1];  // extra bit for full/empty
    logic [ADDR_W:0] rd_ptr [0:NUM_QUEUES-1];

    // Derived signals per queue
    logic [NUM_QUEUES-1:0] q_wr_en, q_rd_en;
    logic [XLEN-1:0]      q_wr_data [0:NUM_QUEUES-1];

    // Status
    genvar g;
    generate
        for (g = 0; g < NUM_QUEUES; g++) begin : gen_status
            assign q_empty[g] = (wr_ptr[g] == rd_ptr[g]);
            assign q_full[g]  = (wr_ptr[g][ADDR_W] != rd_ptr[g][ADDR_W]) &&
                                (wr_ptr[g][ADDR_W-1:0] == rd_ptr[g][ADDR_W-1:0]);
        end
    endgenerate

    // Core port ready/valid: the external port goes first on one queue
    assign wr_full  = q_full[wr_id];
    assign rd_empty = q_empty[rd_id];
    assign wr_ready = ~q_full[wr_id] && !(ext_wr_en && ext_wr_id == wr_id);
    assign rd_valid = ~q_empty[rd_id] && !(ext_rd_en && ext_rd_id == rd_id);
    assign rd_data  = mem[rd_id][rd_ptr[rd_id][ADDR_W-1:0]];

    // External port ready/valid
    assign ext_wr_ready = ~q_full[ext_wr_id];
    assign ext_rd_valid = ~q_empty[ext_rd_id];
    assign ext_rd_data  = mem[ext_rd_id][rd_ptr[ext_rd_id][ADDR_W-1:0]];

    // Write/read arbitration: external has priority over core for same queue
    always_comb begin
        for (int i = 0; i < NUM_QUEUES; i++) begin
            q_wr_en[i]   = 1'b0;
            q_wr_data[i] = '0;
            q_rd_en[i]   = 1'b0;
        end

        // External write
        if (ext_wr_en && ~q_full[ext_wr_id]) begin
            q_wr_en[ext_wr_id]   = 1'b1;
            q_wr_data[ext_wr_id] = ext_wr_data;
        end
        // Core write (only if the external port isn't writing the same queue)
        if (wr_en && wr_ready) begin
            q_wr_en[wr_id]   = 1'b1;
            q_wr_data[wr_id] = wr_data;
        end

        // External read
        if (ext_rd_en && ~q_empty[ext_rd_id])
            q_rd_en[ext_rd_id] = 1'b1;
        // Core read (only if the external port isn't reading the same queue)
        if (rd_en && rd_valid)
            q_rd_en[rd_id] = 1'b1;
    end

    // Sequential: pointer and memory updates
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_QUEUES; i++) begin
                wr_ptr[i] <= '0;
                rd_ptr[i] <= '0;
            end
        end else begin
            for (int i = 0; i < NUM_QUEUES; i++) begin
                if (q_wr_en[i]) begin
                    mem[i][wr_ptr[i][ADDR_W-1:0]] <= q_wr_data[i];
                    wr_ptr[i] <= wr_ptr[i] + 1;
                end
                if (q_rd_en[i]) begin
                    rd_ptr[i] <= rd_ptr[i] + 1;
                end
            end
        end
    end

endmodule
