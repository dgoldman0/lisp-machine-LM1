// ============================================================================
// LM-1 Control Unit — Multi-Cycle FSM
//
// Orchestrates the fetch-decode-execute cycle for the LM-1 processor.
// Multi-cycle design: each instruction takes 2+ cycles.
//
// Architecture:
//   - All state register updates in a single always_ff block
//   - All next-state / output computation in a single always_comb
//   - Bridge via *_nxt signals: comb computes, ff latches
// ============================================================================
module lm1_control
    import lm1_pkg::*;
(
    input  logic                clk,
    input  logic                rst_n,

    // -- Decoded instruction (from decoder, active one cycle after ifetch) --
    input  decoded_t            dec_in,
    input  logic [XLEN-1:0]    imm_sext_in,

    // -- Register-file ports --
    input  logic [XLEN-1:0]    rf_rd1_data,   // read port 1 data
    input  logic [XLEN-1:0]    rf_rd2_data,   // read port 2 data
    output logic                rf_we,
    output logic [FULL_REG_W-1:0] rf_w_addr,
    output logic [XLEN-1:0]    rf_w_data,
    output logic [FULL_REG_W-1:0] rf_rd1_addr,  // read port 1 address
    output logic [FULL_REG_W-1:0] rf_rd2_addr,  // read port 2 address

    // -- ALU interface --
    output opcode_t             alu_op,
    output logic [FUNC_W-1:0]  alu_func,
    output logic [XLEN-1:0]    alu_a,
    output logic [XLEN-1:0]    alu_b,
    output logic                alu_start,
    input  logic [XLEN-1:0]    alu_result,
    input  logic                alu_valid,
    input  logic                alu_trap,
    input  logic [7:0]         alu_trap_code,

    // -- Branch unit interface --
    output logic [XLEN-1:0]    br_pc,
    output logic [XLEN-1:0]    br_val,
    output logic [REG_IDX_W-1:0] br_cond,
    output logic [IMM16_W-1:0] br_offset,
    output logic                br_is_br,
    output logic                br_is_cond,
    input  logic [XLEN-1:0]    br_target,
    input  logic                br_taken,

    // -- LSU interface --
    output logic                lsu_req,
    output logic [3:0]         lsu_op,
    output logic [XLEN-1:0]    lsu_addr,
    output logic [XLEN-1:0]    lsu_wdata,
    input  logic                lsu_ready,
    input  logic                lsu_valid,
    input  logic [XLEN-1:0]    lsu_rdata,
    input  logic [ILEN-1:0]    lsu_inst,

    // -- Header template table --
    output logic [7:0]         tmpl_rd_idx,
    input  logic [XLEN-1:0]    tmpl_rd_data,
    output logic                tmpl_wr_en,
    output logic [7:0]         tmpl_wr_idx,
    output logic [XLEN-1:0]    tmpl_wr_data,

    // -- IC table --
    output logic [XLEN-1:0]    ic_lu_pc,
    output logic [31:0]        ic_lu_shape,
    output logic                ic_lu_valid,
    input  logic [XLEN-1:0]    ic_hit_target,
    input  logic                ic_hit,
    output logic                ic_inst_valid,
    output logic [XLEN-1:0]    ic_inst_pc,
    output logic [31:0]        ic_inst_shape,
    output logic [XLEN-1:0]    ic_inst_target,

    // -- Message queue interface (the tile's 4 hardware queues) --
    output logic                mq_wr_en,
    output logic [1:0]         mq_wr_id,         // which queue
    output logic [XLEN-1:0]    mq_wr_data,
    input  logic                mq_wr_ready,      // a word written now is taken
    input  logic                mq_wr_full,       // the queue is full
    output logic                mq_rd_en,
    output logic [1:0]         mq_rd_id,         // which queue
    input  logic [XLEN-1:0]    mq_rd_data,
    input  logic                mq_rd_valid,      // a word read now is taken
    input  logic                mq_rd_empty,      // the queue is empty

    // -- Remote queue port: a queue of another tile --
    // Driven from registers alone while SEND or RECV waits for it.  The
    // answer comes with rmq_ack, in the same cycle or later: rmq_ok means
    // the word was taken (SEND) or popped into rmq_rdata (RECV), and
    // rmq_noroute that no route reaches the tile; with neither, the queue
    // was full or empty.
    output logic                rmq_req,
    output logic                rmq_write,        // 1 = SEND, 0 = RECV / TRY.RECV
    output logic [XLEN-1:0]    rmq_tile,
    output logic [1:0]         rmq_queue,
    output logic [XLEN-1:0]    rmq_wdata,
    input  logic                rmq_ack,
    input  logic                rmq_ok,
    input  logic                rmq_noroute,
    input  logic [XLEN-1:0]    rmq_rdata,

    // -- GC engine command interface --
    output logic                gc_cmd_valid,
    output logic [3:0]         gc_cmd_op,
    output logic [XLEN-1:0]    gc_cmd_arg0,      // region base / pointer list base
    output logic [XLEN-1:0]    gc_cmd_arg1,      // region size / dest base
    output logic [XLEN-1:0]    gc_cmd_arg2,      // copy: region size (from rd)
    input  logic                gc_cmd_ready,     // engine accepts command
    input  logic                gc_engine_busy,   // any engine currently active

    // -- Scanner result FIFO (cluster → CPU) --
    input  logic [7:0]          scan_fifo_count,
    input  logic [XLEN-1:0]    scan_fifo_head_obj,
    input  logic [15:0]         scan_fifo_head_field,
    input  logic [XLEN-1:0]    scan_fifo_head_ref,
    output logic                scan_fifo_pop,     // pulse to pop head entry

    // -- Performance counter read port --
    output logic [4:0]         ctr_id,
    input  logic [XLEN-1:0]    ctr_value,

    // -- Config --
    input  logic [XLEN-1:0]    cfg_tile_id,
    input  logic [XLEN-1:0]    cfg_thread_id,

    // -- Status --
    output logic                halted,
    output logic [XLEN-1:0]    pc_out,
    output logic [XLEN-1:0]    cycle_count,     // the core's clocks since reset

    // -- Performance counter increment strobes --
    output logic                ctr_alloc_inc,
    output logic [15:0]        ctr_alloc_bytes_inc,
    output logic                ctr_barrier_fire_inc,
    output logic                ctr_barrier_filt_inc,
    output logic                ctr_ic_hit_inc,
    output logic                ctr_ic_miss_inc,
    output logic                ctr_nursery_ovf_inc,

    // -- I-Cache fetch interface --
    output logic                icache_fetch_req,
    output logic [XLEN-1:0]    icache_fetch_addr,
    input  logic                icache_fetch_valid,
    input  logic [ILEN-1:0]    icache_fetch_inst,

    // -- Instruction latch control (for CPU inst_latched) --
    output logic                inst_latch_en,
    output logic [ILEN-1:0]    inst_latch_data
);

    // ---------------------------------------------------------------
    // LSU operation codes (must match lm1_lsu)
    // ---------------------------------------------------------------
    localparam logic [3:0] LSU_NONE    = 4'd0,
                           LSU_IFETCH  = 4'd1,
                           LSU_LOAD64  = 4'd2,
                           LSU_STORE64 = 4'd3,
                           LSU_LOAD32  = 4'd4,
                           LSU_LOAD_BYTE  = 4'd5,
                           LSU_STORE_BYTE = 4'd6,
                           LSU_LOAD_HALF  = 4'd7,
                           LSU_STORE_HALF = 4'd8,
                           LSU_LOAD_WORD  = 4'd9,
                           LSU_STORE_WORD = 4'd10;

    // ---------------------------------------------------------------
    // FSM states
    // ---------------------------------------------------------------
    typedef enum logic [5:0] {
        S_RESET,
        S_FETCH,
        S_FETCH_WAIT,
        S_DECODE,
        S_EXECUTE,
        S_ALU_WAIT,
        S_MEM,
        S_MEM_WAIT,
        S_FIELD_MEM,
        S_FIELD_WAIT,
        S_PUSH_FRAME_0,
        S_PUSH_FRAME_W0,
        S_PUSH_FRAME_1,
        S_PUSH_FRAME_W1,
        S_PUSH_FRAME_2,
        S_POP_FRAME_0,
        S_POP_FRAME_W0,
        S_POP_FRAME_LD_LR,
        S_POP_FRAME_1,
        S_POP_FRAME_W1,
        S_POP_FRAME_2,
        S_MULTI_ITER,
        S_MULTI_PUSH,
        S_MULTI_POP_WAIT,
        S_MULTI_SP_WR,
        S_ALLOC_HDR,
        S_ALLOC_HDR_W,
        S_ALLOC_ZERO,
        S_ALLOC_ZERO_W,
        S_ALLOC_INIT,
        S_ALLOC_INIT_W,
        S_ALLOC_INIT2,
        S_ALLOC_INIT2_W,
        S_ALLOC_RD_CDR,
        S_ALLOC_DONE,
        S_HDR_READ,
        S_HDR_WAIT,
        S_CLOS_CODE_RD,
        S_CLOS_CODE_WAIT,
        S_IC_DISPATCH,
        S_BARRIER_CHECK,
        S_BARRIER_MARK,
        S_BARRIER_MARK_W,
        S_SEND_WAIT,        // SEND waits for a busy or remote queue
        S_RECV_WAIT,        // RECV / TRY.RECV wait for a busy or remote queue
        S_TRY_RECV_WB,
        S_ENQ_WAIT,
        S_FENCE_GC,
        S_ATOMIC_STORE,
        S_ATOMIC_STORE_W,
        S_TRAP_LOOKUP,
        S_TRAP_WAIT,
        S_HALTED
    } state_t;

    // ---------------------------------------------------------------
    // State registers
    // ---------------------------------------------------------------
    state_t          state;
    logic [XLEN-1:0] pc;
    logic [XLEN-1:0] cyc;               // CYCLE: every clock since reset

    // --- FGMT: 4 hardware threads ---
    logic [THREAD_IDX_W-1:0] cur_thread;     // currently active thread (0..3)
    logic [XLEN-1:0] thread_pc    [0:NUM_THREADS-1];  // per-thread PC
    logic [NUM_THREADS-1:0] thread_active;   // which threads are active (not halted)
    // INSTRET: the instructions each thread has retired.  An instruction
    // retires in the state that sets retire, once it has done its work: a
    // fault, and a RECV that ends its turn waiting, retire nothing.
    logic [XLEN-1:0] instret      [0:NUM_THREADS-1];

    // Helper: build banked register address from thread + reg index
    function automatic logic [FULL_REG_W-1:0] banked_addr(
        logic [THREAD_IDX_W-1:0] tid,
        logic [REG_IDX_W-1:0]    ridx
    );
        return {tid, ridx};
    endfunction

    decoded_t        dr;                 // latched instruction
    logic [XLEN-1:0] imm_r;             // latched sign-extended immediate

    logic [XLEN-1:0] opa, opb, opc;     // latched regs[rd], regs[rs1], regs[rs2]

    logic [XLEN-1:0] ta;                // temp address
    logic [XLEN-1:0] td;                // temp data / return addr
    logic [XLEN-1:0] tt;                // temp target
    logic [7:0]      tc;                // temp trap code
    logic [XLEN-1:0] th;                // temp header

    logic [15:0]     mm;                // multi mask
    logic [4:0]      mi;                // multi index
    logic [4:0]      mb;                // multi base
    logic            mdir;              // 0=push, 1=pop

    logic [XLEN-1:0] aa;                // alloc base address
    logic [15:0]     anw;               // alloc n_words
    logic [15:0]     acnt;              // alloc zero-fill counter: payload words left
    logic            acon;              // alloc is_cons

    logic [XLEN-1:0] trap_tbl;
    logic [XLEN-1:0] trap_pc;
    logic [7:0]      trap_cause;
    logic            in_trap;

    // Barrier configuration registers (set via system traps)
    logic [XLEN-1:0] card_table_base;
    logic [5:0]      card_shift;       // log2(card_size), default=6 → 64B cards
    logic [XLEN-1:0] gen_boundary;     // addrs < gen_boundary are nursery (Gen 0)
    logic [XLEN-1:0] queue_base;       // message-queue base address

    // ---------------------------------------------------------------
    // Next-state signals (all computed in always_comb)
    // ---------------------------------------------------------------
    state_t          ns;
    logic [XLEN-1:0] pc_n;
    logic            retire;            // the current thread's instruction retires

    decoded_t        dr_n;
    logic [XLEN-1:0] imm_n;
    logic [XLEN-1:0] opa_n, opb_n, opc_n;
    logic [XLEN-1:0] ta_n, td_n, tt_n, th_n;
    logic [7:0]      tc_n;
    logic [15:0]     mm_n;
    logic [4:0]      mi_n, mb_n;
    logic            mdir_n;
    logic [XLEN-1:0] aa_n;
    logic [15:0]     anw_n;
    logic [15:0]     acnt_n;
    logic            acon_n;
    logic [XLEN-1:0] trap_tbl_n, trap_pc_n;
    logic [7:0]      trap_cause_n;
    logic            in_trap_n;

    logic [XLEN-1:0] card_table_base_n;
    logic [5:0]      card_shift_n;
    logic [XLEN-1:0] gen_boundary_n;
    logic [XLEN-1:0] queue_base_n;

    // ---------------------------------------------------------------
    // Outputs
    // ---------------------------------------------------------------
    assign halted      = (state == S_HALTED) && (thread_active == '0);
    assign pc_out      = pc;
    assign cycle_count = cyc;

    // ---------------------------------------------------------------
    // Queue descriptors (spec/02-isa.md § 8)
    //
    // A descriptor is a fixnum, and a word that is not one traps
    // TRAP_NOT_FIXNUM.  Its value's low two bits pick the queue and the
    // rest, value / 4, the tile; a negative value names no tile and traps
    // TRAP_NO_ROUTE.  qd_value returns a fixnum descriptor's value.
    // ---------------------------------------------------------------
    function automatic logic [XLEN-1:0] qd_value(logic [XLEN-1:0] d);
        return $signed(d) >>> 1;
    endfunction

    // ---------------------------------------------------------------
    // Reserved encodings (spec/07-encoding.md § 2)
    //
    // A value of a selecting field that names no instruction: a func,
    // cond, bank, sub-code, or car/cdr or halt/nop selector the spec does
    // not assign.  S_EXECUTE traps TRAP_UNIMPLEMENTED on one before the
    // instruction does anything, as it does on an opcode no instruction
    // has and on a system trap code no system trap has.
    // ---------------------------------------------------------------
    function automatic logic reserved_encoding(decoded_t d);
        case (d.opcode)
            OP_ARITH_FIX:   return d.func > FUNC_DIV_FIX;
            OP_CMP_TAGGED:  return d.func > FUNC_EQ;
            OP_LD_CAR_CDR:  return d.imm16 > 16'd1;
            OP_ST_CAR_CDR:  return d.rs2 > 5'd1;
            OP_FAA_FENCE:   return d.func != 5'd0 && d.func != FUNC_FENCE_GC;
            OP_ARITH_RAW:   return d.func > FUNC_MOD;
            OP_BITWISE:     return d.func > FUNC_NOT;
            OP_BR_COND:     return d.rs1 > BR_EQ_Z;
            OP_PUSH_POP:    return d.func > FUNC_POP;
            OP_PUSH_MULTI,
            OP_POP_MULTI:   return d.rd > 5'd1;       // two banks of 16
            OP_ERET:        return d.func > FUNC_ERET_NEXT;
            OP_SYS_INFO:    return d.rs1 > SYS_INSTRET;
            OP_HALT_NOP:    return d.rd > SEL_NOP;
            default:        return 1'b0;
        endcase
    endfunction

    // While SEND or RECV waits on another tile's queue, ta holds the tile,
    // tt[1:0] the queue and td SEND's value.
    assign rmq_req   = ((state == S_SEND_WAIT) || (state == S_RECV_WAIT)) &&
                       (ta != cfg_tile_id);
    assign rmq_write = (state == S_SEND_WAIT);
    assign rmq_tile  = ta;
    assign rmq_queue = tt[1:0];
    assign rmq_wdata = td;

    // ---------------------------------------------------------------
    // Sequential: latch next-state values
    // ---------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_RESET;
            pc         <= '0;
            cyc        <= '0;
            cur_thread <= '0;
            for (int t = 0; t < NUM_THREADS; t++) begin
                thread_pc[t] <= '0;
                instret[t]   <= '0;
            end
            thread_active <= {{(NUM_THREADS-1){1'b0}}, 1'b1};  // only thread 0 active at reset
            dr         <= '0;
            imm_r      <= '0;
            opa        <= '0;
            opb        <= '0;
            opc        <= '0;
            ta         <= '0;
            td         <= '0;
            tt         <= '0;
            tc         <= '0;
            th         <= '0;
            mm         <= '0;
            mi         <= '0;
            mb         <= '0;
            mdir       <= 1'b0;
            aa         <= '0;
            anw        <= '0;
            acnt       <= '0;
            acon       <= 1'b0;
            trap_tbl   <= '0;
            trap_pc    <= '0;
            trap_cause <= '0;
            in_trap    <= 1'b0;
            card_table_base <= '0;
            card_shift <= 6'd6;  // default 64-byte cards
            gen_boundary <= '0;
            queue_base <= '0;
        end else begin
            state      <= ns;
            pc         <= pc_n;
            cyc        <= cyc + 64'd1;
            // cur_thread holds the retiring instruction's thread until this
            // edge, the one that may switch threads.
            if (retire)
                instret[cur_thread] <= instret[cur_thread] + 64'd1;

            // FGMT: save/restore thread context at instruction boundaries
            // When entering S_FETCH from a completed instruction, save
            // current thread PC and switch to next active thread.
            if (ns == S_FETCH && state != S_FETCH && state != S_FETCH_WAIT &&
                state != S_RESET) begin
                // Save current thread's PC
                thread_pc[cur_thread] <= pc_n;
                // Round-robin to next active thread
                begin
                    logic [THREAD_IDX_W-1:0] next_t;
                    next_t = cur_thread;
                    for (int i = 1; i <= NUM_THREADS; i++) begin
                        logic [THREAD_IDX_W-1:0] candidate;
                        candidate = THREAD_IDX_W'((THREAD_IDX_W'(cur_thread) + THREAD_IDX_W'(i)) & THREAD_IDX_W'(NUM_THREADS-1));
                        if (thread_active[candidate]) begin
                            next_t = candidate;
                            break;
                        end
                    end
                    // Only override pc when actually switching to a
                    // different thread.  When next_t == cur_thread
                    // (single active thread), the default pc <= pc_n
                    // already has the correct value; overriding it with
                    // thread_pc[next_t] would read the stale (pre-save)
                    // value due to non-blocking assignment semantics and
                    // cause every instruction to execute twice.
                    if (next_t != cur_thread) begin
                        cur_thread <= next_t;
                        pc         <= thread_pc[next_t];
                    end
                end
            end else if (state == S_RESET) begin
                // At reset exit, load thread 0's PC
                pc <= thread_pc[0];
            end

            // Track thread halted state
            if (ns == S_HALTED && state != S_HALTED) begin
                thread_active[cur_thread] <= 1'b0;
                // If other threads are still active, switch to one
                // instead of actually entering S_HALTED.
                begin
                    logic found_active;
                    logic [THREAD_IDX_W-1:0] next_t;
                    found_active = 1'b0;
                    next_t = cur_thread;
                    for (int i = 1; i < NUM_THREADS; i++) begin
                        logic [THREAD_IDX_W-1:0] candidate;
                        candidate = THREAD_IDX_W'((THREAD_IDX_W'(cur_thread) + THREAD_IDX_W'(i)) & THREAD_IDX_W'(NUM_THREADS-1));
                        if (thread_active[candidate] && !found_active) begin
                            next_t = candidate;
                            found_active = 1'b1;
                        end
                    end
                    if (found_active) begin
                        state <= S_FETCH;  // override ns = S_HALTED
                        cur_thread <= next_t;
                        pc <= thread_pc[next_t];
                    end
                end
            end
            dr         <= dr_n;
            imm_r      <= imm_n;
            opa        <= opa_n;
            opb        <= opb_n;
            opc        <= opc_n;
            ta         <= ta_n;
            td         <= td_n;
            tt         <= tt_n;
            tc         <= tc_n;
            th         <= th_n;
            mm         <= mm_n;
            mi         <= mi_n;
            mb         <= mb_n;
            mdir       <= mdir_n;
            aa         <= aa_n;
            anw        <= anw_n;
            acnt       <= acnt_n;
            acon       <= acon_n;
            trap_tbl   <= trap_tbl_n;
            trap_pc    <= trap_pc_n;
            trap_cause <= trap_cause_n;
            in_trap    <= in_trap_n;
            card_table_base <= card_table_base_n;
            card_shift <= card_shift_n;
            gen_boundary <= gen_boundary_n;
            queue_base <= queue_base_n;
        end
    end

    // ---------------------------------------------------------------
    // Combinational: compute all next values and outputs
    //
    // Every signal must be assigned on every path (no latches).
    // Defaults at top → overridden in specific states.
    // ---------------------------------------------------------------
    always_comb begin
        // === Defaults: hold all state ===
        ns          = state;
        pc_n        = pc;
        retire      = 1'b0;
        dr_n        = dr;
        imm_n       = imm_r;
        opa_n       = opa;
        opb_n       = opb;
        opc_n       = opc;
        ta_n        = ta;
        td_n        = td;
        tt_n        = tt;
        tc_n        = tc;
        th_n        = th;
        mm_n        = mm;
        mi_n        = mi;
        mb_n        = mb;
        mdir_n      = mdir;
        aa_n        = aa;
        anw_n       = anw;
        acnt_n      = acnt;
        acon_n      = acon;
        trap_tbl_n  = trap_tbl;
        trap_pc_n   = trap_pc;
        trap_cause_n = trap_cause;
        in_trap_n   = in_trap;

        card_table_base_n = card_table_base;
        card_shift_n = card_shift;
        gen_boundary_n = gen_boundary;
        queue_base_n = queue_base;

        // === Default outputs ===
        rf_we       = 1'b0;
        rf_w_addr   = banked_addr(cur_thread, dr.rd);
        rf_w_data   = '0;
        rf_rd1_addr = banked_addr(cur_thread, dec_in.rd);
        rf_rd2_addr = banked_addr(cur_thread, dec_in.rs1);

        alu_op      = dr.opcode;
        alu_func    = dr.func;
        alu_a       = opb;
        alu_b       = opc;
        alu_start   = 1'b0;

        br_pc       = pc;
        br_val      = opa;
        br_cond     = dr.rs1;
        br_offset   = dr.imm16;
        br_is_br    = 1'b0;
        br_is_cond  = 1'b0;

        lsu_req     = 1'b0;
        lsu_op      = LSU_NONE;
        lsu_addr    = '0;
        lsu_wdata   = '0;

        tmpl_rd_idx = '0;
        tmpl_wr_en  = 1'b0;
        tmpl_wr_idx = '0;
        tmpl_wr_data = '0;

        ic_lu_valid   = 1'b0;
        ic_lu_pc      = '0;
        ic_lu_shape   = '0;
        ic_inst_valid = 1'b0;
        ic_inst_pc    = '0;
        ic_inst_shape = '0;
        ic_inst_target = '0;

        // Message queue defaults
        mq_wr_en    = 1'b0;
        mq_wr_id    = '0;
        mq_wr_data  = '0;
        mq_rd_en    = 1'b0;
        mq_rd_id    = '0;

        // GC engine defaults
        gc_cmd_valid = 1'b0;
        gc_cmd_op    = '0;
        gc_cmd_arg0  = '0;
        gc_cmd_arg1  = '0;
        gc_cmd_arg2  = '0;
        scan_fifo_pop = 1'b0;

        // Perf counter defaults
        ctr_id               = '0;
        ctr_alloc_inc        = 1'b0;
        ctr_alloc_bytes_inc  = '0;
        ctr_barrier_fire_inc = 1'b0;
        ctr_barrier_filt_inc = 1'b0;
        ctr_ic_hit_inc       = 1'b0;
        ctr_ic_miss_inc      = 1'b0;
        ctr_nursery_ovf_inc  = 1'b0;

        // I-Cache fetch defaults
        icache_fetch_req  = 1'b0;
        icache_fetch_addr = '0;
        inst_latch_en     = 1'b0;
        inst_latch_data   = '0;

        // =========================================================
        case (state)

        S_RESET: ns = S_FETCH;

        // ---------------------------------------------------------
        // FETCH — check I-Cache first; LSU fallback only if needed.
        // ---------------------------------------------------------
        S_FETCH: begin
            icache_fetch_req  = 1'b1;
            icache_fetch_addr = pc;
            if (icache_fetch_valid) begin
                // I-Cache hit (or fill just completed)
                inst_latch_en   = 1'b1;
                inst_latch_data = icache_fetch_inst;
                ns = S_DECODE;
            end
            // On miss the I-Cache fill sequencer (in lm1_cpu) handles the
            // burst read from SRAM.  We simply stay in S_FETCH until the
            // fill finishes and icache_fetch_valid asserts.
        end

        S_FETCH_WAIT: begin
            // Legacy LSU-based fetch path (kept for fallback / future use)
            if (lsu_valid) begin
                inst_latch_en   = 1'b1;
                inst_latch_data = lsu_inst;
                ns = S_DECODE;
            end
        end

        // ---------------------------------------------------------
        // DECODE: latch decoded fields + operand values
        //
        // The decoder drives dec_in from the fetched instruction.
        // Port 1 reads regs[rd], port 2 reads regs[rs1].
        // We can capture regs[rs2] next cycle from port 1.
        // For simplicity, capture rs2 immediately via port 1 reuse:
        //   We set rf_rd1_addr = dec_in.rs2, and read rf_rd1_data.
        //   But port 1 is also reading rd this same cycle.
        //   → use two cycles or accept the collission.
        //
        // We do DECODE in one cycle: read rd(port1) + rs1(port2),
        // then EXECUTE reads rs2 on port1. This costs an extra cycle
        // for R-format instructions but simplifies the design.
        // ---------------------------------------------------------
        S_DECODE: begin
            dr_n     = dec_in;
            imm_n    = imm_sext_in;
            // Read rd on port1, rs1 on port2 (banked by thread)
            rf_rd1_addr = banked_addr(cur_thread, dec_in.rd);
            rf_rd2_addr = banked_addr(cur_thread, dec_in.rs1);
            opa_n    = rf_rd1_data;  // regs[rd]
            opb_n    = rf_rd2_data;  // regs[rs1]
            ns       = S_EXECUTE;
        end

        // ---------------------------------------------------------
        // EXECUTE: big dispatch
        //
        // At entry: dr/imm_r latched.
        // opa=regs[rd], opb=regs[rs1].
        // We need regs[rs2] on port 1 this cycle.
        // ---------------------------------------------------------
        S_EXECUTE: begin
            // Read rs2 on port 1 (banked by thread)
            rf_rd1_addr = banked_addr(cur_thread, dr.rs2);
            opc_n       = rf_rd1_data;   // regs[rs2] available
            // Use rf_rd1_data as rs2 value immediate this cycle
            // (wire directly — no latency issue since regfile reads are combinational)

            // Default: advance PC; the instruction retires
            pc_n    = pc + 64'd4;
            retire  = 1'b1;

            if (reserved_encoding(dr)) begin
                // A fault: the PC stays, and nothing else changes
                tc_n    = TRAP_UNIMPLEMENTED;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_TRAP_LOOKUP;
            end else begin
            case (dr.opcode)

            // ===== ALU R-type =====
            OP_ARITH_RAW: begin
                alu_op   = OP_ARITH_RAW;
                alu_func = dr.func;
                alu_a    = opb;            // rs1
                alu_b    = rf_rd1_data;    // rs2 (live)
                alu_start = (dr.func inside {FUNC_DIV, FUNC_MOD});
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_ALU_WAIT;
            end

            OP_BITWISE: begin
                alu_op   = OP_BITWISE;
                alu_func = dr.func;
                alu_a    = opb;
                alu_b    = rf_rd1_data;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_ALU_WAIT;
            end

            OP_ARITH_FIX: begin
                alu_op   = OP_ARITH_FIX;
                alu_func = dr.func;
                alu_a    = opb;
                alu_b    = rf_rd1_data;
                alu_start = (dr.func == FUNC_DIV_FIX);
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_ALU_WAIT;
            end

            OP_ADD_FIX_IMM: begin
                alu_op  = OP_ADD_FIX_IMM;
                alu_a   = opb;
                alu_b   = imm_r;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_ALU_WAIT;
            end

            OP_CMP_TAGGED: begin
                alu_op   = OP_CMP_TAGGED;
                alu_func = dr.func;
                alu_a    = opb;
                alu_b    = rf_rd1_data;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_ALU_WAIT;
            end

            OP_TST: begin
                alu_op  = OP_TST;
                alu_a   = opb;
                alu_b   = {48'b0, dr.imm16};
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_ALU_WAIT;
            end

            OP_TST_SHAPE: begin
                if (is_any_ref(opb)) begin
                    ta_n    = ref_address(opb);
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_HDR_READ;
                end else begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = VAL_NIL;
                    ns        = S_FETCH;
                end
            end

            // ===== Load immediate =====
            OP_LI: begin
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, dr.rd);
                rf_w_data = imm_r;
                ns        = S_FETCH;
            end

            OP_LUI: begin
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, dr.rd);
                rf_w_data = {32'b0, dr.imm16, 16'b0};
                ns        = S_FETCH;
            end

            OP_LI32: begin
                // Fetch 32-bit immediate from PC+4
                lsu_req  = 1'b1;
                lsu_op   = LSU_LOAD32;
                lsu_addr = pc + 64'd4;
                retire   = 1'b0;
                pc_n     = pc;
                if (lsu_ready) ns = S_FIELD_WAIT;
            end

            // ===== Raw memory =====
            OP_LDR: begin
                ta_n    = (opb + imm_r) & ~64'h7;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end

            OP_STR: begin
                ta_n    = (opa + imm_r) & ~64'h7;
                td_n    = opb;   // rs1 value to store
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end

            // ===== Sub-word loads =====
            OP_LDB: begin
                ta_n    = opb + imm_r;   // byte address, no alignment
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end
            OP_LDH: begin
                ta_n    = (opb + imm_r) & ~64'h1;  // halfword aligned
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end
            OP_LDW: begin
                ta_n    = (opb + imm_r) & ~64'h3;  // word aligned
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end

            // ===== Sub-word stores =====
            OP_STB: begin
                ta_n    = opa + imm_r;   // byte address
                td_n    = opb;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end
            OP_STH: begin
                ta_n    = (opa + imm_r) & ~64'h1;
                td_n    = opb;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end
            OP_STW: begin
                ta_n    = (opa + imm_r) & ~64'h3;
                td_n    = opb;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end

            // ===== Tagged field access =====
            OP_LD: begin
                if (!is_any_ref(opb)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opb) + {56'b0, dr.imm16[4:0] + 5'd1, 3'b0};
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_FIELD_MEM;
                end
            end

            OP_LD_CAR_CDR: begin
                if (!is_any_ref(opb)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opb) +
                              (dr.imm16[0] ? 64'd16 : 64'd8);
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_FIELD_MEM;
                end
            end

            OP_ST: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa) + {56'b0, dr.rs2 + 5'd1, 3'b0};
                    td_n    = opb;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_FIELD_MEM;
                end
            end

            // ===== Store with write barrier =====
            // Stores opb (rs1 value) into opa[field], then checks
            // whether a card-table mark is needed.
            //
            // Barrier fires when:
            //   1. Stored value (opb) is a ref (any_ref tag)
            //   2. The stored word's address, and so its container,
            //      is >= gen_boundary (old-gen)
            //   3. Stored ref (opb) address < gen_boundary (nursery)
            // This detects old→young cross-generation stores.
            //
            // If barrier fires: compute the card-table byte address of
            // the card holding the stored word, and issue a byte store
            // to mark the card dirty (0xFF).
            OP_ST_WB: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa) + {56'b0, dr.rs2 + 5'd1, 3'b0};
                    td_n    = opb;              // value being stored
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_FIELD_MEM;      // do the store first
                    // After S_FIELD_MEM completes, S_FIELD_WAIT sends
                    // ST.WB on to the barrier check in S_BARRIER_CHECK
                end
            end

            // ===== ST.CAR / ST.CDR =====
            // Barriered stores to a cons cell's car or cdr, the same as
            // ST.WB on field 0 or 1 (spec/02-isa.md § 5.5): S_FIELD_WAIT
            // sends them on to S_BARRIER_CHECK too.
            OP_ST_CAR_CDR: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa) +
                              (dr.rs2[0] ? 64'd16 : 64'd8);
                    td_n    = opb;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_FIELD_MEM;
                end
            end

            // ===== Branches =====
            OP_BR: begin
                br_is_br = 1'b1;
                if (br_taken) pc_n = br_target;
                ns = S_FETCH;
            end

            OP_BR_COND: begin
                br_is_cond = 1'b1;
                br_val     = opa;
                br_cond    = dr.rs1;
                if (br_taken) pc_n = br_target;
                ns = S_FETCH;
            end

            // ===== Stack PUSH/POP =====
            OP_PUSH_POP: begin
                rf_rd1_addr = banked_addr(cur_thread, REG_SP);
                begin
                    logic [XLEN-1:0] sp_v;
                    sp_v = rf_rd1_data;
                    if (dr.func == FUNC_PUSH) begin
                        ta_n      = sp_v - 64'd8;
                        td_n      = opa;         // value to push
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, REG_SP);
                        rf_w_data = sp_v - 64'd8;
                        retire    = 1'b0;
                        pc_n      = pc;
                        ns        = S_MEM;
                    end else begin
                        ta_n    = sp_v;   // load from current SP
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_MEM;
                    end
                end
            end

            OP_PUSH_MULTI: begin
                mm_n     = dr.imm16;
                mb_n     = {dr.rd[0], 4'b0};
                mdir_n   = 1'b0;
                mi_n     = 5'd0;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_MULTI_ITER;
            end

            OP_POP_MULTI: begin
                mm_n     = dr.imm16;
                mb_n     = {dr.rd[0], 4'b0};
                mdir_n   = 1'b1;
                mi_n     = 5'd15;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_MULTI_ITER;
            end

            // ===== Calls =====
            OP_CALL_DIRECT: begin
                tt_n     = pc + {{(XLEN-18){dr.imm16[15]}}, dr.imm16, 2'b00};
                td_n     = pc + 64'd4;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_PUSH_FRAME_0;
            end

            // CALL.REG Rs: a call to the address in Rs, read before the
            // frame changes LR, FP and SP (spec/02-isa.md § 6.8)
            OP_CALL_REG: begin
                tt_n     = opa;
                td_n     = pc + 64'd4;
                retire   = 1'b0;
                pc_n     = pc;
                ns       = S_PUSH_FRAME_0;
            end

            OP_CALL_CLOSURE: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_CLOSURE;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa);
                    td_n    = pc + 64'd4;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_HDR_READ;
                end
            end

            OP_CALL_IC: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa);
                    td_n    = pc + 64'd4;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_HDR_READ;
                end
            end

            OP_TAILCALL_DIR: begin
                pc_n = pc + {{(XLEN-18){dr.imm16[15]}}, dr.imm16, 2'b00};
                ns   = S_FETCH;
            end

            OP_TAILCALL_IC: begin
                if (!is_any_ref(opa)) begin
                    tc_n    = TRAP_NOT_REF;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    ta_n    = ref_address(opa);
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_HDR_READ;
                end
            end

            OP_RET: begin
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_POP_FRAME_0;
            end

            OP_JR: begin
                pc_n = opa;
                ns   = S_FETCH;
            end

            // ===== IC Install =====
            OP_IC_INSTALL: begin
                if (is_any_ref(opb)) begin
                    ta_n    = ref_address(opb);
                    td_n    = rf_rd1_data;   // rs2 = code entry
                    tt_n    = opa;           // rd = callsite PC
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_HDR_READ;
                end else begin
                    ic_inst_valid  = 1'b1;
                    ic_inst_pc     = opa;
                    ic_inst_shape  = '0;
                    ic_inst_target = rf_rd1_data;
                    ns = S_FETCH;
                end
            end

            // ===== Allocation =====
            OP_ALLOC: begin
                rf_rd1_addr = banked_addr(cur_thread, REG_NP);
                rf_rd2_addr = banked_addr(cur_thread, REG_NL);
                begin
                    logic [XLEN-1:0] np_v, nl_v, new_np;
                    logic [15:0] nw;
                    np_v   = rf_rd1_data;
                    nl_v   = rf_rd2_data;
                    nw     = {11'b0, dr.rs1};
                    new_np = np_v + {45'b0, nw + 16'd1, 3'b0};
                    anw_n  = nw;
                    acon_n = 1'b0;
                    tmpl_rd_idx = dr.imm16[7:0];
                    if (new_np > nl_v) begin
                        tc_n    = TRAP_NURSERY_OVERFLOW;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_TRAP_LOOKUP;
                    end else begin
                        aa_n      = np_v;
                        acnt_n    = nw;
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, REG_NP);
                        rf_w_data = new_np;
                        retire    = 1'b0;
                        pc_n      = pc;
                        ns        = S_ALLOC_HDR;
                    end
                end
            end

            OP_ALLOC_CONS: begin
                rf_rd1_addr = banked_addr(cur_thread, REG_NP);
                rf_rd2_addr = banked_addr(cur_thread, REG_NL);
                begin
                    logic [XLEN-1:0] np_v, nl_v, new_np;
                    np_v   = rf_rd1_data;
                    nl_v   = rf_rd2_data;
                    new_np = np_v + 64'd24;
                    anw_n  = 16'd2;
                    acon_n = 1'b1;
                    tmpl_rd_idx = 8'd0;
                    if (new_np > nl_v) begin
                        tc_n    = TRAP_NURSERY_OVERFLOW;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_TRAP_LOOKUP;
                    end else begin
                        aa_n      = np_v;
                        acnt_n    = 16'd2;
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, REG_NP);
                        rf_w_data = new_np;
                        retire    = 1'b0;
                        pc_n      = pc;
                        // Need an extra cycle to capture rs2 (cdr) since
                        // rf_rd1_addr was overridden to read NP above.
                        ns        = S_ALLOC_RD_CDR;
                    end
                end
            end

            OP_ALLOCV: begin
                rf_rd1_addr = banked_addr(cur_thread, REG_NP);
                rf_rd2_addr = banked_addr(cur_thread, REG_NL);
                if (!is_fixnum(opb) ||
                    $signed(opb) < 0 || $signed(opb) > $signed(64'd131068)) begin
                    // The length must be a fixnum (spec/02-isa.md § 4.3)
                    // from 0 to 65,534, whose payload of length + 1 words
                    // the header's 16-bit size holds
                    tc_n    = is_fixnum(opb) ? TRAP_BAD_LENGTH : TRAP_NOT_FIXNUM;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    logic [XLEN-1:0] np_v, nl_v, new_np;
                    logic signed [XLEN-1:0] len_s;
                    logic [15:0] nw;
                    np_v  = rf_rd1_data;
                    nl_v  = rf_rd2_data;
                    len_s = $signed(opb) >>> 1;
                    nw    = len_s[15:0] + 16'd1;      // at most 65,535
                    anw_n = nw;
                    acon_n = 1'b0;
                    tmpl_rd_idx = dr.imm16[7:0];
                    // The object's words, header included, need 17 bits
                    new_np = np_v + {44'b0, {1'b0, nw} + 17'd1, 3'b0};
                    if (new_np > nl_v) begin
                        tc_n    = TRAP_NURSERY_OVERFLOW;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_TRAP_LOOKUP;
                    end else begin
                        aa_n      = np_v;
                        acnt_n    = nw;
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, REG_NP);
                        rf_w_data = new_np;
                        retire    = 1'b0;
                        pc_n      = pc;
                        ns        = S_ALLOC_HDR;
                    end
                end
            end

            OP_ALLOC_CLOSURE: begin
                rf_rd1_addr = banked_addr(cur_thread, REG_NP);
                rf_rd2_addr = banked_addr(cur_thread, REG_NL);
                begin
                    logic [XLEN-1:0] np_v, nl_v, new_np;
                    logic [15:0] nw;
                    nw     = {11'b0, dr.rs2} + 16'd1;
                    np_v   = rf_rd1_data;
                    nl_v   = rf_rd2_data;
                    new_np = np_v + {45'b0, nw + 16'd1, 3'b0};
                    anw_n  = nw;
                    acon_n = 1'b0;
                    tmpl_rd_idx = 8'd1;
                    if (new_np > nl_v) begin
                        tc_n    = TRAP_NURSERY_OVERFLOW;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_TRAP_LOOKUP;
                    end else begin
                        aa_n      = np_v;
                        acnt_n    = nw;
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, REG_NP);
                        rf_w_data = new_np;
                        retire    = 1'b0;
                        pc_n      = pc;
                        ns        = S_ALLOC_HDR;
                    end
                end
            end

            // ===== TRAP =====
            OP_TRAP: begin
                tc_n = dr.raw26[7:0];
                if (dr.raw26[7]) begin
                    // System traps
                    case (dr.raw26[7:0])
                        8'h90: begin  // SET_TRAP_TABLE
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            trap_tbl_n   = rf_rd1_data;
                        end
                        8'h91: begin  // SET_TEMPLATE
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            rf_rd2_addr = banked_addr(cur_thread, 5'd2);
                            tmpl_wr_en   = 1'b1;
                            tmpl_wr_idx  = rf_rd1_data[7:0];
                            tmpl_wr_data = rf_rd2_data;
                        end
                        8'h92: begin  // SET_CARD_BASE
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            card_table_base_n = rf_rd1_data;
                        end
                        8'h93: begin  // SET_CARD_SHIFT
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            card_shift_n = rf_rd1_data[5:0];
                        end
                        8'h94: begin  // SET_GEN_BOUNDARY
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            gen_boundary_n = rf_rd1_data;
                        end
                        8'h95: begin  // SET_QUEUE_BASE
                            rf_rd1_addr = banked_addr(cur_thread, 5'd1);
                            queue_base_n = rf_rd1_data;
                        end
                        default: begin
                            // A reserved system code: a fault, so the PC
                            // stays at the TRAP
                            tc_n    = TRAP_UNIMPLEMENTED;
                            retire  = 1'b0;
                            pc_n    = pc;
                        end
                    endcase
                    ns = (dr.raw26[7:0] inside {[8'h90:8'h95]}) ? S_FETCH : S_TRAP_LOOKUP;
                end else begin
                    // User trap: advance PC past the TRAP instruction
                    // so that ERET returns to the instruction after TRAP.
                    // (pc_n and retire keep their S_EXECUTE defaults:
                    //  pc_n = pc + 4, retire = 1)
                    ns      = S_TRAP_LOOKUP;
                end
            end

            // ERET (func 0) resumes at the saved PC, so a fault runs again;
            // ERET.NEXT (func 1) resumes at the saved PC + 4, after a
            // faulting instruction the handler carried out itself
            // (spec/03-core.md § 6.3).
            OP_ERET: begin
                if (!in_trap) begin
                    tc_n    = TRAP_UNIMPLEMENTED;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    in_trap_n = 1'b0;
                    pc_n      = (dr.func == FUNC_ERET_NEXT) ? trap_pc + 64'd4 : trap_pc;
                    ns        = S_FETCH;
                end
            end

            // ===== SYS_INFO =====
            OP_SYS_INFO: begin
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, dr.rd);
                case (dr.rs1)
                    SYS_TILE_ID:      rf_w_data = cfg_tile_id;
                    SYS_THREAD_ID:    rf_w_data = cfg_thread_id;
                    SYS_CYCLE:        rf_w_data = cyc;
                    SYS_INSTRET:      rf_w_data = instret[cur_thread];
                    SYS_TRAP_CAUSE:   rf_w_data = {56'b0, trap_cause};
                    SYS_TRAP_PC:      rf_w_data = trap_pc;
                    SYS_CARD_BASE:    rf_w_data = card_table_base;
                    SYS_CARD_SHIFT:   rf_w_data = {58'b0, card_shift};
                    SYS_GEN_BOUNDARY: rf_w_data = gen_boundary;
                    SYS_QUEUE_BASE:   rf_w_data = queue_base;
                    SYS_GC_STATUS:    rf_w_data = {63'b0, gc_engine_busy};
                    SYS_SCAN_COUNT:     rf_w_data = {56'b0, scan_fifo_count};
                    SYS_SCAN_HEAD_OBJ:  rf_w_data = scan_fifo_head_obj;
                    SYS_SCAN_HEAD_FIELD: rf_w_data = {48'b0, scan_fifo_head_field};
                    SYS_SCAN_POP_REF: begin
                        rf_w_data     = scan_fifo_head_ref;
                        scan_fifo_pop = (scan_fifo_count != 8'd0);
                    end
                    SYS_PERF_CTR: begin
                        // Counter ID in imm16[4:0]
                        ctr_id    = dr.imm16[4:0];
                        rf_w_data = ctr_value;
                    end
                    default:          rf_w_data = '0;
                endcase
                ns = S_FETCH;
            end

            // ===== HALT / NOP =====
            OP_HALT_NOP: begin
                if (dr.rd == 5'd0)
                    ns = S_HALTED;
                else
                    ns = S_FETCH;
            end

            // ===== No-ops: Prefetch family =====
            OP_PREFETCH_REF, OP_PREFETCH_FLD,
            OP_PREFETCH_CDR, OP_GATHER_PRE: begin
                ns = S_FETCH;
            end

            // ===== GC engine commands =====
            // ENQ.SCAN/COPY/FIXUP/COMPACT: issue a command to the
            // cluster's movement engines.
            //   rs1 = region base address (opb)
            //   rs2 = region size / dest addr (read from port 1)
            OP_ENQ_SCAN: begin
                gc_cmd_valid = 1'b1;
                gc_cmd_op    = GC_CMD_SCAN;
                gc_cmd_arg0  = opb;             // region base
                gc_cmd_arg1  = rf_rd1_data;     // region size (rs2 on port1)
                rf_rd1_addr  = banked_addr(cur_thread, dr.rs2);
                if (gc_cmd_ready) begin
                    ns = S_FETCH;
                end else begin
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_ENQ_WAIT;
                end
            end

            OP_ENQ_COPY: begin
                gc_cmd_valid = 1'b1;
                gc_cmd_op    = GC_CMD_COPY;
                gc_cmd_arg0  = opb;
                gc_cmd_arg1  = rf_rd1_data;
                gc_cmd_arg2  = opa;             // rd = region size
                rf_rd1_addr  = banked_addr(cur_thread, dr.rs2);
                if (gc_cmd_ready) begin
                    ns = S_FETCH;
                end else begin
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_ENQ_WAIT;
                end
            end

            OP_ENQ_FIXUP: begin
                gc_cmd_valid = 1'b1;
                gc_cmd_op    = GC_CMD_FIXUP;
                gc_cmd_arg0  = opb;
                gc_cmd_arg1  = rf_rd1_data;
                rf_rd1_addr  = banked_addr(cur_thread, dr.rs2);
                if (gc_cmd_ready) begin
                    ns = S_FETCH;
                end else begin
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_ENQ_WAIT;
                end
            end

            OP_ENQ_COMPACT: begin
                gc_cmd_valid = 1'b1;
                gc_cmd_op    = GC_CMD_COMPACT;
                gc_cmd_arg0  = opb;
                gc_cmd_arg1  = rf_rd1_data;
                rf_rd1_addr  = banked_addr(cur_thread, dr.rs2);
                if (gc_cmd_ready) begin
                    ns = S_FETCH;
                end else begin
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_ENQ_WAIT;
                end
            end

            // ===== FAA / FENCE.GC =====
            OP_FAA_FENCE: begin
                if (dr.func == FUNC_FENCE_GC) begin
                    // FENCE.GC: wait until all GC engines are idle
                    // and all pending barrier stores are drained
                    if (gc_engine_busy) begin
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_FENCE_GC;
                    end else begin
                        ns = S_FETCH;
                    end
                end else begin
                    ta_n    = is_any_ref(opb) ? ref_address(opb) : opb;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_MEM;
                end
            end

            // ===== CAS.TAGGED =====
            OP_CAS_TAGGED: begin
                ta_n    = is_any_ref(opb) ?
                          (ref_address(opb) & ~64'h7) : (opb & ~64'h7);
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_MEM;
            end

            // ===== SEND/RECV =====
            //
            // SEND Rs_queue, Rt_value — opa (bits 25:21) is the descriptor,
            //   opb (bits 20:16) the value.  A full queue traps
            //   TRAP_QUEUE_FULL.
            // RECV Rd, Rs_queue / TRY.RECV Rd, Rd2, Rs_queue — opb (bits
            //   20:16) is the descriptor; func != 0 makes it TRY.RECV,
            //   whose func names Rd2.  On an empty queue RECV waits: it
            //   ends its turn without retiring, so the scheduler moves on
            //   and the thread runs the RECV again at its next turn; and
            //   TRY.RECV gives nil, nil.
            //
            // A descriptor that is not a fixnum traps TRAP_NOT_FIXNUM, and
            // one naming a tile no route reaches TRAP_NO_ROUTE.  The tile's
            // own queues answer here; a busy one, or another tile's, makes
            // the instruction wait in S_SEND_WAIT or S_RECV_WAIT.
            OP_SEND: begin
                logic [XLEN-1:0] qv;
                qv         = qd_value(opa);
                ta_n       = {2'b00, qv[XLEN-1:2]};   // the tile
                tt_n       = {62'b0, qv[1:0]};        // the queue
                td_n       = opb;                     // the value
                mq_wr_id   = qv[1:0];
                mq_wr_data = opb;
                if (!is_fixnum(opa) || qv[XLEN-1]) begin
                    tc_n    = is_fixnum(opa) ? TRAP_NO_ROUTE : TRAP_NOT_FIXNUM;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else if ({2'b00, qv[XLEN-1:2]} == cfg_tile_id && mq_wr_ready) begin
                    mq_wr_en = 1'b1;
                    ns       = S_FETCH;
                end else if ({2'b00, qv[XLEN-1:2]} == cfg_tile_id && mq_wr_full) begin
                    tc_n    = TRAP_QUEUE_FULL;
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_TRAP_LOOKUP;
                end else begin
                    retire  = 1'b0;
                    pc_n    = pc;
                    ns      = S_SEND_WAIT;
                end
            end

            OP_RECV: begin
                logic [XLEN-1:0] qv;
                qv       = qd_value(opb);
                ta_n     = {2'b00, qv[XLEN-1:2]};
                tt_n     = {62'b0, qv[1:0]};
                mq_rd_id = qv[1:0];
                retire   = 1'b0;
                pc_n     = pc;
                if (!is_fixnum(opb) || qv[XLEN-1]) begin
                    tc_n = is_fixnum(opb) ? TRAP_NO_ROUTE : TRAP_NOT_FIXNUM;
                    ns   = S_TRAP_LOOKUP;
                end else if ({2'b00, qv[XLEN-1:2]} == cfg_tile_id && mq_rd_empty) begin
                    // An empty queue of the tile's own
                    if (dr.func != '0) begin
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, dr.rd);
                        rf_w_data = VAL_NIL;
                        opa_n     = VAL_NIL;     // TRY.RECV's status
                        ns        = S_TRY_RECV_WB;
                    end else begin
                        ns = S_FETCH;            // RECV waits for a word
                    end
                end else if ({2'b00, qv[XLEN-1:2]} == cfg_tile_id && mq_rd_valid) begin
                    mq_rd_en  = 1'b1;
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = mq_rd_data;
                    if (dr.func != '0) begin
                        opa_n = VAL_T;           // TRY.RECV's status
                        ns    = S_TRY_RECV_WB;
                    end else begin
                        retire  = 1'b1;
                        pc_n    = pc + 64'd4;
                        ns      = S_FETCH;
                    end
                end else begin
                    ns = S_RECV_WAIT;
                end
            end

            default: begin
                tc_n    = TRAP_UNIMPLEMENTED;
                retire  = 1'b0;
                pc_n    = pc;
                ns      = S_TRAP_LOOKUP;
            end

            endcase  // dr.opcode in S_EXECUTE
            end  // not a reserved encoding
        end  // S_EXECUTE

        // ---------------------------------------------------------
        // ALU_WAIT: wait for ALU result (single or multi-cycle)
        // ---------------------------------------------------------
        S_ALU_WAIT: begin
            alu_op   = dr.opcode;
            alu_func = dr.func;
            alu_a    = opb;
            alu_b    = (dr.opcode == OP_ADD_FIX_IMM) ? imm_r :
                       (dr.opcode == OP_TST)         ? {48'b0, dr.imm16} : opc;

            if (alu_valid) begin
                if (alu_trap) begin
                    // A fault: the PC stays at the faulting instruction,
                    // which S_TRAP_LOOKUP saves, so ERET runs it again
                    // (spec/03-core.md § 6.3).
                    tc_n = alu_trap_code;
                    ns   = S_TRAP_LOOKUP;
                end else begin
                    retire    = 1'b1;
                    pc_n      = pc + 64'd4;
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = alu_result;
                    ns        = S_FETCH;
                end
            end
        end

        // ---------------------------------------------------------
        // MEM: issue load/store
        // ---------------------------------------------------------
        S_MEM: begin
            lsu_req = 1'b1;
            case (dr.opcode)
                OP_STR: begin
                    lsu_op    = LSU_STORE64;
                    lsu_addr  = ta;
                    lsu_wdata = td;
                end
                OP_PUSH_POP: begin
                    if (dr.func == FUNC_PUSH) begin
                        lsu_op    = LSU_STORE64;
                        lsu_addr  = ta;
                        lsu_wdata = td;
                    end else begin
                        lsu_op   = LSU_LOAD64;
                        lsu_addr = ta;
                    end
                end
                OP_FAA_FENCE: begin
                    lsu_op   = LSU_LOAD64;
                    lsu_addr = ta;
                end
                OP_CAS_TAGGED: begin
                    lsu_op   = LSU_LOAD64;
                    lsu_addr = ta;
                end
                // Sub-word loads
                OP_LDB: begin
                    lsu_op   = LSU_LOAD_BYTE;
                    lsu_addr = ta;
                end
                OP_LDH: begin
                    lsu_op   = LSU_LOAD_HALF;
                    lsu_addr = ta;
                end
                OP_LDW: begin
                    lsu_op   = LSU_LOAD_WORD;
                    lsu_addr = ta;
                end
                // Sub-word stores
                OP_STB: begin
                    lsu_op    = LSU_STORE_BYTE;
                    lsu_addr  = ta;
                    lsu_wdata = td;
                end
                OP_STH: begin
                    lsu_op    = LSU_STORE_HALF;
                    lsu_addr  = ta;
                    lsu_wdata = td;
                end
                OP_STW: begin
                    lsu_op    = LSU_STORE_WORD;
                    lsu_addr  = ta;
                    lsu_wdata = td;
                end
                default: begin
                    lsu_op   = LSU_LOAD64;
                    lsu_addr = ta;
                end
            endcase
            if (lsu_ready) ns = S_MEM_WAIT;
        end

        // ---------------------------------------------------------
        // MEM_WAIT
        // ---------------------------------------------------------
        S_MEM_WAIT: begin
            if (lsu_valid) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;

                case (dr.opcode)
                OP_LDR: begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = lsu_rdata;
                    ns        = S_FETCH;
                end

                OP_STR: begin
                    ns = S_FETCH;
                end

                // Sub-word loads: LSU already extracted and zero-extended
                OP_LDB, OP_LDH, OP_LDW: begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = lsu_rdata;
                    ns        = S_FETCH;
                end

                // Sub-word stores: done
                OP_STB, OP_STH, OP_STW: begin
                    ns = S_FETCH;
                end

                OP_PUSH_POP: begin
                    if (dr.func == FUNC_PUSH) begin
                        ns = S_FETCH;
                    end else begin
                        // POP: write loaded value to rd
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, dr.rd);
                        rf_w_data = lsu_rdata;
                        // SP += 8
                        ta_n = ta + 64'd8;
                        // Defer PC advance and retirement to S_MULTI_SP_WR
                        // (S_MEM_WAIT default already set retire=1, pc_n=pc+4
                        //  but S_MULTI_SP_WR will also set them → double advance)
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns   = S_MULTI_SP_WR;  // write SP
                    end
                end

                OP_FAA_FENCE: begin
                    if (!is_fixnum(lsu_rdata) || !is_fixnum(opc)) begin
                        // The word in memory and the delta must both be
                        // fixnums (spec/02-isa.md § 8.5).  A fault: nothing
                        // is stored or written, and the PC stays.
                        tc_n    = TRAP_NOT_FIXNUM;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_TRAP_LOOKUP;
                    end else begin
                        // old = mem[addr], store old + delta, return old
                        rf_we     = 1'b1;
                        rf_w_addr = banked_addr(cur_thread, dr.rd);
                        rf_w_data = lsu_rdata;
                        // Save computed new value for write-back via S_ATOMIC_STORE
                        // (cannot issue store here — LSU is still in WAIT_RD,
                        //  lsu_ready is 0, so the store would be silently dropped)
                        td_n    = lsu_rdata + opc;
                        retire  = 1'b0;
                        pc_n    = pc;
                        ns      = S_ATOMIC_STORE;
                    end
                end

                OP_CAS_TAGGED: begin
                    if (lsu_rdata == opc) begin
                        // Match → save new value for write-back via S_ATOMIC_STORE
                        // (cannot issue store here — LSU is still in WAIT_RD,
                        //  lsu_ready is 0, so the store would be silently dropped)
                        rf_rd1_addr = banked_addr(cur_thread, dr.func);
                        td_n        = rf_rd1_data;
                        rf_we       = 1'b1;
                        rf_w_addr   = banked_addr(cur_thread, dr.rd);
                        rf_w_data   = VAL_T;
                        retire      = 1'b0;
                        pc_n        = pc;
                        ns          = S_ATOMIC_STORE;
                    end else begin
                        rf_we       = 1'b1;
                        rf_w_addr   = banked_addr(cur_thread, dr.rd);
                        rf_w_data   = VAL_NIL;
                        ns          = S_FETCH;
                    end
                end

                default: ns = S_FETCH;
                endcase
            end
        end

        // ---------------------------------------------------------
        // FIELD_MEM: tagged field load/store
        // ---------------------------------------------------------
        S_FIELD_MEM: begin
            lsu_req = 1'b1;
            case (dr.opcode)
                OP_LD, OP_LD_CAR_CDR: begin
                    lsu_op   = LSU_LOAD64;
                    lsu_addr = ta;
                end
                default: begin
                    lsu_op    = LSU_STORE64;
                    lsu_addr  = ta;
                    lsu_wdata = td;
                end
            endcase
            if (lsu_ready) ns = S_FIELD_WAIT;
        end

        // ---------------------------------------------------------
        // FIELD_WAIT
        // ---------------------------------------------------------
        S_FIELD_WAIT: begin
            if (lsu_valid) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;

                case (dr.opcode)
                OP_LD, OP_LD_CAR_CDR: begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = lsu_rdata;
                    ns = S_FETCH;
                end
                OP_LI32: begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = lsu_rdata;
                    pc_n      = pc + 64'd8;
                    ns = S_FETCH;
                end
                OP_ST_WB, OP_ST_CAR_CDR: begin
                    // Store completed — now check barrier
                    ns = S_BARRIER_CHECK;
                end
                default: begin
                    ns = S_FETCH;
                end
                endcase
            end
        end

        // ---------------------------------------------------------
        // BARRIER_CHECK: decide if card-table mark is needed
        //
        // ST.WB, ST.CAR and ST.CDR come here after their store.
        // td = stored value (set in S_EXECUTE)
        // ta = address of the stored word (set in S_EXECUTE; the store
        //      states only read it)
        //
        // Filter 1: skip if stored value is not a ref
        // Filter 2: skip if same generation (both nursery or both old)
        //
        // A cross-gen store is: container in old-gen AND stored ref
        // points to nursery.  With gen_boundary configured:
        //   word_addr >= gen_boundary  AND  ref_addr < gen_boundary
        // The word lies inside its container, so it is old exactly when
        // the container is.
        // ---------------------------------------------------------
        S_BARRIER_CHECK: begin
            if (!is_any_ref(td)) begin
                // Filter 1: not a ref — no barrier needed
                ctr_barrier_filt_inc = 1'b1;
                ns = S_FETCH;
            end else begin
                // td is a ref — check generations
                begin
                    logic [XLEN-1:0] stored_addr;
                    stored_addr = ref_address(td);
                    if (gen_boundary != '0 &&
                        ta >= gen_boundary &&
                        stored_addr < gen_boundary) begin
                        // Cross-gen old→young: FIRE barrier
                        // Mark the card holding the stored word
                        // (spec/02-isa.md § 5.4, spec/05-memory-gc.md
                        // § 4.2), so a field past its header's card is
                        // found by the collector's scan of that card:
                        //   card_index = (word_addr - 0) >> card_shift
                        //   card_addr  = card_table_base + card_index
                        ta_n = card_table_base + (ta >> card_shift);
                        td_n = {56'hFF_FFFF_FFFF_FFFF, 8'hFF};  // dirty marker
                        ctr_barrier_fire_inc = 1'b1;
                        ns   = S_BARRIER_MARK;
                    end else begin
                        // Same-gen or gen_boundary not configured — filtered
                        ctr_barrier_filt_inc = 1'b1;
                        ns = S_FETCH;
                    end
                end
            end
        end

        // ---------------------------------------------------------
        // BARRIER_MARK: issue byte store to card table
        // ---------------------------------------------------------
        S_BARRIER_MARK: begin
            lsu_req   = 1'b1;
            lsu_op    = LSU_STORE_BYTE;  // single-byte store to card table
            lsu_addr  = ta;
            lsu_wdata = td;              // low byte = 0xFF (marks card dirty)
            if (lsu_ready) ns = S_BARRIER_MARK_W;
        end

        S_BARRIER_MARK_W: begin
            if (lsu_valid) begin
                ns = S_FETCH;
            end
        end

        // ---------------------------------------------------------
        // PUSH_FRAME: save LR, save FP, set LR=ret, FP=SP
        //   State 0: store LR at SP-8, SP-=8
        //   State W0: wait for store
        //   State 1: store FP at SP-8, SP-=8
        //   State W1: wait for store
        //   State 2: LR=ret_addr(td), FP=SP
        // ---------------------------------------------------------
        S_PUSH_FRAME_0: begin
            rf_rd1_addr = banked_addr(cur_thread, REG_SP);
            rf_rd2_addr = banked_addr(cur_thread, REG_LR);
            begin
                logic [XLEN-1:0] sp_v;
                sp_v = rf_rd1_data;
                ta_n  = sp_v - 64'd8;
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, REG_SP);
                rf_w_data = sp_v - 64'd8;
                lsu_req   = 1'b1;
                lsu_op    = LSU_STORE64;
                lsu_addr  = sp_v - 64'd8;
                lsu_wdata = rf_rd2_data;  // LR value
                if (lsu_ready) ns = S_PUSH_FRAME_W0;
            end
        end

        S_PUSH_FRAME_W0: begin
            if (lsu_valid) ns = S_PUSH_FRAME_1;
        end

        S_PUSH_FRAME_1: begin
            rf_rd1_addr = banked_addr(cur_thread, REG_SP);
            rf_rd2_addr = banked_addr(cur_thread, REG_FP);
            begin
                logic [XLEN-1:0] sp_v;
                sp_v = rf_rd1_data;
                ta_n  = sp_v - 64'd8;
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, REG_SP);
                rf_w_data = sp_v - 64'd8;
                lsu_req   = 1'b1;
                lsu_op    = LSU_STORE64;
                lsu_addr  = sp_v - 64'd8;
                lsu_wdata = rf_rd2_data;  // FP value
                if (lsu_ready) ns = S_PUSH_FRAME_W1;
            end
        end

        S_PUSH_FRAME_W1: begin
            if (lsu_valid) ns = S_PUSH_FRAME_2;
        end

        S_PUSH_FRAME_2: begin
            // LR = return address (td), FP = current SP
            rf_rd1_addr = banked_addr(cur_thread, REG_SP);
            rf_we       = 1'b1;
            rf_w_addr   = banked_addr(cur_thread, REG_LR);
            rf_w_data   = td;
            ta_n        = rf_rd1_data;  // save SP for FP write next cycle
            // Need one more cycle to write FP
            ns = S_POP_FRAME_2;  // reuse: writes FP=ta, then jumps to tt
        end

        // ---------------------------------------------------------
        // POP_FRAME: restore FP, LR, SP, return
        //   State 0: save LR→tt, load FP from mem[FP]
        //   State W0: wait; restore FP from rdata, load LR from mem[FP+8]
        //   State 1: wait; restore LR from rdata, SP = FP+16
        //   State W1: wait for any pending
        //   State 2: write FP=ta (reused from push_frame too), jump to tt
        // ---------------------------------------------------------
        S_POP_FRAME_0: begin
            rf_rd1_addr = banked_addr(cur_thread, REG_LR);
            rf_rd2_addr = banked_addr(cur_thread, REG_FP);
            tt_n        = rf_rd1_data;    // save LR as return address
            ta_n        = rf_rd2_data;    // FP value = restore point
            // SP = FP
            rf_we       = 1'b1;
            rf_w_addr   = banked_addr(cur_thread, REG_SP);
            rf_w_data   = rf_rd2_data;
            // Load saved FP from mem[FP]
            lsu_req     = 1'b1;
            lsu_op      = LSU_LOAD64;
            lsu_addr    = rf_rd2_data;
            if (lsu_ready) ns = S_POP_FRAME_W0;
        end

        S_POP_FRAME_W0: begin
            if (lsu_valid) begin
                // Restore FP from loaded data
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, REG_FP);
                rf_w_data = lsu_rdata;
                // Next cycle: issue load for saved LR
                ns = S_POP_FRAME_LD_LR;
            end
        end

        S_POP_FRAME_LD_LR: begin
            // Issue load of saved LR from mem[FP+8]
            // (LSU is back in IDLE now after the previous load completed)
            lsu_req   = 1'b1;
            lsu_op    = LSU_LOAD64;
            lsu_addr  = ta + 64'd8;
            if (lsu_ready) ns = S_POP_FRAME_1;
        end

        S_POP_FRAME_1: begin
            if (lsu_valid) begin
                // Restore LR
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, REG_LR);
                rf_w_data = lsu_rdata;
                // SP = FP + 16  (two saved words)
                ta_n = ta + 64'd16;
                ns   = S_POP_FRAME_W1;
            end
        end

        S_POP_FRAME_W1: begin
            // Write SP = restored FP+16
            rf_we     = 1'b1;
            rf_w_addr = banked_addr(cur_thread, REG_SP);
            rf_w_data = ta;
            ns        = S_POP_FRAME_2;
        end

        S_POP_FRAME_2: begin
            // For PUSH_FRAME completion: write FP = SP (ta holds SP)
            // For POP_FRAME / RET: jump to tt
            case (dr.opcode)
            OP_RET: begin
                retire  = 1'b1;
                pc_n    = tt;
                ns      = S_FETCH;
            end
            OP_PUSH_POP: begin
                // POP single — not used here (handled by MEM_WAIT)
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
            default: begin
                // CALL variants: write FP = ta (SP), jump to tt
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, REG_FP);
                rf_w_data = ta;
                retire    = 1'b1;
                pc_n      = tt;
                ns        = S_FETCH;
            end
            endcase
        end

        // ---------------------------------------------------------
        // MULTI: push/pop multiple registers
        // ---------------------------------------------------------
        S_MULTI_ITER: begin
            if (mm == 16'd0) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end else if (mm[mi[3:0]]) begin
                rf_rd1_addr = banked_addr(cur_thread, REG_SP);
                rf_rd2_addr = banked_addr(cur_thread, mb + mi);  // register to push/pop

                if (mdir) begin
                    // POP: load from [SP]
                    lsu_req  = 1'b1;
                    lsu_op   = LSU_LOAD64;
                    lsu_addr = rf_rd1_data;  // SP
                    if (lsu_ready) ns = S_MULTI_POP_WAIT;
                end else begin
                    // PUSH: SP -= 8, store reg at new SP
                    begin
                        logic [XLEN-1:0] new_sp;
                        new_sp = rf_rd1_data - 64'd8;
                        rf_we      = 1'b1;
                        rf_w_addr  = banked_addr(cur_thread, REG_SP);
                        rf_w_data  = new_sp;
                        lsu_req    = 1'b1;
                        lsu_op     = LSU_STORE64;
                        lsu_addr   = new_sp;
                        lsu_wdata  = rf_rd2_data;
                        if (lsu_ready) ns = S_MULTI_PUSH;
                    end
                end
            end else begin
                // Skip: advance index
                if (mdir) begin
                    if (mi == 5'd0) mm_n = 16'd0;
                    else            mi_n = mi - 5'd1;
                end else begin
                    if (mi == 5'd15) mm_n = 16'd0;
                    else             mi_n = mi + 5'd1;
                end
            end
        end

        S_MULTI_PUSH: begin
            // Wait for store to complete, advance index
            if (lsu_valid) begin
                if (mi == 5'd15) mm_n = 16'd0;
                else             mi_n = mi + 5'd1;
                ns = S_MULTI_ITER;
            end
        end

        S_MULTI_POP_WAIT: begin
            if (lsu_valid) begin
                // Write loaded value into register
                rf_we     = 1'b1;
                rf_w_addr = banked_addr(cur_thread, mb + mi);
                rf_w_data = lsu_rdata;
                // SP += 8 — need to do in next state
                ns = S_MULTI_SP_WR;
            end
        end

        S_MULTI_SP_WR: begin
            // Update SP after POP
            rf_rd1_addr = banked_addr(cur_thread, REG_SP);
            rf_we       = 1'b1;
            rf_w_addr   = banked_addr(cur_thread, REG_SP);
            rf_w_data   = rf_rd1_data + 64'd8;

            case (dr.opcode)
            OP_POP_MULTI: begin
                if (mi == 5'd0) mm_n = 16'd0;
                else            mi_n = mi - 5'd1;
                ns = S_MULTI_ITER;
            end
            OP_PUSH_POP: begin
                // Single POP done
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
            default: begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
            endcase
        end

        // ---------------------------------------------------------
        // ALLOCATION
        // ---------------------------------------------------------
        S_ALLOC_HDR: begin
            tmpl_rd_idx = (dr.opcode == OP_ALLOC_CONS)    ? 8'd0 :
                          (dr.opcode == OP_ALLOC_CLOSURE)  ? 8'd1 :
                          dr.imm16[7:0];
            lsu_req     = 1'b1;
            lsu_op      = LSU_STORE64;
            lsu_addr    = aa;
            lsu_wdata   = header_patch_size(tmpl_rd_data, anw);
            if (lsu_ready) ns = S_ALLOC_HDR_W;
        end

        S_ALLOC_HDR_W: begin
            if (lsu_valid) begin
                ta_n = aa + 64'd8;
                if (acnt == 16'd0)
                    ns = S_ALLOC_INIT;
                else
                    ns = S_ALLOC_ZERO;
            end
        end

        S_ALLOC_ZERO: begin
            lsu_req   = 1'b1;
            lsu_op    = LSU_STORE64;
            lsu_addr  = ta;
            lsu_wdata = '0;
            if (lsu_ready) ns = S_ALLOC_ZERO_W;
        end

        S_ALLOC_ZERO_W: begin
            if (lsu_valid) begin
                ta_n   = ta + 64'd8;
                acnt_n = acnt - 16'd1;
                if (acnt == 16'd1)
                    ns = S_ALLOC_INIT;
                else
                    ns = S_ALLOC_ZERO;
            end
        end

        S_ALLOC_INIT: begin
            case (dr.opcode)
            OP_ALLOC_CONS: begin
                lsu_req   = 1'b1;
                lsu_op    = LSU_STORE64;
                lsu_addr  = aa + 64'd8;
                lsu_wdata = (dr.rs1 == 5'd0) ? VAL_NIL : opb;
                if (lsu_ready) ns = S_ALLOC_INIT_W;
            end
            OP_ALLOC_CLOSURE: begin
                lsu_req   = 1'b1;
                lsu_op    = LSU_STORE64;
                lsu_addr  = aa + 64'd8;
                lsu_wdata = opb;
                if (lsu_ready) ns = S_ALLOC_DONE;
            end
            OP_ALLOCV: begin
                lsu_req   = 1'b1;
                lsu_op    = LSU_STORE64;
                lsu_addr  = aa + 64'd8;
                lsu_wdata = opb;  // tagged length
                if (lsu_ready) ns = S_ALLOC_DONE;
            end
            default: ns = S_ALLOC_DONE;
            endcase
        end

        S_ALLOC_INIT_W: begin
            // Wait for car write, then write cdr
            if (lsu_valid) ns = S_ALLOC_INIT2;
        end

        // ---------------------------------------------------------
        // ALLOC_RD_CDR: capture rs2 (cdr value) for ALLOC.CONS
        //
        // In S_EXECUTE, rf_rd1_addr was overridden to read NP/NL,
        // which prevents the default opc_n = rf_rd1_data from
        // capturing regs[rs2]. This state reads rs2 on port 1.  The
        // operands are read before the allocation (spec/02-isa.md
        // § 4.2), and S_EXECUTE has already bumped NP, so a cdr named
        // np takes NP's value from before the bump, which aa holds.
        // ---------------------------------------------------------
        S_ALLOC_RD_CDR: begin
            rf_rd1_addr = banked_addr(cur_thread, dr.rs2);
            opc_n       = (dr.rs2 == REG_NP) ? aa : rf_rd1_data;
            ns          = S_ALLOC_HDR;
        end

        S_ALLOC_INIT2: begin
            // CONS: write cdr at aa+16
            lsu_req   = 1'b1;
            lsu_op    = LSU_STORE64;
            lsu_addr  = aa + 64'd16;
            lsu_wdata = (dr.rs2 == 5'd0) ? VAL_NIL : opc;
            if (lsu_ready) ns = S_ALLOC_INIT2_W;
        end

        S_ALLOC_INIT2_W: begin
            if (lsu_valid) ns = S_ALLOC_DONE;
        end

        S_ALLOC_DONE: begin
            rf_we     = 1'b1;
            rf_w_addr = banked_addr(cur_thread, dr.rd);
            rf_w_data = make_ref(aa, acon);
            retire    = 1'b1;
            pc_n      = pc + 64'd4;
            ctr_alloc_inc       = 1'b1;
            ctr_alloc_bytes_inc = (anw + 16'd1) << 3;  // (nwords+1)*8
            ns        = S_FETCH;
        end

        // ---------------------------------------------------------
        // HDR_READ / HDR_WAIT: read object header
        // ---------------------------------------------------------
        S_HDR_READ: begin
            lsu_req  = 1'b1;
            lsu_op   = LSU_LOAD64;
            lsu_addr = ta;
            if (lsu_ready) ns = S_HDR_WAIT;
        end

        S_HDR_WAIT: begin
            if (lsu_valid) begin
                th_n = lsu_rdata;

                case (dr.opcode)
                OP_TST_SHAPE: begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    if (is_header(lsu_rdata) &&
                        (header_shape_id(lsu_rdata) & 32'hFFFF) ==
                        {16'b0, dr.imm16})
                        rf_w_data = VAL_T;
                    else
                        rf_w_data = VAL_NIL;
                    retire  = 1'b1;
                    pc_n    = pc + 64'd4;
                    ns      = S_FETCH;
                end

                OP_CALL_CLOSURE: begin
                    if (!is_header(lsu_rdata) ||
                        header_subtype(lsu_rdata) != HDR_CLOSURE) begin
                        tc_n = TRAP_NOT_CLOSURE;
                        ns   = S_TRAP_LOOKUP;
                    end else begin
                        // Read code entry at addr+8
                        ns = S_CLOS_CODE_RD;
                    end
                end

                OP_CALL_IC, OP_TAILCALL_IC: begin
                    begin
                        logic [31:0] shp;
                        shp = is_header(lsu_rdata) ?
                              header_shape_id(lsu_rdata) : 32'd0;
                        ic_lu_valid = 1'b1;
                        ic_lu_pc    = pc;
                        ic_lu_shape = shp;
                        ns          = S_IC_DISPATCH;
                    end
                end

                OP_IC_INSTALL: begin
                    begin
                        logic [31:0] shp;
                        shp = is_header(lsu_rdata) ?
                              header_shape_id(lsu_rdata) : 32'd0;
                        ic_inst_valid  = 1'b1;
                        ic_inst_pc     = tt;
                        ic_inst_shape  = shp;
                        ic_inst_target = td;
                        retire  = 1'b1;
                        pc_n    = pc + 64'd4;
                        ns      = S_FETCH;
                    end
                end

                default: ns = S_FETCH;
                endcase
            end
        end

        // ---------------------------------------------------------
        // CLOS_CODE_RD / WAIT: read closure code entry
        // ---------------------------------------------------------
        S_CLOS_CODE_RD: begin
            lsu_req  = 1'b1;
            lsu_op   = LSU_LOAD64;
            lsu_addr = ta + 64'd8;
            if (lsu_ready) ns = S_CLOS_CODE_WAIT;
        end

        S_CLOS_CODE_WAIT: begin
            if (lsu_valid) begin
                tt_n = lsu_rdata;
                ns   = S_PUSH_FRAME_0;
            end
        end

        // ---------------------------------------------------------
        // IC_DISPATCH
        // ---------------------------------------------------------
        S_IC_DISPATCH: begin
            case (dr.opcode)
            OP_CALL_IC: begin
                if (ic_hit) begin
                    tt_n = ic_hit_target;
                    ctr_ic_hit_inc = 1'b1;
                    ns   = S_PUSH_FRAME_0;
                end else begin
                    tc_n = TRAP_IC_MISS;
                    ctr_ic_miss_inc = 1'b1;
                    ns   = S_TRAP_LOOKUP;
                end
            end
            OP_TAILCALL_IC: begin
                if (ic_hit) begin
                    pc_n    = ic_hit_target;
                    retire  = 1'b1;
                    ctr_ic_hit_inc = 1'b1;
                    ns      = S_FETCH;
                end else begin
                    tc_n = TRAP_IC_MISS;
                    ctr_ic_miss_inc = 1'b1;
                    ns   = S_TRAP_LOOKUP;
                end
            end
            default: ns = S_FETCH;
            endcase
        end

        // ---------------------------------------------------------
        // SEND_WAIT: SEND waits for a busy queue of the tile's own, or
        // for another tile's queue through the remote port
        //   ta = tile, tt[1:0] = queue, td = value
        // ---------------------------------------------------------
        S_SEND_WAIT: begin
            if (ta == cfg_tile_id) begin
                mq_wr_id   = tt[1:0];
                mq_wr_data = td;
                if (mq_wr_ready) begin
                    mq_wr_en = 1'b1;
                    retire   = 1'b1;
                    pc_n     = pc + 64'd4;
                    ns       = S_FETCH;
                end else if (mq_wr_full) begin
                    tc_n = TRAP_QUEUE_FULL;
                    ns   = S_TRAP_LOOKUP;
                end
            end else if (rmq_ack) begin
                if (rmq_ok) begin
                    retire  = 1'b1;
                    pc_n    = pc + 64'd4;
                    ns      = S_FETCH;
                end else begin
                    tc_n = rmq_noroute ? TRAP_NO_ROUTE : TRAP_QUEUE_FULL;
                    ns   = S_TRAP_LOOKUP;
                end
            end
        end

        // ---------------------------------------------------------
        // RECV_WAIT: RECV and TRY.RECV wait for a busy queue of the
        // tile's own, or for another tile's queue through the remote port
        //   ta = tile, tt[1:0] = queue
        // ---------------------------------------------------------
        S_RECV_WAIT: begin
            logic          got, none;
            logic [XLEN-1:0] word;
            got  = 1'b0;
            none = 1'b0;
            word = '0;
            if (ta == cfg_tile_id) begin
                mq_rd_id = tt[1:0];
                if (mq_rd_valid) begin
                    mq_rd_en = 1'b1;
                    got      = 1'b1;
                    word     = mq_rd_data;
                end else if (mq_rd_empty) begin
                    none = 1'b1;
                end
            end else if (rmq_ack && rmq_noroute) begin
                tc_n = TRAP_NO_ROUTE;
                ns   = S_TRAP_LOOKUP;
            end else if (rmq_ack) begin
                got  = rmq_ok;
                none = !rmq_ok;
                word = rmq_rdata;
            end
            if (got || none) begin
                if (dr.func != '0 || got) begin
                    rf_we     = 1'b1;
                    rf_w_addr = banked_addr(cur_thread, dr.rd);
                    rf_w_data = got ? word : VAL_NIL;
                end
                if (dr.func != '0) begin
                    opa_n = got ? VAL_T : VAL_NIL;   // TRY.RECV's status
                    ns    = S_TRY_RECV_WB;
                end else if (got) begin
                    retire  = 1'b1;
                    pc_n    = pc + 64'd4;
                    ns      = S_FETCH;
                end else begin
                    ns = S_FETCH;                     // RECV waits for a word
                end
            end
        end

        // ---------------------------------------------------------
        // TRY_RECV_WB: write Rd2 = T (success) or NIL (empty)
        //   opa holds VAL_T or VAL_NIL from previous cycle
        //   dr.func[4:0] holds the Rd2 register index
        // ---------------------------------------------------------
        S_TRY_RECV_WB: begin
            rf_we     = 1'b1;
            rf_w_addr = banked_addr(cur_thread, dr.func);
            rf_w_data = opa;  // VAL_T or VAL_NIL
            retire    = 1'b1;
            pc_n      = pc + 64'd4;
            ns        = S_FETCH;
        end

        // ---------------------------------------------------------
        // ENQ_WAIT: wait for GC engine to accept command
        // ---------------------------------------------------------
        S_ENQ_WAIT: begin
            gc_cmd_valid = 1'b1;
            gc_cmd_op    = (dr.opcode == OP_ENQ_SCAN)    ? GC_CMD_SCAN :
                           (dr.opcode == OP_ENQ_COPY)    ? GC_CMD_COPY :
                           (dr.opcode == OP_ENQ_FIXUP)   ? GC_CMD_FIXUP :
                                                           GC_CMD_COMPACT;
            gc_cmd_arg0  = opb;
            gc_cmd_arg1  = opc;
            gc_cmd_arg2  = opa;    // region size for COPY
            if (gc_cmd_ready) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
        end

        // ---------------------------------------------------------
        // FENCE_GC: stall until all GC engines are idle
        // ---------------------------------------------------------
        S_FENCE_GC: begin
            if (!gc_engine_busy) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
        end

        // ---------------------------------------------------------
        // ATOMIC_STORE: issue the store part of FAA / CAS.TAGGED
        //
        // The load response was consumed in S_MEM_WAIT.  The LSU was
        // still in WAIT_RD that cycle (lsu_ready == 0), so we could
        // NOT issue the store there.  By the time we reach this state
        // the LSU has returned to IDLE (lsu_ready == 1) and we can
        // safely issue the store.
        // ---------------------------------------------------------
        S_ATOMIC_STORE: begin
            lsu_req   = 1'b1;
            lsu_op    = LSU_STORE64;
            lsu_addr  = ta;
            lsu_wdata = td;
            if (lsu_ready) ns = S_ATOMIC_STORE_W;
        end

        S_ATOMIC_STORE_W: begin
            if (lsu_valid) begin
                retire  = 1'b1;
                pc_n    = pc + 64'd4;
                ns      = S_FETCH;
            end
        end

        // ---------------------------------------------------------
        // TRAP LOOKUP
        // ---------------------------------------------------------
        S_TRAP_LOOKUP: begin
            trap_pc_n    = pc;
            trap_cause_n = tc;
            in_trap_n    = 1'b1;

            // Perf counter for nursery overflow
            if (tc == TRAP_NURSERY_OVERFLOW)
                ctr_nursery_ovf_inc = 1'b1;

            if (trap_tbl != '0 && tc < 8'h80) begin
                lsu_req  = 1'b1;
                lsu_op   = LSU_LOAD64;
                lsu_addr = trap_tbl + {52'b0, tc, 3'b0};
                if (lsu_ready) ns = S_TRAP_WAIT;
            end else begin
                ns = S_HALTED;
            end
        end

        S_TRAP_WAIT: begin
            if (lsu_valid) begin
                if (lsu_rdata == '0) begin
                    ns = S_HALTED;
                end else begin
                    pc_n = lsu_rdata;
                    ns   = S_FETCH;
                end
            end
        end

        // ---------------------------------------------------------
        S_HALTED: begin
            ns = S_HALTED;
        end

        default: ns = S_HALTED;

        endcase  // state
    end

endmodule
