// LM-1 chiplet module: instruction execution, traps and the run loops.
//
// Model::step is one turn of the reference's run loop (Emulator.run in
// emu/lm1/execute.py): fetch, execute, answer a trap, then advance the PC
// and the counters.  execute() follows Emulator._execute case by case, in
// the same order of effects, so a fault part way through an instruction
// leaves the state the reference leaves.
#include <algorithm>
#include <cstring>
#include <iterator>

#include "memory.h"
#include "model.h"

namespace lm1 {

namespace {

// Opcodes (bits 31:26, emu/lm1/decode.py).
enum Op : unsigned {
    OP_TST = 0, OP_TST_SHAPE = 1, OP_ARITH_FIX = 2, OP_ADD_FIX_IMM = 3, OP_CMP_TAGGED = 4,
    OP_LDW = 5, OP_STW = 6,
    OP_ALLOC = 8, OP_ALLOC_CONS = 9, OP_ALLOCV = 10, OP_ALLOC_CLOSURE = 11,
    OP_LD = 16, OP_LD_CAR_CDR = 17, OP_ST = 18, OP_ST_WB = 19, OP_ST_CAR_CDR = 20,
    OP_CALL_REG = 23,
    OP_CALL_IC = 24, OP_IC_INSTALL = 25, OP_CALL_DIRECT = 26, OP_CALL_CLOSURE = 27,
    OP_RET = 28, OP_TAILCALL_IC = 29, OP_TAILCALL_DIR = 30, OP_JR = 31,
    OP_PREFETCH_REF = 32, OP_PREFETCH_FLD = 33, OP_PREFETCH_CDR = 34, OP_GATHER_PRE = 35,
    OP_SEND = 36, OP_RECV = 37, OP_CAS_TAGGED = 38, OP_FAA_FENCE = 39,
    OP_LDB = 44, OP_STB = 45, OP_LDH = 46, OP_STH = 47,
    OP_ARITH_RAW = 48, OP_BITWISE = 49, OP_LDR = 50, OP_STR = 51, OP_BR = 52, OP_BR_COND = 53,
    OP_PUSH_POP = 54, OP_LI = 55, OP_LUI = 56, OP_PUSH_MULTI = 57, OP_POP_MULTI = 58,
    OP_LI32 = 59,
    OP_TRAP = 60, OP_ERET = 61, OP_SYS_INFO = 62, OP_HALT_NOP = 63
};

// System traps: the core's configuration traps, and the emulator's bridge
// to its host's console and block device (design/emulator.md section 7).
enum : uint32_t {
    EMU_TRAP_PUTCHAR = 0x80, EMU_TRAP_GETCHAR = 0x81, EMU_TRAP_BLOCK_IO = 0x82,
    EMU_TRAP_MEM_BYTE = 0x84, EMU_TRAP_SET_TRAP_TABLE = 0x90, EMU_TRAP_SET_TEMPLATE = 0x91,
    EMU_TRAP_SET_CARD_BASE = 0x92, EMU_TRAP_SET_CARD_SHIFT = 0x93,
    EMU_TRAP_SET_GEN_BOUNDARY = 0x94, EMU_TRAP_SET_QUEUE_BASE = 0x95,
    EMU_TRAP_DEBUG_PRINT = 0x9F
};

// ERET's func: 0 resumes at the saved PC, 1 (ERET.NEXT) after it.
constexpr unsigned FUNC_ERET_NEXT = 1;
constexpr unsigned FUNC_FENCE_GC = 0x1F;

struct Inst {
    unsigned op, rd, rs1, rs2, func;
    i64 imm16;     // sign-extended
    u64 raw26;
};

inline Inst decode(u64 raw) {
    Inst d;
    d.op = unsigned(raw >> 26) & 0x3F;
    d.rd = unsigned(raw >> 21) & 0x1F;
    d.rs1 = unsigned(raw >> 16) & 0x1F;
    d.rs2 = unsigned(raw >> 11) & 0x1F;
    d.func = unsigned(raw >> 6) & 0x1F;
    d.imm16 = int16_t(uint16_t(raw & 0xFFFF));
    d.raw26 = raw & 0x03FFFFFFull;
    return d;
}

// Whether an instruction carries a value of a selecting field that names no
// instruction (spec/07-encoding.md section 2, emu/lm1/decode.py reserved):
// it traps TRAP_UNIMPLEMENTED before it does anything.
inline bool reserved(const Inst &in) {
    switch (in.op) {
    case OP_ARITH_FIX: return in.func > 3;
    case OP_CMP_TAGGED: return in.func > 1;
    case OP_LD_CAR_CDR: return (u64(in.imm16) & 0xFFFF) > 1;
    case OP_ST_CAR_CDR: return in.rs2 > 1;
    case OP_FAA_FENCE: return in.func != 0 && in.func != FUNC_FENCE_GC;
    case OP_ARITH_RAW: return in.func > 4;
    case OP_BITWISE: return in.func > 6;
    case OP_BR_COND: return in.rs1 > 5;
    case OP_PUSH_POP: return in.func > 1;
    case OP_PUSH_MULTI: case OP_POP_MULTI: return in.rd > 1;  // two banks of 16
    case OP_ERET: return in.func > FUNC_ERET_NEXT;
    case OP_SYS_INFO: return in.rs1 > 15;  // 15, INSTRET, is the last assigned
    case OP_HALT_NOP: return in.rd > 1;
    default: return false;
    }
}

// The value of a fixnum, or the raw word when it is not one, as the
// reference decodes the emulator's byte-access trap's arguments.
inline i128 descriptor_value(u64 desc) {
    return is_fixnum(desc) ? i128(untag_fixnum(desc)) : i128(desc);
}

class Executor {
public:
    Executor(Model &m, Tile &tile, Thread &t) : m_(m), tile_(tile), t_(t) {}

    Flow raise(uint32_t code) {
        m_.trap.code = code;
        m_.trap.resumes = false;
        m_.trap.alloc_bytes = 0;
        return Flow::Trap;
    }

    Flow execute(const Inst &in, u64 &next_pc);

private:
    Flow write_barrier(u64 store_addr, u64 stored);
    Flow bump_alloc(i128 total_bytes, u64 &addr);
    Flow alloc_object(unsigned rd, i128 total_bytes, u64 tmpl_idx, i64 n_words,
                      bool init, u64 init_value);
    Flow alloc_cons(unsigned rd, u64 car, u64 cdr);
    Flow push_frame(u64 return_addr);
    Flow pop_frame(u64 &return_addr);
    Flow system_trap(uint32_t code);
    Flow queue_of(u64 desc, i64 &value, Queue *&queue);
    Flow block_io();
    Flow shape_of(u64 receiver, u64 &shape);
    // A value a tagged store may write into the shared heap: one that is no
    // reference, or a reference into the shared heap, whose metadata bits
    // the store clears (spec/05-memory-gc.md section 4.5).
    bool shared_value(u64 &v) const {
        if (!is_any_ref(v)) return true;
        if (!in_shared_heap(m_, ref_address(v))) return false;
        v &= ~REF_METADATA;
        return true;
    }

    u64 &reg(unsigned i) { return t_.regs[i & 31]; }
    uint32_t tile_index() const { return uint32_t(&tile_ - m_.tiles.data()); }

    Model &m_;
    Tile &tile_;
    Thread &t_;
};

// The card-table barrier of ST.WB, ST.CAR, ST.CDR and CAS.TAGGED: a young
// ref stored into an old word marks the card holding that word, in the
// model's table and in the one the runtime configured in memory
// (spec/02-isa.md section 5.4, spec/05-memory-gc.md section 4.2).
Flow Executor::write_barrier(u64 store_addr, u64 stored) {
    if (!is_any_ref(stored)) return Flow::Next;
    u64 val_addr = ref_address(stored);
    if (store_addr < tile_.gen_boundary) return Flow::Next;
    if (val_addr >= tile_.gen_boundary) return Flow::Next;
    u64 card_idx = tile_.card_shift < 64 ? store_addr >> tile_.card_shift : 0;
    if (tile_.card_table_base != 0) {
        if (!store_byte(m_, tile_, u128(tile_.card_table_base) + card_idx, 0xFF)) return Flow::Stop;
    }
    if (card_idx < tile_.cards.size()) tile_.cards[card_idx] = 0xFF;
    return Flow::Next;
}

Flow Executor::bump_alloc(i128 total_bytes, u64 &addr) {
    addr = t_.regs[REG_NP];
    i128 new_np = i128(addr) + total_bytes;
    if (new_np > i128(t_.regs[REG_NL])) {
        Flow f = raise(TRAP_NURSERY_OVERFLOW);
        m_.trap.alloc_bytes = total_bytes > i128(~0ull) ? ~0ull : u64(total_bytes);
        return f;
    }
    t_.regs[REG_NP] = u64(new_np);
    return Flow::Next;
}

Flow Executor::alloc_object(unsigned rd, i128 total_bytes, u64 tmpl_idx, i64 n_words,
                            bool init, u64 init_value) {
    u64 addr;
    Flow f = bump_alloc(total_bytes, addr);
    if (f != Flow::Next) return f;
    // The header comes from the template, with the object's size patched in.
    u64 tmpl = tmpl_idx < HEADER_TEMPLATES ? tile_.templates[tmpl_idx] : 0;
    u64 hdr = (tmpl & ~(0xFFFFull << 8)) | ((u64(n_words) & 0xFFFF) << 8);
    if (!store_word(m_, tile_, addr, hdr)) return Flow::Stop;
    for (i64 i = 1; i <= n_words; ++i) {
        if (!store_word(m_, tile_, u128(addr) + u128(i) * 8, 0)) return Flow::Stop;
    }
    if (init && !store_word(m_, tile_, u128(addr) + 8, init_value)) return Flow::Stop;
    reg(rd) = make_ref(addr, false);
    return Flow::Next;
}

Flow Executor::alloc_cons(unsigned rd, u64 car, u64 cdr) {
    u64 addr;
    Flow f = bump_alloc(24, addr);
    if (f != Flow::Next) return f;
    // Template 0 with the cons's two payload words as its size.
    u64 hdr = (tile_.templates[0] & ~(0xFFFFull << 8)) | (u64(2) << 8);
    if (!store_word(m_, tile_, addr, hdr)) return Flow::Stop;
    if (!store_word(m_, tile_, u128(addr) + 8, car)) return Flow::Stop;
    if (!store_word(m_, tile_, u128(addr) + 16, cdr)) return Flow::Stop;
    reg(rd) = make_ref(addr, true);
    return Flow::Next;
}

// A call frame: LR then FP pushed, LR set to the return address, FP to SP.
Flow Executor::push_frame(u64 return_addr) {
    u64 *r = t_.regs;
    r[REG_SP] -= 8;
    if (!store_word(m_, tile_, r[REG_SP], r[REG_LR])) return Flow::Stop;
    r[REG_SP] -= 8;
    if (!store_word(m_, tile_, r[REG_SP], r[REG_FP])) return Flow::Stop;
    r[REG_LR] = return_addr;
    r[REG_FP] = r[REG_SP];
    return Flow::Next;
}

Flow Executor::pop_frame(u64 &return_addr) {
    u64 *r = t_.regs;
    return_addr = r[REG_LR];
    r[REG_SP] = r[REG_FP];
    u64 v;
    if (!load_word(m_, tile_, r[REG_SP], v)) return Flow::Stop;
    r[REG_FP] = v;
    r[REG_SP] += 8;
    if (!load_word(m_, tile_, r[REG_SP], v)) return Flow::Stop;
    r[REG_LR] = v;
    r[REG_SP] += 8;
    return Flow::Next;
}

// The queue a descriptor names (spec/02-isa.md section 8, the reference's
// _descriptor and _queue_of): a descriptor that is not a fixnum traps
// TRAP_NOT_FIXNUM, and one naming a tile the model lacks, or a negative
// value, TRAP_NO_ROUTE.
Flow Executor::queue_of(u64 desc, i64 &value, Queue *&queue) {
    if (!is_fixnum(desc)) return raise(TRAP_NOT_FIXNUM);
    value = untag_fixnum(desc);
    Tile *tile = value >> 2 == i64(tile_.id) ? &tile_ : m_.tile_by_id(value >> 2);
    if (tile == nullptr) return raise(TRAP_NO_ROUTE);
    queue = &tile->queues[value & 3];
    return Flow::Next;
}

// The shape a receiver's header names, 0 for a header-less word.
Flow Executor::shape_of(u64 receiver, u64 &shape) {
    u64 hdr;
    if (!load_word(m_, tile_, ref_address(receiver), hdr)) return Flow::Stop;
    shape = is_header(hdr) ? header_shape_id(hdr) : 0;
    return Flow::Next;
}

Flow Executor::block_io() {
    u64 func = t_.regs[1], block = t_.regs[2], mem_addr = t_.regs[3];
    if (m_.host.block_io == nullptr) {  // no block device attached
        t_.regs[0] = tag_fixnum(-1);
        return Flow::Next;
    }
    uint8_t data[BLOCK_SIZE];
    if (func == 0) {
        int r = m_.host.block_io(m_.host.context, tile_index(), 0, block, data, BLOCK_SIZE);
        if (r == LM1C_HOST_FAIL) return m_.host_failure();
        if (r != LM1C_HOST_OK) { t_.regs[0] = tag_fixnum(-1); return Flow::Next; }
        for (u64 i = 0; i < BLOCK_SIZE; ++i) {
            if (!store_byte(m_, tile_, mem_addr + i, data[i])) return Flow::Stop;
        }
        t_.regs[0] = tag_fixnum(0);
    } else if (func == 1) {
        for (u64 i = 0; i < BLOCK_SIZE; ++i) {
            u64 b;
            if (!load_byte(m_, tile_, mem_addr + i, b)) return Flow::Stop;
            data[i] = uint8_t(b);
        }
        int r = m_.host.block_io(m_.host.context, tile_index(), 1, block, data, BLOCK_SIZE);
        if (r == LM1C_HOST_FAIL) return m_.host_failure();
        t_.regs[0] = tag_fixnum(r == LM1C_HOST_OK ? 0 : -1);
    } else {
        t_.regs[0] = tag_fixnum(-2);  // no such sub-function
    }
    return Flow::Next;
}

// TRAP #code for a code from 0x80 up (Emulator._system_trap): the
// configuration traps and the emulator's host bridge; any other code is
// reserved and traps TRAP_UNIMPLEMENTED, a fault that saves the TRAP's PC.
Flow Executor::system_trap(uint32_t code) {
    u64 *r = t_.regs;
    switch (code) {
    case EMU_TRAP_PUTCHAR: {
        if (m_.host.put_char == nullptr) return raise(code);  // no console
        u64 val = r[1];
        u64 ch;
        if (is_fixnum(val)) ch = u64(untag_fixnum(val)) & 0xFF;
        else if ((val & 0xFF) == 0x35) ch = (val >> 8) & 0x1FFFFF;  // a character
        else ch = val & 0xFF;
        if (m_.host.put_char(m_.host.context, tile_index(), uint32_t(ch)) != LM1C_HOST_OK)
            return m_.host_failure();
        return Flow::Next;
    }
    case EMU_TRAP_GETCHAR: {
        if (m_.host.get_char == nullptr) return raise(code);  // no console
        int64_t ch = -1;
        if (m_.host.get_char(m_.host.context, tile_index(), &ch) != LM1C_HOST_OK)
            return m_.host_failure();
        r[0] = tag_fixnum(ch < 0 ? -1 : ch);
        return Flow::Next;
    }
    case EMU_TRAP_BLOCK_IO:
        return block_io();
    case EMU_TRAP_SET_TRAP_TABLE:
        t_.trap_table_base = r[1];
        return Flow::Next;
    case EMU_TRAP_SET_TEMPLATE:
        tile_.templates[r[1] & 0xFF] = r[2];
        return Flow::Next;
    case EMU_TRAP_SET_CARD_BASE:
        tile_.card_table_base = r[1];
        return Flow::Next;
    case EMU_TRAP_SET_CARD_SHIFT:
        tile_.card_shift = r[1] & 0x3F;
        card_table_reset(tile_);
        return Flow::Next;
    case EMU_TRAP_SET_GEN_BOUNDARY:
        tile_.gen_boundary = r[1];
        return Flow::Next;
    case EMU_TRAP_SET_QUEUE_BASE:  // recorded; the modeled queues are not in memory
        tile_.queue_base = r[1];
        return Flow::Next;
    case EMU_TRAP_MEM_BYTE: {
        // r1 = 0 to load or 1 to store, r2 = address, r3 = offset, r4 = value
        i128 sub = descriptor_value(r[1]);
        u64 addr = r[2] + u64(descriptor_value(r[3]));
        if (sub == 0) {
            u64 b;
            if (!load_byte(m_, tile_, addr, b)) return Flow::Stop;
            r[1] = tag_fixnum(i64(b));
        } else if (sub == 1) {
            u64 val = u64(descriptor_value(r[4]));
            if (!store_byte(m_, tile_, addr, val & 0xFF)) return Flow::Stop;
            r[1] = tag_fixnum(0);
        }
        return Flow::Next;
    }
    case EMU_TRAP_DEBUG_PRINT: {
        // r1 = address of a byte string, r2 = its length.  The string is
        // read in full before anything is written.
        if (m_.host.put_char == nullptr) return raise(code);  // no console
        u64 addr = r[1], length = r[2];
        std::string text;
        for (u64 i = 0; i < length; ++i) {
            u64 b;
            if (!load_byte(m_, tile_, addr + i, b)) return Flow::Stop;
            text.push_back(char(b));
        }
        for (unsigned char c : text) {
            if (m_.host.put_char(m_.host.context, tile_index(), c) != LM1C_HOST_OK)
                return m_.host_failure();
        }
        return Flow::Next;
    }
    default:
        return raise(TRAP_UNIMPLEMENTED);
    }
}

Flow Executor::execute(const Inst &in, u64 &next_pc) {
    u64 *r = t_.regs;
    if (reserved(in)) return raise(TRAP_UNIMPLEMENTED);
    switch (in.op) {

    // ---- Scalar arithmetic (raw, untagged 64-bit) ----
    case OP_ARITH_RAW: {
        u64 a = r[in.rs1], b = r[in.rs2];
        switch (in.func) {
        case 0: r[in.rd] = a + b; break;
        case 1: r[in.rd] = a - b; break;
        case 2: r[in.rd] = a * b; break;
        case 3: if (b == 0) return raise(TRAP_DIVIDE_BY_ZERO); r[in.rd] = a / b; break;
        case 4: if (b == 0) return raise(TRAP_DIVIDE_BY_ZERO); r[in.rd] = a % b; break;
        default: break;
        }
        return Flow::Next;
    }

    case OP_BITWISE: {
        u64 a = r[in.rs1], b = r[in.rs2];
        unsigned shift = unsigned(b & 63);
        switch (in.func) {
        case 0: r[in.rd] = a & b; break;
        case 1: r[in.rd] = a | b; break;
        case 2: r[in.rd] = a ^ b; break;
        case 3: r[in.rd] = a << shift; break;
        case 4: r[in.rd] = a >> shift; break;
        case 5: r[in.rd] = u64(i64(a) >> shift); break;
        case 6: r[in.rd] = ~a; break;
        default: break;
        }
        return Flow::Next;
    }

    // ---- Raw loads and stores ----
    case OP_LDR: {
        u64 addr = (r[in.rs1] + u64(in.imm16)) & ~7ull;
        u64 v;
        if (!load_word(m_, tile_, addr, v)) return Flow::Stop;
        r[in.rd] = v;
        return Flow::Next;
    }
    case OP_STR: {
        u64 addr = (r[in.rd] + u64(in.imm16)) & ~7ull;  // Format S: Rs in rd
        if (!store_word(m_, tile_, addr, r[in.rs1])) return Flow::Stop;
        return Flow::Next;
    }
    case OP_LDB: case OP_LDH: case OP_LDW: {
        u64 addr = r[in.rs1] + u64(in.imm16);
        unsigned width = in.op == OP_LDB ? 8 : in.op == OP_LDH ? 16 : 32;
        if (width == 16) addr &= ~1ull;
        if (width == 32) addr &= ~3ull;
        u64 v;
        if (!load_bits(m_, tile_, addr, width, v)) return Flow::Stop;
        r[in.rd] = v;
        return Flow::Next;
    }
    case OP_STB: case OP_STH: case OP_STW: {
        u64 addr = r[in.rd] + u64(in.imm16);
        unsigned width = in.op == OP_STB ? 8 : in.op == OP_STH ? 16 : 32;
        if (width == 16) addr &= ~1ull;
        if (width == 32) addr &= ~3ull;
        if (!store_bits(m_, tile_, addr, width, r[in.rs1])) return Flow::Stop;
        return Flow::Next;
    }

    // ---- Immediates ----
    case OP_LI:
        r[in.rd] = u64(in.imm16);
        return Flow::Next;
    case OP_LI32: {
        u64 imm;
        if (!load_u32(m_, tile_, next_pc, imm)) return Flow::Stop;
        r[in.rd] = imm;
        next_pc += 4;  // skip the immediate word
        return Flow::Next;
    }
    case OP_LUI:
        r[in.rd] = (u64(in.imm16) & 0xFFFF) << 16;
        return Flow::Next;

    // ---- Branches ----
    case OP_BR:
        next_pc = t_.pc + u64(in.imm16) * 4;
        return Flow::Next;
    case OP_BR_COND: {
        // B format: bits 25:21 hold the register, bits 20:16 the condition.
        u64 v = r[in.rd];
        bool taken = false;
        switch (in.rs1) {
        case 0: taken = v != NIL && v != 0; break;              // BR.T
        case 1: taken = v == NIL; break;                        // BR.NIL
        case 2: taken = is_fixnum(v) && i64(v) < 0; break;      // BR.FIX.LT
        case 3: taken = v == 0; break;                          // BR.FIX.EQ
        case 4: taken = is_fixnum(v) && i64(v) > 0; break;      // BR.FIX.GT
        case 5: taken = v == 0; break;                          // BR.EQ
        default: break;
        }
        if (taken) next_pc = t_.pc + u64(in.imm16) * 4;
        return Flow::Next;
    }

    // ---- Stack ----
    case OP_PUSH_POP:
        if (in.func == 0) {
            r[REG_SP] -= 8;
            if (!store_word(m_, tile_, r[REG_SP], r[in.rd])) return Flow::Stop;
        } else if (in.func == 1) {
            u64 v;
            if (!load_word(m_, tile_, r[REG_SP], v)) return Flow::Stop;
            r[in.rd] = v;
            r[REG_SP] += 8;
        }
        return Flow::Next;
    case OP_PUSH_MULTI: {
        // Format I: rd = bank of 16 registers, 0 or 1, imm16 = register mask.
        unsigned base = in.rd * 16;
        u64 mask = u64(in.imm16) & 0xFFFF;
        for (unsigned i = 0; i < 16; ++i) {
            if (mask & (1u << i)) {
                r[REG_SP] -= 8;
                if (!store_word(m_, tile_, r[REG_SP], r[base + i])) return Flow::Stop;
            }
        }
        return Flow::Next;
    }
    case OP_POP_MULTI: {
        unsigned base = in.rd * 16;
        u64 mask = u64(in.imm16) & 0xFFFF;
        for (int i = 15; i >= 0; --i) {
            if (mask & (1u << i)) {
                u64 v;
                if (!load_word(m_, tile_, r[REG_SP], v)) return Flow::Stop;
                r[base + unsigned(i)] = v;
                r[REG_SP] += 8;
            }
        }
        return Flow::Next;
    }

    // ---- System ----
    case OP_TRAP: {
        uint32_t code = uint32_t(in.raw26 & 0xFF);
        if (code >= SYSTEM_TRAPS) return system_trap(code);
        // A trap for the trap table: the instruction's whole work, so it
        // resumes after itself, as a system call returns.
        raise(code);
        m_.trap.resumes = true;
        m_.trap.resume_pc = next_pc;
        return Flow::Trap;
    }
    case OP_ERET:
        // ERET (func 0) resumes at the saved PC, so a fault runs again;
        // ERET.NEXT (func 1) resumes after it (spec/03-core.md section 6.3).
        if (!t_.in_trap) return raise(TRAP_UNIMPLEMENTED);
        t_.in_trap = false;
        next_pc = in.func == FUNC_ERET_NEXT ? t_.trap_pc + 4 : t_.trap_pc;
        return Flow::Next;
    case OP_SYS_INFO: {
        // Bits 25:21 name Rd, bits 20:16 select what is read.
        u64 v;
        switch (in.rs1) {
        case 0: v = t_.tile_id; break;
        case 1: v = t_.thread_id; break;
        case 2: v = t_.cycle_count; break;
        case 3: v = t_.trap_cause; break;
        case 4: v = t_.trap_pc; break;
        case 5: v = tile_.card_table_base; break;
        case 6: v = tile_.card_shift; break;
        case 7: v = tile_.gen_boundary; break;
        case 8: v = tile_.queue_base; break;
        case 15: v = t_.instret; break;  // the instructions retired before this one
        default:
            v = 0;  // GC status, counters and scan results: no engines are modeled
            break;
        }
        r[in.rd] = v;
        return Flow::Next;
    }
    case OP_HALT_NOP:
        if (((in.raw26 >> 21) & 0x1F) == 0) t_.halted = true;  // else NOP
        return Flow::Next;

    // ---- Tagged arithmetic ----
    case OP_ARITH_FIX: {
        u64 a = r[in.rs1], b = r[in.rs2];
        if (!is_fixnum(a) || !is_fixnum(b)) return raise(TRAP_NOT_FIXNUM);
        switch (in.func) {
        case 0: {
            i64 s;
            if (__builtin_add_overflow(i64(a), i64(b), &s)) return raise(TRAP_FIXNUM_OVERFLOW);
            r[in.rd] = u64(s);
            break;
        }
        case 1: {
            i64 s;
            if (__builtin_sub_overflow(i64(a), i64(b), &s)) return raise(TRAP_FIXNUM_OVERFLOW);
            r[in.rd] = u64(s);
            break;
        }
        case 2: {
            // untag(a) times the tagged b is the tagged product.
            i64 p;
            if (__builtin_mul_overflow(untag_fixnum(a), i64(b), &p))
                return raise(TRAP_FIXNUM_OVERFLOW);
            r[in.rd] = u64(p);
            break;
        }
        case 3: {
            if (b == 0) return raise(TRAP_DIVIDE_BY_ZERO);
            i64 va = untag_fixnum(a), vb = untag_fixnum(b);
            u64 q = (va < 0 ? 0 - u64(va) : u64(va)) / (vb < 0 ? 0 - u64(vb) : u64(vb));
            bool negative = (va < 0) != (vb < 0);
            if (!negative && q >= u64(FIXNUM_LIMIT)) return raise(TRAP_FIXNUM_OVERFLOW);
            r[in.rd] = tag_fixnum(negative ? -i64(q) : i64(q));
            break;
        }
        default:
            break;
        }
        return Flow::Next;
    }
    case OP_ADD_FIX_IMM: {
        u64 a = r[in.rs1];
        if (!is_fixnum(a)) return raise(TRAP_NOT_FIXNUM);
        i64 s;
        if (__builtin_add_overflow(i64(a), in.imm16, &s)) return raise(TRAP_FIXNUM_OVERFLOW);
        r[in.rd] = u64(s);
        return Flow::Next;
    }
    case OP_CMP_TAGGED: {
        u64 a = r[in.rs1], b = r[in.rs2];
        if (in.func == 0) {
            if (is_fixnum(a) && is_fixnum(b)) {
                i64 va = i64(a), vb = i64(b);
                r[in.rd] = tag_fixnum(va < vb ? -1 : va == vb ? 0 : 1);
            } else if ((a & 7) == (b & 7)) {
                r[in.rd] = tag_fixnum(a == b ? 0 : 1);
            } else {
                return raise(TRAP_TYPE_MISMATCH);
            }
        } else if (in.func == 1) {
            r[in.rd] = a == b ? T : NIL;
        }
        return Flow::Next;
    }
    case OP_TST: {
        u64 v = r[in.rs1];
        bool m = false;
        switch (u64(in.imm16) & 7) {
        case 0: m = is_fixnum(v); break;
        case 1: m = (v & 3) == 1; break;
        case 2: m = (v & 7) == 3; break;
        case 3: m = (v & 7) == 5; break;
        case 4: m = v == NIL; break;
        case 5: m = (v & 0xFF) == 0x35; break;
        case 6: m = (v & 0xFF) == 0x3D; break;
        case 7: m = (v & 7) == 7; break;
        }
        r[in.rd] = m ? T : NIL;
        return Flow::Next;
    }

    case OP_PREFETCH_REF: case OP_PREFETCH_FLD: case OP_PREFETCH_CDR: case OP_GATHER_PRE:
        return Flow::Next;

    // ---- Allocation ----
    case OP_ALLOC: {
        unsigned rd = unsigned(in.raw26 >> 21) & 0x1F;
        i64 n_words = i64(in.raw26 >> 16) & 0x1F;
        return alloc_object(rd, i128(1 + n_words) * 8, in.raw26 & 0xFFFF, n_words, false, 0);
    }
    case OP_ALLOC_CONS: {
        unsigned rd = unsigned(in.raw26 >> 21) & 0x1F;
        u64 car = in.rs1 != 0 ? r[in.rs1] : NIL;  // register 0 stands for nil
        u64 cdr = in.rs2 != 0 ? r[in.rs2] : NIL;
        return alloc_cons(rd, car, cdr);
    }
    case OP_ALLOCV: {
        unsigned rd = unsigned(in.raw26 >> 21) & 0x1F;
        u64 length = r[(in.raw26 >> 16) & 0x1F];
        if (!is_fixnum(length)) return raise(TRAP_NOT_FIXNUM);
        // A header's 16-bit size holds the length word and the elements, so
        // a length runs from 0 to 65,534 (spec/02-isa.md section 4.3).
        if (untag_fixnum(length) < 0 || untag_fixnum(length) > 0xFFFF - 1)
            return raise(TRAP_BAD_LENGTH);
        // A header, the length word, then the elements.
        i64 n_words = 1 + untag_fixnum(length);
        return alloc_object(rd, (1 + i128(n_words)) * 8, in.raw26 & 0xFFFF, n_words,
                            true, length);
    }
    case OP_ALLOC_CLOSURE: {
        unsigned rd = unsigned(in.raw26 >> 21) & 0x1F;
        u64 code_ptr = r[(in.raw26 >> 16) & 0x1F];
        i64 n_words = 1 + (i64(in.raw26 >> 11) & 0x1F);  // the code entry, then the env
        return alloc_object(rd, i128(1 + n_words) * 8, 1, n_words, true, code_ptr);
    }

    // ---- Tagged field access ----
    case OP_LD: case OP_LD_CAR_CDR: {
        u64 obj = r[in.rs1];
        if (!is_any_ref(obj)) return raise(TRAP_NOT_REF);
        const u64 field = in.op == OP_LD ? (u64(in.imm16) & 0x1F) : (u64(in.imm16) & 1);
        const u64 field_addr = ref_address(obj) + (field + 1) * 8;
        u64 v;
        if (!load_word(m_, tile_, field_addr, v)) return Flow::Stop;
        // A reference loaded from the shared heap names the shared heap, or
        // the load traps and leaves its destination as it was.
        if (shared_route(m_, field_addr) && is_any_ref(v) && !in_shared_heap(m_, ref_address(v)))
            return raise(TRAP_SHARED_REF);
        r[in.rd] = v;
        return Flow::Next;
    }
    case OP_ST: case OP_ST_WB: case OP_ST_CAR_CDR: {
        // Format S: Rs (the object) in rd, Rt (the value) in rs1, the field
        // in rs2.  ST stores without the barrier; ST.WB, ST.CAR and ST.CDR
        // run it after the store.
        u64 obj = r[in.rd], val = r[in.rs1];
        if (!is_any_ref(obj)) return raise(TRAP_NOT_REF);
        u64 field_addr = ref_address(obj) + (u64(in.rs2) + 1) * 8;
        const bool shared = shared_route(m_, field_addr);
        if (shared && !shared_value(val)) return raise(TRAP_SHARED_STORE);
        if (!store_word(m_, tile_, field_addr, val)) return Flow::Stop;
        if (in.op == OP_ST) return Flow::Next;
        if (shared) {
            standin_barrier(m_, field_addr, val);
            return Flow::Next;
        }
        return write_barrier(field_addr, val);
    }
    case OP_TST_SHAPE: {
        u64 v = r[in.rs1];
        u64 shape_test = u64(in.imm16) & 0xFFFF;
        if (is_any_ref(v)) {
            u64 hdr;
            if (!load_word(m_, tile_, ref_address(v), hdr)) return Flow::Stop;
            r[in.rd] = is_header(hdr) && (header_shape_id(hdr) & 0xFFFF) == shape_test ? T : NIL;
        } else {
            r[in.rd] = NIL;
        }
        return Flow::Next;
    }

    // ---- Dispatch ----
    case OP_CALL_IC: case OP_TAILCALL_IC: {
        u64 receiver = r[in.rd];
        if (!is_any_ref(receiver)) return raise(TRAP_NOT_REF);
        u64 shape;
        if (shape_of(receiver, shape) != Flow::Next) return Flow::Stop;
        auto hit = tile_.ic.find(IcKey{t_.pc, shape});  // the call site is the PC
        if (hit == tile_.ic.end()) return raise(TRAP_IC_MISS);
        if (in.op == OP_CALL_IC) {
            Flow f = push_frame(next_pc);
            if (f != Flow::Next) return f;
        }
        next_pc = hit->second;
        return Flow::Next;
    }
    case OP_IC_INSTALL: {
        // Format R: Rs1 = receiver, Rs2 = code entry, Rd = call-site PC.
        u64 receiver = r[in.rs1], code_entry = r[in.rs2], callsite = r[in.rd];
        u64 shape = 0;
        if (is_any_ref(receiver) && shape_of(receiver, shape) != Flow::Next) return Flow::Stop;
        tile_.ic[IcKey{callsite, shape}] = code_entry;
        return Flow::Next;
    }
    case OP_CALL_DIRECT: {
        Flow f = push_frame(next_pc);
        if (f != Flow::Next) return f;
        next_pc = t_.pc + u64(in.imm16) * 4;
        return Flow::Next;
    }
    case OP_CALL_CLOSURE: {
        u64 closure = r[in.rd];
        if (!is_any_ref(closure)) return raise(TRAP_NOT_CLOSURE);
        u64 caddr = ref_address(closure), hdr, code_entry;
        if (!load_word(m_, tile_, caddr, hdr)) return Flow::Stop;
        if (!is_header(hdr) || header_subtype(hdr) != HDR_CLOSURE) return raise(TRAP_NOT_CLOSURE);
        if (!load_word(m_, tile_, caddr + 8, code_entry)) return Flow::Stop;
        Flow f = push_frame(next_pc);
        if (f != Flow::Next) return f;
        next_pc = code_entry;
        return Flow::Next;
    }
    case OP_CALL_REG: {
        // The target, read before the frame changes LR, FP and SP.
        u64 target = r[in.rd];
        Flow f = push_frame(next_pc);
        if (f != Flow::Next) return f;
        next_pc = target;
        return Flow::Next;
    }
    case OP_RET: {
        u64 ret;
        Flow f = pop_frame(ret);
        if (f != Flow::Next) return f;
        next_pc = ret;
        return Flow::Next;
    }
    case OP_JR:
        next_pc = r[in.rd];
        return Flow::Next;
    case OP_TAILCALL_DIR:
        next_pc = t_.pc + u64(in.imm16) * 4;
        return Flow::Next;

    // ---- Messaging and atomics ----
    case OP_SEND: {
        // Format S: Rs (bits 25:21) holds the queue descriptor, a fixnum
        // tile * 4 + queue, and Rt (bits 20:16) the value.
        i64 desc;
        Queue *q;
        if (queue_of(r[in.rd], desc, q) != Flow::Next) return Flow::Trap;
        if (q->full()) return raise(TRAP_QUEUE_FULL);
        q->push(r[in.rs1]);
        return Flow::Next;
    }
    case OP_RECV: {
        i64 desc;
        Queue *q;
        if (queue_of(r[in.rs1], desc, q) != Flow::Next) return Flow::Trap;
        bool any = q->size() > 0;
        if (in.func != 0) {
            // TRY.RECV: Rd gets the value or nil, the register named by func
            // gets t or nil.
            if (any) { r[in.rd] = q->pop(); r[in.func] = T; }
            else { r[in.rd] = NIL; r[in.func] = NIL; }
        } else if (any) {
            r[in.rd] = q->pop();
        } else {
            // RECV on an empty queue stalls the thread until the queue holds
            // a word, to run the RECV again.
            t_.stalled = true;
            t_.stall_queue = desc;
            next_pc = t_.pc;
        }
        return Flow::Next;
    }
    case OP_CAS_TAGGED: {
        // Format X: Rd, Rs_addr (bits 20:16), Rs_expected (15:11),
        // Rt_new (10:6).
        u64 a = r[in.rs1];
        u64 addr = is_any_ref(a) ? ref_address(a) : a;
        const bool shared = shared_route(m_, addr);
        u64 old;
        if (!load_word(m_, tile_, addr, old)) return Flow::Stop;
        if (shared && is_any_ref(old) && !in_shared_heap(m_, ref_address(old)))
            return raise(TRAP_SHARED_REF);
        if (old == r[in.rs2]) {
            u64 nv = r[in.func];
            if (shared && !shared_value(nv)) return raise(TRAP_SHARED_STORE);
            if (!store_word(m_, tile_, addr, nv)) return Flow::Stop;
            if (shared) standin_barrier(m_, addr, nv);
            else if (is_any_ref(nv) && write_barrier(addr, nv) != Flow::Next) return Flow::Stop;
            r[in.rd] = T;
        } else {
            r[in.rd] = NIL;
        }
        return Flow::Next;
    }
    case OP_FAA_FENCE: {
        if (in.func == 0x1F) return Flow::Next;  // FENCE.GC: nothing is pending
        u64 a = r[in.rs1];
        u64 addr = is_any_ref(a) ? ref_address(a) : a;
        u64 old;
        if (!load_word(m_, tile_, addr, old)) return Flow::Stop;
        u64 delta = r[in.rs2];
        if (!is_fixnum(old) || !is_fixnum(delta)) return raise(TRAP_NOT_FIXNUM);
        if (!store_word(m_, tile_, addr, old + delta)) return Flow::Stop;
        r[in.rd] = old;
        return Flow::Next;
    }

    default:
        return raise(TRAP_UNIMPLEMENTED);
    }
}

}  // namespace

Model::Model(const Config &config_)
    : config(config_), first_tile_id(config_.tile_id),
      queue_depth(size_t(config_.queue_depth)) {
    init_tiles();
    link.ranges.assign(size_t(config.lend_ranges), LentRange{});
}

void Model::init_tiles() {
    // Each tile starts over, so nothing of an earlier run survives: memory,
    // threads, tables and counters alike.  Once the tiles exist their
    // storage is reused, so a reset allocates nothing.
    tiles.resize(size_t(config.tiles));
    for (size_t i = 0; i < tiles.size(); ++i) {
        Tile &tile = tiles[i];
        tile.id = config.tile_id + i;
        tile.mem_size = config.mem;
        tile.mem.assign(size_t(config.mem / 8), 0);
        tile.current = 0;
        std::fill(std::begin(tile.templates), std::end(tile.templates), 0);
        tile.gc_count = 0;
        tile.instruction_count = 0;
        tile.card_table_base = 0;
        tile.queue_base = 0;
        tile.ic.clear();
        tile.nursery_base = config.nursery_base;
        tile.nursery_size = config.nursery_size;
        tile.oldgen_base = config.oldgen_base;
        tile.oldgen_size = config.oldgen_size;
        tile.oldgen_ptr = config.oldgen_base;
        tile.card_shift = config.card_shift;
        // The barrier counts addresses at or above the boundary as old, so
        // it starts at the nursery's end, with old-gen above.
        tile.gen_boundary = config.nursery_base + config.nursery_size;
        card_table_reset(tile);
        for (Queue &q : tile.queues) q.reset(queue_depth);
        // The tile's threads share its nursery as the runtime divides it
        // (spec/04-soc.md section 4.3): each bumps its own pointer through a
        // chunk of it, nursery_chunk bytes and at most an equal slice.
        u64 n = config.threads;
        u64 slice = (config.nursery_size / n) & ~7ull;
        tile.nursery_chunk = std::min(config.nursery_chunk, slice);
        tile.threads.assign(size_t(n), Thread{});
        for (u64 tid = 0; tid < n; ++tid) {
            Thread &t = tile.threads[size_t(tid)];
            t.tile_id = tile.id;
            t.thread_id = tid;
        }
        fresh_chunks(tile);
        // The default templates: a cons, a closure and a vector.
        tile.templates[0] = make_header(HDR_CONS, 2, 0);
        tile.templates[1] = make_header(HDR_CLOSURE, 0, 1);
        tile.templates[2] = make_header(HDR_VECTOR, 0, 2);
    }
}

void Model::reset_chiplet() {
    init_tiles();
    // LM-1's runtime loses everything it keeps in LM-1's memory.
    standin = Standin{};
    for (Tile &tile : tiles) {
        for (Thread &t : tile.threads) t.halted = true;
    }
    trap = {};
    stop = {};
    // The lend table keeps its storage; its entries are cleared.
    std::vector<LentRange> ranges = std::move(link.ranges);
    link = Link{};
    ranges.assign(size_t(config.lend_ranges), LentRange{});
    link.ranges = std::move(ranges);
}

Tile *Model::tile_by_id(i64 id) {
    if (id < 0 || u64(id) < first_tile_id) return nullptr;
    u64 index = u64(id) - first_tile_id;
    return index < tiles.size() ? &tiles[size_t(index)] : nullptr;
}

const Tile *Model::tile_by_id(i64 id) const {
    return const_cast<Model *>(this)->tile_by_id(id);
}

Flow Model::memory_fault(u128 address) {
    stop.stop = LM1C_STOP_MEMORY;
    stop.address = address > ~0ull ? ~0ull : u64(address);
    return Flow::Stop;
}

Flow Model::host_failure() {
    stop.stop = LM1C_STOP_HOST;
    return Flow::Stop;
}

void Model::fresh_chunks(Tile &tile) {
    for (size_t tid = 0; tid < tile.threads.size(); ++tid) {
        Thread &t = tile.threads[tid];
        t.slice_base = tile.nursery_base + tid * tile.nursery_chunk;
        t.regs[REG_NP] = t.slice_base;
        t.regs[REG_NL] = t.slice_base + tile.nursery_chunk;
    }
    tile.nursery_free = tile.nursery_base + tile.threads.size() * tile.nursery_chunk;
}

void card_table_reset(Tile &tile) {
    u64 entries = tile.card_shift < 64 ? tile.mem_size >> tile.card_shift : 0;
    tile.cards.assign(size_t(entries), 0);
}

// One step of a thread, a turn of the reference's run loop.  A trap below
// 0x80 vectors through the thread's trap table (Emulator._take_trap): ERET
// resumes a fault at the faulting instruction and an explicit TRAP after
// itself.  TRAP_NURSERY_OVERFLOW with no handler in the table is answered as
// the runtime answers it (spec/04-soc.md section 4.3), after which the
// allocation is retried: a fresh chunk for the thread from the nursery's
// free space, the allocation's size at least, while any is left, and a minor
// collection only when none is.  When the thread's chunk is already empty
// and no thread has taken free space since the last collection, no
// collection makes room, and the trap stands.
//
// The instruction retires, and adds one to the thread's INSTRET, when it has
// done its work (Emulator._step): a fault and a RECV that waits retire
// nothing, and an explicit TRAP retires before its trap is taken, whether a
// handler answers it or not.
Flow Model::step(Tile &tile, Thread &t) {
    u64 raw;
    if (!load_u32(*this, tile, t.pc, raw)) return Flow::Stop;
    u64 next_pc = t.pc + 4;
    Executor ex(*this, tile, t);
    Flow f = ex.execute(decode(raw), next_pc);
    if ((f == Flow::Next && !t.stalled) || (f == Flow::Trap && trap.resumes)) t.instret += 1;
    if (f == Flow::Trap) {
        uint32_t code = trap.code;
        bool handled_by_table = false;
        if (code == TRAP_NURSERY_OVERFLOW && t.trap_table_base != 0) {
            u64 handler;
            if (!load_word(*this, tile, u128(t.trap_table_base) + u128(code) * 8, handler))
                return Flow::Stop;
            handled_by_table = handler != 0;
        }
        if (code == TRAP_NURSERY_OVERFLOW && !handled_by_table) {
            const u128 need = (u128(trap.alloc_bytes) + 7) & ~u128(7);
            const u128 chunk = std::max<u128>(tile.nursery_chunk, need);
            if (u128(tile.nursery_free) + chunk <= u128(tile.nursery_base) + tile.nursery_size) {
                t.slice_base = tile.nursery_free;
                t.regs[REG_NP] = tile.nursery_free;
                t.regs[REG_NL] = u64(tile.nursery_free + chunk);
                tile.nursery_free = u64(tile.nursery_free + chunk);
            } else {
                const u64 fresh = tile.nursery_base + tile.threads.size() * tile.nursery_chunk;
                if (t.regs[REG_NP] == t.slice_base && tile.nursery_free == fresh) {
                    stop.stop = LM1C_STOP_TRAP;
                    stop.trap = code;
                    return Flow::Stop;
                }
                Flow c = collect(tile);
                if (c != Flow::Next) return c;
            }
            next_pc = t.pc;
        } else if (t.trap_table_base != 0 && code < SYSTEM_TRAPS) {
            t.trap_pc = trap.resumes ? trap.resume_pc : t.pc;
            t.trap_cause = code;
            t.in_trap = true;
            u64 handler;
            if (!load_word(*this, tile, u128(t.trap_table_base) + u128(code) * 8, handler))
                return Flow::Stop;
            if (handler == 0) {
                stop.stop = LM1C_STOP_TRAP;
                stop.trap = code;
                return Flow::Stop;
            }
            next_pc = handler;
        } else {
            stop.stop = LM1C_STOP_TRAP;
            stop.trap = code;
            return Flow::Stop;
        }
    } else if (f == Flow::Stop) {
        return f;
    }
    t.pc = next_pc;
    t.cycle_count += 1;
    tile.instruction_count += 1;
    return Flow::Next;
}

}  // namespace lm1
