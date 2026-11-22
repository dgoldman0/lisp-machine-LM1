// LM-1 chiplet module: the core model's state.
//
// The model follows the reference emulator in emu/lm1 (execute.py, word.py,
// memory.py, core.py, traps.py) step for step, so that both leave the same
// state after the same program, the reference's partial updates before a
// fault included.  Where the reference does arithmetic on Python's
// unbounded integers, the model works in 128 bits or checks for overflow,
// so that an address past 2^64 lies outside memory here as it does there.
#ifndef LM1_CHIPLET_MODEL_H
#define LM1_CHIPLET_MODEL_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "lm1_chiplet.h"

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "the model keeps memory as host words and reads bytes from them in place"
#endif

namespace lm1 {

using u64 = uint64_t;
using i64 = int64_t;
using u128 = unsigned __int128;
using i128 = __int128;

// Tagged words (spec/01-object-model.md, emu/lm1/word.py).
constexpr u64 NIL = 0x05;
constexpr u64 T = 0x0D;
constexpr u64 REF_ADDR_MASK = 0x0007FFFFFFFFFFF8ull;  // bits 50:3
constexpr u64 TAG_REF = 0b001;
constexpr u64 TAG_CONS = 0b011;
constexpr i64 FIXNUM_LIMIT = i64(1) << 62;

inline bool is_fixnum(u64 w) { return (w & 1) == 0; }
inline u64 tag_fixnum(i64 v) { return u64(v) << 1; }
inline i64 untag_fixnum(u64 w) { return i64(w) >> 1; }
inline bool is_any_ref(u64 w) { u64 t = w & 7; return t == 1 || t == 3; }
inline u64 ref_address(u64 w) { return w & REF_ADDR_MASK; }
inline u64 make_ref(u64 addr, bool cons) { return (addr & REF_ADDR_MASK) | (cons ? TAG_CONS : TAG_REF); }
inline bool is_header(u64 w) { return (w & 7) == 7; }
inline u64 header_size(u64 h) { return (h >> 8) & 0xFFFF; }
inline u64 header_shape_id(u64 h) { return (h >> 24) & 0xFFFFFFFFull; }
inline u64 header_subtype(u64 h) { return (h >> 3) & 0x1F; }
inline u64 make_header(u64 sub, u64 size, u64 shape) {
    return 0b111 | ((sub & 0x1F) << 3) | ((size & 0xFFFF) << 8) | ((shape & 0xFFFFFFFFull) << 24);
}
constexpr u64 HDR_CONS = 0b00001;
constexpr u64 HDR_VECTOR = 0b00010;
constexpr u64 HDR_CLOSURE = 0b00100;

// Registers (spec/02-isa.md section 2.1).
constexpr unsigned REGS = 32;
constexpr unsigned REG_NL = 25, REG_NP = 26, REG_LR = 28, REG_FP = 29, REG_SP = 30;

// Trap codes (spec/02-isa.md section 12, emu/lm1/traps.py).
constexpr uint32_t TRAP_NOT_FIXNUM = 0x01;
constexpr uint32_t TRAP_FIXNUM_OVERFLOW = 0x02;
constexpr uint32_t TRAP_DIVIDE_BY_ZERO = 0x03;
constexpr uint32_t TRAP_TYPE_MISMATCH = 0x04;
constexpr uint32_t TRAP_NOT_REF = 0x05;
constexpr uint32_t TRAP_NOT_CLOSURE = 0x07;
constexpr uint32_t TRAP_NURSERY_OVERFLOW = 0x10;
constexpr uint32_t TRAP_IC_MISS = 0x20;
constexpr uint32_t TRAP_QUEUE_FULL = 0x30;
constexpr uint32_t TRAP_UNIMPLEMENTED = 0xFE;

// A tile's header-template table: SET_TEMPLATE names an entry with eight
// bits, and an allocation's index past the table reads a zero template.
constexpr unsigned HEADER_TEMPLATES = 256;
// Queue descriptors name a queue with two bits (spec/04-soc.md).
constexpr unsigned QUEUES = 4;
// The block device's block (emulator TRAP 0x82).
constexpr u64 BLOCK_SIZE = 4096;

struct Thread {
    u64 regs[REGS] = {};
    u64 pc = 0;
    u64 cycle_count = 0;
    bool halted = false;
    bool stalled = false;
    i64 stall_queue = -1;
    u64 trap_table_base = 0;
    u64 trap_cause = 0;
    u64 trap_pc = 0;
    bool in_trap = false;
    u64 tile_id = 0;
    u64 thread_id = 0;
    u64 slice_base = 0;  // where this thread's slice of the nursery begins
};

// A bounded FIFO.  SEND refuses a full queue; a host push onto one drops
// the oldest value, as the reference's deque with a maximum length does.
class Queue {
public:
    void reset(size_t depth) { items_.assign(depth, 0); head_ = count_ = 0; }
    size_t size() const { return count_; }
    bool full() const { return count_ == items_.size(); }
    u64 at(size_t i) const { return items_[(head_ + i) % items_.size()]; }
    void push(u64 v) {
        if (items_.empty()) return;
        if (full()) { head_ = (head_ + 1) % items_.size(); --count_; }
        items_[(head_ + count_) % items_.size()] = v;
        ++count_;
    }
    u64 pop() {
        u64 v = items_[head_];
        head_ = (head_ + 1) % items_.size();
        --count_;
        return v;
    }
private:
    std::vector<u64> items_;
    size_t head_ = 0, count_ = 0;
};

struct IcKey {
    u64 callsite, shape;
    bool operator==(const IcKey &o) const { return callsite == o.callsite && shape == o.shape; }
};
struct IcHash {
    size_t operator()(const IcKey &k) const {
        u64 h = k.callsite * 0x9E3779B97F4A7C15ull ^ (k.shape + 0x7F4A7C159E3779B9ull);
        return size_t(h ^ (h >> 29));
    }
};

struct Tile {
    u64 id = 0;
    std::vector<u64> mem;  // the tile's memory, one host word per LM-1 word
    u64 mem_size = 0;      // bytes
    std::vector<Thread> threads;
    size_t current = 0;    // the thread that lm1c_run runs
    u64 templates[HEADER_TEMPLATES] = {};
    u64 nursery_base = 0, nursery_size = 0;
    u64 oldgen_base = 0, oldgen_size = 0, oldgen_ptr = 0;
    u64 card_shift = 6, card_table_base = 0, gen_boundary = 0;
    std::vector<uint8_t> cards;  // one byte per card, 0xFF when dirty
    u64 gc_count = 0;
    u64 instruction_count = 0;
    std::unordered_map<IcKey, u64, IcHash> ic;
    Queue queues[QUEUES];
};

// What an instruction, a trap or a collection did.
enum class Flow {
    Next,    // carry on at the next PC
    Trap,    // a trap was raised: Model::trap holds it
    Stop     // the run ends: Model::stop holds why
};

struct PendingTrap {
    uint32_t code = 0;
    bool resumes = false;  // an explicit TRAP: ERET resumes at resume_pc
    u64 resume_pc = 0;
};

// A range of LM-1's memory lent to MegaPad, which MegaPad's tile engines
// store into (rtl/doc/08-host-link.md).  The limit is exclusive.
struct LentRange {
    u64 base = 0;
    u64 limit = 0;
    bool write = false;
};

struct Config {
    u64 tiles = 1;
    u64 threads = 1;
    u64 mem = 4u << 20;
    u64 tile_id = 0;
    u64 nursery_base = 0x30000;
    u64 nursery_size = 0x10000;
    u64 oldgen_base = 0x80000;
    u64 oldgen_size = 0x40000;
    u64 card_shift = 6;
    u64 queue_depth = 512;
    // LM-1's end of the link (link.cpp), with the RTL's parameters as
    // defaults: lm1_link_boot's LOAD_BASE and BURST_WORDS, and lm1_link's
    // RANGES.  lend holds the ranges the runtime's stand-in lends once the
    // image is loaded, as (base, length), until LEND entries exist.
    u64 load_base = 0;
    u64 burst_words = 8;
    u64 lend_ranges = 2;
    u64 quanta = 1;
    std::vector<std::pair<u64, u64>> lend;
};

// Parses a configuration string; on failure returns false with a reason.
bool parse_config(const char *text, Config &config, std::string &error);
// The configuration as one canonical string, every key in a fixed order,
// which a saved state carries so that it restores only where it fits.
std::string config_text(const Config &config);

// Where LM-1's end of the link stands (link.cpp).
enum class Boot : uint8_t {
    Reset = 0,    // held in reset
    Fetch = 1,    // released; the image is still being fetched
    Running = 2,  // the image is loaded and the runtime is up
    Failed = 3    // a request or a word of the fetch failed
};

struct Link {
    Boot boot = Boot::Reset;
    u64 boot_addr = 0, boot_len = 0, boot_arg = 0;
    u64 fetched = 0;          // words of the image loaded so far
    bool stop_req = false;
    // What MegaPad was last told on the sideband.
    bool told_ready = false, told_stopped = false;
    std::vector<LentRange> ranges;  // the lend table, lend_ranges entries
    // The scheduler's place: the next thread, by its index over every
    // tile's threads, and the part of its quantum it has used.
    u64 cursor = 0, used = 0;
    // Threads that stopped on a trap no handler answers or a fault, and
    // the first of them.
    u64 faults = 0;
    lm1c_result first_fault = {};
    u64 doorbells_in = 0, doorbells_out = 0, doorbells_dropped = 0;
};

struct Model {
    Config config;
    std::vector<Tile> tiles;
    u64 first_tile_id = 0;
    size_t queue_depth = 0;
    lm1c_host host = {};
    PendingTrap trap;
    lm1c_result stop = {};  // why the run is ending, once a step says Stop
    Link link;
    // MegaPad's services, when the model is a chiplet module; not state.
    const struct mp64_chiplet_host *link_host = nullptr;

    explicit Model(const Config &config);

    // Every tile back to the state the configuration gives it.
    void init_tiles();
    // The chiplet held in reset: the configured state, every thread halted,
    // nothing lent and no work in hand.
    void reset_chiplet();

    Tile *tile_by_id(i64 id);

    // One step of a thread: fetch, execute, answer any trap, advance.
    Flow step(Tile &tile, Thread &t);
    // The minor collection, as TRAP_NURSERY_OVERFLOW's stand-in runs it.
    Flow collect(Tile &tile);

    // The run loops (run.cpp).  run_tiles gives the threads of tiles
    // [first, last) their quanta in turn.
    lm1c_result run_thread(size_t tile, u64 max_instructions);
    lm1c_result run_tiles(size_t first, size_t last, u64 max_instructions, u64 quanta);
    lm1c_result collect_now(size_t tile);

    // Ends the run with a fault at ADDRESS, outside the tile's memory.
    Flow memory_fault(u128 address);
    Flow host_failure();
};

// The whole state of a model, for the chiplet module's save and restore
// (state.cpp).  restore_state leaves MODEL unchanged when it fails.
std::vector<uint8_t> save_state(const Model &model);
bool restore_state(Model &model, const uint8_t *data, size_t size);

// Memory access as the reference's Memory does it: a word access uses the
// word holding the address, and an address outside memory is a fault.
inline bool word_index(const Tile &tile, u128 address, size_t &index) {
    u128 i = address >> 3;
    if (i >= tile.mem.size()) return false;
    index = size_t(i);
    return true;
}

void card_table_reset(Tile &tile);

// The queue a stalled RECV waits on, counted as the reference's list index
// counts, a negative index from the end; or null for no such queue.
inline const Queue *stall_queue_of(const Tile &tile, const Thread &t) {
    i64 q = t.stall_queue;
    if (q < 0) q += QUEUES;
    return q < 0 || q >= i64(QUEUES) ? nullptr : &tile.queues[q];
}

// Whether a thread that is not halted may run: a stalled one may once the
// queue its RECV waits on holds a value.
inline bool can_run(const Tile &tile, const Thread &t) {
    if (t.halted) return false;
    if (!t.stalled) return true;
    const Queue *q = stall_queue_of(tile, t);
    return q != nullptr && q->size() > 0;
}

// Whether a stalled thread may run again, which clears its stall.
inline bool unstall(Tile &tile, Thread &t) {
    if (!t.stalled) return true;
    const Queue *q = stall_queue_of(tile, t);
    if (q == nullptr || q->size() == 0) return false;
    t.stalled = false;
    t.stall_queue = -1;
    return true;
}

}  // namespace lm1

struct lm1c_model : lm1::Model {
    using lm1::Model::Model;
};

#endif
