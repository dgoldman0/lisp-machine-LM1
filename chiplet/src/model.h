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
#include <cstdio>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "lm1_chiplet.h"
#include "managed.h"

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "the model keeps memory as host words and reads bytes from them in place"
#endif

struct lm1c_model;

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
constexpr u64 REF_METADATA = ~0ull << 51;  // bits 63:51 of a reference
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
constexpr uint32_t TRAP_BAD_LENGTH = 0x11;
constexpr uint32_t TRAP_IC_MISS = 0x20;
constexpr uint32_t TRAP_QUEUE_FULL = 0x30;
constexpr uint32_t TRAP_NO_ROUTE = 0x32;
constexpr uint32_t TRAP_SHARED_STORE = 0x51;
constexpr uint32_t TRAP_SHARED_REF = 0x61;
constexpr uint32_t TRAP_UNIMPLEMENTED = 0x7E;
// Codes below this vector through the trap table; TRAP #code from here up
// is a system trap the core carries out itself.
constexpr uint32_t SYSTEM_TRAPS = 0x80;

// A tile's header-template table: SET_TEMPLATE names an entry with eight
// bits, and an allocation's index past the table reads a zero template.
constexpr unsigned HEADER_TEMPLATES = 256;
// Queue descriptors name a queue with two bits (spec/04-soc.md).
constexpr unsigned QUEUES = 4;
// The block device's block (emulator TRAP 0x82).
constexpr u64 BLOCK_SIZE = 4096;

// LM-1's architectural timing model: the cycles each instruction costs,
// from timing.h, the table the Python emulator reads too (spec/02-isa.md
// section 11.6).
namespace timing {
#define LM1_TIMING(name, cycles) constexpr u64 name = cycles;
#include "timing.h"
#undef LM1_TIMING
}  // namespace timing

// The performance counters SYS_INFO 10 reads, by the number in bits 4:0 of
// its immediate (spec/02-isa.md section 11.6); a number past them reads 0.
constexpr unsigned CTR_ALLOC_COUNT = 0, CTR_ALLOC_BYTES = 1, CTR_BARRIER_FIRES = 2,
                   CTR_BARRIER_FILTERED = 3, CTR_IC_HITS = 4, CTR_IC_MISSES = 5,
                   CTR_GC_CYCLES = 6, CTR_NURSERY_OVERFLOWS = 7, CTR_MEMORY_STALLS = 8;
constexpr unsigned PERF_COUNTERS = 9;

struct Thread {
    u64 regs[REGS] = {};
    u64 pc = 0;
    u64 instret = 0;  // INSTRET: the instructions the thread has retired
    bool halted = false;
    bool stalled = false;
    i64 stall_queue = -1;  // the descriptor value, tile * 4 + queue, a RECV waits on
    u64 trap_table_base = 0;
    u64 trap_cause = 0;
    u64 trap_pc = 0;
    bool in_trap = false;
    u64 tile_id = 0;
    u64 thread_id = 0;
    u64 slice_base = 0;  // where this thread's chunk of the nursery begins
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
    // The chunk a thread takes at a time, and the nursery's first free byte
    // after the chunks threads hold (spec/04-soc.md section 4.3).
    u64 nursery_chunk = 0, nursery_free = 0;
    u64 oldgen_base = 0, oldgen_size = 0, oldgen_ptr = 0;
    u64 card_shift = 6, card_table_base = 0, gen_boundary = 0;
    u64 queue_base = 0;  // SET_QUEUE_BASE records it, SYS_INFO 8 reads it back
    std::vector<uint8_t> cards;  // one byte per card, 0xFF when dirty
    u64 gc_count = 0;
    // The instructions the core has issued, whatever became of them; the
    // cycles the timing model charged them, which CYCLE reads; and the
    // performance counters SYS_INFO 10 reads.
    u64 instruction_count = 0;
    u64 cycles = 0;
    u64 perf[PERF_COUNTERS] = {};
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
    u64 alloc_bytes = 0;   // TRAP_NURSERY_OVERFLOW: the bytes the allocation needs
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
    u64 nursery_chunk = 0x1000;   // at most an equal slice for each thread
    // LM-1's end of the link (link.cpp), with the RTL's parameters as
    // defaults: lm1_link_boot's LOAD_BASE and BURST_WORDS, and lm1_link's
    // RANGES.  lend holds the ranges the runtime's stand-in lends once the
    // image is loaded, as (base, length), until LEND entries exist.
    u64 load_base = 0;
    u64 burst_words = 8;
    u64 lend_ranges = 2;
    u64 quanta = 1;
    std::vector<std::pair<u64, u64>> lend;
    // The stand-in for LM-1's runtime (managed.cpp): whether it serves the
    // control queues MegaPad names at release, and its sizes and bounds.
    // Times are nanoseconds of LM-1's time, of which each step of the
    // chiplet module stands for its budget times unit_ns.
    u64 runtime = 0;
    u64 heap_block = 0x40000;     // the shared heap's first block, asked in the OFFER
    u64 queue_entries = 4;        // entries asked for in each work queue
    u64 region_bytes = 0x1000;    // a core's region by default
    u64 window_limit = 1000000;   // WINDOW_LIMIT, HOLD_LIMIT and HOLD_GAP
    u64 hold_limit = 1000000;
    u64 hold_gap = 100000;
    u64 young_trigger = 0;        // MegaPad's bytes between young collections; 0: half the heap
    // The nanoseconds of LM-1's time a unit of a step's budget stands for.
    // MegaPad's emulator steps a chiplet about every 13 microseconds of its
    // own time with a budget of 64 units, and the hosted simulator about
    // every 20 microseconds of a test's clock (measured 2026-11-22), so 200
    // keeps LM-1's clock and MegaPad's within a small factor of each other.
    u64 unit_ns = 200;
    u64 hook_id = 0;              // names the model to the test hooks
    // The per-function profile (profile.cpp): a sample every profile cycles
    // of the timing model, 0 for none, a record every profile_interval
    // cycles, appended to profile_file.  They change nothing the model
    // computes, so the configuration a saved state carries leaves them out.
    u64 profile = 0;
    u64 profile_interval = 100000000;
    std::string profile_file;
};

// Parses a configuration string; on failure returns false with a reason.
bool parse_config(const char *text, Config &config, std::string &error);
// The configuration as one canonical string, every key in a fixed order,
// which a saved state carries so that it restores only where it fits.
std::string config_text(const Config &config);

// The per-function profile (profile.cpp, docs/profile.md).  Every profile
// cycles of the timing model it samples a core's current thread: the
// function holding its PC and the callers its frame chain names, from the
// symbol table an image carries at its end.  Each core's samples go into a
// record every profile_interval cycles, in the shape of MegaPad's
// guest-words.jsonl.  It reads the model and never writes it, so a run
// computes the same with it as without.
struct Symbol {
    u64 address;
    std::string name;
};

struct CoreProfile {
    u64 next_sample = 0;  // the cycle count at which the next sample falls
    u64 start = 0;        // the cycle count the record in hand began at
    u64 cycles = 0, samples = 0;
    std::map<std::string, u64> inclusive, leaf, chains;
};

class Profiler {
public:
    Profiler() = default;
    Profiler(const Profiler &) = delete;
    Profiler &operator=(const Profiler &) = delete;
    ~Profiler();

    // The symbol table at the end of SIZE bytes of an image: false, and no
    // table, when the bytes end in none.
    bool load_symbols(const uint8_t *data, size_t size);
    std::vector<Symbol> symbols;  // by address
    std::vector<CoreProfile> cores;
    std::string path;
    u64 interval = 0;
    std::FILE *out = nullptr;
};

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
    // The stand-in for LM-1's runtime at the link (managed.h).
    Standin standin;
    // MegaPad's services, when the model is a chiplet module; not state.
    const struct mp64_chiplet_host *link_host = nullptr;
    // Whether a step of the chiplet module runs, the one time LM-1 may
    // reach MegaPad's memory; not state.
    bool in_step = false;
    // Data reads so far, which the timing model's memory latency follows,
    // and the cards the current instruction marked; not state.
    u64 data_reads = 0;
    u64 marks = 0;
    // The per-function profile; not state.
    Profiler profiler;

    explicit Model(const Config &config);
    // Writes the profile's records in hand.
    ~Model();

    // Every tile back to the state the configuration gives it.
    void init_tiles();
    // The chiplet held in reset: the configured state, every thread halted,
    // nothing lent and no work in hand.
    void reset_chiplet();

    Tile *tile_by_id(i64 id);
    const Tile *tile_by_id(i64 id) const;

    // One step of a thread: fetch, execute, answer any trap, advance.
    Flow step(Tile &tile, Thread &t);
    // The minor collection, as TRAP_NURSERY_OVERFLOW's stand-in runs it.
    Flow collect(Tile &tile);
    // Every thread's chunk from the nursery's base on, the rest free.
    void fresh_chunks(Tile &tile);

    // The run loops (run.cpp).  run_tiles gives the threads of tiles
    // [first, last) their quanta in turn.
    lm1c_result run_thread(size_t tile, u64 max_instructions);
    lm1c_result run_tiles(size_t first, size_t last, u64 max_instructions, u64 quanta);
    lm1c_result collect_now(size_t tile);

    // Ends the run with a fault at ADDRESS, outside the tile's memory.
    Flow memory_fault(u128 address);
    Flow host_failure();

    // The per-function profile (profile.cpp).  profile_sample takes the
    // samples due once an instruction of thread T has charged TILE's
    // cycles; profile_restart writes the records in hand and starts each
    // core's sampling over from its cycle count, as after a reset or a
    // restore; profile_flush writes the records in hand.
    void profile_sample(Tile &tile, const Thread &t);
    void profile_restart();
    void profile_flush();
};

// Opens the profile's file when the configuration asks for a profile, and
// starts each core's sampling; on failure returns false with a reason.
bool profile_open(Model &m, std::string &error);

// The models mp64_chiplet_create made, which lm1c_chiplet_find searches.
void register_module_model(lm1c_model *model);
void unregister_module_model(lm1c_model *model);

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

// The queue a stalled RECV waits on, which its descriptor's value names, on
// the thread's own tile or another of the model's; null for none.
inline const Queue *stall_queue_of(const Model &m, const Thread &t) {
    if (t.stall_queue < 0) return nullptr;
    const Tile *tile = m.tile_by_id(t.stall_queue >> 2);
    return tile == nullptr ? nullptr : &tile->queues[t.stall_queue & 3];
}

// Whether a thread that is not halted may run: a stalled one may once the
// queue its RECV waits on holds a value.
inline bool can_run(const Model &m, const Thread &t) {
    if (t.halted) return false;
    if (!t.stalled) return true;
    const Queue *q = stall_queue_of(m, t);
    return q != nullptr && q->size() > 0;
}

// Whether a stalled thread may run again, which clears its stall.
inline bool unstall(const Model &m, Thread &t) {
    if (!t.stalled) return true;
    const Queue *q = stall_queue_of(m, t);
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
