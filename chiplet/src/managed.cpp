// LM-1 chiplet module: the stand-in for LM-1's runtime at the link, and the
// managed-data function it serves.
//
// The stand-in comes up once the image is in.  It reads the control block
// MegaPad names in the boot argument (MegaPad's docs/chiplet-link.md section
// 8), offers class 1, managed data, asking for the shared heap's first block,
// and serves the function on the grant's queue pair and on each MegaPad
// core's own pair (MegaPad's docs/managed-data.md sections 9 and 10; LM-1's
// spec/05-memory-gc.md section 12 and spec/06-runtime.md section 12).
//
// Every access to MegaPad's memory is one request through the link's
// host-memory port, made only from inside a step, each answered before the
// next is made (spec/05-memory-gc.md section 6.4).  A doorbell means "look at
// your queues": the stand-in reads its queues when it is rung, and rings the
// core a pair names after it puts an entry there.
//
// The stand-in's work on the link goes in chunks that finish inside one
// step: an entry taken and answered, a log drained, and a hold's whole
// ACTIVE work, from draining the logs to the last rewrite.  A stop request,
// which MegaPad makes between steps, therefore never finds a hold in the
// middle of its moves, and LM-1 reports STOPPED with MOVE reading NONE or
// FIXED (spec/05-memory-gc.md section 12.9).  A test hook makes the request
// arrive as a hold begins flipping; the stand-in then finishes the hold's
// moves and rewrites, writes FIXED, and writes nothing more, leaving the
// hold ACTIVE, which is how LM-1 answers a stop only between moves.  Each
// step stands for the budget
// it was given, budget times unit_ns nanoseconds of LM-1's time, by which the
// stand-in keeps HOLD_GAP, WINDOW_LIMIT and the test hook's timed holds.
//
// The collector keeps the order of writes recovery reads from the records
// (stage 0 of managed-data.md section 11): a moving hold writes its move
// set, then MOVE_SET, then MOVE reading FLIPPING, before its first
// forwarding pointer; it copies each object whole before writing the
// forwarding pointer over its old header, in a request of its own; it
// rewrites a reference only after the pointer to its new place exists; it
// copies only into space that was free when the hold began; and it writes
// MOVE reading FIXED after its last rewrite, the handle slots' included,
// before it ends the hold.  Its marks, its index of objects and its
// remembered set are LM-1's own memory, never a header's GC bits.  The
// moving work follows the movement engines' four rules (decision G of
// design/chiplet-module.md): marks outside headers, layout read from each
// header so that raw words are never read as references, references written
// with zero in bits 63:51, and each step's work complete only once its last
// write has been answered, which a request through the host's callback is
// when it returns.
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <unordered_set>

#include "chiplet_module.h"
#include "managed.h"
#include "model.h"

namespace lm1 {

namespace {

// ---- The formats (MegaPad's docs/chiplet-link.md section 8) ----

constexpr u64 ascii8(const char (&s)[9]) {
    u64 v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | u64(uint8_t(s[i]));
    return v;
}
constexpr u64 MAGIC_CHIPCTRL = ascii8("CHIPCTRL");
constexpr u64 MAGIC_MANAGED = ascii8("MANAGED\0");

constexpr u64 REPLY = u64(1) << 16;
// Operations of every pair and of the control queues.
constexpr u64 OP_CANCEL = 1, OP_OFFER = 2;
// Statuses of every class, and of managed data.
constexpr u64 ST_OK = 0, ST_UNSUPPORTED = 1, ST_CANCELLED = 2, ST_FAILED = 3, ST_REFUSED = 4;
constexpr u64 ST_NO_MEMORY = 0x100, ST_BAD_RECORD = 0x101, ST_NO_RESUME = 0x102;
// Managed data, class 1 (managed-data.md section 10).
constexpr u64 CLASS_MANAGED = 1, MANAGED_VERSION = 1;
constexpr u64 OP_ATTACH = 0x100, OP_REGION = 0x101, OP_SHAPE = 0x102, OP_COLLECT = 0x103;
constexpr u64 OP_GROW = 0x180, OP_RETURN = 0x181, OP_NOTICE = 0x182;
constexpr u64 NOTICE_WINDOW = 1, NOTICE_CORRUPT = 2, NOTICE_LOG = 3;

// The records (managed-data.md section 9), by offset.
constexpr u64 MC_MAGIC = 0x00, MC_VERSION = 0x08, MC_CORES = 0x10, MC_CORE_TABLE = 0x18,
              MC_LM1_BLOCK = 0x20, MC_HANDLE_DIR = 0x28, MC_STATE = 0x30, MC_BLOCKS = 0x40,
              MC_SHAPES = 0x48, MC_SIZE = 0x80;
constexpr u64 CR_WINDOW = 0x00, CR_REGION = 0x08, CR_FILL = 0x10, CR_LOG_RING = 0x18,
              CR_LOG_CAPACITY = 0x20, CR_TAIL = 0x28, CR_PAIR = 0x40, CR_SIZE = 0x80;
constexpr u64 LB_HOLD = 0x00, LB_WINDOW_LIMIT = 0x08, LB_SHAPE_CONS = 0x20, LB_MOVE = 0x40,
              LB_MOVE_SET = 0x48, LB_SHAPE_RANGE = 0x50, LB_CORES = 0x80, LB_ENTRY = 0x40;
constexpr u64 CE_LOG_HEAD = 0x00, CE_NEXT_BASE = 0x10, CE_NEXT_LIMIT = 0x18, CE_NEXT_SERIAL = 0x20;
constexpr u64 HD_FIRST = 0x00, HD_SEGMENTS = 0x08, HD_BASES = 0x10, HD_MAX_SEGMENTS = 64;
constexpr u64 WINDOW_CLOSED = 0, WINDOW_OPEN = 1, WINDOW_WAITING = 2;
constexpr u64 HOLD_HELD = 1, HOLD_ACTIVE = 2, HOLD_FAILED = 4;
constexpr u64 MOVE_FLIPPING = 1, MOVE_FIXED = 2;
// The shape description of a SHAPE request (managed-data.md section 8).
constexpr u64 SD_TAGGED = 0x00, SD_RAW = 0x08, SD_NAME_LEN = 0x10, SD_KINDS = 0x18;
constexpr u64 KIND_RECORD = 9, KIND_RAW_FIRST = 16, KIND_RAW_LAST = 18, KIND_TAGGED_LAST = 9;

// Object headers (spec/01-object-model.md section 3.5).
constexpr u64 SUB_CONS = 0b00001, SUB_VECTOR = 0b00010, SUB_BYTES = 0b00011, SUB_RECORD = 0b10000;
constexpr u64 SUB_BIGNUM = 0b00110, SUB_DOUBLE = 0b01000, SUB_FOREIGN = 0b01110;
constexpr u64 GC_FORWARD = 0xFF;
constexpr u64 METADATA = ~0ull << 51;   // bits 63:51 of a reference

// The shape IDs LM-1's image fixes (spec/05-memory-gc.md section 12.6): the
// built-in kinds, and the range set aside for the kinds MegaPad registers,
// whose upper part belongs to MegaPad's stand-in.
constexpr u64 SHAPE_CONS = 0x10, SHAPE_VECTOR = 0x11, SHAPE_BYTES = 0x12, SHAPE_STRING = 0x13;
constexpr u64 SHAPE_REGISTRY_FIRST = 0x100, SHAPE_STANDIN_FIRST = 0x80000000ull;

// A burst carries at most 256 words.
constexpr u64 MAX_BURST = 256;

bool aligned8(u64 a) { return a != 0 && a % 8 == 0; }
bool power_of_two(u64 v) { return v != 0 && (v & (v - 1)) == 0; }
u64 forwarding_pointer(u64 to) { return (GC_FORWARD << 56) | (to & 0x00FFFFFFFFFFFFF8ull) | 0b111; }
bool is_forwarding(u64 w) { return is_header(w) && (w >> 56) == GC_FORWARD; }
u64 header_gc(u64 h) { return h >> 56; }

// Why the stand-in stopped serving, kept for tests (lm1c_md_get).
enum MdWhy : u64 {
    WHY_NONE = 0, WHY_CONTROL = 1, WHY_LINK = 2, WHY_QUEUE = 3, WHY_RECORD = 4,
    WHY_WINDOW = 5, WHY_LOG = 6, WHY_OBJECT = 7, WHY_REFERENCE = 8, WHY_SLOT = 9
};

class Service {
public:
    Service(Model &m, u64 budget) : m_(m), s_(m.standin), budget_(budget) {}
    u64 left() const { return budget_ > used_ ? budget_ - used_ : 0; }
    void run();

private:
    // ---- The host-memory port ----
    bool read(u64 addr, u64 &v) { return request(addr, 0, 0, &v); }
    bool write(u64 addr, u64 v) { return request(addr, 0, 1, &v); }
    bool request(u64 addr, unsigned len, int wen, u64 *data) {
        used_ += 1;
        unsigned done = 0;
        const int code = m_.link_host->mem(m_.link_host->ctx, addr, 3, len, wen, data, &done);
        if (wen && s_.tracing) {
            const unsigned landed = code == MP64_CHIPLET_OK ? len + 1 : std::min(done, len + 1);
            for (unsigned i = 0; i < landed; ++i) s_.trace.emplace_back(addr + 8 * i, data[i]);
        }
        return code == MP64_CHIPLET_OK;
    }
    bool read_words(u64 addr, u64 *out, u64 n) {
        while (n > 0) {
            const u64 k = std::min(n, MAX_BURST);
            if (!request(addr, unsigned(k - 1), 0, out)) return false;
            addr += 8 * k;
            out += k;
            n -= k;
        }
        return true;
    }
    bool write_words(u64 addr, const u64 *in, u64 n) {
        std::vector<u64> data(in, in + n);
        u64 *p = data.data();
        while (n > 0) {
            const u64 k = std::min(n, MAX_BURST);
            if (!request(addr, unsigned(k - 1), 1, p)) return false;
            addr += 8 * k;
            p += k;
            n -= k;
        }
        return true;
    }
    void ring(u64 core) {
        m_.link_host->doorbell(m_.link_host->ctx, uint8_t(core));
        m_.link.doorbells_out += 1;
    }

    // ---- Queues ----
    bool read_pair(u64 addr, LinkPair &p);
    // Takes the entries waiting on a queue LM-1 reads; false when it broke.
    bool take(LinkQueue &q, std::vector<QEntry> &out);
    // Puts an entry on a queue LM-1 writes, or keeps it in the outbox when
    // the queue is full.
    void put(u64 pair_index, const QEntry &e);
    bool try_put(u64 pair_index, const QEntry &e, bool &placed);
    void flush_outbox();
    LinkPair *pair_of(u64 index) {
        if (index == 0) return &s_.control;
        if (index == 1) return &s_.grant;
        return index - 2 < s_.core.size() ? &s_.core[size_t(index - 2)].pair : nullptr;
    }
    void reply(u64 pair_index, const QEntry &request, u64 status, u64 arg0 = 0, u64 arg1 = 0) {
        QEntry e;
        e.w[0] = (request.w[0] & 0xFFFF) | REPLY | (status << 32);
        e.w[1] = request.w[1];
        e.w[6] = arg0;
        e.w[7] = arg1;
        put(pair_index, e);
    }
    void ask(u64 op, u64 in_addr, u64 in_len, u64 arg0, u64 arg1) {
        QEntry e;
        e.w[0] = op;
        e.w[1] = s_.next_tag++;
        e.w[2] = in_addr;
        e.w[3] = in_len;
        e.w[6] = arg0;
        e.w[7] = arg1;
        s_.asked[e.w[1]] = op;
        put(1, e);
    }

    // ---- The function ----
    void start();
    void serve_control();
    void serve_grant();
    void serve_core(u64 c);
    void attach(const QEntry &e);
    bool check_records(u64 control);
    void region(u64 c, const QEntry &e);
    bool grant_region(u64 c, u64 bytes, u64 &base, u64 &limit);
    void shape(u64 c, const QEntry &e);
    void collect_request(u64 c, const QEntry &e);
    void cancel(u64 c, const QEntry &e);
    void grown(const QEntry &e);
    bool drain_log(u64 c);
    void replay(u64 word);
    void ask_grow(u64 at_least);

    // ---- Holds and the collector ----
    void holds();
    void begin_hold();
    bool read_window(u64 c, u64 &w);
    void active_work();
    void end_hold();
    bool retire_regions(std::vector<Extent> &released);
    bool walk_runs();
    bool check_object(u64 addr, const std::vector<u64> &words, u64 end);
    bool collect(u64 mode, std::vector<Extent> &released);
    bool read_object(u64 addr, std::vector<u64> &words);
    void tagged_fields(const std::vector<u64> &words, std::vector<u64> &fields) const;
    bool object_at(u64 ref_addr, bool &is_old) const;
    bool check_ref(u64 v);
    bool take_space(u64 bytes, u64 &at, std::vector<Extent> &taken);
    void give_back(u64 base, u64 limit);
    void hook_writes();
    void notice(u64 kind, u64 arg1, u64 addr);
    void fail(u64 kind, u64 why, u64 addr);

    u64 epoch() const { return s_.hold_word >> 3; }
    u64 heap_bytes() const {
        u64 n = 0;
        for (const Extent &b : s_.blocks) n += b.limit - b.base;
        return n;
    }
    u64 free_bytes() const {
        u64 n = 0;
        for (const auto &f : s_.free) n += f.second - f.first;
        return n;
    }
    u64 rec(u64 c) const { return s_.core_table + CR_SIZE * c; }
    u64 entry(u64 c) const { return s_.lm1_block + LB_CORES + LB_ENTRY * c; }
    bool serving() const { return s_.phase == MdPhase::Serving; }

    Model &m_;
    Standin &s_;
    u64 budget_, used_ = 0;
    // The participants that rang since the last step.
    std::vector<bool> rung_ = std::vector<bool>(256, false);
    bool any_rung_ = false;
};

// ---------------------------------------------------------------------------
//  Queues
// ---------------------------------------------------------------------------

bool Service::read_pair(u64 addr, LinkPair &p) {
    u64 w[6];
    if (!read_words(addr, w, 6)) return false;
    const u64 in_cap = w[4] & 0xFFFFFFFFull, out_cap = w[4] >> 32;
    if (!aligned8(w[0]) || !aligned8(w[1]) || !aligned8(w[2]) || !aligned8(w[3]) ||
        !power_of_two(in_cap) || !power_of_two(out_cap))
        return false;
    p = LinkPair{};
    p.valid = true;
    p.in.ring = w[0];
    p.in.head_word = w[1];
    p.in.capacity = in_cap;
    p.out.ring = w[2];
    p.out.head_word = w[3];
    p.out.capacity = out_cap;
    p.notify = w[5] & 0xFF;
    return true;
}

bool Service::take(LinkQueue &q, std::vector<QEntry> &out) {
    u64 tail;
    if (!read(q.ring, tail)) return false;
    // A TAIL that moved backwards, or runs more than CAPACITY ahead of
    // HEAD, marks the queue broken.
    if (tail < q.other || tail - q.count > q.capacity) return false;
    q.other = tail;
    if (tail == q.count) return true;
    while (q.count < tail) {
        QEntry e;
        if (!read_words(q.ring + 64 + 64 * (q.count % q.capacity), e.w, 8)) return false;
        out.push_back(e);
        q.count += 1;
    }
    return write(q.head_word, q.count);
}

bool Service::try_put(u64 pair_index, const QEntry &e, bool &placed) {
    placed = false;
    LinkPair *p = pair_of(pair_index);
    if (p == nullptr || !p->valid) return true;  // nothing to put it on: dropped
    LinkQueue &q = p->out;
    if (q.count - q.other >= q.capacity) {
        u64 head;
        if (!read(q.head_word, head)) return false;
        if (head < q.other || head > q.count) return false;  // MegaPad's HEAD broke
        q.other = head;
        if (q.count - q.other >= q.capacity) return true;    // still full
    }
    if (!write_words(q.ring + 64 + 64 * (q.count % q.capacity), e.w, 8)) return false;
    q.count += 1;
    if (!write(q.ring, q.count)) return false;
    ring(p->notify);
    placed = true;
    return true;
}

void Service::put(u64 pair_index, const QEntry &e) {
    // Entries go in order: one waiting already keeps its place ahead.
    for (const Outgoing &o : s_.outbox) {
        if (o.pair == pair_index) {
            s_.outbox.push_back(Outgoing{pair_index, e});
            return;
        }
    }
    bool placed;
    if (!try_put(pair_index, e, placed)) {
        fail(NOTICE_CORRUPT, WHY_QUEUE, pair_of(pair_index) ? pair_of(pair_index)->out.head_word : 0);
        return;
    }
    if (!placed) s_.outbox.push_back(Outgoing{pair_index, e});
}

void Service::flush_outbox() {
    std::vector<Outgoing> waiting;
    waiting.swap(s_.outbox);
    std::vector<bool> blocked(2 + s_.core.size(), false);
    for (const Outgoing &o : waiting) {
        if (o.pair < blocked.size() && blocked[size_t(o.pair)]) {
            s_.outbox.push_back(o);
            continue;
        }
        bool placed;
        if (!try_put(o.pair, o.entry, placed)) {
            fail(NOTICE_CORRUPT, WHY_QUEUE, 0);
            return;
        }
        if (!placed) {
            s_.outbox.push_back(o);
            if (o.pair < blocked.size()) blocked[size_t(o.pair)] = true;
        }
    }
}

// ---------------------------------------------------------------------------
//  Starting: the control block, the offer and the grant
// ---------------------------------------------------------------------------

void Service::start() {
    // The control block MegaPad names at release: CHIPCTRL, version 1, the
    // control queues' pair.
    const u64 at = m_.link.boot_arg;
    u64 w[2];
    s_.control_block = at;
    if (!aligned8(at) || !read_words(at, w, 2) || w[0] != MAGIC_CHIPCTRL ||
        (w[1] & 0xFFFFFFFFull) != 1 || !read_pair(at + 0x10, s_.control)) {
        s_.phase = MdPhase::Broken;
        s_.why = WHY_CONTROL;
        s_.why_addr = at;
        return;
    }
    // Offer managed data: class 1 version 1, the entries asked for in each
    // work queue, and the shared heap's first block as working memory.
    QEntry e;
    e.w[0] = OP_OFFER;
    e.w[1] = s_.offer_tag = s_.next_tag++;
    e.w[5] = m_.config.heap_block;
    e.w[6] = CLASS_MANAGED | (MANAGED_VERSION << 32);
    e.w[7] = m_.config.queue_entries;
    s_.phase = MdPhase::Offered;
    put(0, e);
}

void Service::serve_control() {
    std::vector<QEntry> entries;
    if (!s_.control.valid) return;
    if (!take(s_.control.in, entries)) {
        s_.phase = MdPhase::Broken;
        s_.why = WHY_QUEUE;
        s_.why_addr = s_.control.in.ring;
        return;
    }
    for (const QEntry &e : entries) {
        const u64 op = e.w[0] & 0xFFFF;
        if (e.w[0] & REPLY) {
            if (e.w[1] != s_.offer_tag || s_.phase != MdPhase::Offered) continue;
            if ((e.w[0] >> 32) != ST_OK) {  // MegaPad refused the function
                s_.phase = MdPhase::Off;
                continue;
            }
            // The grant record: the function's first pair, then its memory.
            u64 g[8];
            if (!read_words(e.w[2], g, 8) || !read_pair(e.w[2], s_.grant) || !aligned8(g[6]) ||
                g[7] < 64 || g[7] % 8 != 0) {
                s_.phase = MdPhase::Broken;
                s_.why = WHY_RECORD;
                s_.why_addr = e.w[2];
                continue;
            }
            s_.blocks.assign(1, Extent{g[6], g[6] + g[7]});
            s_.free.clear();
            s_.free[g[6]] = g[6] + g[7];
            s_.phase = MdPhase::Granted;
        } else if (op != OP_CANCEL) {
            reply(0, e, ST_UNSUPPORTED);
        }
    }
}

// ---------------------------------------------------------------------------
//  The grant's pair: ATTACH, and the replies to LM-1's own requests
// ---------------------------------------------------------------------------

void Service::serve_grant() {
    std::vector<QEntry> entries;
    if (!s_.grant.valid) return;
    if (!take(s_.grant.in, entries)) {
        fail(NOTICE_CORRUPT, WHY_QUEUE, s_.grant.in.ring);
        return;
    }
    for (const QEntry &e : entries) {
        const u64 op = e.w[0] & 0xFFFF;
        if (e.w[0] & REPLY) {
            auto asked = s_.asked.find(e.w[1]);
            if (asked == s_.asked.end()) continue;
            const u64 what = asked->second;
            s_.asked.erase(asked);
            if (what == OP_GROW) grown(e);
            continue;
        }
        if (op == OP_ATTACH) attach(e);
        else if (op != OP_CANCEL) reply(1, e, ST_UNSUPPORTED);
    }
}

// The records MegaPad names at ATTACH, checked as managed-data.md section 9
// lists: a sound control block, core records whose window words hold one of
// their three values, a handle directory LM-1 can read, and stage 0's block
// list, naming the heap's first block, and shape registry, empty for a
// function started afresh.
bool Service::check_records(u64 control) {
    u64 c[16];
    if (!aligned8(control) || !read_words(control, c, 16)) return false;
    if (c[MC_MAGIC / 8] != MAGIC_MANAGED || c[MC_VERSION / 8] != 1) return false;
    const u64 cores = c[MC_CORES / 8];
    if (cores < 1 || cores > 256) return false;
    if (!aligned8(c[MC_CORE_TABLE / 8]) || !aligned8(c[MC_LM1_BLOCK / 8]) ||
        !aligned8(c[MC_HANDLE_DIR / 8]) || !aligned8(c[MC_BLOCKS / 8]) || !aligned8(c[MC_SHAPES / 8]))
        return false;
    if (c[MC_STATE / 8] != 0) return false;
    for (u64 i = 0; i < cores; ++i) {
        u64 w;
        if (!read(c[MC_CORE_TABLE / 8] + CR_SIZE * i + CR_WINDOW, w) || w > WINDOW_WAITING) return false;
    }
    u64 dir[2];
    if (!read_words(c[MC_HANDLE_DIR / 8], dir, 2) || !power_of_two(dir[0]) || dir[0] > (u64(1) << 32) ||
        dir[1] > HD_MAX_SEGMENTS)
        return false;
    u64 list[2];
    if (!read_words(c[MC_BLOCKS / 8], list, 2) || list[0] < 1 || list[1] < list[0]) return false;
    bool first = false;
    for (u64 i = 0; i < list[0]; ++i) {
        u64 b[2];
        if (!read_words(c[MC_BLOCKS / 8] + 0x10 + 16 * i, b, 2)) return false;
        if (b[0] == s_.blocks[0].base && b[1] == s_.blocks[0].limit - s_.blocks[0].base) first = true;
    }
    u64 reg[2];
    if (!first || !read_words(c[MC_SHAPES / 8], reg, 2) || reg[0] != 0) return false;
    s_.records = control;
    s_.cores = cores;
    s_.core_table = c[MC_CORE_TABLE / 8];
    s_.lm1_block = c[MC_LM1_BLOCK / 8];
    s_.handle_dir = c[MC_HANDLE_DIR / 8];
    return true;
}

void Service::attach(const QEntry &e) {
    if (e.w[6] != 0) {  // taking a heap back is stage 2 of recovery
        reply(1, e, ST_NO_RESUME);
        return;
    }
    if (s_.phase != MdPhase::Granted || e.w[3] < MC_SIZE || !check_records(e.w[2])) {
        reply(1, e, ST_BAD_RECORD);
        return;
    }
    s_.core.assign(size_t(s_.cores), MdCore{});
    for (MdCore &c : s_.core) c.region_size = m_.config.region_bytes;
    s_.next_shape = SHAPE_REGISTRY_FIRST;
    // LM-1's block: the hold word clear, the three bounds, the built-in
    // shape IDs, MOVE reading NONE, the registry's range, and every core's
    // entry zero.
    std::vector<u64> block(size_t((LB_CORES + LB_ENTRY * s_.cores) / 8), 0);
    block[1] = m_.config.window_limit;
    block[2] = m_.config.hold_limit;
    block[3] = m_.config.hold_gap;
    block[LB_SHAPE_CONS / 8 + 0] = SHAPE_CONS;
    block[LB_SHAPE_CONS / 8 + 1] = SHAPE_VECTOR;
    block[LB_SHAPE_CONS / 8 + 2] = SHAPE_BYTES;
    block[LB_SHAPE_CONS / 8 + 3] = SHAPE_STRING;
    block[LB_SHAPE_RANGE / 8] = SHAPE_REGISTRY_FIRST | (SHAPE_STANDIN_FIRST << 32);
    if (!write_words(s_.lm1_block, block.data(), block.size())) {
        reply(1, e, ST_BAD_RECORD);
        return;
    }
    s_.hold_word = 0;
    s_.move_word = 0;
    s_.phase = MdPhase::Serving;
    reply(1, e, ST_OK);
}

// ---------------------------------------------------------------------------
//  A core's pair: REGION, SHAPE and COLLECT
// ---------------------------------------------------------------------------

void Service::serve_core(u64 c) {
    MdCore &core = s_.core[size_t(c)];
    if (!core.pair_known) {
        // The module makes a core's pair before its first request and
        // records it in the core record, where LM-1 reads it at the core's
        // first ring.
        u64 first;
        if (!read(rec(c) + CR_PAIR, first)) {
            fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_PAIR);
            return;
        }
        if (first == 0) return;
        if (!read_pair(rec(c) + CR_PAIR, core.pair)) {
            fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_PAIR);
            return;
        }
        core.pair_known = true;
    }
    std::vector<QEntry> entries;
    if (!take(core.pair.in, entries)) {
        fail(NOTICE_CORRUPT, WHY_QUEUE, core.pair.in.ring);
        return;
    }
    for (const QEntry &e : entries) {
        if (e.w[0] & REPLY) continue;  // LM-1 asks a core nothing
        const u64 op = e.w[0] & 0xFFFF;
        if (!serving()) {
            if (op != OP_CANCEL) reply(2 + c, e, ST_FAILED);
            continue;
        }
        switch (op) {
        case OP_REGION: region(c, e); break;
        case OP_SHAPE: shape(c, e); break;
        case OP_COLLECT: collect_request(c, e); break;
        case OP_CANCEL: cancel(c, e); break;
        default: reply(2 + c, e, ST_UNSUPPORTED); break;
        }
        if (!serving()) return;
    }
}

// Space for a region of BYTES: the first free extent that holds it.  A
// core's record may still name a region a hold retired, until the core
// next opens a window, so no region the core is granted starts at the base
// its record names: the base alone tells LM-1 which grant the record means.
bool Service::grant_region(u64 c, u64 bytes, u64 &base, u64 &limit) {
    const u64 avoid = s_.core[size_t(c)].recorded_region;
    for (auto it = s_.free.begin(); it != s_.free.end(); ++it) {
        const u64 first = it->first, end = it->second;
        const u64 start = first == avoid ? first + 64 : first;
        if (start >= end || end - start < bytes) continue;
        base = start;
        limit = base + bytes;
        s_.free.erase(it);
        if (first < base) s_.free[first] = base;
        if (limit < end) s_.free[limit] = end;
        MdCore &core = s_.core[size_t(c)];
        core.grants.push_back(Grant{base, limit, false, 0});
        core.next_serial += 1;
        // NEXT_BASE and NEXT_LIMIT, then NEXT_SERIAL raised, so a core
        // that reads them after the serial never takes half of each.
        const u64 next[2] = {base, limit};
        if (!write_words(entry(c) + CE_NEXT_BASE, next, 2) ||
            !write(entry(c) + CE_NEXT_SERIAL, core.next_serial))
            return false;
        s_.young_since += bytes;
        return true;
    }
    base = limit = 0;
    return true;
}

void Service::region(u64 c, const QEntry &e) {
    MdCore &core = s_.core[size_t(c)];
    if (core.region_parked) {  // one request at a time
        reply(2 + c, e, ST_REFUSED);
        return;
    }
    // The core stored FILL before it asked, so the region it leaves holds
    // whole objects from its base to FILL, which the next hold walks.
    u64 rf[2];
    if (!read_words(rec(c) + CR_REGION, rf, 2)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_REGION);
        return;
    }
    core.recorded_region = rf[0];
    for (Grant &g : core.grants) {
        if (g.base != rf[0] || rf[1] < g.base || rf[1] > g.limit) continue;
        g.left = true;
        g.fill = std::max(g.fill, rf[1]);
    }
    core.makes_objects = true;
    // Regions grow for a core that fills them quickly: within one epoch,
    // each request doubles the core's next default size.
    if (core.asked_epoch == epoch()) core.region_size *= 2;
    core.asked_epoch = epoch();
    const u64 asked = (std::max<u64>(e.w[6], 8) + 63) & ~63ull;
    const u64 bytes = std::max(asked, core.region_size);
    u64 base, limit;
    if (!grant_region(c, bytes, base, limit)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, entry(c));
        return;
    }
    if (base == 0 && bytes > asked && !grant_region(c, asked, base, limit)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, entry(c));
        return;
    }
    if (base == 0) {
        // No free extent holds it: the request waits for a block, which
        // doubles the heap at least.
        core.region_parked = true;
        core.parked_tag = e.w[1];
        core.parked_bytes = asked;
        ask_grow(asked);
        return;
    }
    reply(2 + c, e, ST_OK);
    // LM-1 keeps free space in reserve, a quarter of the heap, and asks for
    // a block before it runs out.
    if (free_bytes() < heap_bytes() / 4) ask_grow(0);
}

void Service::ask_grow(u64 at_least) {
    if (s_.grow_asked || s_.asked.size() >= s_.grant.in.capacity) return;
    s_.grow_asked = true;
    ask(OP_GROW, 0, 0, std::max(heap_bytes(), at_least), 0);
}

void Service::grown(const QEntry &e) {
    s_.grow_asked = false;
    const bool ok = (e.w[0] >> 32) == ST_OK && aligned8(e.w[6]) && e.w[7] >= 64 && e.w[7] % 8 == 0;
    if (ok) {
        s_.grows += 1;
        s_.blocks.push_back(Extent{e.w[6], e.w[6] + e.w[7]});
        give_back(e.w[6], e.w[6] + e.w[7]);
    }
    for (u64 c = 0; c < s_.core.size(); ++c) {
        MdCore &core = s_.core[size_t(c)];
        if (!core.region_parked) continue;
        QEntry request;
        request.w[0] = OP_REGION;
        request.w[1] = core.parked_tag;
        u64 base, limit;
        if (!grant_region(c, core.parked_bytes, base, limit)) {
            fail(NOTICE_CORRUPT, WHY_RECORD, entry(c));
            return;
        }
        if (base == 0 && ok) continue;  // still short: it waits for the next block
        core.region_parked = false;
        reply(2 + c, request, base != 0 ? ST_OK : ST_NO_MEMORY);
        if (base == 0) continue;
    }
    for (const MdCore &core : s_.core) {
        if (core.region_parked && ok) {
            ask_grow(core.parked_bytes);
            break;
        }
    }
}

void Service::shape(u64 c, const QEntry &e) {
    const u64 addr = e.w[2], len = e.w[3];
    u64 head[3];
    if (len < SD_KINDS || !aligned8(addr) || !read_words(addr, head, 3)) {
        reply(2 + c, e, ST_BAD_RECORD);
        return;
    }
    MdShape s;
    s.tagged = head[0];
    s.raw = head[1];
    const u64 name_len = head[2];
    // A record's header counts its count word, its tagged fields and its
    // raw words in 16 bits.
    if (s.tagged > 0xFFFF || s.raw > 0xFFFF || 1 + s.tagged + s.raw > 0xFFFF || name_len > len ||
        SD_KINDS + 8 * (s.tagged + s.raw) > len - name_len) {
        reply(2 + c, e, ST_BAD_RECORD);
        return;
    }
    s.kinds.resize(size_t(s.tagged + s.raw));
    if (!read_words(addr + SD_KINDS, s.kinds.data(), s.kinds.size())) {
        reply(2 + c, e, ST_BAD_RECORD);
        return;
    }
    for (u64 i = 0; i < s.kinds.size(); ++i) {
        const u64 k = s.kinds[size_t(i)];
        bool sound;
        if (i < s.tagged) {
            const u64 kind = k & 0xFFFFFFFFull, shape_id = k >> 32;
            sound = kind < KIND_RECORD ? shape_id == 0 : kind == KIND_RECORD;
            if (kind == KIND_RECORD) {
                sound = false;
                for (const MdShape &t : s_.shapes) sound = sound || t.id == shape_id;
            }
        } else {
            sound = k >= KIND_RAW_FIRST && k <= KIND_RAW_LAST;
        }
        if (!sound) {
            reply(2 + c, e, ST_BAD_RECORD);
            return;
        }
    }
    // The name, UTF-8.
    const u64 name_at = addr + SD_KINDS + 8 * s.kinds.size();
    std::vector<u64> words(size_t((name_len + 7) / 8 + 1), 0);
    if (name_len > 0 && !read_words(name_at, words.data(), (name_len + 7) / 8)) {
        reply(2 + c, e, ST_BAD_RECORD);
        return;
    }
    s.name.assign(reinterpret_cast<const char *>(words.data()), size_t(name_len));
    for (size_t i = 0; i < s.name.size();) {
        const unsigned char b = s.name[i];
        const size_t n = b < 0x80 ? 1 : (b >> 5) == 6 ? 2 : (b >> 4) == 14 ? 3 : (b >> 3) == 30 ? 4 : 0;
        bool ok = n != 0 && i + n <= s.name.size();
        for (size_t j = 1; ok && j < n; ++j) ok = (static_cast<unsigned char>(s.name[i + j]) >> 6) == 2;
        if (!ok) {
            reply(2 + c, e, ST_BAD_RECORD);
            return;
        }
        i += n;
    }
    // The same description always gets the same ID.
    for (const MdShape &t : s_.shapes) {
        if (t.tagged == s.tagged && t.raw == s.raw && t.kinds == s.kinds && t.name == s.name) {
            reply(2 + c, e, ST_OK, t.id);
            return;
        }
    }
    if (s_.next_shape >= SHAPE_STANDIN_FIRST) {
        reply(2 + c, e, ST_NO_MEMORY);
        return;
    }
    s.id = s_.next_shape++;
    s_.shapes.push_back(s);
    reply(2 + c, e, ST_OK, s.id);
}

void Service::collect_request(u64 c, const QEntry &e) {
    if (e.w[6] > 2) {
        reply(2 + c, e, ST_UNSUPPORTED);
        return;
    }
    HoldAsk a;
    a.mode = e.w[6];
    a.from_core = true;
    a.core = c;
    a.tag = e.w[1];
    s_.holds.push_back(a);
}

// A CANCEL's one answer is its request's reply: CANCELLED for a request
// still waiting, the ordinary result for one already at work.
void Service::cancel(u64 c, const QEntry &e) {
    MdCore &core = s_.core[size_t(c)];
    QEntry request;
    request.w[1] = e.w[1];
    if (core.region_parked && core.parked_tag == e.w[1]) {
        core.region_parked = false;
        request.w[0] = OP_REGION;
        reply(2 + c, request, ST_CANCELLED);
        return;
    }
    for (auto it = s_.holds.begin(); it != s_.holds.end(); ++it) {
        if (it->from_core && it->core == c && it->tag == e.w[1]) {
            s_.holds.erase(it);
            request.w[0] = OP_COLLECT;
            reply(2 + c, request, ST_CANCELLED, s_.live_bytes, epoch());
            return;
        }
    }
}

// ---------------------------------------------------------------------------
//  Store logs (managed-data.md section 7, spec/05-memory-gc.md section 12.4)
// ---------------------------------------------------------------------------

// Takes every entry the core's log holds and replays it through LM-1's
// barrier.  TAIL is read before the ring's address and capacity, since a
// core may move to a larger ring.
bool Service::drain_log(u64 c) {
    MdCore &core = s_.core[size_t(c)];
    u64 tail;
    if (!read(rec(c) + CR_TAIL, tail)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_TAIL);
        return false;
    }
    if (tail == core.log_head) return true;
    u64 ring_cap[2];
    if (!read_words(rec(c) + CR_LOG_RING, ring_cap, 2)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_LOG_RING);
        return false;
    }
    const u64 ring_at = ring_cap[0], capacity = ring_cap[1];
    // TAIL never moves back, and never runs more than LOG_CAPACITY ahead of
    // LOG_HEAD.
    if (tail < core.tail_seen || tail < core.log_head || !aligned8(ring_at) || !power_of_two(capacity) ||
        tail - core.log_head > capacity) {
        fail(NOTICE_LOG, WHY_LOG, rec(c) + CR_TAIL);
        return false;
    }
    core.tail_seen = tail;
    const bool full = tail - core.log_head == capacity;
    std::vector<u64> words;
    for (u64 n = core.log_head; n < tail;) {
        const u64 slot = n % capacity;
        const u64 run = std::min(tail - n, capacity - slot);
        words.resize(size_t(run));
        if (!read_words(ring_at + 8 * slot, words.data(), run)) {
            fail(NOTICE_LOG, WHY_LOG, ring_at + 8 * slot);
            return false;
        }
        for (u64 w : words) {
            replay(w);
            if (!serving()) return false;
        }
        n += run;
    }
    core.log_head = tail;
    if (!write(entry(c) + CE_LOG_HEAD, tail)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, entry(c));
        return false;
    }
    if (full) ring(c);  // the core waits for LOG_HEAD to move
    return true;
}

// One logged store, replayed as LM-1's own store of that value would be:
// a word of an old object that now refers to a young one goes into the
// remembered set.  Handle slots are roots in every collection, and young
// objects are traced whole, so neither needs a record.
void Service::replay(u64 word) {
    bool in_heap = false;
    for (const Extent &b : s_.blocks) in_heap = in_heap || (word >= b.base && word < b.limit);
    if (!in_heap) {
        // The handle table: read the directory to know its segments.
        u64 dir[2];
        if (!read_words(s_.handle_dir, dir, 2) || !power_of_two(dir[0]) || dir[1] > HD_MAX_SEGMENTS) {
            fail(NOTICE_CORRUPT, WHY_RECORD, s_.handle_dir);
            return;
        }
        std::vector<u64> bases(size_t(dir[1]), 0);
        if (dir[1] > 0 && !read_words(s_.handle_dir + HD_BASES, bases.data(), dir[1])) {
            fail(NOTICE_CORRUPT, WHY_RECORD, s_.handle_dir);
            return;
        }
        for (u64 k = 0; k < dir[1]; ++k) {
            const u64 slots = dir[0] << k;
            if (word >= bases[size_t(k)] && word < bases[size_t(k)] + 8 * slots && word % 8 == 0) return;
        }
        fail(NOTICE_LOG, WHY_LOG, word);  // names a word outside both
        return;
    }
    if (word % 8 != 0) {
        fail(NOTICE_LOG, WHY_LOG, word);
        return;
    }
    auto it = s_.old.upper_bound(word);
    if (it == s_.old.begin()) return;
    --it;
    if (word >= it->first + it->second) return;  // not in an old object
    u64 v;
    if (!read(word, v)) {
        fail(NOTICE_CORRUPT, WHY_LINK, word);
        return;
    }
    if (is_any_ref(v) && s_.old.find(ref_address(v)) == s_.old.end()) s_.remembered.insert(word);
}

// ---------------------------------------------------------------------------
//  Holds (managed-data.md section 3, spec/05-memory-gc.md section 12.3)
// ---------------------------------------------------------------------------

bool Service::read_window(u64 c, u64 &w) {
    if (!read(rec(c) + CR_WINDOW, w)) {
        fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_WINDOW);
        return false;
    }
    if (w > WINDOW_WAITING) {
        fail(NOTICE_CORRUPT, WHY_WINDOW, rec(c) + CR_WINDOW);
        return false;
    }
    return true;
}

void Service::begin_hold() {
    s_.current = s_.holds.front();
    s_.holds.erase(s_.holds.begin());
    // HELD, and the store's answer, before any window word is read.
    s_.hold_word = (epoch() << 3) | HOLD_HELD;
    if (!write(s_.lm1_block + LB_HOLD, s_.hold_word)) {
        fail(NOTICE_CORRUPT, WHY_LINK, s_.lm1_block);
        return;
    }
    s_.hold = HoldPhase::Held;
    s_.held_at_ns = s_.clock_ns;
    for (u64 c = 0; c < s_.cores; ++c) {
        MdCore &core = s_.core[size_t(c)];
        core.noticed = false;
        u64 w;
        if (!read_window(c, w)) return;
        core.waited = w == WINDOW_OPEN;
    }
}

void Service::holds() {
    if (!serving()) return;
    if (s_.hold == HoldPhase::None) {
        // LM-1's own policy: a young collection once MegaPad has made half
        // as many bytes as the heap holds since the last one.
        if (s_.holds.empty() && m_.config.young_trigger != 0 && s_.young_since >= m_.config.young_trigger) {
            HoldAsk a;
            s_.holds.push_back(a);
        } else if (s_.holds.empty() && m_.config.young_trigger == 0 && s_.young_since > 0 &&
                   s_.young_since >= heap_bytes() / 2) {
            HoldAsk a;
            s_.holds.push_back(a);
        }
        if (s_.holds.empty() || s_.clock_ns < s_.gap_until_ns) return;
        begin_hold();
        if (!serving()) return;
    }
    if (s_.hold == HoldPhase::Held) {
        if (s_.current.mode == 4) return;   // the hook's hold that never goes on
        // A core's window word is read again when the core rings.
        bool open = false;
        for (u64 c = 0; c < s_.cores; ++c) {
            MdCore &core = s_.core[size_t(c)];
            if (!core.waited) continue;
            if (rung_[size_t(c)]) {
                u64 w;
                if (!read_window(c, w)) return;
                core.waited = w == WINDOW_OPEN;
            }
            if (!core.waited) continue;
            open = true;
            // A window still open WINDOW_LIMIT after HELD was set is
            // MegaPad's fault: LM-1 waits on, and reports the core.
            const u64 open_ns = s_.clock_ns - s_.held_at_ns;
            if (!core.noticed && open_ns >= m_.config.window_limit) {
                core.noticed = true;
                notice(NOTICE_WINDOW, (c & 0xFFFF) | (open_ns << 16), rec(c) + CR_WINDOW);
            }
        }
        if (open) return;
        active_work();
        if (!serving() || m_.link.stop_req) return;
    }
    if (s_.hold == HoldPhase::Active && s_.current.keep_ns != ~0ull && s_.clock_ns >= s_.keep_until_ns)
        end_hold();
}

void Service::active_work() {
    // Every window has closed and none can open: ACTIVE.
    s_.hold_word |= HOLD_ACTIVE;
    if (!write(s_.lm1_block + LB_HOLD, s_.hold_word)) {
        fail(NOTICE_CORRUPT, WHY_LINK, s_.lm1_block);
        return;
    }
    s_.hold = HoldPhase::Active;
    // Every log drained to the TAIL read here, before anything is decided.
    for (u64 c = 0; c < s_.cores; ++c) {
        if (!drain_log(c)) return;
    }
    std::vector<Extent> released;
    if (!retire_regions(released) || !walk_runs()) return;
    if (s_.current.mode <= 2 && !collect(s_.current.mode, released)) return;
    // A stop that came while the hold flipped is answered here, with MOVE
    // reading FIXED: LM-1 writes nothing more, and the hold stays ACTIVE
    // for MegaPad's rule to end (MegaPad's docs/managed-data.md section 11,
    // row 6).
    if (m_.link.stop_req) return;
    // What the hold freed, and the rest of every region it retired, are
    // free once its moves are done.
    for (const Extent &x : released) give_back(x.base, x.limit);
    // A fresh region for each core that makes objects, waiting at its first
    // open after the hold.
    for (u64 c = 0; c < s_.cores; ++c) {
        MdCore &core = s_.core[size_t(c)];
        if (!core.makes_objects) continue;
        u64 base, limit;
        if (!grant_region(c, core.region_size, base, limit)) {
            fail(NOTICE_CORRUPT, WHY_RECORD, entry(c));
            return;
        }
    }
    if (free_bytes() < heap_bytes() / 4) ask_grow(0);
    // A block that a collection of the whole heap emptied goes back to
    // MegaPad, the first one excepted, while the free space left keeps the
    // reserve.
    if (s_.current.mode == 1 || s_.current.mode == 2) {
        for (size_t i = 1; i < s_.blocks.size();) {
            const Extent b = s_.blocks[i];
            auto f = s_.free.find(b.base);
            const u64 len = b.limit - b.base;
            if (f == s_.free.end() || f->second < b.limit || free_bytes() - len < (heap_bytes() - len) / 4 ||
                s_.asked.size() >= s_.grant.in.capacity) {
                ++i;
                continue;
            }
            if (f->second > b.limit) s_.free[b.limit] = f->second;
            s_.free.erase(f);
            s_.blocks.erase(s_.blocks.begin() + std::ptrdiff_t(i));
            s_.returns += 1;
            ask(OP_RETURN, b.base, len, 0, 0);
        }
    }
    s_.keep_until_ns = s_.current.keep_ns == ~0ull ? ~0ull : s_.clock_ns + s_.current.keep_ns;
    if (s_.current.keep_ns == 0) end_hold();
}

void Service::end_hold() {
    s_.hold_word = (epoch() + 1) << 3;
    if (!write(s_.lm1_block + LB_HOLD, s_.hold_word)) {
        fail(NOTICE_CORRUPT, WHY_LINK, s_.lm1_block);
        return;
    }
    s_.hold = HoldPhase::None;
    s_.holds_done += 1;
    s_.gap_until_ns = s_.clock_ns + m_.config.hold_gap;
    // Every core found WAITING is rung, and the COLLECT that asked for the
    // hold is answered with the live bytes and the epoch.
    for (u64 c = 0; c < s_.cores; ++c) {
        u64 w;
        if (!read_window(c, w)) return;
        if (w == WINDOW_WAITING) {
            ring(c);
            s_.wakes += 1;
        }
    }
    if (s_.current.from_core) {
        QEntry request;
        request.w[0] = OP_COLLECT;
        request.w[1] = s_.current.tag;
        reply(2 + s_.current.core, request, ST_OK, s_.live_bytes, epoch());
    }
    s_.current = HoldAsk{};
}

// Every region LM-1 gave MegaPad retires: the whole objects each holds,
// from its base to the FILL the core published, join the runs the hold
// walks, and the rest of each returns to free space once the hold is done.
bool Service::retire_regions(std::vector<Extent> &released) {
    for (u64 c = 0; c < s_.cores; ++c) {
        MdCore &core = s_.core[size_t(c)];
        if (core.grants.empty()) continue;
        u64 rf[2];
        if (!read_words(rec(c) + CR_REGION, rf, 2)) {
            fail(NOTICE_CORRUPT, WHY_RECORD, rec(c) + CR_REGION);
            return false;
        }
        core.recorded_region = rf[0];
        for (const Grant &g : core.grants) {
            // The region the core makes objects in ends at the FILL it
            // stored at its last close; one it left, at the fill it stored
            // before asking for the next.  A core that kept making objects
            // in a region after a refused request still names it.
            u64 end = g.left ? g.fill : g.base;
            if (rf[0] == g.base && rf[1] >= g.base && rf[1] <= g.limit) end = std::max(end, rf[1]);
            if (end > g.base) s_.runs.push_back(Extent{g.base, end});
            if (end < g.limit) released.push_back(Extent{end, g.limit});
        }
        core.grants.clear();
    }
    return true;
}

// A shared object's tagged words, by its header (spec/01-object-model.md
// section 3.5): a bytevector's bytes, a record's words after its tagged
// fields and the payloads of the raw kinds are never read as references.
void Service::tagged_fields(const std::vector<u64> &words, std::vector<u64> &fields) const {
    fields.clear();
    const u64 h = words[0];
    const u64 sub = header_subtype(h), size = header_size(h);
    u64 traced = size;
    if (sub == SUB_BYTES) traced = std::min<u64>(size, 1);
    else if (sub == SUB_RECORD) traced = size == 0 ? 0 : std::min<u64>(size, 1 + u64(untag_fixnum(words[1])));
    else if (sub == SUB_BIGNUM || sub == SUB_DOUBLE || sub == SUB_FOREIGN) traced = 0;
    for (u64 i = 0; i < traced; ++i) fields.push_back(i);
}

bool Service::read_object(u64 addr, std::vector<u64> &words) {
    u64 h;
    if (!read(addr, h)) return false;
    words.assign(size_t(1 + header_size(h)), 0);
    words[0] = h;
    return header_size(h) == 0 || read_words(addr + 8, words.data() + 1, header_size(h));
}

// One object MegaPad made, as the hold walks its region: a sound header of
// one of the four kinds, a size inside the run, a registered shape, and the
// layout its kind requires.
bool Service::check_object(u64 addr, const std::vector<u64> &words, u64 end) {
    const u64 h = words[0];
    if (!is_header(h) || header_gc(h) != 0) return false;
    const u64 sub = header_subtype(h), size = header_size(h), shape = header_shape_id(h);
    if (u128(addr) + 8 * (1 + size) > end) return false;
    switch (sub) {
    case SUB_CONS:
        return size == 2 && shape == SHAPE_CONS;
    case SUB_VECTOR:
        return size >= 1 && shape == SHAPE_VECTOR && is_fixnum(words[1]) &&
               untag_fixnum(words[1]) == i64(size - 1);
    case SUB_BYTES: {
        if (size < 1 || (shape != SHAPE_BYTES && shape != SHAPE_STRING) || !is_fixnum(words[1])) return false;
        const i64 n = untag_fixnum(words[1]);
        return n >= 0 && u64(n + 7) / 8 == size - 1;
    }
    case SUB_RECORD: {
        if (size < 1 || !is_fixnum(words[1])) return false;
        for (const MdShape &s : s_.shapes) {
            if (s.id == shape)
                return untag_fixnum(words[1]) == i64(s.tagged) && size == 1 + s.tagged + s.raw;
        }
        return false;
    }
    default:
        return false;
    }
}

// The runs the hold retired, walked object by object from each base; each
// object found sound becomes young.
bool Service::walk_runs() {
    std::vector<u64> words;
    for (const Extent &run : s_.runs) {
        for (u64 at = run.base; at < run.limit;) {
            if (!read_object(at, words)) {
                fail(NOTICE_CORRUPT, WHY_LINK, at);
                return false;
            }
            if (!check_object(at, words, run.limit)) {
                fail(NOTICE_CORRUPT, WHY_OBJECT, at);
                return false;
            }
            const u64 bytes = 8 * words.size();
            s_.young[at] = bytes;
            at += bytes;
        }
    }
    s_.runs.clear();
    // Each object's references lie in the shared heap, at the start of an
    // object, and carry no metadata.
    std::vector<u64> fields;
    for (const auto &o : s_.young) {
        if (!read_object(o.first, words)) {
            fail(NOTICE_CORRUPT, WHY_LINK, o.first);
            return false;
        }
        tagged_fields(words, fields);
        for (u64 i : fields) {
            if (!check_ref(words[size_t(1 + i)])) {
                fail(NOTICE_CORRUPT, WHY_REFERENCE, o.first + 8 * (1 + i));
                return false;
            }
        }
    }
    return true;
}

bool Service::object_at(u64 a, bool &is_old) const {
    is_old = s_.old.count(a) != 0;
    return is_old || s_.young.count(a) != 0;
}

// A tagged word as the collector reads it from the shared heap or a handle
// slot: a header-tagged word never stands in a traced word, and a reference
// names the start of an object in the shared heap and carries zero in bits
// 63:51 (spec/05-memory-gc.md sections 12.1 and 12.8).
bool Service::check_ref(u64 v) {
    if (is_header(v)) return false;
    if (!is_any_ref(v)) return true;
    if (v & METADATA) return false;
    bool is_old;
    return object_at(ref_address(v), is_old);
}

// ---- Space ----

bool Service::take_space(u64 bytes, u64 &at, std::vector<Extent> &taken) {
    for (auto it = s_.free.begin(); it != s_.free.end(); ++it) {
        if (it->second - it->first < bytes) continue;
        at = it->first;
        const u64 end = it->second;
        s_.free.erase(it);
        if (at + bytes < end) s_.free[at + bytes] = end;
        taken.push_back(Extent{at, at + bytes});
        return true;
    }
    return false;
}

void Service::give_back(u64 base, u64 limit) {
    if (limit <= base) return;
    auto next = s_.free.lower_bound(base);
    if (next != s_.free.end() && next->first == limit) {
        limit = next->second;
        next = s_.free.erase(next);
    }
    if (next != s_.free.begin()) {
        auto prev = std::prev(next);
        if (prev->second == base) {
            prev->second = limit;
            return;
        }
    }
    s_.free[base] = limit;
}

// ---------------------------------------------------------------------------
//  The collection
// ---------------------------------------------------------------------------

// Mode 0 collects the young objects: roots are the handle slots, the words
// of the remembered set and LM-1's registers, and it traces young objects
// alone.  Modes 1 and 2 trace the whole shared heap from the handle slots
// and LM-1's registers.  Modes 0 and 1 copy the young survivors out of the
// regions they were made in; mode 2 copies every live object that space
// free when the hold began can hold.  The rest of what lives stays in
// place, and every object that lives is old afterwards.
bool Service::collect(u64 mode, std::vector<Extent> &released) {
    const bool whole = mode != 0;
    // ---- The roots ----
    u64 dir[2];
    if (!read_words(s_.handle_dir, dir, 2) || !power_of_two(dir[0]) || dir[1] > HD_MAX_SEGMENTS) {
        fail(NOTICE_CORRUPT, WHY_RECORD, s_.handle_dir);
        return false;
    }
    std::vector<u64> bases(size_t(dir[1]), 0);
    if (dir[1] > 0 && !read_words(s_.handle_dir + HD_BASES, bases.data(), dir[1])) {
        fail(NOTICE_CORRUPT, WHY_RECORD, s_.handle_dir);
        return false;
    }
    struct Root { u64 where; u64 value; };   // a slot's address, and what it holds
    std::vector<Root> slots;
    for (u64 k = 0; k < dir[1]; ++k) {
        const u64 n = dir[0] << k;
        if (!aligned8(bases[size_t(k)])) {
            fail(NOTICE_CORRUPT, WHY_RECORD, s_.handle_dir + HD_BASES + 8 * k);
            return false;
        }
        std::vector<u64> values(n);
        if (!read_words(bases[size_t(k)], values.data(), n)) {
            fail(NOTICE_CORRUPT, WHY_LINK, bases[size_t(k)]);
            return false;
        }
        for (u64 i = 0; i < n; ++i) {
            const u64 v = values[size_t(i)];
            if (!is_any_ref(v)) continue;
            if (!check_ref(v)) {
                fail(NOTICE_CORRUPT, WHY_SLOT, bases[size_t(k)] + 8 * i);
                return false;
            }
            slots.push_back(Root{bases[size_t(k)] + 8 * i, v});
        }
    }
    // ---- The mark, kept in LM-1's own memory ----
    std::unordered_set<u64> marked;
    std::vector<u64> work;
    auto reach = [&](u64 v) {
        if (!is_any_ref(v)) return;
        const u64 a = ref_address(v);
        bool is_old;
        if (!object_at(a, is_old)) return;
        if (!whole && is_old) return;  // a young collection counts old objects live
        if (marked.insert(a).second) work.push_back(a);
    };
    for (const Root &r : slots) reach(r.value);
    std::vector<u64> remembered_values;
    if (!whole) {
        for (u64 word : s_.remembered) {
            u64 v;
            if (!read(word, v)) {
                fail(NOTICE_CORRUPT, WHY_LINK, word);
                return false;
            }
            if (is_any_ref(v) && !check_ref(v)) {
                fail(NOTICE_CORRUPT, WHY_REFERENCE, word);
                return false;
            }
            reach(v);
        }
    }
    for (Tile &tile : m_.tiles) {
        for (Thread &t : tile.threads) {
            for (u64 r : t.regs) {
                if (is_any_ref(r) && in_shared_heap(m_, ref_address(r))) reach(r & ~METADATA);
            }
        }
    }
    for (const Standin::Churn &ch : s_.churns) {
        if (ch.left == 0) continue;
        reach(ch.a);
        reach(ch.b);
    }
    std::unordered_map<u64, std::vector<u64>> contents;   // what each marked object holds
    std::vector<u64> fields;
    while (!work.empty()) {
        const u64 a = work.back();
        work.pop_back();
        std::vector<u64> &words = contents[a];
        if (!read_object(a, words)) {
            fail(NOTICE_CORRUPT, WHY_LINK, a);
            return false;
        }
        if (!is_header(words[0]) || is_forwarding(words[0])) {
            fail(NOTICE_CORRUPT, WHY_OBJECT, a);
            return false;
        }
        tagged_fields(words, fields);
        for (u64 i : fields) {
            const u64 v = words[size_t(1 + i)];
            if (!check_ref(v)) {
                fail(NOTICE_CORRUPT, WHY_REFERENCE, a + 8 * (1 + i));
                return false;
            }
            reach(v);
        }
    }
    // ---- What moves, and where to ----
    std::vector<u64> moving;
    for (u64 a : marked) {
        if (mode == 2 || s_.young.count(a) != 0) moving.push_back(a);
    }
    std::sort(moving.begin(), moving.end());
    std::unordered_map<u64, u64> to;   // old address -> new
    std::vector<Extent> taken;
    std::vector<Extent> sources, destinations;
    u64 move_set = 0;
    if (!moving.empty()) {
        // The move set's own space first, then each object's copy, all in
        // space free when the hold began.  What finds no space stays put.
        // The move set's space is free again once the hold ends.
        const u64 most = (16 + 16 * (2 * moving.size()) + 63) & ~63ull;
        if (take_space(most, move_set, taken)) released.push_back(Extent{move_set, move_set + most});
        else moving.clear();
        std::vector<u64> placed;
        for (u64 a : moving) {
            const u64 bytes = 8 * contents[a].size();
            u64 at;
            if (!take_space(bytes, at, taken)) continue;
            to[a] = at;
            placed.push_back(a);
            if (!sources.empty() && sources.back().limit == a) sources.back().limit = a + bytes;
            else sources.push_back(Extent{a, a + bytes});
            if (!destinations.empty() && destinations.back().limit == at) destinations.back().limit = at + bytes;
            else destinations.push_back(Extent{at, at + bytes});
        }
        moving.swap(placed);
    }
    if (!moving.empty()) {
        // The move set, MOVE_SET naming it, then MOVE reading FLIPPING with
        // this hold's epoch, before the first forwarding pointer.
        std::vector<u64> set;
        set.push_back(sources.size());
        set.push_back(destinations.size());
        for (const Extent &x : sources) { set.push_back(x.base); set.push_back(x.limit); }
        for (const Extent &x : destinations) { set.push_back(x.base); set.push_back(x.limit); }
        s_.move_word = (epoch() << 3) | MOVE_FLIPPING;
        if (!write_words(move_set, set.data(), set.size()) || !write(s_.lm1_block + LB_MOVE_SET, move_set) ||
            !write(s_.lm1_block + LB_MOVE, s_.move_word)) {
            fail(NOTICE_CORRUPT, WHY_LINK, move_set);
            return false;
        }
        // The test hook's stop arrives now, while the hold flips.  LM-1
        // answers a stop only between moves (spec/05-memory-gc.md section
        // 12.9): it finishes every move and rewrite below, writes FIXED,
        // and only then stops.
        if (s_.stop_at_flip) {
            s_.stop_at_flip = false;
            m_.link.stop_req = true;
        }
        // Each copy whole, then the forwarding pointer over the old
        // header, in a request of its own.
        for (u64 a : moving) {
            const std::vector<u64> &words = contents[a];
            if (!write_words(to[a], words.data(), words.size()) || !write(a, forwarding_pointer(to[a]))) {
                fail(NOTICE_CORRUPT, WHY_LINK, a);
                return false;
            }
            s_.objects_moved += 1;
        }
        // Then every reference to a moved object, written anew without
        // metadata: in each live object, the copies included, and in the
        // handle slots, the remembered words and LM-1's registers.
        auto moved = [&](u64 v, u64 &nv) {
            if (!is_any_ref(v)) return false;
            auto f = to.find(ref_address(v));
            if (f == to.end()) return false;
            nv = make_ref(f->second, (v & 7) == TAG_CONS);
            return true;
        };
        for (auto &obj : contents) {
            const u64 where = to.count(obj.first) ? to[obj.first] : obj.first;
            tagged_fields(obj.second, fields);
            for (u64 i : fields) {
                u64 nv;
                if (moved(obj.second[size_t(1 + i)], nv)) {
                    obj.second[size_t(1 + i)] = nv;
                    if (!write(where + 8 * (1 + i), nv)) {
                        fail(NOTICE_CORRUPT, WHY_LINK, where);
                        return false;
                    }
                }
            }
        }
        if (!whole) {
            for (u64 word : s_.remembered) {
                u64 v, nv;
                if (!read(word, v)) {
                    fail(NOTICE_CORRUPT, WHY_LINK, word);
                    return false;
                }
                if (moved(v, nv) && !write(word, nv)) {
                    fail(NOTICE_CORRUPT, WHY_LINK, word);
                    return false;
                }
            }
        }
        for (const Root &r : slots) {
            u64 nv;
            if (moved(r.value, nv) && !write(r.where, nv)) {
                fail(NOTICE_CORRUPT, WHY_LINK, r.where);
                return false;
            }
        }
        for (Tile &tile : m_.tiles) {
            for (Thread &t : tile.threads) {
                for (u64 &r : t.regs) {
                    u64 nv;
                    if (is_any_ref(r) && in_shared_heap(m_, ref_address(r)) && moved(r & ~METADATA, nv)) r = nv;
                }
            }
        }
        for (Standin::Churn &ch : s_.churns) {
            u64 nv;
            if (moved(ch.a, nv)) ch.a = nv;
            if (moved(ch.b, nv)) ch.b = nv;
        }
        // FIXED, after the last rewrite.
        s_.move_word = (epoch() << 3) | MOVE_FIXED;
        if (!write(s_.lm1_block + LB_MOVE, s_.move_word)) {
            fail(NOTICE_CORRUPT, WHY_LINK, s_.lm1_block);
            return false;
        }
    }
    // ---- What lives on, and what is free once the hold is done ----
    std::map<u64, u64> next_old;
    for (const auto &o : s_.old) {
        if (whole && marked.count(o.first) == 0) {
            released.push_back(Extent{o.first, o.first + o.second});
            continue;
        }
        if (to.count(o.first)) {
            next_old[to[o.first]] = o.second;
            released.push_back(Extent{o.first, o.first + o.second});
            continue;
        }
        next_old[o.first] = o.second;
    }
    for (const auto &o : s_.young) {
        if (marked.count(o.first) == 0) {
            released.push_back(Extent{o.first, o.first + o.second});
        } else if (to.count(o.first)) {
            next_old[to[o.first]] = o.second;
            released.push_back(Extent{o.first, o.first + o.second});
        } else {
            next_old[o.first] = o.second;
        }
    }
    s_.old.swap(next_old);
    s_.young.clear();
    s_.remembered.clear();
    s_.young_since = 0;
    u64 live = 0;
    for (const auto &o : s_.old) live += o.second;
    s_.live_bytes = live;
    return true;
}

// ---------------------------------------------------------------------------
//  The test hooks' writes, and failure
// ---------------------------------------------------------------------------

void Service::hook_writes() {
    for (const auto &p : s_.pokes) {
        if (!write(p.first, p.second)) {
            continue;
        }
        standin_barrier(m_, p.first, p.second);
    }
    s_.pokes.clear();
    for (Standin::Churn &ch : s_.churns) {
        if (ch.left == 0) continue;
        const u64 v = (ch.turn & 1) ? ch.b : ch.a;
        if (!write(ch.addr, v)) {
            ch.left = 0;
            continue;
        }
        standin_barrier(m_, ch.addr, v);
        ch.turn += 1;
        ch.left -= 1;
    }
    s_.churns.erase(std::remove_if(s_.churns.begin(), s_.churns.end(),
                                   [](const Standin::Churn &c) { return c.left == 0; }),
                    s_.churns.end());
}

void Service::notice(u64 kind, u64 arg1, u64 addr) {
    s_.notices += 1;
    s_.last_notice_kind = kind;
    s_.last_notice_arg = arg1;
    s_.last_notice_addr = addr;
    if (s_.asked.size() < s_.grant.in.capacity) ask(OP_NOTICE, addr, 0, kind, arg1);
}

// LM-1 found corrupt data: it reports it, sets FAILED, and serves the
// shared heap no more (spec/05-memory-gc.md section 12.9).  Every core that
// waits is rung, so that it reads FAILED, and every COLLECT waiting for a
// hold is answered FAILED.
void Service::fail(u64 kind, u64 why, u64 addr) {
    if (s_.phase != MdPhase::Serving) {
        if (s_.phase == MdPhase::Granted || s_.phase == MdPhase::Offered) {
            s_.phase = MdPhase::Broken;
            s_.why = why;
            s_.why_addr = addr;
        }
        return;
    }
    s_.phase = MdPhase::Failed;
    s_.why = why;
    s_.why_addr = addr;
    notice(kind, 0, addr);
    s_.hold_word = (epoch() << 3) | HOLD_FAILED;
    write(s_.lm1_block + LB_HOLD, s_.hold_word);
    s_.hold = HoldPhase::None;
    std::vector<HoldAsk> answered = std::move(s_.holds);
    s_.holds.clear();
    if (s_.current.from_core) answered.insert(answered.begin(), s_.current);
    s_.current = HoldAsk{};
    for (const HoldAsk &a : answered) {
        if (!a.from_core) continue;
        QEntry request;
        request.w[0] = OP_COLLECT;
        request.w[1] = a.tag;
        reply(2 + a.core, request, ST_FAILED);
    }
    for (u64 c = 0; c < s_.cores; ++c) {
        u64 w;
        if (read(rec(c) + CR_WINDOW, w) && w == WINDOW_WAITING) ring(c);
        MdCore &core = s_.core[size_t(c)];
        if (core.region_parked) {
            core.region_parked = false;
            QEntry request;
            request.w[0] = OP_REGION;
            request.w[1] = core.parked_tag;
            reply(2 + c, request, ST_FAILED);
        }
    }
}

// ---------------------------------------------------------------------------
//  A step
// ---------------------------------------------------------------------------

void Service::run() {
    // The runtime comes up once: it reads the control block the boot
    // argument names, and serves nothing when that names none.
    if (s_.phase == MdPhase::Off && s_.control_block == 0) {
        if (m_.link.boot_arg == 0) return;
        start();
        any_rung_ = true;
    }
    if (s_.phase == MdPhase::Broken || !s_.control.valid) return;
    // The runtime takes the doorbells MegaPad rang from its tile's queue 0.
    Queue &q = m_.tiles[LM1C_LINK_TILE].queues[LM1C_DOORBELL_IN_QUEUE];
    while (q.size() > 0) {
        const u64 v = q.pop();
        if (is_fixnum(v) && untag_fixnum(v) >= 0 && untag_fixnum(v) < 256) {
            rung_[size_t(untag_fixnum(v))] = true;
            any_rung_ = true;
        }
    }
    if (!s_.outbox.empty()) flush_outbox();
    if (any_rung_) {
        serve_control();
        if (s_.phase >= MdPhase::Granted) serve_grant();
        if (s_.phase == MdPhase::Serving || s_.phase == MdPhase::Failed) {
            for (u64 c = 0; c < s_.cores && c < s_.core.size(); ++c) {
                if (!rung_[size_t(c)] && !s_.core[size_t(c)].pair_known) continue;
                serve_core(c);
                // A core rings when its store log is half full or full.
                if (serving() && rung_[size_t(c)] && s_.hold != HoldPhase::Active && !s_.no_drain)
                    drain_log(c);
            }
        }
    }
    if (serving()) {
        hook_writes();
        holds();
    }
    // A stop that arrived in this step's hold: nothing more goes out.
    if (m_.link.stop_req) return;
    if (!s_.outbox.empty()) flush_outbox();
}

}  // namespace

u64 standin_step(Model &m, u64 budget) {
    if (!m.config.runtime || m.link_host == nullptr) return budget;
    m.standin.clock_ns += budget * m.config.unit_ns;
    Service s(m, budget);
    s.run();
    return s.left();
}

bool standin_waits(const Model &m) {
    const Standin &s = m.standin;
    if (!m.config.runtime) return false;
    if (s.phase == MdPhase::Off && s.control_block == 0 && m.link.boot_arg != 0) return true;
    if (!s.outbox.empty()) return true;
    if (s.phase != MdPhase::Serving) return false;
    if (s.hold == HoldPhase::Held && s.current.mode == 4) return false;   // stopped at HELD
    if (s.hold != HoldPhase::None || !s.holds.empty() || !s.pokes.empty() || !s.churns.empty()) return true;
    return false;
}

bool link_access(Model &m, u128 addr, unsigned width, bool write, u64 &value) {
    if (m.link_host == nullptr || addr > ~0ull) return false;
    const unsigned size = width >= 64 ? 3 : width == 32 ? 2 : width == 16 ? 1 : 0;
    const u64 at = size == 3 ? u64(addr) & ~7ull : u64(addr);
    u64 data = value;
    unsigned done = 0;
    const int code = m.link_host->mem(m.link_host->ctx, at, size, 0, write ? 1 : 0, &data, &done);
    if (code != MP64_CHIPLET_OK) return false;
    if (!write) value = data;
    return true;
}

bool in_shared_heap(const Model &m, u64 addr) {
    for (const Extent &b : m.standin.blocks) {
        if (addr >= b.base && addr < b.limit) return true;
    }
    return false;
}

void standin_barrier(Model &m, u64 word_addr, u64 value) {
    Standin &s = m.standin;
    if (!is_any_ref(value) || s.old.count(ref_address(value)) != 0) return;
    auto it = s.old.upper_bound(word_addr);
    if (it == s.old.begin()) return;
    --it;
    if (word_addr < it->first + it->second) s.remembered.insert(word_addr);
}

}  // namespace lm1
