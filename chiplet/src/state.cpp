// LM-1 chiplet module: the model's whole state as bytes, for save and
// restore.
//
// A state carries the configuration it was made under, as config_text
// writes it, and restores only into a model of the same configuration, so
// every size it holds fits.  Memory travels as runs of zero words and runs
// of words as they are, so the state of a mostly empty memory stays small.
// The host's callbacks, MegaPad's services and the trap in hand during one
// instruction are no part of it: none outlives a call.
#include <algorithm>
#include <cstring>

#include "model.h"

namespace lm1 {

namespace {

constexpr char MAGIC[8] = {'L', 'M', '1', 'C', 'H', 'I', 'P', 'S'};
constexpr uint32_t VERSION = 4;  // 2: a tile's queue base; 3: the runtime's stand-in;
                                 // 4: a thread's INSTRET

class Writer {
public:
    void raw(const void *data, size_t size) {
        const uint8_t *bytes = static_cast<const uint8_t *>(data);
        out_.insert(out_.end(), bytes, bytes + size);
    }
    void byte(uint8_t v) { out_.push_back(v); }
    void word32(uint32_t v) { raw(&v, sizeof v); }
    void word64(uint64_t v) { raw(&v, sizeof v); }
    void flag(bool v) { byte(v ? 1 : 0); }
    void text(const std::string &s) {
        word64(s.size());
        raw(s.data(), s.size());
    }
    // Words as runs: a count of zero words, a count of words as they are,
    // then those words, until every word is covered.
    void words(const std::vector<u64> &w) {
        size_t i = 0;
        while (i < w.size()) {
            size_t zeros = 0;
            while (i + zeros < w.size() && w[i + zeros] == 0) ++zeros;
            size_t literal = 0;
            while (i + zeros + literal < w.size() && w[i + zeros + literal] != 0) ++literal;
            word64(zeros);
            word64(literal);
            raw(w.data() + i + zeros, literal * 8);
            i += zeros + literal;
        }
    }
    void bytes(const std::vector<uint8_t> &b) {
        size_t i = 0;
        while (i < b.size()) {
            size_t zeros = 0;
            while (i + zeros < b.size() && b[i + zeros] == 0) ++zeros;
            size_t literal = 0;
            while (i + zeros + literal < b.size() && b[i + zeros + literal] != 0) ++literal;
            word64(zeros);
            word64(literal);
            raw(b.data() + i + zeros, literal);
            i += zeros + literal;
        }
    }
    void result(const lm1c_result &r) {
        word32(uint32_t(r.stop));
        word32(r.trap);
        word64(r.address);
        word64(r.executed);
        word32(r.tile);
        word32(r.thread);
    }
    std::vector<uint8_t> take() { return std::move(out_); }

private:
    std::vector<uint8_t> out_;
};

class Reader {
public:
    Reader(const uint8_t *data, size_t size) : at_(data), end_(data + size) {}
    bool raw(void *out, size_t size) {
        if (size > size_t(end_ - at_)) return false;
        std::memcpy(out, at_, size);
        at_ += size;
        return true;
    }
    bool byte(uint8_t &v) { return raw(&v, 1); }
    bool word32(uint32_t &v) { return raw(&v, sizeof v); }
    bool word64(uint64_t &v) { return raw(&v, sizeof v); }
    bool signed64(int64_t &v) { return raw(&v, sizeof v); }
    bool flag(bool &v) {
        uint8_t b;
        if (!byte(b) || b > 1) return false;
        v = b != 0;
        return true;
    }
    bool text(std::string &s) {
        uint64_t n;
        if (!word64(n) || n > size_t(end_ - at_)) return false;
        s.assign(reinterpret_cast<const char *>(at_), size_t(n));
        at_ += n;
        return true;
    }
    bool words(std::vector<u64> &w) {
        size_t i = 0;
        while (i < w.size()) {
            uint64_t zeros, literal;
            if (!word64(zeros) || !word64(literal)) return false;
            if (zeros > w.size() - i || literal > w.size() - i - zeros) return false;
            std::fill(w.begin() + i, w.begin() + i + zeros, 0);
            if (!raw(w.data() + i + zeros, literal * 8)) return false;
            i += zeros + literal;
            if (zeros + literal == 0) return false;
        }
        return true;
    }
    bool bytes(std::vector<uint8_t> &b) {
        size_t i = 0;
        while (i < b.size()) {
            uint64_t zeros, literal;
            if (!word64(zeros) || !word64(literal)) return false;
            if (zeros > b.size() - i || literal > b.size() - i - zeros) return false;
            std::fill(b.begin() + i, b.begin() + i + zeros, 0);
            if (!raw(b.data() + i + zeros, literal)) return false;
            i += zeros + literal;
            if (zeros + literal == 0) return false;
        }
        return true;
    }
    bool result(lm1c_result &r) {
        uint32_t s;
        if (!word32(s) || !word32(r.trap) || !word64(r.address) || !word64(r.executed) || !word32(r.tile) ||
            !word32(r.thread))
            return false;
        r.stop = int32_t(s);
        return true;
    }
    bool done() const { return at_ == end_; }
    size_t left() const { return size_t(end_ - at_); }

private:
    const uint8_t *at_;
    const uint8_t *end_;
};

void save_thread(Writer &w, const Thread &t) {
    for (u64 r : t.regs) w.word64(r);
    w.word64(t.pc);
    w.word64(t.cycle_count);
    w.word64(t.instret);
    w.flag(t.halted);
    w.flag(t.stalled);
    w.word64(u64(t.stall_queue));
    w.word64(t.trap_table_base);
    w.word64(t.trap_cause);
    w.word64(t.trap_pc);
    w.flag(t.in_trap);
    w.word64(t.tile_id);
    w.word64(t.thread_id);
    w.word64(t.slice_base);
}

bool restore_thread(Reader &r, Thread &t) {
    for (u64 &reg : t.regs) {
        if (!r.word64(reg)) return false;
    }
    return r.word64(t.pc) && r.word64(t.cycle_count) && r.word64(t.instret) && r.flag(t.halted) &&
           r.flag(t.stalled) && r.signed64(t.stall_queue) && r.word64(t.trap_table_base) &&
           r.word64(t.trap_cause) && r.word64(t.trap_pc) && r.flag(t.in_trap) &&
           r.word64(t.tile_id) && r.word64(t.thread_id) && r.word64(t.slice_base);
}

void save_tile(Writer &w, const Tile &tile) {
    w.word64(tile.id);
    w.word64(tile.current);
    w.words(tile.mem);
    w.word64(tile.threads.size());
    for (const Thread &t : tile.threads) save_thread(w, t);
    for (u64 v : tile.templates) w.word64(v);
    w.word64(tile.nursery_base);
    w.word64(tile.nursery_size);
    w.word64(tile.nursery_chunk);
    w.word64(tile.nursery_free);
    w.word64(tile.oldgen_base);
    w.word64(tile.oldgen_size);
    w.word64(tile.oldgen_ptr);
    w.word64(tile.card_shift);
    w.word64(tile.card_table_base);
    w.word64(tile.gen_boundary);
    w.word64(tile.queue_base);
    w.word64(tile.cards.size());
    w.bytes(tile.cards);
    w.word64(tile.gc_count);
    w.word64(tile.instruction_count);
    // The inline cache in key order, so one state has one form.
    std::vector<std::pair<IcKey, u64>> ic(tile.ic.begin(), tile.ic.end());
    std::sort(ic.begin(), ic.end(), [](const auto &a, const auto &b) {
        return a.first.callsite != b.first.callsite ? a.first.callsite < b.first.callsite
                                                    : a.first.shape < b.first.shape;
    });
    w.word64(ic.size());
    for (const auto &entry : ic) {
        w.word64(entry.first.callsite);
        w.word64(entry.first.shape);
        w.word64(entry.second);
    }
    for (const Queue &q : tile.queues) {
        w.word64(q.size());
        for (size_t i = 0; i < q.size(); ++i) w.word64(q.at(i));
    }
}

bool restore_tile(Reader &r, Tile &tile, size_t queue_depth) {
    uint64_t current, threads, cards, ic_count;
    if (!r.word64(tile.id) || !r.word64(current) || !r.words(tile.mem) || !r.word64(threads) ||
        threads != tile.threads.size() || current >= threads)
        return false;
    tile.current = size_t(current);
    for (Thread &t : tile.threads) {
        if (!restore_thread(r, t)) return false;
    }
    for (u64 &v : tile.templates) {
        if (!r.word64(v)) return false;
    }
    if (!r.word64(tile.nursery_base) || !r.word64(tile.nursery_size) || !r.word64(tile.nursery_chunk) ||
        !r.word64(tile.nursery_free) || !r.word64(tile.oldgen_base) ||
        !r.word64(tile.oldgen_size) || !r.word64(tile.oldgen_ptr) || !r.word64(tile.card_shift) ||
        tile.card_shift > 63 || !r.word64(tile.card_table_base) || !r.word64(tile.gen_boundary) ||
        !r.word64(tile.queue_base) || !r.word64(cards))
        return false;
    // SET_CARD_SHIFT sizes the card table from the shift; a table the shift
    // does not size came from no model.
    if (cards != (tile.mem_size >> tile.card_shift)) return false;
    tile.cards.assign(size_t(cards), 0);
    if (!r.bytes(tile.cards) || !r.word64(tile.gc_count) || !r.word64(tile.instruction_count) ||
        !r.word64(ic_count))
        return false;
    tile.ic.clear();
    for (uint64_t i = 0; i < ic_count; ++i) {
        IcKey key;
        u64 target;
        if (!r.word64(key.callsite) || !r.word64(key.shape) || !r.word64(target)) return false;
        tile.ic[key] = target;
    }
    for (Queue &q : tile.queues) {
        uint64_t count;
        if (!r.word64(count) || count > queue_depth) return false;
        q.reset(queue_depth);
        for (uint64_t i = 0; i < count; ++i) {
            u64 v;
            if (!r.word64(v)) return false;
            q.push(v);
        }
    }
    return true;
}

// ---- The stand-in for LM-1's runtime (managed.h) ----

void save_queue(Writer &w, const LinkQueue &q) {
    w.word64(q.ring);
    w.word64(q.head_word);
    w.word64(q.capacity);
    w.word64(q.count);
    w.word64(q.other);
}

bool restore_queue(Reader &r, LinkQueue &q) {
    return r.word64(q.ring) && r.word64(q.head_word) && r.word64(q.capacity) && r.word64(q.count) &&
           r.word64(q.other);
}

void save_pair(Writer &w, const LinkPair &p) {
    w.flag(p.valid);
    save_queue(w, p.in);
    save_queue(w, p.out);
    w.word64(p.notify);
}

bool restore_pair(Reader &r, LinkPair &p) {
    return r.flag(p.valid) && restore_queue(r, p.in) && restore_queue(r, p.out) && r.word64(p.notify);
}

void save_entry(Writer &w, const QEntry &e) {
    for (u64 v : e.w) w.word64(v);
}

bool restore_entry(Reader &r, QEntry &e) {
    for (u64 &v : e.w) {
        if (!r.word64(v)) return false;
    }
    return true;
}

void save_map(Writer &w, const std::map<u64, u64> &m) {
    w.word64(m.size());
    for (const auto &kv : m) {
        w.word64(kv.first);
        w.word64(kv.second);
    }
}

// A count read from a state is never trusted to size an allocation beyond
// what the rest of the state could hold.
bool count_fits(Reader &r, uint64_t count, size_t each) { return count <= r.left() / each; }

bool restore_map(Reader &r, std::map<u64, u64> &m) {
    uint64_t n;
    if (!r.word64(n) || !count_fits(r, n, 16)) return false;
    m.clear();
    for (uint64_t i = 0; i < n; ++i) {
        u64 k, v;
        if (!r.word64(k) || !r.word64(v)) return false;
        m[k] = v;
    }
    return true;
}

void save_extents(Writer &w, const std::vector<Extent> &v) {
    w.word64(v.size());
    for (const Extent &x : v) {
        w.word64(x.base);
        w.word64(x.limit);
    }
}

bool restore_extents(Reader &r, std::vector<Extent> &v) {
    uint64_t n;
    if (!r.word64(n) || !count_fits(r, n, 16)) return false;
    v.assign(size_t(n), Extent{});
    for (Extent &x : v) {
        if (!r.word64(x.base) || !r.word64(x.limit)) return false;
    }
    return true;
}

void save_hold_ask(Writer &w, const HoldAsk &a) {
    w.word64(a.mode);
    w.word64(a.keep_ns);
    w.flag(a.from_core);
    w.word64(a.core);
    w.word64(a.tag);
}

bool restore_hold_ask(Reader &r, HoldAsk &a) {
    return r.word64(a.mode) && r.word64(a.keep_ns) && r.flag(a.from_core) && r.word64(a.core) &&
           r.word64(a.tag);
}

void save_standin(Writer &w, const Standin &s) {
    w.byte(uint8_t(s.phase));
    w.word64(s.clock_ns);
    w.word64(s.control_block);
    save_pair(w, s.control);
    save_pair(w, s.grant);
    w.word64(s.outbox.size());
    for (const Outgoing &o : s.outbox) {
        w.word64(o.pair);
        save_entry(w, o.entry);
    }
    w.word64(s.next_tag);
    w.word64(s.offer_tag);
    save_map(w, s.asked);
    w.flag(s.grow_asked);
    w.word64(s.records);
    w.word64(s.cores);
    w.word64(s.core_table);
    w.word64(s.lm1_block);
    w.word64(s.handle_dir);
    w.word64(s.core.size());
    for (const MdCore &c : s.core) {
        w.flag(c.pair_known);
        save_pair(w, c.pair);
        w.word64(c.grants.size());
        for (const Grant &g : c.grants) {
            w.word64(g.base);
            w.word64(g.limit);
            w.flag(g.left);
            w.word64(g.fill);
        }
        w.word64(c.next_serial);
        w.word64(c.log_head);
        w.word64(c.tail_seen);
        w.word64(c.region_size);
        w.word64(c.recorded_region);
        w.word64(c.asked_epoch);
        w.flag(c.makes_objects);
        w.flag(c.waited);
        w.flag(c.noticed);
        w.flag(c.region_parked);
        w.word64(c.parked_tag);
        w.word64(c.parked_bytes);
    }
    save_extents(w, s.blocks);
    save_map(w, s.free);
    save_map(w, s.old);
    save_map(w, s.young);
    save_extents(w, s.runs);
    w.word64(s.remembered.size());
    for (u64 a : s.remembered) w.word64(a);
    w.word64(s.young_since);
    w.word64(s.live_bytes);
    w.word64(s.shapes.size());
    for (const MdShape &sh : s.shapes) {
        w.word64(sh.id);
        w.word64(sh.tagged);
        w.word64(sh.raw);
        w.word64(sh.kinds.size());
        for (u64 k : sh.kinds) w.word64(k);
        w.text(sh.name);
    }
    w.word64(s.next_shape);
    w.word64(s.hold_word);
    w.byte(uint8_t(s.hold));
    save_hold_ask(w, s.current);
    w.word64(s.holds.size());
    for (const HoldAsk &a : s.holds) save_hold_ask(w, a);
    w.word64(s.held_at_ns);
    w.word64(s.keep_until_ns);
    w.word64(s.gap_until_ns);
    w.word64(s.move_word);
    w.word64(s.pokes.size());
    for (const auto &p : s.pokes) {
        w.word64(p.first);
        w.word64(p.second);
    }
    w.flag(s.no_drain);
    w.word64(s.churns.size());
    for (const Standin::Churn &c : s.churns) {
        w.word64(c.addr);
        w.word64(c.a);
        w.word64(c.b);
        w.word64(c.left);
        w.word64(c.turn);
    }
    for (u64 v : {s.holds_done, s.notices, s.last_notice_kind, s.last_notice_arg, s.last_notice_addr,
                  s.objects_moved, s.grows, s.returns, s.why, s.why_addr, s.wakes})
        w.word64(v);
}

bool restore_standin(Reader &r, Standin &s) {
    uint8_t phase, hold;
    uint64_t n;
    if (!r.byte(phase) || phase > uint8_t(MdPhase::Broken) || !r.word64(s.clock_ns) ||
        !r.word64(s.control_block) || !restore_pair(r, s.control) || !restore_pair(r, s.grant) ||
        !r.word64(n) || !count_fits(r, n, 72))
        return false;
    s.phase = MdPhase(phase);
    s.outbox.assign(size_t(n), Outgoing{});
    for (Outgoing &o : s.outbox) {
        if (!r.word64(o.pair) || !restore_entry(r, o.entry)) return false;
    }
    if (!r.word64(s.next_tag) || !r.word64(s.offer_tag) || !restore_map(r, s.asked) || !r.flag(s.grow_asked) ||
        !r.word64(s.records) || !r.word64(s.cores) || !r.word64(s.core_table) || !r.word64(s.lm1_block) ||
        !r.word64(s.handle_dir) || !r.word64(n) || !count_fits(r, n, 64))
        return false;
    s.core.assign(size_t(n), MdCore{});
    for (MdCore &c : s.core) {
        uint64_t g;
        if (!r.flag(c.pair_known) || !restore_pair(r, c.pair) || !r.word64(g) || !count_fits(r, g, 25))
            return false;
        c.grants.assign(size_t(g), Grant{});
        for (Grant &x : c.grants) {
            if (!r.word64(x.base) || !r.word64(x.limit) || !r.flag(x.left) || !r.word64(x.fill)) return false;
        }
        if (!r.word64(c.next_serial) || !r.word64(c.log_head) || !r.word64(c.tail_seen) ||
            !r.word64(c.region_size) || !r.word64(c.recorded_region) || !r.word64(c.asked_epoch) ||
            !r.flag(c.makes_objects) ||
            !r.flag(c.waited) || !r.flag(c.noticed) || !r.flag(c.region_parked) || !r.word64(c.parked_tag) ||
            !r.word64(c.parked_bytes))
            return false;
    }
    if (!restore_extents(r, s.blocks) || !restore_map(r, s.free) || !restore_map(r, s.old) ||
        !restore_map(r, s.young) || !restore_extents(r, s.runs) || !r.word64(n) || !count_fits(r, n, 8))
        return false;
    s.remembered.clear();
    for (uint64_t i = 0; i < n; ++i) {
        u64 a;
        if (!r.word64(a)) return false;
        s.remembered.insert(a);
    }
    if (!r.word64(s.young_since) || !r.word64(s.live_bytes) || !r.word64(n) || !count_fits(r, n, 40))
        return false;
    s.shapes.assign(size_t(n), MdShape{});
    for (MdShape &sh : s.shapes) {
        uint64_t k;
        if (!r.word64(sh.id) || !r.word64(sh.tagged) || !r.word64(sh.raw) || !r.word64(k) ||
            !count_fits(r, k, 8))
            return false;
        sh.kinds.assign(size_t(k), 0);
        for (u64 &v : sh.kinds) {
            if (!r.word64(v)) return false;
        }
        if (!r.text(sh.name)) return false;
    }
    if (!r.word64(s.next_shape) || !r.word64(s.hold_word) || !r.byte(hold) || hold > uint8_t(HoldPhase::Active) ||
        !restore_hold_ask(r, s.current) || !r.word64(n) || !count_fits(r, n, 33))
        return false;
    s.hold = HoldPhase(hold);
    s.holds.assign(size_t(n), HoldAsk{});
    for (HoldAsk &a : s.holds) {
        if (!restore_hold_ask(r, a)) return false;
    }
    if (!r.word64(s.held_at_ns) || !r.word64(s.keep_until_ns) || !r.word64(s.gap_until_ns) ||
        !r.word64(s.move_word) || !r.word64(n) || !count_fits(r, n, 16))
        return false;
    s.pokes.assign(size_t(n), {0, 0});
    for (auto &p : s.pokes) {
        if (!r.word64(p.first) || !r.word64(p.second)) return false;
    }
    if (!r.flag(s.no_drain) || !r.word64(n) || !count_fits(r, n, 40)) return false;
    s.churns.assign(size_t(n), Standin::Churn{});
    for (Standin::Churn &c : s.churns) {
        if (!r.word64(c.addr) || !r.word64(c.a) || !r.word64(c.b) || !r.word64(c.left) || !r.word64(c.turn))
            return false;
    }
    for (u64 *v : {&s.holds_done, &s.notices, &s.last_notice_kind, &s.last_notice_arg, &s.last_notice_addr,
                   &s.objects_moved, &s.grows, &s.returns, &s.why, &s.why_addr, &s.wakes}) {
        if (!r.word64(*v)) return false;
    }
    return true;
}

}  // namespace

std::vector<uint8_t> save_state(const Model &m) {
    Writer w;
    w.raw(MAGIC, sizeof MAGIC);
    w.word32(VERSION);
    w.text(config_text(m.config));
    const Link &l = m.link;
    w.byte(uint8_t(l.boot));
    w.word64(l.boot_addr);
    w.word64(l.boot_len);
    w.word64(l.boot_arg);
    w.word64(l.fetched);
    w.flag(l.stop_req);
    w.flag(l.told_ready);
    w.flag(l.told_stopped);
    w.word64(l.ranges.size());
    for (const LentRange &range : l.ranges) {
        w.word64(range.base);
        w.word64(range.limit);
        w.flag(range.write);
    }
    w.word64(l.cursor);
    w.word64(l.used);
    w.word64(l.faults);
    w.result(l.first_fault);
    w.word64(l.doorbells_in);
    w.word64(l.doorbells_out);
    w.word64(l.doorbells_dropped);
    for (const Tile &tile : m.tiles) save_tile(w, tile);
    save_standin(w, m.standin);
    return w.take();
}

bool restore_state(Model &m, const uint8_t *data, size_t size) {
    Reader r(data, size);
    char magic[sizeof MAGIC];
    uint32_t version;
    std::string config;
    if (!r.raw(magic, sizeof magic) || std::memcmp(magic, MAGIC, sizeof MAGIC) != 0 ||
        !r.word32(version) || version != VERSION || !r.text(config) ||
        config != config_text(m.config))
        return false;
    // Every size is the configuration's from here on.  The state is read
    // into a model made from the same configuration, and taken whole only
    // once all of it has been read.
    Model fresh(m.config);
    Link &l = fresh.link;
    uint8_t boot;
    uint64_t ranges;
    if (!r.byte(boot) || boot > uint8_t(Boot::Failed) || !r.word64(l.boot_addr) ||
        !r.word64(l.boot_len) || !r.word64(l.boot_arg) || !r.word64(l.fetched) || !r.flag(l.stop_req) ||
        !r.flag(l.told_ready) || !r.flag(l.told_stopped) || !r.word64(ranges) ||
        ranges != l.ranges.size())
        return false;
    l.boot = Boot(boot);
    for (LentRange &range : l.ranges) {
        if (!r.word64(range.base) || !r.word64(range.limit) || !r.flag(range.write)) return false;
    }
    const u64 threads = fresh.config.threads * fresh.tiles.size();
    if (!r.word64(l.cursor) || l.cursor >= threads || !r.word64(l.used) || !r.word64(l.faults) ||
        !r.result(l.first_fault) || !r.word64(l.doorbells_in) || !r.word64(l.doorbells_out) ||
        !r.word64(l.doorbells_dropped))
        return false;
    for (Tile &tile : fresh.tiles) {
        if (!restore_tile(r, tile, fresh.queue_depth)) return false;
    }
    if (!restore_standin(r, fresh.standin) || !r.done()) return false;
    m.tiles = std::move(fresh.tiles);
    m.link = std::move(fresh.link);
    m.standin = std::move(fresh.standin);
    m.trap = {};
    m.stop = {};
    return true;
}

}  // namespace lm1
