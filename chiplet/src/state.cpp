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
constexpr uint32_t VERSION = 1;

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

private:
    const uint8_t *at_;
    const uint8_t *end_;
};

void save_thread(Writer &w, const Thread &t) {
    for (u64 r : t.regs) w.word64(r);
    w.word64(t.pc);
    w.word64(t.cycle_count);
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
    return r.word64(t.pc) && r.word64(t.cycle_count) && r.flag(t.halted) && r.flag(t.stalled) &&
           r.signed64(t.stall_queue) && r.word64(t.trap_table_base) && r.word64(t.trap_cause) &&
           r.word64(t.trap_pc) && r.flag(t.in_trap) && r.word64(t.tile_id) && r.word64(t.thread_id) &&
           r.word64(t.slice_base);
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
    w.word64(tile.oldgen_base);
    w.word64(tile.oldgen_size);
    w.word64(tile.oldgen_ptr);
    w.word64(tile.card_shift);
    w.word64(tile.card_table_base);
    w.word64(tile.gen_boundary);
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
    if (!r.word64(tile.nursery_base) || !r.word64(tile.nursery_size) || !r.word64(tile.oldgen_base) ||
        !r.word64(tile.oldgen_size) || !r.word64(tile.oldgen_ptr) || !r.word64(tile.card_shift) ||
        tile.card_shift > 63 || !r.word64(tile.card_table_base) || !r.word64(tile.gen_boundary) ||
        !r.word64(cards))
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
    if (!r.done()) return false;
    m.tiles = std::move(fresh.tiles);
    m.link = std::move(fresh.link);
    m.trap = {};
    m.stop = {};
    return true;
}

}  // namespace lm1
