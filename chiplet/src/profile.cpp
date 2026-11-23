// LM-1 chiplet module: the per-function profile (docs/profile.md).
//
// With profile=N in the configuration, every N cycles of the timing model
// (timing.h) a core's current thread is sampled: the function that holds
// its PC, then the callers its frame chain names, innermost first.  Names
// come from the symbol table an image carries at its end; a PC no symbol
// covers is named by its address.  Each sample charges its N cycles, as
// MegaPad's screen profile charges guest steps, to the function it stopped
// in (leaf), to every function of the chain once (inclusive), and to the
// chain itself.  Every profile_interval cycles a core's samples become one
// record, a line of JSON in the shape of MegaPad's guest-words.jsonl, with
// the core's cycle counts where MegaPad has host nanoseconds: at the Normal
// operating point of spec/04-soc.md section 11.2, 1.0 GHz, a cycle lasts a
// nanosecond.
//
// The profile reads the model and never writes it: no cycle, counter or
// memory word changes, so a run computes the same with it as without, and
// with profile=0 the model does no more than test one number each step.
#include <algorithm>
#include <cstring>
#include <set>
#include <utility>

#include "model.h"

namespace lm1 {

namespace {

// "LM1SYMS1", the last word of an image that carries a symbol table.
constexpr u64 SYMBOLS_MAGIC = 0x31534D5953314D4Cull;
// As MegaPad's screen profile keeps them: the 40 most frequent chains of
// each record, eight functions of each, innermost first.
constexpr size_t CHAINS_KEPT = 40;
constexpr size_t CHAIN_DEPTH = 8;
// The frames a chain follows at most, against a frame chain that loops.
constexpr unsigned FRAMES_FOLLOWED = 64;

u64 le64(const uint8_t *p) {
    u64 v;
    std::memcpy(&v, p, sizeof v);
    return v;
}

std::string hex(u64 v) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "0x%llx", static_cast<unsigned long long>(v));
    return buf;
}

// A word of the tile's memory, read without touching what the model counts.
bool peek(const Tile &tile, u64 address, u64 &out) {
    size_t i;
    if (!word_index(tile, address, i)) return false;
    out = tile.mem[i];
    return true;
}

const std::string &name_of(const std::vector<Symbol> &symbols, u64 pc, std::string &raw) {
    auto above = std::upper_bound(symbols.begin(), symbols.end(), pc,
                                  [](u64 a, const Symbol &s) { return a < s.address; });
    if (above == symbols.begin()) {
        raw = hex(pc);
        return raw;
    }
    return std::prev(above)->name;
}

// The function holding the thread's PC, its caller, which LR names, and
// the callers each frame's saved LR names (the frame a call pushes holds
// the caller's FP at FP and its LR at FP + 8, spec/02-isa.md section 6).
std::vector<std::string> frame_chain(const Profiler &p, const Tile &tile, const Thread &t) {
    std::vector<std::string> chain;
    std::string raw;
    chain.push_back(name_of(p.symbols, t.pc, raw));
    u64 fp = t.regs[REG_FP];
    if (fp == 0) return chain;  // no call has made a frame
    chain.push_back(name_of(p.symbols, t.regs[REG_LR], raw));
    for (unsigned i = 0; i < FRAMES_FOLLOWED; ++i) {
        u64 saved_fp, ret;
        if (!peek(tile, fp, saved_fp) || !peek(tile, fp + 8, ret) || ret == 0) break;
        chain.push_back(name_of(p.symbols, ret, raw));
        // Each caller's frame lies above its callee's on the stack.
        if (saved_fp <= fp) break;
        fp = saved_fp;
    }
    return chain;
}

void json_string(std::string &out, const std::string &s) {
    out += '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += char(c);
        } else if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof buf, "\\u%04x", c);
            out += buf;
        } else {
            out += char(c);
        }
    }
    out += '"';
}

// COUNTS as a JSON object, largest first, at most LIMIT entries.
void json_counts(std::string &out, const std::map<std::string, u64> &counts, size_t limit) {
    std::vector<std::pair<std::string, u64>> sorted(counts.begin(), counts.end());
    std::stable_sort(sorted.begin(), sorted.end(),
                     [](const auto &a, const auto &b) { return a.second > b.second; });
    if (sorted.size() > limit) sorted.resize(limit);
    out += '{';
    for (size_t i = 0; i < sorted.size(); ++i) {
        if (i) out += ", ";
        json_string(out, sorted[i].first);
        out += ": " + std::to_string(sorted[i].second);
    }
    out += '}';
}

// Writes the core's record from its start to END cycles and starts the next.
void write_record(Profiler &p, CoreProfile &c, u64 core_id, u64 end) {
    if (c.samples != 0 && p.out != nullptr) {
        std::string line = "{\"start_ns\": " + std::to_string(c.start) +
                           ", \"t_ns\": " + std::to_string(end) +
                           ", \"steps\": " + std::to_string(c.cycles) +
                           ", \"samples\": " + std::to_string(c.samples) + ", \"inclusive\": ";
        json_counts(line, c.inclusive, c.inclusive.size());
        line += ", \"leaf\": ";
        json_counts(line, c.leaf, c.leaf.size());
        line += ", \"chains\": ";
        json_counts(line, c.chains, CHAINS_KEPT);
        line += ", \"core\": " + std::to_string(core_id) + "}\n";
        std::fputs(line.c_str(), p.out);
        std::fflush(p.out);
    }
    c.start = end;
    c.cycles = c.samples = 0;
    c.inclusive.clear();
    c.leaf.clear();
    c.chains.clear();
}

}  // namespace

Profiler::~Profiler() {
    if (out != nullptr) std::fclose(out);
}

// The table: entries of an address, a name's length in bytes and the name,
// each name padded with zeros to a whole word; then the table's length in
// bytes and SYMBOLS_MAGIC, the image's last two words (emu/lm1/image.py).
bool Profiler::load_symbols(const uint8_t *data, size_t size) {
    if (data == nullptr || size < 16 || le64(data + size - 8) != SYMBOLS_MAGIC) return false;
    const u64 length = le64(data + size - 16);
    if (length > size - 16) return false;
    const uint8_t *p = data + (size - 16 - length), *end = data + size - 16;
    std::vector<Symbol> table;
    while (p < end) {
        if (end - p < 16) return false;
        const u64 address = le64(p), name_len = le64(p + 8);
        p += 16;
        const u64 padded = (name_len + 7) & ~u64(7);
        if (name_len > u64(end - p) || padded > u64(end - p)) return false;
        table.push_back(Symbol{address, std::string(reinterpret_cast<const char *>(p), name_len)});
        p += padded;
    }
    std::stable_sort(table.begin(), table.end(),
                     [](const Symbol &a, const Symbol &b) { return a.address < b.address; });
    symbols = std::move(table);
    return true;
}

void Model::profile_sample(Tile &tile, const Thread &t) {
    CoreProfile &c = profiler.cores[size_t(tile.id - config.tile_id)];
    if (tile.cycles < c.next_sample) return;
    const u64 period = config.profile;
    const u64 samples = (tile.cycles - c.next_sample) / period + 1;
    c.next_sample += samples * period;
    const std::vector<std::string> chain = frame_chain(profiler, tile, t);
    const u64 charged = samples * period;
    c.cycles += charged;
    c.samples += samples;
    c.leaf[chain.front()] += charged;
    for (const std::string &name : std::set<std::string>(chain.begin(), chain.end()))
        c.inclusive[name] += charged;
    std::string joined;
    for (size_t i = 0; i < chain.size() && i < CHAIN_DEPTH; ++i) {
        if (i) joined += " < ";
        joined += chain[i];
    }
    c.chains[joined] += charged;
    if (tile.cycles - c.start >= profiler.interval) write_record(profiler, c, tile.id, tile.cycles);
}

Model::~Model() { profile_flush(); }

void Model::profile_flush() {
    for (size_t i = 0; i < profiler.cores.size() && i < tiles.size(); ++i)
        write_record(profiler, profiler.cores[i], tiles[i].id, tiles[i].cycles);
}

void Model::profile_restart() {
    profile_flush();
    profiler.cores.resize(tiles.size());
    for (size_t i = 0; i < tiles.size(); ++i) {
        CoreProfile &c = profiler.cores[i];
        c.start = tiles[i].cycles;
        c.next_sample = tiles[i].cycles + config.profile;
    }
}

bool profile_open(Model &m, std::string &error) {
    Profiler &p = m.profiler;
    p.interval = m.config.profile_interval;
    p.path = m.config.profile_file;
    if (m.config.profile == 0) return true;
    p.out = std::fopen(p.path.c_str(), "a");
    if (p.out == nullptr) {
        error = "profile_file '" + p.path + "' cannot be opened for appending";
        return false;
    }
    m.profile_restart();
    return true;
}

}  // namespace lm1
