// LM-1 chiplet module: the configuration string.
//
// A model's sizes come from its configuration (decision E of
// design/chiplet-module.md), with the reference emulator's present
// defaults for every key left out.
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "model.h"

namespace lm1 {

namespace {

bool parse_number(const std::string &text, u64 &value) {
    if (text.empty()) return false;
    const char *s = text.c_str();
    char *end = nullptr;
    errno = 0;
    unsigned long long v = std::strtoull(s, &end, 0);
    if (errno != 0 || end == s || text[0] == '-' || text[0] == '+') return false;
    u64 scale = 1;
    if (*end == 'K' || *end == 'k') { scale = u64(1) << 10; ++end; }
    else if (*end == 'M' || *end == 'm') { scale = u64(1) << 20; ++end; }
    else if (*end == 'G' || *end == 'g') { scale = u64(1) << 30; ++end; }
    if (*end != '\0') return false;
    if (v != 0 && scale > ~0ull / v) return false;
    value = v * scale;
    return true;
}

struct Key {
    const char *name;
    u64 Config::*field;
};

const Key KEYS[] = {
    {"tiles", &Config::tiles},
    {"threads", &Config::threads},
    {"mem", &Config::mem},
    {"tile_id", &Config::tile_id},
    {"nursery_base", &Config::nursery_base},
    {"nursery_size", &Config::nursery_size},
    {"oldgen_base", &Config::oldgen_base},
    {"oldgen_size", &Config::oldgen_size},
    {"card_shift", &Config::card_shift},
    {"queue_depth", &Config::queue_depth},
    {"nursery_chunk", &Config::nursery_chunk},
    {"load_base", &Config::load_base},
    {"burst_words", &Config::burst_words},
    {"lend_ranges", &Config::lend_ranges},
    {"quanta", &Config::quanta},
    {"runtime", &Config::runtime},
    {"heap_block", &Config::heap_block},
    {"queue_entries", &Config::queue_entries},
    {"region_bytes", &Config::region_bytes},
    {"window_limit", &Config::window_limit},
    {"hold_limit", &Config::hold_limit},
    {"hold_gap", &Config::hold_gap},
    {"young_trigger", &Config::young_trigger},
    {"unit_ns", &Config::unit_ns},
    {"hook_id", &Config::hook_id},
};

// "BASE:LENGTH", two numbers.
bool parse_range(const std::string &text, u64 &base, u64 &length) {
    size_t colon = text.find(':');
    return colon != std::string::npos && parse_number(text.substr(0, colon), base) &&
           parse_number(text.substr(colon + 1), length);
}

std::string hex(u64 v) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "0x%llx", static_cast<unsigned long long>(v));
    return buf;
}

}  // namespace

std::string config_text(const Config &config) {
    std::string text;
    for (const Key &k : KEYS) {
        if (!text.empty()) text += ',';
        text += k.name;
        text += '=';
        text += hex(config.*(k.field));
    }
    for (const auto &range : config.lend) text += ",lend=" + hex(range.first) + ":" + hex(range.second);
    return text;
}

bool parse_config(const char *text, Config &config, std::string &error) {
    std::string s = text ? text : "";
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t comma = s.find(',', pos);
        if (comma == std::string::npos) comma = s.size();
        std::string item = s.substr(pos, comma - pos);
        pos = comma + 1;
        // Spaces around an item are allowed.
        size_t first = item.find_first_not_of(" \t");
        size_t last = item.find_last_not_of(" \t");
        if (first == std::string::npos) {
            if (comma == s.size()) break;
            continue;
        }
        item = item.substr(first, last - first + 1);
        size_t eq = item.find('=');
        if (eq == std::string::npos) {
            error = "configuration item '" + item + "' is not key=value";
            return false;
        }
        std::string key = item.substr(0, eq), value = item.substr(eq + 1);
        if (key == "lend") {  // one range each time it is named
            u64 base, length;
            if (!parse_range(value, base, length)) {
                error = "configuration key 'lend' takes BASE:LENGTH, not '" + value + "'";
                return false;
            }
            config.lend.emplace_back(base, length);
            continue;
        }
        const Key *found = nullptr;
        for (const Key &k : KEYS) {
            if (key == k.name) found = &k;
        }
        if (found == nullptr) {
            error = "unknown configuration key '" + key + "'";
            return false;
        }
        u64 v;
        if (!parse_number(value, v)) {
            error = "configuration key '" + key + "' takes a number, not '" + value + "'";
            return false;
        }
        config.*(found->field) = v;
    }
    if (config.tiles < 1) { error = "a model has at least one tile"; return false; }
    if (config.threads < 1) { error = "a tile has at least one thread"; return false; }
    if (config.mem % 8 != 0) { error = "a tile's memory is a whole number of 8-byte words"; return false; }
    if (config.card_shift > 63) { error = "card_shift is at most 63"; return false; }
    if (config.queue_depth < 1) { error = "a queue holds at least one message"; return false; }
    if (config.nursery_chunk < 8 || config.nursery_chunk % 8 != 0) {
        error = "nursery_chunk is a multiple of 8 from 8 up";
        return false;
    }
    if (u128(config.nursery_base) + config.nursery_size > ~0ull ||
        u128(config.oldgen_base) + config.oldgen_size > ~0ull ||
        u128(config.tile_id) + config.tiles > u128(INT64_MAX)) {
        error = "a region or the tile IDs run past the 64-bit address space";
        return false;
    }
    // The barrier's one generation boundary starts at the nursery's end and
    // counts addresses at or above it as old, so old-gen lies above.
    if (config.oldgen_base < config.nursery_base + config.nursery_size) {
        error = "old-gen lies below the nursery's end; the barrier's generation "
                "boundary needs old-gen above the nursery";
        return false;
    }
    // The link: a burst carries up to 256 words (its length field holds
    // 0 to 255 beyond the first), the lend port's index has eight bits, and
    // the image lands in whole words.
    if (config.load_base % 8 != 0) { error = "load_base is a multiple of 8"; return false; }
    if (config.burst_words < 1 || config.burst_words > 256) {
        error = "burst_words lies from 1 to 256";
        return false;
    }
    if (config.lend_ranges < 1 || config.lend_ranges > 255) {
        error = "lend_ranges lies from 1 to 255";
        return false;
    }
    if (config.quanta < 1) { error = "a thread's quantum is at least one instruction"; return false; }
    if (config.lend.size() > config.lend_ranges) {
        error = "more ranges to lend than the lend table's lend_ranges entries";
        return false;
    }
    for (const auto &range : config.lend) {
        if (range.second == 0 || range.first > config.mem || range.second > config.mem - range.first) {
            error = "a lent range is empty or lies outside the first tile's memory";
            return false;
        }
    }
    // The stand-in: a queue holds at least one entry, and a block, a region
    // and the three bounds are not empty.
    if (config.runtime > 1) { error = "runtime is 0 or 1"; return false; }
    if (config.queue_entries < 1 || config.queue_entries > 0xFFFFFFFFull) {
        error = "queue_entries lies from 1 to 2^32 - 1";
        return false;
    }
    if (config.heap_block < 64 || config.heap_block % 8 != 0 || config.region_bytes < 64 ||
        config.region_bytes % 8 != 0) {
        error = "heap_block and region_bytes are multiples of 8 from 64 up";
        return false;
    }
    if (config.window_limit == 0 || config.hold_limit == 0 || config.hold_gap == 0) {
        error = "window_limit, hold_limit and hold_gap are nanoseconds above zero";
        return false;
    }
    return true;
}

}  // namespace lm1
