// LM-1 chiplet module: the configuration string.
//
// A model's sizes come from its configuration (decision E of
// design/chiplet-module.md), with the reference emulator's present
// defaults for every key left out.
#include <cerrno>
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
};

}  // namespace

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
    return true;
}

}  // namespace lm1
