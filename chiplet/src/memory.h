// LM-1 chiplet module: access to a tile's memory.
//
// Each helper mirrors a method of the reference's Memory (emu/lm1/memory.py):
// a word access uses the word holding the address and ignores its low
// bits, and a sub-word access shifts within that word.  An address outside
// the tile's memory, including one that a sum carried past 2^64, ends the
// run with a memory fault and returns false.
#ifndef LM1_CHIPLET_MEMORY_H
#define LM1_CHIPLET_MEMORY_H

#include "model.h"

namespace lm1 {

inline bool load_word(Model &m, Tile &tile, u128 addr, u64 &out) {
    size_t i;
    if (!word_index(tile, addr, i)) { m.memory_fault(addr); return false; }
    out = tile.mem[i];
    return true;
}

inline bool store_word(Model &m, Tile &tile, u128 addr, u64 value) {
    size_t i;
    if (!word_index(tile, addr, i)) { m.memory_fault(addr); return false; }
    tile.mem[i] = value;
    return true;
}

// Instructions: the low half of a word at an address with bit 2 clear, the
// high half with it set.
inline bool load_u32(Model &m, Tile &tile, u128 addr, u64 &out) {
    size_t i;
    if (!word_index(tile, addr, i)) { m.memory_fault(addr); return false; }
    out = (u64(addr) & 4) ? tile.mem[i] >> 32 : tile.mem[i] & 0xFFFFFFFFull;
    return true;
}

// A field of WIDTH bits at the address's byte offset within its word.  As
// in the reference, a field that would cross the word's top keeps only the
// bits inside the word.
inline bool load_bits(Model &m, Tile &tile, u128 addr, unsigned width, u64 &out) {
    size_t i;
    if (!word_index(tile, addr, i)) { m.memory_fault(addr); return false; }
    unsigned shift = unsigned(u64(addr) & 7) * 8;
    u64 field = width == 64 ? ~0ull : (1ull << width) - 1;
    out = (tile.mem[i] >> shift) & field;
    return true;
}

inline bool store_bits(Model &m, Tile &tile, u128 addr, unsigned width, u64 value) {
    size_t i;
    if (!word_index(tile, addr, i)) { m.memory_fault(addr); return false; }
    unsigned shift = unsigned(u64(addr) & 7) * 8;
    u64 field = (1ull << width) - 1;
    tile.mem[i] = (tile.mem[i] & ~(field << shift)) | ((value & field) << shift);
    return true;
}

inline bool load_byte(Model &m, Tile &tile, u128 addr, u64 &out) { return load_bits(m, tile, addr, 8, out); }
inline bool store_byte(Model &m, Tile &tile, u128 addr, u64 value) { return store_bits(m, tile, addr, 8, value); }

}  // namespace lm1

#endif
