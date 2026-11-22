// LM-1 chiplet module: the minor collection.
//
// On the chip, LM-1's runtime answers TRAP_NURSERY_OVERFLOW with a minor
// collection (spec/02-isa.md section 4.1, spec/05-memory-gc.md section
// 5.1).  Until that runtime runs on the model, the model stands in for it
// as the reference emulator does (Emulator._gc_collect), with the same
// Cheney copy in the same order, so both promote every object to the same
// address: each thread's registers and then its stack, the words of every
// dirty card, and then the copies themselves.
#include "memory.h"
#include "model.h"

namespace lm1 {

namespace {

class Collector {
public:
    Collector(Model &m, Tile &tile) : m_(m), tile_(tile) {}

    // A word that refers into the nursery is copied to old-gen once, and
    // every later reference to it finds the forwarding pointer left in its
    // header.  Other words come back unchanged.
    Flow forward(u64 w, u64 &out) {
        out = w;
        if (!is_any_ref(w)) return Flow::Next;
        u64 addr = ref_address(w);
        if (!(tile_.nursery_base <= addr && u128(addr) < u128(tile_.nursery_base) + tile_.nursery_size))
            return Flow::Next;
        u64 tag = w & 7;
        u64 hdr;
        if (!load_word(m_, tile_, addr, hdr)) return Flow::Stop;
        if ((hdr & 3) == 1 || (hdr & 7) == 3) {  // already forwarded
            out = ref_address(hdr) | tag;
            return Flow::Next;
        }
        if (!is_header(hdr)) return Flow::Next;  // not an object
        u64 total_words = 1 + header_size(hdr);
        u64 total_bytes = total_words * 8;
        u64 new_addr = tile_.oldgen_ptr;
        if (u128(new_addr) + total_bytes > u128(tile_.oldgen_base) + tile_.oldgen_size) {
            m_.stop.stop = LM1C_STOP_OLDGEN;
            m_.stop.address = total_bytes;
            return Flow::Stop;
        }
        tile_.oldgen_ptr += total_bytes;
        for (u64 i = 0; i < total_words; ++i) {
            u64 v;
            if (!load_word(m_, tile_, u128(addr) + i * 8, v)) return Flow::Stop;
            if (!store_word(m_, tile_, u128(new_addr) + i * 8, v)) return Flow::Stop;
        }
        if (!store_word(m_, tile_, addr, make_ref(new_addr, false))) return Flow::Stop;
        out = new_addr | tag;
        return Flow::Next;
    }

    // Forwards the word at ADDR in place.
    Flow forward_at(u128 addr) {
        u64 w, f;
        if (!load_word(m_, tile_, addr, w)) return Flow::Stop;
        if (forward(w, f) != Flow::Next) return Flow::Stop;
        if (!store_word(m_, tile_, addr, f)) return Flow::Stop;
        return Flow::Next;
    }

    Flow run() {
        u64 scan_start = tile_.oldgen_ptr;

        // Roots: each thread's registers, then the stack it grows down to
        // its SP from just below the nursery, the emulator's convention.
        for (Thread &t : tile_.threads) {
            for (unsigned i = 0; i < REGS; ++i) {
                u64 f;
                if (forward(t.regs[i], f) != Flow::Next) return Flow::Stop;
                t.regs[i] = f;
            }
            for (u128 sp = t.regs[REG_SP]; sp < tile_.nursery_base; sp += 8) {
                if (forward_at(sp) != Flow::Next) return Flow::Stop;
            }
        }

        // Dirty cards hold every old-to-young ref the barrier saw.  Each is
        // scanned word by word and then cleared, here and in the table the
        // runtime configured in memory.
        u64 card_size = u64(1) << tile_.card_shift;
        for (size_t idx = 0; idx < tile_.cards.size(); ++idx) {
            if (tile_.cards[idx] != 0xFF) continue;
            u64 a = u64(idx) << tile_.card_shift;
            for (u64 end = a + card_size; a < end; a += 8) {
                if (forward_at(a) != Flow::Next) return Flow::Stop;
            }
            tile_.cards[idx] = 0;
            if (tile_.card_table_base != 0 &&
                !store_byte(m_, tile_, u128(tile_.card_table_base) + idx, 0))
                return Flow::Stop;
        }

        // Cheney scan: the copies' fields, as the copying goes on.
        u128 scan = scan_start;
        while (scan < tile_.oldgen_ptr) {
            u64 w;
            if (!load_word(m_, tile_, scan, w)) return Flow::Stop;
            if (is_header(w)) {
                u64 n = header_size(w);
                for (u64 i = 1; i <= n; ++i) {
                    if (forward_at(scan + i * 8) != Flow::Next) return Flow::Stop;
                }
                scan += (1 + n) * 8;
            } else {
                scan += 8;
            }
        }

        // Every thread's slice of the nursery is empty again.
        for (Thread &t : tile_.threads) t.regs[REG_NP] = t.slice_base;
        tile_.gc_count += 1;
        return Flow::Next;
    }

private:
    Model &m_;
    Tile &tile_;
};

}  // namespace

Flow Model::collect(Tile &tile) {
    return Collector(*this, tile).run();
}

}  // namespace lm1
