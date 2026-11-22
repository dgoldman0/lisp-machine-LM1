// LM-1 chiplet module: LM-1's end of the link, as a MegaPad chiplet module.
//
// MegaPad's emulator and simulator attach the model to a chiplet port
// through MegaPad's module interface, shared/accel/chiplet_module.h in the
// workspace's MegaPad checkout, which carries what the link carries
// (MegaPad's docs/chiplet-link.md).  This file is LM-1's end of it, after the
// RTL's endpoint and boot fetch (rtl/doc/08-host-link.md):
//
// - The identity record, readable in reset: "CHIP" over contract version 1,
//   then the device name "LM1", and zeros beyond.
// - Release starts the boot fetch, which reads the image MegaPad names
//   through MegaPad's memory requests, burst_words words to a request as
//   lm1_link_boot reads it, a length rounded up to whole words, and writes
//   each word into LM-1's memory from load_base on.  A response other than
//   OK, or a word outside LM-1's memory, fails the fetch, and the words
//   before it stay written.
// - Once the image is in, the stand-in for LM-1's runtime lends the ranges
//   the configuration names, raises READY, and starts thread 0 of the first
//   tile at load_base with the boot argument, the address of the control
//   block MegaPad lends, in r1.  A length of zero loads no image and starts
//   no thread.
// - A doorbell from MegaPad arrives as fixnum(src) on queue 0 of the first
//   tile, and the link rings the participant named by each fixnum from 0 to
//   255 that LM-1 code sends to queue 1 of that tile: the tile's queues are
//   the external ports the RTL's doorbell ports are meant for.  A doorbell
//   that finds queue 0 full stays pending at MegaPad.
// - MegaPad's writes into memory LM-1 lends are admitted when aligned to a
//   unit and held whole by one range of the lend table with the write
//   right, zero units included, and are refused untouched otherwise.
// - A stop pauses LM-1's threads, the fetch and the doorbells out, and LM-1
//   reports STOPPED in its next step: nothing is ever in flight between
//   calls, since each memory request completes inside the call that makes
//   it.  MegaPad's own writes still land while LM-1 is stopped.
// - Reset returns the model to the state its configuration gives it, every
//   thread halted and nothing lent.
//
// A step's budget counts LM-1's own units: one instruction of any thread,
// or one memory request of the boot fetch.  Threads run in round-robin,
// tile by tile, quanta instructions each, and the scheduler keeps its
// place between steps, so how MegaPad divides the work into steps changes
// nothing in what LM-1 computes.
#include <algorithm>
#include <cstring>
#include <new>
#include <string>

#include "chiplet_module.h"
#include "model.h"

static_assert(MP64_CHIPLET_ABI == 2,
              "the module implements version 2 of MegaPad's chiplet_module.h");
static_assert(MP64_CHIPLET_UNIT_BYTES == 64,
              "MegaPad's writes into lent memory come in 64-byte units");

namespace lm1 {

namespace {

// Contract version 1 below the ASCII tag "CHIP", then "LM1", first
// character in the low byte, as lm1_link's ID_WORD0 and DEVICE_NAME.
constexpr u64 IDENTITY_0 = 0x4348495000000001ull;
constexpr u64 IDENTITY_1 = 0x0000000000314D4Cull;
// The tile whose memory and queues the link reaches, and its queues for
// doorbells in and out.
constexpr size_t LINK_TILE = LM1C_LINK_TILE;
constexpr unsigned QUEUE_IN = LM1C_DOORBELL_IN_QUEUE;
constexpr unsigned QUEUE_OUT = LM1C_DOORBELL_OUT_QUEUE;
// A burst carries at most 256 words: its length field counts 0 to 255
// words beyond the first.
constexpr unsigned MAX_BURST = 256;

Model &model_of(void *self) { return *static_cast<Model *>(self); }

u128 image_words(const Link &l) { return (u128(l.boot_len) + 7) >> 3; }

bool ready_now(const Link &l) { return l.boot == Boot::Running; }

// MegaPad hears of a change in READY or STOPPED.  Between calls nothing of
// LM-1's is in flight, so LM-1 has stopped whenever MegaPad asks it to.
void tell(Model &m) {
    Link &l = m.link;
    const bool ready = ready_now(l), stopped = l.stop_req;
    if (ready == l.told_ready && stopped == l.told_stopped) return;
    l.told_ready = ready;
    l.told_stopped = stopped;
    m.link_host->status(m.link_host->ctx, ready ? 1 : 0, stopped ? 1 : 0);
}

// The image is in: the runtime's stand-in lends the configured ranges, as
// the runtime installs a range at the lend port before it announces it,
// and starts the image.
void start(Model &m) {
    Link &l = m.link;
    l.boot = Boot::Running;
    for (size_t i = 0; i < m.config.lend.size(); ++i) {
        const auto &range = m.config.lend[i];
        l.ranges[i] = LentRange{range.first, range.first + range.second, true};
    }
    if (l.boot_len != 0) {
        Thread &t = m.tiles[LINK_TILE].threads[0];
        t.halted = false;
        t.pc = m.config.load_base;
        t.regs[1] = l.boot_arg;
    }
}

// One request of the boot fetch.
void fetch(Model &m) {
    Link &l = m.link;
    const mp64_chiplet_host &host = *m.link_host;
    const u128 left = image_words(l) - l.fetched;
    const unsigned words = unsigned(std::min<u128>(left, m.config.burst_words));
    u64 data[MAX_BURST];
    unsigned done = 0;
    const int code = host.mem(host.ctx, l.boot_addr + l.fetched * 8, 3, words - 1, 0, data, &done);
    // The words MegaPad answered OK land in order, as lm1_link_boot writes
    // each one as it arrives; a failing response ends the fetch after them.
    const unsigned good = code == MP64_CHIPLET_OK ? words : std::min(done, words);
    Tile &tile = m.tiles[LINK_TILE];
    for (unsigned i = 0; i < good; ++i) {
        size_t index;
        if (!word_index(tile, u128(m.config.load_base) + u128(l.fetched) * 8, index)) {
            l.boot = Boot::Failed;  // a word outside LM-1's memory
            return;
        }
        tile.mem[index] = data[i];
        l.fetched += 1;
    }
    if (code != MP64_CHIPLET_OK) {
        l.boot = Boot::Failed;
        return;
    }
    if (l.fetched == image_words(l)) start(m);
}

// The link rings MegaPad for each participant LM-1 code queued.
void ring_out(Model &m) {
    Link &l = m.link;
    if (l.stop_req || l.boot != Boot::Running) return;
    Queue &q = m.tiles[LINK_TILE].queues[QUEUE_OUT];
    while (q.size() > 0) {
        const u64 v = q.pop();
        if (is_fixnum(v) && untag_fixnum(v) >= 0 && untag_fixnum(v) <= 0xFF) {
            m.link_host->doorbell(m.link_host->ctx, uint8_t(untag_fixnum(v)));
            l.doorbells_out += 1;
        } else {
            l.doorbells_dropped += 1;  // names no participant
        }
    }
}

// A thread that ends on a trap no handler answers, or on a fault, stays
// where it ended and runs no more; the first such end is kept.
void fault(Model &m, size_t tile, size_t thread) {
    Link &l = m.link;
    if (l.faults == 0) {
        l.first_fault = m.stop;
        l.first_fault.tile = uint32_t(tile);
        l.first_fault.thread = uint32_t(thread);
    }
    l.faults += 1;
    m.tiles[tile].threads[thread].halted = true;
}

// LM-1's threads in round-robin, up to BUDGET instructions; returns the
// budget left.
u64 run_threads(Model &m, u64 budget) {
    Link &l = m.link;
    const u64 per_tile = m.config.threads;
    const u64 total = per_tile * m.tiles.size();
    const u64 quanta = m.config.quanta;
    u64 passed = 0;  // threads passed over in a row
    while (budget > 0 && passed < total) {
        const size_t ti = size_t(l.cursor / per_tile), tid = size_t(l.cursor % per_tile);
        Tile &tile = m.tiles[ti];
        Thread &t = tile.threads[tid];
        if (t.halted || !unstall(tile, t)) {
            l.cursor = (l.cursor + 1) % total;
            l.used = 0;
            passed += 1;
            continue;
        }
        passed = 0;
        tile.current = tid;
        while (budget > 0 && l.used < quanta && !t.halted && !t.stalled) {
            m.stop = {};
            const Flow f = m.step(tile, t);
            budget -= 1;
            if (f == Flow::Stop) {
                fault(m, ti, tid);
                break;
            }
            l.used += 1;
        }
        if (l.used >= quanta || t.halted || t.stalled) {
            l.cursor = (l.cursor + 1) % total;
            l.used = 0;
        }
        ring_out(m);
    }
    return budget;
}

bool work_waits(const Model &m) {
    const Link &l = m.link;
    if (l.boot == Boot::Reset) return false;
    if (ready_now(l) != l.told_ready || l.stop_req != l.told_stopped) return true;
    if (l.stop_req) return false;
    if (l.boot == Boot::Fetch) return true;
    if (l.boot != Boot::Running) return false;
    if (m.tiles[LINK_TILE].queues[QUEUE_OUT].size() > 0) return true;
    for (const Tile &tile : m.tiles) {
        for (const Thread &t : tile.threads) {
            if (can_run(tile, t)) return true;
        }
    }
    return false;
}

// ---- The module's functions (chiplet_module.h) ----

void bind(void *self, const mp64_chiplet_host *host) { model_of(self).link_host = host; }

uint64_t identity(void *, uint8_t index) {
    return index == 0 ? IDENTITY_0 : index == 1 ? IDENTITY_1 : 0;
}

void reset(void *self) { model_of(self).reset_chiplet(); }

void release(void *self, uint64_t boot_addr, uint64_t boot_len, uint64_t boot_arg) {
    Model &m = model_of(self);
    // MegaPad releases a chiplet it held in reset; a release in any other
    // state starts it over from reset.
    if (m.link.boot != Boot::Reset) m.reset_chiplet();
    Link &l = m.link;
    l.boot = Boot::Fetch;
    l.boot_addr = boot_addr;
    l.boot_len = boot_len;
    l.boot_arg = boot_arg;
}

void stop(void *self, int stop_req) { model_of(self).link.stop_req = stop_req != 0; }

int doorbell(void *self, uint8_t src) {
    Model &m = model_of(self);
    if (m.link.boot == Boot::Reset) return 0;
    Queue &q = m.tiles[LINK_TILE].queues[QUEUE_IN];
    if (q.full()) return 0;  // it stays pending at MegaPad
    q.push(tag_fixnum(src));
    m.link.doorbells_in += 1;
    return 1;
}

int step(void *self, uint32_t budget) {
    Model &m = model_of(self);
    Link &l = m.link;
    if (l.boot == Boot::Reset || m.link_host == nullptr) return 0;
    tell(m);
    if (l.stop_req) return work_waits(m) ? 1 : 0;
    u64 left = budget;
    while (l.boot == Boot::Fetch && left > 0) {
        if (l.fetched == image_words(l)) {  // a length of zero: no image
            start(m);
            break;
        }
        fetch(m);
        left -= 1;
    }
    tell(m);
    if (l.boot == Boot::Running) {
        left = run_threads(m, left);
        ring_out(m);
    }
    return work_waits(m) ? 1 : 0;
}

int lent_write(void *self, uint64_t addr, const uint8_t *data, unsigned units, unsigned zeros) {
    Model &m = model_of(self);
    constexpr u64 unit = MP64_CHIPLET_UNIT_BYTES;
    if (m.link.boot == Boot::Reset || data == nullptr || units == 0 || addr % unit != 0)
        return MP64_CHIPLET_REFUSED;
    const u128 span = (u128(units) + zeros) * unit;
    const u128 end = u128(addr) + span;
    bool lent = false;
    for (const LentRange &r : m.link.ranges) {
        if (r.write && addr >= r.base && end <= r.limit) lent = true;
    }
    if (!lent) return MP64_CHIPLET_REFUSED;
    // The units, then the zeros, a word at a time in order; a word outside
    // LM-1's memory fails the write after the words before it.
    Tile &tile = m.tiles[LINK_TILE];
    const u128 fits = addr >= tile.mem_size ? 0 : std::min<u128>(span, tile.mem_size - addr);
    if (fits != 0) {
        uint8_t *memory = reinterpret_cast<uint8_t *>(tile.mem.data()) + addr;
        const size_t from_data = size_t(std::min<u128>(fits, u128(units) * unit));
        std::memcpy(memory, data, from_data);
        std::memset(memory + from_data, 0, size_t(fits) - from_data);
    }
    return fits == span ? MP64_CHIPLET_OK : MP64_CHIPLET_MEM_ERROR;
}

size_t save(void *self, uint8_t *buffer, size_t capacity) {
    try {
        const std::vector<uint8_t> state = save_state(model_of(self));
        if (buffer != nullptr && capacity >= state.size())
            std::memcpy(buffer, state.data(), state.size());
        return state.size();
    } catch (const std::bad_alloc &) {
        return 0;
    }
}

int restore(void *self, const uint8_t *buffer, size_t length) {
    Model &m = model_of(self);
    bool ok = false;
    try {
        ok = buffer != nullptr && restore_state(m, buffer, length);
    } catch (const std::bad_alloc &) {
        ok = false;
    }
    if (ok) return 0;
    m.reset_chiplet();
    return 1;
}

void destroy(void *self) { delete static_cast<lm1c_model *>(self); }

}  // namespace

}  // namespace lm1

extern "C" {

LM1C_API uint32_t lm1c_chiplet_abi(void) { return MP64_CHIPLET_ABI; }

// The one function a MegaPad module library exports.  The module's self is
// the model, so LM-1's own C interface (lm1_chiplet.h) reaches it too.
LM1C_API int mp64_chiplet_create(const char *config, mp64_chiplet_module *out) {
    using namespace lm1;
    if (out == nullptr) return 1;
    Config c;
    std::string why;
    if (!parse_config(config, c, why)) return 1;
    lm1c_model *model = nullptr;
    try {
        model = new lm1c_model(c);
        model->reset_chiplet();
    } catch (const std::bad_alloc &) {
        delete model;
        return 1;
    }
    *out = mp64_chiplet_module{};
    out->abi = MP64_CHIPLET_ABI;
    out->self = model;
    out->bind = bind;
    out->identity = identity;
    out->reset = reset;
    out->release = release;
    out->stop = stop;
    out->doorbell = doorbell;
    out->step = step;
    out->lent_write = lent_write;
    out->save = save;
    out->restore = restore;
    out->destroy = destroy;
    return 0;
}

}  // extern "C"
