// LM-1 chiplet module: the core model's C interface (include/lm1_chiplet.h).
#include <algorithm>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <vector>

#include "model.h"

#ifndef LM1C_SOURCE_DIGEST
#error "the Makefile defines LM1C_SOURCE_DIGEST, the digest of the sources"
#endif

using namespace lm1;

namespace {

// The digest sits behind a marker so that a reader of the library's file
// can find it without loading the library.
const char DIGEST_MARKER[] = "LM1-CHIPLET-SOURCE-DIGEST=" LM1C_SOURCE_DIGEST;
constexpr size_t DIGEST_PREFIX = sizeof("LM1-CHIPLET-SOURCE-DIGEST=") - 1;

Tile *tile_at(const lm1c_model *model, uint32_t tile) {
    if (model == nullptr || tile >= model->tiles.size()) return nullptr;
    return const_cast<Tile *>(&model->tiles[tile]);
}

Thread *thread_at(const lm1c_model *model, uint32_t tile, uint32_t thread) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || thread >= t->threads.size()) return nullptr;
    return &t->threads[thread];
}

bool range_ok(const Tile &tile, uint64_t address, uint64_t size) {
    return address <= tile.mem_size && size <= tile.mem_size - address;
}

void write_error(char *error, size_t size, const std::string &text) {
    if (error == nullptr || size == 0) return;
    size_t n = std::min(size - 1, text.size());
    std::memcpy(error, text.data(), n);
    error[n] = '\0';
}

}  // namespace

extern "C" {

LM1C_API const char *lm1c_source_digest(void) { return DIGEST_MARKER + DIGEST_PREFIX; }

LM1C_API uint32_t lm1c_interface_version(void) { return LM1C_INTERFACE_VERSION; }

LM1C_API lm1c_model *lm1c_create(const char *config, char *error, size_t error_size) {
    Config c;
    std::string why;
    if (!parse_config(config, c, why)) {
        write_error(error, error_size, why);
        return nullptr;
    }
    lm1c_model *model = nullptr;
    try {
        model = new lm1c_model(c);
    } catch (const std::bad_alloc &) {
        write_error(error, error_size, "not enough host memory for the model");
        return nullptr;
    }
    if (!profile_open(*model, why)) {
        delete model;
        write_error(error, error_size, why);
        return nullptr;
    }
    return model;
}

LM1C_API void lm1c_destroy(lm1c_model *model) { delete model; }

LM1C_API void lm1c_set_host(lm1c_model *model, const lm1c_host *host) {
    if (model == nullptr) return;
    model->host = host ? *host : lm1c_host{};
}

LM1C_API uint32_t lm1c_tiles(const lm1c_model *model) {
    return model ? uint32_t(model->tiles.size()) : 0;
}

LM1C_API int lm1c_tile_get(const lm1c_model *model, uint32_t tile, int field, uint64_t *value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    switch (field) {
    case LM1C_TILE_ID: *value = t->id; break;
    case LM1C_TILE_MEM_SIZE: *value = t->mem_size; break;
    case LM1C_TILE_THREADS: *value = t->threads.size(); break;
    case LM1C_TILE_CURRENT_THREAD: *value = t->current; break;
    case LM1C_TILE_NURSERY_BASE: *value = t->nursery_base; break;
    case LM1C_TILE_NURSERY_SIZE: *value = t->nursery_size; break;
    case LM1C_TILE_OLDGEN_BASE: *value = t->oldgen_base; break;
    case LM1C_TILE_OLDGEN_SIZE: *value = t->oldgen_size; break;
    case LM1C_TILE_OLDGEN_PTR: *value = t->oldgen_ptr; break;
    case LM1C_TILE_CARD_SHIFT: *value = t->card_shift; break;
    case LM1C_TILE_CARD_TABLE_BASE: *value = t->card_table_base; break;
    case LM1C_TILE_GEN_BOUNDARY: *value = t->gen_boundary; break;
    case LM1C_TILE_CARD_TABLE_SIZE: *value = t->cards.size(); break;
    case LM1C_TILE_GC_COUNT: *value = t->gc_count; break;
    case LM1C_TILE_INSTRUCTION_COUNT: *value = t->instruction_count; break;
    case LM1C_TILE_NURSERY_CHUNK: *value = t->nursery_chunk; break;
    case LM1C_TILE_NURSERY_FREE: *value = t->nursery_free; break;
    case LM1C_TILE_CYCLES: *value = t->cycles; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_tile_perf(const lm1c_model *model, uint32_t tile, uint32_t index,
                            uint64_t *value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || index >= PERF_COUNTERS) return LM1C_BAD_INDEX;
    *value = t->perf[index];
    return LM1C_OK;
}

LM1C_API int lm1c_profile_symbols(lm1c_model *model, const uint8_t *image, size_t size) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    return model->profiler.load_symbols(image, size) ? LM1C_OK : LM1C_BAD_VALUE;
}

LM1C_API int lm1c_profile_flush(lm1c_model *model) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    model->profile_flush();
    return LM1C_OK;
}

LM1C_API int lm1c_tile_set(lm1c_model *model, uint32_t tile, int field, uint64_t value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    switch (field) {
    case LM1C_TILE_CURRENT_THREAD:
        if (value >= t->threads.size()) return LM1C_BAD_VALUE;
        t->current = size_t(value);
        break;
    case LM1C_TILE_NURSERY_BASE: t->nursery_base = value; break;
    case LM1C_TILE_NURSERY_SIZE: t->nursery_size = value; break;
    case LM1C_TILE_OLDGEN_BASE: t->oldgen_base = value; break;
    case LM1C_TILE_OLDGEN_SIZE: t->oldgen_size = value; break;
    case LM1C_TILE_OLDGEN_PTR: t->oldgen_ptr = value; break;
    case LM1C_TILE_CARD_SHIFT:
        // The shift is a six-bit register; setting it keeps the card table,
        // as only SET_CARD_SHIFT rebuilds it.
        if (value > 63) return LM1C_BAD_VALUE;
        t->card_shift = value;
        break;
    case LM1C_TILE_CARD_TABLE_BASE: t->card_table_base = value; break;
    case LM1C_TILE_GEN_BOUNDARY: t->gen_boundary = value; break;
    case LM1C_TILE_GC_COUNT: t->gc_count = value; break;
    case LM1C_TILE_INSTRUCTION_COUNT: t->instruction_count = value; break;
    case LM1C_TILE_NURSERY_FREE: t->nursery_free = value; break;
    case LM1C_TILE_CYCLES: t->cycles = value; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_thread_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                             int field, uint64_t *value) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr) return LM1C_BAD_INDEX;
    switch (field) {
    case LM1C_THREAD_PC: *value = t->pc; break;
    case LM1C_THREAD_INSTRET: *value = t->instret; break;
    case LM1C_THREAD_HALTED: *value = t->halted; break;
    case LM1C_THREAD_STALLED: *value = t->stalled; break;
    case LM1C_THREAD_STALL_QUEUE: *value = uint64_t(t->stall_queue); break;
    case LM1C_THREAD_TRAP_TABLE_BASE: *value = t->trap_table_base; break;
    case LM1C_THREAD_TRAP_CAUSE: *value = t->trap_cause; break;
    case LM1C_THREAD_TRAP_PC: *value = t->trap_pc; break;
    case LM1C_THREAD_IN_TRAP: *value = t->in_trap; break;
    case LM1C_THREAD_TILE_ID: *value = t->tile_id; break;
    case LM1C_THREAD_THREAD_ID: *value = t->thread_id; break;
    case LM1C_THREAD_SLICE_BASE: *value = t->slice_base; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_thread_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                             int field, uint64_t value) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr) return LM1C_BAD_INDEX;
    switch (field) {
    case LM1C_THREAD_PC: t->pc = value; break;
    case LM1C_THREAD_INSTRET: t->instret = value; break;
    case LM1C_THREAD_HALTED: t->halted = value != 0; break;
    case LM1C_THREAD_STALLED: t->stalled = value != 0; break;
    case LM1C_THREAD_STALL_QUEUE: t->stall_queue = int64_t(value); break;
    case LM1C_THREAD_TRAP_TABLE_BASE: t->trap_table_base = value; break;
    case LM1C_THREAD_TRAP_CAUSE: t->trap_cause = value; break;
    case LM1C_THREAD_TRAP_PC: t->trap_pc = value; break;
    case LM1C_THREAD_IN_TRAP: t->in_trap = value != 0; break;
    case LM1C_THREAD_TILE_ID: t->tile_id = value; break;
    case LM1C_THREAD_THREAD_ID: t->thread_id = value; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_reg_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                          uint32_t reg, uint64_t *value) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr || reg >= REGS) return LM1C_BAD_INDEX;
    *value = t->regs[reg];
    return LM1C_OK;
}

LM1C_API int lm1c_reg_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                          uint32_t reg, uint64_t value) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr || reg >= REGS) return LM1C_BAD_INDEX;
    t->regs[reg] = value;
    return LM1C_OK;
}

LM1C_API int lm1c_regs_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                           uint64_t *values) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr) return LM1C_BAD_INDEX;
    std::memcpy(values, t->regs, sizeof t->regs);
    return LM1C_OK;
}

LM1C_API int lm1c_regs_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                           const uint64_t *values) {
    Thread *t = thread_at(model, tile, thread);
    if (t == nullptr) return LM1C_BAD_INDEX;
    std::memcpy(t->regs, values, sizeof t->regs);
    return LM1C_OK;
}

LM1C_API int lm1c_mem_read(const lm1c_model *model, uint32_t tile, uint64_t address,
                           void *data, uint64_t size) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    if (!range_ok(*t, address, size)) return LM1C_BAD_RANGE;
    std::memcpy(data, reinterpret_cast<const uint8_t *>(t->mem.data()) + address, size_t(size));
    return LM1C_OK;
}

LM1C_API int lm1c_mem_write(lm1c_model *model, uint32_t tile, uint64_t address,
                            const void *data, uint64_t size) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    if (!range_ok(*t, address, size)) return LM1C_BAD_RANGE;
    std::memcpy(reinterpret_cast<uint8_t *>(t->mem.data()) + address, data, size_t(size));
    return LM1C_OK;
}

LM1C_API uint32_t lm1c_template_count(void) { return HEADER_TEMPLATES; }

LM1C_API int lm1c_template_get(const lm1c_model *model, uint32_t tile, uint32_t index,
                               uint64_t *value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || index >= HEADER_TEMPLATES) return LM1C_BAD_INDEX;
    *value = t->templates[index];
    return LM1C_OK;
}

LM1C_API int lm1c_template_set(lm1c_model *model, uint32_t tile, uint32_t index,
                               uint64_t value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || index >= HEADER_TEMPLATES) return LM1C_BAD_INDEX;
    t->templates[index] = value;
    return LM1C_OK;
}

LM1C_API int lm1c_cards_read(const lm1c_model *model, uint32_t tile, uint64_t first,
                             uint8_t *data, uint64_t count) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    if (first > t->cards.size() || count > t->cards.size() - first) return LM1C_BAD_RANGE;
    if (count) std::memcpy(data, t->cards.data() + first, size_t(count));
    return LM1C_OK;
}

LM1C_API int lm1c_ic_install(lm1c_model *model, uint32_t tile, uint64_t callsite,
                             uint64_t shape, uint64_t target) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    t->ic[IcKey{callsite, shape}] = target;
    return LM1C_OK;
}

LM1C_API int lm1c_ic_entries(const lm1c_model *model, uint32_t tile, uint64_t *entries,
                             uint64_t capacity, uint64_t *count) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr) return LM1C_BAD_INDEX;
    std::vector<std::pair<IcKey, uint64_t>> all(t->ic.begin(), t->ic.end());
    std::sort(all.begin(), all.end(), [](const auto &a, const auto &b) {
        return a.first.callsite != b.first.callsite ? a.first.callsite < b.first.callsite
                                                    : a.first.shape < b.first.shape;
    });
    *count = all.size();
    for (size_t i = 0; i < all.size() && i < capacity; ++i) {
        entries[3 * i] = all[i].first.callsite;
        entries[3 * i + 1] = all[i].first.shape;
        entries[3 * i + 2] = all[i].second;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_queue_items(const lm1c_model *model, uint32_t tile, uint32_t queue,
                              uint64_t *values, uint64_t capacity, uint64_t *count) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || queue >= QUEUES) return LM1C_BAD_INDEX;
    const Queue &q = t->queues[queue];
    *count = q.size();
    for (size_t i = 0; i < q.size() && i < capacity; ++i) values[i] = q.at(i);
    return LM1C_OK;
}

LM1C_API int lm1c_queue_push(lm1c_model *model, uint32_t tile, uint32_t queue, uint64_t value) {
    Tile *t = tile_at(model, tile);
    if (t == nullptr || queue >= QUEUES) return LM1C_BAD_INDEX;
    t->queues[queue].push(value);
    return LM1C_OK;
}

LM1C_API int lm1c_run(lm1c_model *model, uint32_t tile, uint64_t max_instructions,
                      lm1c_result *result) {
    if (tile_at(model, tile) == nullptr) return LM1C_BAD_INDEX;
    *result = model->run_thread(tile, max_instructions);
    return LM1C_OK;
}

LM1C_API int lm1c_run_round_robin(lm1c_model *model, uint32_t tile, uint64_t max_instructions,
                                  uint64_t quanta, lm1c_result *result) {
    if (tile_at(model, tile) == nullptr) return LM1C_BAD_INDEX;
    if (quanta == 0) return LM1C_BAD_VALUE;
    *result = model->run_tiles(tile, tile + 1, max_instructions, quanta);
    return LM1C_OK;
}

LM1C_API int lm1c_run_cluster(lm1c_model *model, uint64_t max_instructions, uint64_t quanta,
                              lm1c_result *result) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    if (quanta == 0) return LM1C_BAD_VALUE;
    *result = model->run_tiles(0, model->tiles.size(), max_instructions, quanta);
    return LM1C_OK;
}

LM1C_API int lm1c_collect(lm1c_model *model, uint32_t tile, lm1c_result *result) {
    if (tile_at(model, tile) == nullptr) return LM1C_BAD_INDEX;
    *result = model->collect_now(tile);
    return LM1C_OK;
}

LM1C_API int lm1c_link_get(const lm1c_model *model, int field, uint64_t *value) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    const Link &l = model->link;
    switch (field) {
    case LM1C_LINK_BOOT: *value = uint64_t(l.boot); break;
    case LM1C_LINK_BOOT_ADDR: *value = l.boot_addr; break;
    case LM1C_LINK_BOOT_LEN: *value = l.boot_len; break;
    case LM1C_LINK_BOOT_ARG: *value = l.boot_arg; break;
    case LM1C_LINK_FETCHED: *value = l.fetched; break;
    case LM1C_LINK_STOP_REQ: *value = l.stop_req; break;
    case LM1C_LINK_READY: *value = l.told_ready; break;
    case LM1C_LINK_STOPPED: *value = l.told_stopped; break;
    case LM1C_LINK_RANGES: *value = l.ranges.size(); break;
    case LM1C_LINK_FAULTS: *value = l.faults; break;
    case LM1C_LINK_DOORBELLS_IN: *value = l.doorbells_in; break;
    case LM1C_LINK_DOORBELLS_OUT: *value = l.doorbells_out; break;
    case LM1C_LINK_DOORBELLS_DROPPED: *value = l.doorbells_dropped; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

LM1C_API int lm1c_link_range(const lm1c_model *model, uint32_t index, uint64_t *base,
                             uint64_t *limit, int *write) {
    if (model == nullptr || index >= model->link.ranges.size()) return LM1C_BAD_INDEX;
    const LentRange &r = model->link.ranges[index];
    *base = r.base;
    *limit = r.limit;
    *write = r.write ? 1 : 0;
    return LM1C_OK;
}

LM1C_API int lm1c_link_lend(lm1c_model *model, uint32_t index, uint64_t base, uint64_t limit,
                            int write) {
    if (model == nullptr || index >= model->link.ranges.size()) return LM1C_BAD_INDEX;
    // The lend port takes nothing while LM-1 is held in reset, whose end
    // clears the table.
    if (model->link.boot == Boot::Reset) return LM1C_BAD_VALUE;
    model->link.ranges[index] = LentRange{base, limit, write != 0};
    return LM1C_OK;
}

LM1C_API int lm1c_link_fault(const lm1c_model *model, lm1c_result *first) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    *first = model->link.first_fault;
    return LM1C_OK;
}

}  // extern "C"

// ---- The stand-in's test hooks ----

namespace lm1 {

namespace {

std::mutex registry_lock;
std::vector<lm1c_model *> registry;

}  // namespace

void register_module_model(lm1c_model *model) {
    std::lock_guard<std::mutex> guard(registry_lock);
    registry.push_back(model);
}

void unregister_module_model(lm1c_model *model) {
    std::lock_guard<std::mutex> guard(registry_lock);
    registry.erase(std::remove(registry.begin(), registry.end(), model), registry.end());
}

}  // namespace lm1

extern "C" {

LM1C_API lm1c_model *lm1c_chiplet_find(uint64_t id) {
    std::lock_guard<std::mutex> guard(lm1::registry_lock);
    for (auto it = lm1::registry.rbegin(); it != lm1::registry.rend(); ++it) {
        if ((*it)->config.hook_id == id) return *it;
    }
    return nullptr;
}

LM1C_API int lm1c_md_hold(lm1c_model *model, uint32_t mode, uint64_t keep_ns) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    if (mode > 4) return LM1C_BAD_VALUE;
    HoldAsk a;
    a.mode = mode;
    a.keep_ns = keep_ns;
    model->standin.holds.push_back(a);
    return LM1C_OK;
}

LM1C_API int lm1c_md_write(lm1c_model *model, uint64_t addr, uint64_t value) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    if (addr % 8 != 0) return LM1C_BAD_VALUE;
    model->standin.pokes.emplace_back(addr, value);
    return LM1C_OK;
}

LM1C_API int lm1c_md_churn(lm1c_model *model, uint64_t addr, uint64_t a, uint64_t b,
                           uint64_t times) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    if (addr % 8 != 0) return LM1C_BAD_VALUE;
    Standin::Churn c;
    c.addr = addr;
    c.a = a;
    c.b = b;
    c.left = times;
    model->standin.churns.push_back(c);
    return LM1C_OK;
}

LM1C_API int lm1c_md_drain(lm1c_model *model, int on) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    model->standin.no_drain = on == 0;
    return LM1C_OK;
}

LM1C_API int lm1c_md_stop_at_flip(lm1c_model *model, int on) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    model->standin.stop_at_flip = on != 0;
    return LM1C_OK;
}

LM1C_API int lm1c_md_trace(lm1c_model *model, int on) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    model->standin.tracing = on != 0;
    if (!on) model->standin.trace.clear();
    return LM1C_OK;
}

LM1C_API int lm1c_md_trace_take(lm1c_model *model, uint64_t *pairs, uint64_t capacity,
                                uint64_t *count) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    auto &trace = model->standin.trace;
    *count = trace.size();
    const size_t n = size_t(std::min<uint64_t>(capacity, trace.size()));
    for (size_t i = 0; i < n; ++i) {
        pairs[2 * i] = trace[i].first;
        pairs[2 * i + 1] = trace[i].second;
    }
    trace.erase(trace.begin(), trace.begin() + std::ptrdiff_t(n));
    return LM1C_OK;
}

LM1C_API int lm1c_md_get(const lm1c_model *model, int field, uint64_t *value) {
    if (model == nullptr) return LM1C_BAD_INDEX;
    const Standin &s = model->standin;
    u64 heap = 0, free = 0;
    for (const Extent &b : s.blocks) heap += b.limit - b.base;
    for (const auto &f : s.free) free += f.second - f.first;
    switch (field) {
    case LM1C_MD_PHASE: *value = u64(s.phase); break;
    case LM1C_MD_EPOCH: *value = s.hold_word >> 3; break;
    case LM1C_MD_HOLD: *value = s.hold_word; break;
    case LM1C_MD_MOVE: *value = s.move_word; break;
    case LM1C_MD_LIVE_BYTES: *value = s.live_bytes; break;
    case LM1C_MD_OLD_OBJECTS: *value = s.old.size(); break;
    case LM1C_MD_YOUNG_OBJECTS: *value = s.young.size(); break;
    case LM1C_MD_BLOCKS: *value = s.blocks.size(); break;
    case LM1C_MD_HEAP_BYTES: *value = heap; break;
    case LM1C_MD_FREE_BYTES: *value = free; break;
    case LM1C_MD_NOTICES: *value = s.notices; break;
    case LM1C_MD_NOTICE_KIND: *value = s.last_notice_kind; break;
    case LM1C_MD_NOTICE_ARG: *value = s.last_notice_arg; break;
    case LM1C_MD_NOTICE_ADDR: *value = s.last_notice_addr; break;
    case LM1C_MD_MOVED: *value = s.objects_moved; break;
    case LM1C_MD_GROWS: *value = s.grows; break;
    case LM1C_MD_RETURNS: *value = s.returns; break;
    case LM1C_MD_WHY: *value = s.why; break;
    case LM1C_MD_WHY_ADDR: *value = s.why_addr; break;
    case LM1C_MD_SHAPES: *value = s.shapes.size(); break;
    case LM1C_MD_CLOCK_NS: *value = s.clock_ns; break;
    case LM1C_MD_WAKES: *value = s.wakes; break;
    default: return LM1C_BAD_FIELD;
    }
    return LM1C_OK;
}

}  // extern "C"
