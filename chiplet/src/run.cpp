// LM-1 chiplet module: the run loops.
//
// The three loops of the reference emulator: Emulator.run runs a tile's
// current thread, Emulator.run_round_robin gives each runnable thread of a
// tile its quanta in turn, and Cluster.run does that over every tile.  A
// stalled thread runs again once the queue its RECV waits on holds a value.
#include "model.h"

namespace lm1 {

namespace {

lm1c_result ended(Model &m, size_t tile, size_t thread, u64 executed) {
    lm1c_result r = m.stop;
    r.tile = uint32_t(tile);
    r.thread = uint32_t(thread);
    r.executed = executed;
    return r;
}

// Whether a stalled thread may run again, which clears its stall.
bool unstall(Tile &tile, Thread &t) {
    if (!t.stalled) return true;
    i64 q = t.stall_queue;
    if (q < 0) q += QUEUES;  // as the reference's list index counts from its end
    if (q < 0 || q >= i64(QUEUES) || tile.queues[q].size() == 0) return false;
    t.stalled = false;
    t.stall_queue = -1;
    return true;
}

}  // namespace

lm1c_result Model::run_thread(size_t tile_index, u64 max_instructions) {
    stop = {};
    Tile &tile = tiles[tile_index];
    Thread &t = tile.threads[tile.current];
    u64 count = 0;
    while (!t.halted && !t.stalled) {
        if (step(tile, t) == Flow::Stop) return ended(*this, tile_index, tile.current, count);
        count += 1;
        if (max_instructions && count >= max_instructions) {
            stop.stop = LM1C_STOP_BUDGET;
            return ended(*this, tile_index, tile.current, count);
        }
    }
    stop.stop = LM1C_STOP_IDLE;
    return ended(*this, tile_index, tile.current, count);
}

lm1c_result Model::run_tiles(size_t first, size_t last, u64 max_instructions, u64 quanta) {
    stop = {};
    u64 count = 0;
    for (;;) {
        bool any_runnable = false;
        for (size_t ti = first; ti < last; ++ti) {
            Tile &tile = tiles[ti];
            for (size_t tid = 0; tid < tile.threads.size(); ++tid) {
                tile.current = tid;
                Thread &t = tile.threads[tid];
                if (t.halted || !unstall(tile, t)) continue;
                any_runnable = true;
                for (u64 q = 0; q < quanta; ++q) {
                    if (t.halted || t.stalled) break;
                    if (step(tile, t) == Flow::Stop) return ended(*this, ti, tid, count);
                    count += 1;
                    if (max_instructions && count >= max_instructions) {
                        stop.stop = LM1C_STOP_BUDGET;
                        return ended(*this, ti, tid, count);
                    }
                }
            }
        }
        if (!any_runnable) {
            stop.stop = LM1C_STOP_IDLE;
            return ended(*this, first, tiles[first].current, count);
        }
    }
}

lm1c_result Model::collect_now(size_t tile_index) {
    stop = {};
    Tile &tile = tiles[tile_index];
    if (collect(tile) == Flow::Stop) return ended(*this, tile_index, tile.current, 0);
    stop.stop = LM1C_STOP_IDLE;
    return ended(*this, tile_index, tile.current, 0);
}

}  // namespace lm1
