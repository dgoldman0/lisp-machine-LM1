// LM-1 chiplet module: the stand-in for LM-1's runtime at the link.
//
// On the chip, LM-1's runtime is LM-1 software, which reads its queues
// from the control block MegaPad names at release, offers its functions,
// and serves them (MegaPad's docs/chiplet-link.md section 8).  Until that
// runtime runs on the model, the module serves the first of them natively:
// managed data, class 1, whose records, requests and rules are MegaPad's
// docs/managed-data.md and LM-1's spec/05-memory-gc.md section 12 (decision A
// of design/chiplet-module.md).  The moving work on the shared heap follows
// the movement engines' four rules of decision G.
//
// Everything here is LM-1's own memory: MegaPad sees only what the stand-in
// writes across the link.
#ifndef LM1_CHIPLET_MANAGED_H
#define LM1_CHIPLET_MANAGED_H

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace lm1 {

using u64 = uint64_t;

struct Model;

// One direction of a queue pair, as LM-1 keeps it: the ring block, which
// its writer owns, with TAIL in its first word; the head word, which its
// reader owns; and LM-1's own count, entries taken from a queue it reads or
// put on one it writes.
struct LinkQueue {
    u64 ring = 0, head_word = 0, capacity = 0;
    u64 count = 0;   // LM-1's count: HEAD of a queue it reads, TAIL of one it writes
    u64 other = 0;   // the other chip's count as LM-1 last read it
};

struct LinkPair {
    bool valid = false;
    LinkQueue in;    // MegaPad's queue to LM-1
    LinkQueue out;   // LM-1's queue to MegaPad
    u64 notify = 0;  // the core LM-1 rings for this pair
};

// An entry of a queue (chiplet-link.md section 8): header, tag, the input
// and output areas, two arguments.
struct QEntry {
    u64 w[8] = {};
};

// A request LM-1 sends, or a reply it owes, waiting for room in the queue
// it goes on.  Which pair: 0 the control pair, 1 the grant pair, 2 + c core
// c's pair.
struct Outgoing {
    u64 pair = 0;
    QEntry entry;
};

struct Extent {
    u64 base = 0, limit = 0;  // the limit is exclusive
};

// A region LM-1 granted a core.  A core leaves a region when it asks for
// the next, having stored FILL first; LM-1 then notes that fill, and the
// region's objects and rest wait for the next hold, which retires it.
struct Grant {
    u64 base = 0, limit = 0;
    bool left = false;
    u64 fill = 0;
};

// What LM-1 keeps about one of MegaPad's cores.
struct MdCore {
    bool pair_known = false;
    LinkPair pair;
    std::vector<Grant> grants;   // regions granted and not yet retired, oldest first
    u64 next_serial = 0;
    u64 log_head = 0;            // LOG_HEAD as LM-1 last wrote it
    u64 tail_seen = 0;           // TAIL as LM-1 last read it
    u64 region_size = 0;         // the size of the core's next region by default
    u64 recorded_region = 0;     // REGION as LM-1 last read it from the core's record
    u64 asked_epoch = ~0ull;     // the epoch of the core's last REGION request
    bool makes_objects = false;
    // In a hold: whether LM-1 found the core's window OPEN and waits for it,
    // and whether it has reported the window overdue.
    bool waited = false, noticed = false;
    // A REGION request waiting for space, by its tag and the bytes asked.
    bool region_parked = false;
    u64 parked_tag = 0, parked_bytes = 0;
};

// A record kind MegaPad registered with SHAPE.
struct MdShape {
    u64 id = 0;
    u64 tagged = 0, raw = 0;
    std::vector<u64> kinds;  // tagged fields' kinds, then raw words'
    std::string name;
};

// A hold asked for: by a core's COLLECT, by LM-1's own policy, or by the
// test hook.  Mode 0 collects the young objects, 1 the whole shared heap, 2
// the whole heap moving every object it can, 3 (the hook's) nothing, and 4
// (the hook's) stops at HELD, as an LM-1 that stopped there would.
struct HoldAsk {
    u64 mode = 0;
    u64 keep_ns = 0;       // the hook keeps the hold ACTIVE this long; ~0 for ever
    bool from_core = false;
    u64 core = 0, tag = 0; // the COLLECT that asked, answered when the hold ends
};

enum class MdPhase : uint8_t {
    Off = 0,        // no control block, or the stand-in is configured off
    Offered = 1,    // the OFFER of MANAGED waits for its grant
    Granted = 2,    // the grant is in; ATTACH has not come yet
    Serving = 3,    // the function serves
    Failed = 4,     // FAILED is set: LM-1 serves the shared heap no more
    Broken = 5      // a queue or a record the stand-in needs broke
};

enum class HoldPhase : uint8_t {
    None = 0,    // no hold
    Held = 1,    // HELD is set; LM-1 waits for windows to close
    Active = 2,  // the work is done; the hook keeps the hold ACTIVE
};

struct Standin {
    MdPhase phase = MdPhase::Off;
    u64 clock_ns = 0;          // LM-1's time, as the steps it was given count it
    u64 control_block = 0;     // CHIPCTRL, from the boot argument
    LinkPair control, grant;
    std::vector<Outgoing> outbox;   // entries waiting for room, in order
    u64 next_tag = 1;
    u64 offer_tag = 0;
    // LM-1's own requests on the grant pair that wait for their replies:
    // tag -> operation.
    std::map<u64, u64> asked;
    bool grow_asked = false;

    // The function's records (managed-data.md section 9).
    u64 records = 0;           // the managed function's control block
    u64 cores = 0, core_table = 0, lm1_block = 0, handle_dir = 0;
    std::vector<MdCore> core;

    // The shared heap.
    std::vector<Extent> blocks;
    std::map<u64, u64> free;       // free extents, base -> limit
    std::map<u64, u64> old;        // old objects, address -> bytes
    std::map<u64, u64> young;      // young objects walked from retired regions
    std::vector<Extent> runs;      // finished parts of regions not yet walked
    std::set<u64> remembered;      // words of old objects that may refer to young ones
    u64 young_since = 0;           // bytes MegaPad made since the last collection
    u64 live_bytes = 0;            // as the last collection found them

    // Shapes MegaPad registered, and the next ID in the lower part of the
    // registry's range.
    std::vector<MdShape> shapes;
    u64 next_shape = 0;

    // Holds.
    u64 hold_word = 0;             // HOLD as LM-1 last wrote it
    HoldPhase hold = HoldPhase::None;
    HoldAsk current;
    std::vector<HoldAsk> holds;    // asked for, in order
    u64 held_at_ns = 0, keep_until_ns = 0, gap_until_ns = 0;
    u64 move_word = 0;

    // The test hooks' writes: a word written as LM-1's store would, and a
    // word into which two values are written in turn.
    std::vector<std::pair<u64, u64>> pokes;
    struct Churn {
        u64 addr = 0, a = 0, b = 0, left = 0, turn = 0;
    };
    std::vector<Churn> churns;

    // The test hook that stops LM-1 draining logs when a core rings, as an
    // LM-1 that stopped serving would leave a full log.
    bool no_drain = false;

    // The test hook's record of every word the stand-in writes across the
    // link, in order, as (address, value), while tracing is on.  Not saved.
    bool tracing = false;
    std::vector<std::pair<u64, u64>> trace;

    // What happened, for tests.
    u64 holds_done = 0, notices = 0, last_notice_kind = 0, last_notice_arg = 0;
    u64 last_notice_addr = 0, objects_moved = 0, grows = 0, returns = 0;
    u64 wakes = 0;                 // cores rung at a hold's end, found WAITING
    u64 why = 0;                   // the first failure's reason (enum MdWhy)
    u64 why_addr = 0;
};

// The stand-in's one entry point each step, with the budget left; returns
// the budget it leaves for LM-1's threads.
u64 standin_step(Model &m, u64 budget);
// Whether the stand-in has work a further step would advance.
bool standin_waits(const Model &m);
// Whether an address lies in the shared heap, as the hardware's shared-heap
// ranges classify it (spec/05-memory-gc.md section 12.1, item 6).
bool in_shared_heap(const Model &m, u64 addr);
// The barrier of LM-1's own stores into a shared object (spec/05-memory-gc.md
// section 4.5): the word may now refer to a young object.
void standin_barrier(Model &m, u64 word_addr, u64 value);

}  // namespace lm1

#endif
