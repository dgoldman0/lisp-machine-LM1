/*
 * LM-1 chiplet module: the core model's C interface.
 *
 * The library models LM-1 as the reference emulator in emu/lm1 runs it:
 * tagged words, each tile's memory, every instruction family the reference
 * implements, traps and the trap table, allocation and the write barrier,
 * the minor collection that stands in for an absent runtime, the hardware
 * threads of a tile in round-robin, and a cluster of tiles that route
 * messages to one another.  design/chiplet-module.md is the plan; this
 * interface serves LM-1's own tests, and MegaPad's chiplet interface joins
 * it in the same library in stage 2.
 *
 * A model calls its host only through the callbacks of lm1c_host, and only
 * from inside a run, starts no thread and reads no clock, so its results
 * depend only on its state and the calls made to it.  Models share no
 * state, so separate models may be driven from separate threads.
 *
 * Every function that takes a tile or thread index returns LM1C_BAD_INDEX
 * for one the model lacks, and LM1C_OK on success.
 */
#ifndef LM1_CHIPLET_H
#define LM1_CHIPLET_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define LM1C_API __declspec(dllexport)
#else
#define LM1C_API __attribute__((visibility("default")))
#endif

/* Raised when a function of this interface changes or is added. */
#define LM1C_INTERFACE_VERSION 1

#define LM1C_OK          0
#define LM1C_BAD_INDEX  (-1)  /* no such tile, thread, register or entry */
#define LM1C_BAD_FIELD  (-2)  /* no such field, or a field that cannot be set */
#define LM1C_BAD_RANGE  (-3)  /* an address range outside the tile's memory */
#define LM1C_BAD_VALUE  (-4)  /* an argument outside what the call accepts */

typedef struct lm1c_model lm1c_model;

/* Why a run returned. */
enum lm1c_stop {
    LM1C_STOP_BUDGET = 0,  /* it executed the instructions it was given */
    LM1C_STOP_IDLE   = 1,  /* every thread it runs has halted or stalled */
    LM1C_STOP_TRAP   = 2,  /* a trap that no handler answers stands */
    LM1C_STOP_MEMORY = 3,  /* an access fell outside the tile's memory */
    LM1C_STOP_OLDGEN = 4,  /* the collector found old-gen too full to copy */
    LM1C_STOP_HOST   = 5   /* a host callback reported a failure */
};

typedef struct lm1c_result {
    int32_t  stop;      /* enum lm1c_stop */
    uint32_t trap;      /* LM1C_STOP_TRAP: the trap's code */
    uint64_t address;   /* LM1C_STOP_MEMORY: the address outside memory;
                           LM1C_STOP_OLDGEN: the bytes the copy needed */
    uint64_t executed;  /* instructions this call counted */
    uint32_t tile;      /* the tile and thread whose instruction ended it */
    uint32_t thread;
} lm1c_result;

/* What a host callback returns. */
#define LM1C_HOST_OK        0
#define LM1C_HOST_IO_ERROR  1  /* the block device failed: r0 gets fixnum -1 */
#define LM1C_HOST_FAIL      2  /* the host failed: the run stops, LM1C_STOP_HOST */

/*
 * The emulator-level console and block traps reach the host through these
 * callbacks.  They are not part of LM-1's ISA: on the chip, I/O travels
 * the link to MegaPad.  A null put_char or get_char fails its trap with
 * LM1C_STOP_HOST; a null block_io means no block device is attached, and
 * TRAP 0x82 answers fixnum -1 in r0.
 */
typedef struct lm1c_host {
    void *context;
    /* TRAP 0x80 and 0x9F on tile TILE (an index): write one character, a
       code point up to 0x1FFFFF. */
    int (*put_char)(void *context, uint32_t tile, uint32_t code_point);
    /* TRAP 0x81: read one character into *code_point, -1 at end of input. */
    int (*get_char)(void *context, uint32_t tile, int64_t *code_point);
    /* TRAP 0x82: read (write == 0) or write one block of size bytes.  A
       read fills data in full, padding past the device's end with zeros. */
    int (*block_io)(void *context, uint32_t tile, int write, uint64_t block,
                    uint8_t *data, uint64_t size);
} lm1c_host;

/* Fields of a hardware thread, read and written as 64-bit values. */
enum lm1c_thread_field {
    LM1C_THREAD_PC = 0,
    LM1C_THREAD_CYCLE_COUNT,
    LM1C_THREAD_HALTED,          /* 0 or 1 */
    LM1C_THREAD_STALLED,         /* 0 or 1 */
    LM1C_THREAD_STALL_QUEUE,     /* the queue a stalled RECV waits on, or -1 */
    LM1C_THREAD_TRAP_TABLE_BASE,
    LM1C_THREAD_TRAP_CAUSE,
    LM1C_THREAD_TRAP_PC,
    LM1C_THREAD_IN_TRAP,         /* 0 or 1 */
    LM1C_THREAD_TILE_ID,
    LM1C_THREAD_THREAD_ID,
    LM1C_THREAD_SLICE_BASE,      /* where its slice of the nursery begins; read only */
    LM1C_THREAD_FIELDS
};

/* Fields of a tile. */
enum lm1c_tile_field {
    LM1C_TILE_ID = 0,            /* read only */
    LM1C_TILE_MEM_SIZE,          /* bytes; read only */
    LM1C_TILE_THREADS,           /* read only */
    LM1C_TILE_CURRENT_THREAD,    /* the thread lm1c_run runs */
    LM1C_TILE_NURSERY_BASE,
    LM1C_TILE_NURSERY_SIZE,
    LM1C_TILE_OLDGEN_BASE,
    LM1C_TILE_OLDGEN_SIZE,
    LM1C_TILE_OLDGEN_PTR,
    LM1C_TILE_CARD_SHIFT,
    LM1C_TILE_CARD_TABLE_BASE,   /* 0: the card table lives only in the model */
    LM1C_TILE_GEN_BOUNDARY,
    LM1C_TILE_CARD_TABLE_SIZE,   /* entries; read only */
    LM1C_TILE_GC_COUNT,          /* minor collections so far */
    LM1C_TILE_INSTRUCTION_COUNT,
    LM1C_TILE_FIELDS
};

/* The digest of the sources this library was built from, 64 hex digits:
   the SHA-256 of `sha256sum` over the Makefile, the export list, the
   headers and the sources. */
LM1C_API const char *lm1c_source_digest(void);
LM1C_API uint32_t lm1c_interface_version(void);

/*
 * Make a model from a configuration string of comma-separated key=value
 * pairs, numbers in decimal or 0x hex with an optional K, M or G suffix
 * for KiB, MiB or GiB.  Keys and their defaults, the reference's own:
 *   tiles=1 threads=1 mem=4M tile_id=0 nursery_base=0x30000
 *   nursery_size=0x10000 oldgen_base=0x80000 oldgen_size=0x40000
 *   card_shift=6 queue_depth=512
 * threads counts a tile's hardware threads, mem the bytes of each tile's
 * memory (a multiple of 8), tile_id the first tile's ID.  On failure it
 * returns NULL and, given room, writes the reason to error.
 */
LM1C_API lm1c_model *lm1c_create(const char *config, char *error, size_t error_size);
LM1C_API void lm1c_destroy(lm1c_model *model);

/* Copies *host; NULL removes the console and the block device. */
LM1C_API void lm1c_set_host(lm1c_model *model, const lm1c_host *host);

LM1C_API uint32_t lm1c_tiles(const lm1c_model *model);

LM1C_API int lm1c_tile_get(const lm1c_model *model, uint32_t tile, int field,
                           uint64_t *value);
LM1C_API int lm1c_tile_set(lm1c_model *model, uint32_t tile, int field,
                           uint64_t value);
LM1C_API int lm1c_thread_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                             int field, uint64_t *value);
LM1C_API int lm1c_thread_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                             int field, uint64_t value);
LM1C_API int lm1c_reg_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                          uint32_t reg, uint64_t *value);
LM1C_API int lm1c_reg_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                          uint32_t reg, uint64_t value);
/* All 32 registers at once. */
LM1C_API int lm1c_regs_get(const lm1c_model *model, uint32_t tile, uint32_t thread,
                           uint64_t *values);
LM1C_API int lm1c_regs_set(lm1c_model *model, uint32_t tile, uint32_t thread,
                           const uint64_t *values);

/* Bytes of a tile's memory, little-endian within each word. */
LM1C_API int lm1c_mem_read(const lm1c_model *model, uint32_t tile, uint64_t address,
                           void *data, uint64_t size);
LM1C_API int lm1c_mem_write(lm1c_model *model, uint32_t tile, uint64_t address,
                            const void *data, uint64_t size);

/* A tile's header-template table, which its threads share. */
LM1C_API uint32_t lm1c_template_count(void);
LM1C_API int lm1c_template_get(const lm1c_model *model, uint32_t tile, uint32_t index,
                               uint64_t *value);
LM1C_API int lm1c_template_set(lm1c_model *model, uint32_t tile, uint32_t index,
                               uint64_t value);

/* COUNT entries of a tile's card table from entry FIRST, one byte each. */
LM1C_API int lm1c_cards_read(const lm1c_model *model, uint32_t tile, uint64_t first,
                             uint8_t *data, uint64_t count);

/* A tile's inline-cache table, keyed by call site and shape. */
LM1C_API int lm1c_ic_install(lm1c_model *model, uint32_t tile, uint64_t callsite,
                             uint64_t shape, uint64_t target);
/* Sets *count to the entries and copies at most CAPACITY of them to
   entries as (callsite, shape, target) triples, ordered by key. */
LM1C_API int lm1c_ic_entries(const lm1c_model *model, uint32_t tile, uint64_t *entries,
                             uint64_t capacity, uint64_t *count);

/* A tile's four message queues.  lm1c_queue_items copies at most CAPACITY
   values, oldest first, and sets *count to the queue's length.  A push
   onto a full queue drops its oldest value. */
LM1C_API int lm1c_queue_items(const lm1c_model *model, uint32_t tile, uint32_t queue,
                              uint64_t *values, uint64_t capacity, uint64_t *count);
LM1C_API int lm1c_queue_push(lm1c_model *model, uint32_t tile, uint32_t queue,
                             uint64_t value);

/*
 * Runs.  MAX_INSTRUCTIONS of 0 runs without a budget.  lm1c_run runs the
 * tile's current thread until it halts or stalls; lm1c_run_round_robin
 * gives each of a tile's runnable threads QUANTA instructions in turn, and
 * lm1c_run_cluster does the same over every tile, until none can run.
 * lm1c_collect runs a minor collection of the tile now.
 */
LM1C_API int lm1c_run(lm1c_model *model, uint32_t tile, uint64_t max_instructions,
                      lm1c_result *result);
LM1C_API int lm1c_run_round_robin(lm1c_model *model, uint32_t tile,
                                  uint64_t max_instructions, uint64_t quanta,
                                  lm1c_result *result);
LM1C_API int lm1c_run_cluster(lm1c_model *model, uint64_t max_instructions,
                              uint64_t quanta, lm1c_result *result);
LM1C_API int lm1c_collect(lm1c_model *model, uint32_t tile, lm1c_result *result);

#ifdef __cplusplus
}
#endif

#endif /* LM1_CHIPLET_H */
