// LM-1's architectural timing model: the cycles the Python emulator and the
// chiplet module charge each instruction, the one table both read.  The
// module includes this file with LM1_TIMING defined; the emulator's
// lm1/timing.py reads its LM1_TIMING lines.  spec/02-isa.md section 11.6
// states the model and how each figure was picked from the spec's ranges.
//
// An instruction costs its issue, or the cycles of its own entry when it
// has one.  One that reads data memory, other than the stack, adds one
// memory latency, however many words it reads.  A store that marks a card
// adds the mark.  An instruction that raises a trap below 0x80 costs its
// issue, any read before the trap, and the trap's entry.
#ifndef LM1_TIMING
#error "timing.h is a table: define LM1_TIMING(name, cycles) before including it"
#endif

LM1_TIMING(ISSUE, 1)           // every instruction the spec gives 1 or no figure
LM1_TIMING(MUL, 3)             // MUL.FIX's 3 to 5, and the raw MUL
LM1_TIMING(DIV, 8)             // DIV.FIX's 8 to 32, and the raw DIV and MOD
LM1_TIMING(ALLOCV, 2)          // ALLOCV's 2
LM1_TIMING(MEMORY_LATENCY, 4)  // tile SRAM's 4 (spec/03-core.md section 4.2)
LM1_TIMING(BARRIER_MARK, 1)    // ST.WB's 1 to 2: the card mark's store
LM1_TIMING(TRAP_ENTRY, 3)      // spec/03-core.md section 6.2
