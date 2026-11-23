# The Per-Function Profile

The chiplet module can profile the LM-1 code it runs: every so many cycles it
looks at what each core is doing and charges the time to the function running
and to the functions that called it. The records it writes have the shape of
MegaPad's screen profile of guest words, so the analysis that reads one reads
the other.

## What it samples

The profile counts in the cycles of the models' timing model
(`spec/02-isa.md` § 11.6, `chiplet/src/timing.h`). With `profile=N` in the
module's configuration, each time a core's cycle count passes a multiple of N
the module samples the core's current thread, the one whose instruction has
just run:

- the function that holds its PC;
- its caller, the function that holds its LR, once a call has made a frame
  (FP is not 0);
- each caller further out, the function that holds the LR saved at FP + 8 of
  each frame, following the FP saved at each frame's FP, until a saved LR is
  0, a frame lies outside memory, a saved FP does not lie above the frame
  that saved it, or 64 frames have been followed.

The chain runs innermost first. Each sample charges N cycles: an instruction
whose cycles pass several multiples of N charges the chain once for each.
A chain is charged to the function it starts with (`leaf`), once to every
function it names (`inclusive`), and to itself (`chains`), its first eight
functions joined with ` < `.

Names come from the symbol table the image carries (below). A PC that no
symbol covers, or any PC when the image carries no table, is named by its
address in hexadecimal, `0x1a4`.

The profile reads the model and never writes it. No cycle, counter, register
or memory word changes, so a profiled run computes the same as an unprofiled
one, cycle for cycle, and with `profile=0`, the default, each instruction
costs the module one more comparison. The profile keeps its own state apart
from the model's: a saved state carries none of it and restores into a model
profiled or not, and a reset or a restore starts each core's sampling over
from its cycle count.

## Configuration

| Key | Default | Meaning |
|-----|---------|---------|
| `profile` | 0 | Cycles of the timing model between samples; 0 for no profile |
| `profile_interval` | 100000000 | Cycles of a core that one record covers |
| `profile_file` | none | The file the records are appended to; a profile needs one, and its path holds no comma |

MegaPad passes them in the configuration string of `attach_chiplet` or
`attach_module`, for example
`profile=1000,profile_file=/dev/shm/lm1-profile.jsonl`. LM-1's own tests pass
them through the `config` argument of `lm1.chiplet.Emulator` or the
configuration of `lm1.chiplet.Model`. The module opens the file when it is
made and fails to be made if it cannot. It writes a core's record when the
core's interval ends, when `lm1c_profile_flush` asks, and when the module is
reset, restored or destroyed.

## The symbol table

An image carries its symbol table at its end, after its code and data:

| Field | Size | Contents |
|-------|------|----------|
| Each entry's address | 8 bytes | The function's first instruction |
| Each entry's name length | 8 bytes | The name's length in bytes |
| Each entry's name | the length, padded to whole words with zero bytes | The name, UTF-8 |
| The table's length | 8 bytes | The entries' bytes |
| The magic word | 8 bytes | `LM1SYMS1` |

Every field is little-endian. A chiplet module that boots an image reads the
table from the image's last bytes once the boot fetch has loaded it, before
any LM-1 instruction runs, so the program may reuse those words. A model made
with `lm1c_create` reads it with `lm1c_profile_symbols` from the bytes of an
image (`Emulator.profile_symbols` in the Python binding).

`lm1.image.build` assembles a program into an image that carries its table,
naming the functions a mapping gives, from labels to names, or else every
label except the compiler's own, whose names begin with two underscores.
`Compiler.symbols()` gives the mapping for the functions LM-1's compiler
emitted, its runtime helpers among them, by their Lisp names, and
`lm1.image.compile_program` compiles a Lisp program into an image with its
entry code, `_start` and `_call_main`, and the table. With `symbols=False`
either builds the image without a table.

## The records

Each record is one line of JSON, for one core and one interval:

```json
{"start_ns": 0, "t_ns": 9233, "steps": 9233, "samples": 1319,
 "inclusive": {"_call_main": 9233, "main": 9226, "middle": 9219, "busy": 9212},
 "leaf": {"busy": 9212, "_call_main": 7, "main": 7, "middle": 7},
 "chains": {"busy < middle < main < _call_main": 9212, "_call_main": 7,
            "main < _call_main": 7, "middle < main < _call_main": 7},
 "core": 0}
```

| Key | Meaning |
|-----|---------|
| `start_ns`, `t_ns` | The core's cycle counts at the interval's start and end |
| `steps` | The cycles the interval's samples charged, N for each |
| `samples` | The samples taken |
| `inclusive` | Cycles charged to each function, with everything it called |
| `leaf` | Cycles charged to each function the core was in |
| `chains` | Cycles charged to each chain, the 40 most charged |
| `core` | The core's tile ID |

The keys are those of MegaPad's `guest-words.jsonl` (`shared/screen_profile.py`
in MegaPad), with the same meanings and orders: functions stand where MegaPad
has Forth words, cycles where it has guest steps, and the core's cycle counts
where it has host nanoseconds, which they equal at the Normal operating point
of `spec/04-soc.md` § 11.2, 1.0 GHz. `core` is LM-1's own, since a chiplet has
several cores and each writes its own records. The counts come in descending
order. The 40 kept chains and the 8 functions of each are MegaPad's defaults.
