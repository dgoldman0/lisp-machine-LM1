"""LM-1's architectural timing model: the cycles a model charges each
instruction (spec/02-isa.md section 11.6).

The table lives in one file, chiplet/src/timing.h, which the chiplet module
includes; this module reads its LM1_TIMING lines, so the two models charge
the same cycles.
"""

from __future__ import annotations

import re
from pathlib import Path

TABLE = Path(__file__).resolve().parents[2] / "chiplet" / "src" / "timing.h"

CYCLES: dict[str, int] = {
    name: int(cycles)
    for name, cycles in re.findall(r"^LM1_TIMING\((\w+),\s*(\d+)\)", TABLE.read_text(), re.M)
}

ISSUE = CYCLES["ISSUE"]
MUL = CYCLES["MUL"]
DIV = CYCLES["DIV"]
ALLOCV = CYCLES["ALLOCV"]
MEMORY_LATENCY = CYCLES["MEMORY_LATENCY"]
BARRIER_MARK = CYCLES["BARRIER_MARK"]
TRAP_ENTRY = CYCLES["TRAP_ENTRY"]
