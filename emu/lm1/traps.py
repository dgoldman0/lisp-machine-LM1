"""LM-1 trap codes.

Mirrors spec/02-isa.md § 12.
"""

# Family 1 — tagged arith
TRAP_NOT_FIXNUM       = 0x01
TRAP_FIXNUM_OVERFLOW  = 0x02
TRAP_DIVIDE_BY_ZERO   = 0x03
TRAP_TYPE_MISMATCH    = 0x04

# Family 3 — field access
TRAP_NOT_REF          = 0x05
TRAP_NOT_CONS         = 0x06
TRAP_NOT_CLOSURE      = 0x07

# Family 2 — allocation
TRAP_NURSERY_OVERFLOW = 0x10

# Family 4 — dispatch
TRAP_IC_MISS          = 0x20

# Family 6 — concurrency
TRAP_QUEUE_FULL       = 0x30
TRAP_QUEUE_EMPTY      = 0x31

# Family 7 — movement engines
TRAP_ENGINE_BUSY      = 0x40

# Family 3 — barriers
TRAP_BARRIER_OVERFLOW = 0x50

# Family 8 — capability
TRAP_CAPABILITY_VIOLATION = 0x60

# The thread's own: the stack check's code and the user's calls into the
# runtime, both raised by explicit TRAPs, and an encoding no instruction has
TRAP_STACK_OVERFLOW   = 0x70
TRAP_UNIMPLEMENTED    = 0x7E
TRAP_USER             = 0x7F

# Codes below 0x80 vector through the trap table.  TRAP #code with a code
# from 0x80 up is a system trap the core carries out itself, never vectored:
# 0x90 to 0x95 configure the core, and the emulator's bridge to its host,
# console and block device, uses 0x80 to 0x82, 0x84 and 0x9F (execute.py).

# Human-readable names
_NAMES: dict[int, str] = {
    0x01: "TRAP_NOT_FIXNUM",
    0x02: "TRAP_FIXNUM_OVERFLOW",
    0x03: "TRAP_DIVIDE_BY_ZERO",
    0x04: "TRAP_TYPE_MISMATCH",
    0x05: "TRAP_NOT_REF",
    0x06: "TRAP_NOT_CONS",
    0x07: "TRAP_NOT_CLOSURE",
    0x10: "TRAP_NURSERY_OVERFLOW",
    0x20: "TRAP_IC_MISS",
    0x30: "TRAP_QUEUE_FULL",
    0x31: "TRAP_QUEUE_EMPTY",
    0x40: "TRAP_ENGINE_BUSY",
    0x50: "TRAP_BARRIER_OVERFLOW",
    0x60: "TRAP_CAPABILITY_VIOLATION",
    0x70: "TRAP_STACK_OVERFLOW",
    0x7E: "TRAP_UNIMPLEMENTED",
    0x7F: "TRAP_USER",
}


def trap_name(code: int) -> str:
    return _NAMES.get(code, f"TRAP_{code:#04x}")


class LM1Trap(Exception):
    """Raised by instruction execution when a trap fires.

    The emulator's main loop catches this and dispatches to the trap handler
    (or, in Phase 1, just prints a diagnostic and halts).
    """

    def __init__(self, code: int, message: str = ""):
        self.code = code
        self.message = message or trap_name(code)
        # Where ERET resumes when the trap table answers this trap.  A fault
        # leaves it None, so ERET retries the faulting instruction; an
        # explicit TRAP sets the address of the instruction after it.
        self.resume_pc: int | None = None
        super().__init__(f"{self.message} (code={code:#04x})")
