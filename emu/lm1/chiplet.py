"""LM-1's chiplet module from Python: the C++ core model behind ctypes.

The chiplet module (chiplet/ at the repository's root, planned in
design/chiplet-module.md) is one shared library.  This module finds it,
builds it again whenever its sources differ from the ones it was built
from, and offers three ways in:

- ``Model``, a thin handle on the library's C interface
  (chiplet/include/lm1_chiplet.h);
- ``Emulator`` and ``Cluster``, which offer the part of the reference
  emulator's interface (``lm1.execute``) that LM-1's tests use, backed by
  the model, so that the same test programs run on both;
- ``Module``, the library as MegaPad's port bridge drives it: a chiplet
  module made by ``mp64_chiplet_create``, whose host services are Python
  callables, for LM-1's own tests of its end of the link.

The library builds against MegaPad's ``shared/accel/chiplet_module.h``
(decision C of the plan), in the MegaPad checkout that ``MEGAPAD_ROOT``
names, or in the one beside this repository's.  No test runs against a
library older than its sources or that header.  The library carries the
digest of the files it was built from, and before loading it this module
computes the same digest from the files in the checkouts; when the two
differ it runs make, under a lock, and checks again.
"""

from __future__ import annotations

import ctypes
import fcntl
import hashlib
import os
import subprocess
import sys
import threading
import weakref
from pathlib import Path
from typing import TextIO

from .decode import PERF_COUNTERS
from .traps import LM1Trap
from .word import WORD_MASK

REPO = Path(__file__).resolve().parents[2]
CHIPLET_DIR = REPO / "chiplet"
LIBRARY = CHIPLET_DIR / "build" / "lm1_chiplet.so"
_DIGEST_MARKER = b"LM1-CHIPLET-SOURCE-DIGEST="
# MegaPad's module interface, by its path inside a MegaPad checkout.
MP64_HEADER = "shared/accel/chiplet_module.h"

# Results of a run (enum lm1c_stop).
STOP_BUDGET, STOP_IDLE, STOP_TRAP, STOP_MEMORY, STOP_OLDGEN, STOP_HOST = range(6)
HOST_OK, HOST_IO_ERROR, HOST_FAIL = 0, 1, 2

# enum lm1c_thread_field
(THREAD_PC, THREAD_INSTRET, THREAD_HALTED, THREAD_STALLED, THREAD_STALL_QUEUE,
 THREAD_TRAP_TABLE_BASE, THREAD_TRAP_CAUSE, THREAD_TRAP_PC, THREAD_IN_TRAP,
 THREAD_TILE_ID, THREAD_THREAD_ID, THREAD_SLICE_BASE) = range(12)
# enum lm1c_tile_field
(TILE_ID, TILE_MEM_SIZE, TILE_THREADS, TILE_CURRENT_THREAD, TILE_NURSERY_BASE,
 TILE_NURSERY_SIZE, TILE_OLDGEN_BASE, TILE_OLDGEN_SIZE, TILE_OLDGEN_PTR,
 TILE_CARD_SHIFT, TILE_CARD_TABLE_BASE, TILE_GEN_BOUNDARY, TILE_CARD_TABLE_SIZE,
 TILE_GC_COUNT, TILE_INSTRUCTION_COUNT, TILE_NURSERY_CHUNK, TILE_NURSERY_FREE,
 TILE_CYCLES) = range(18)

BLOCK_SIZE = 4096

# Managed data's stand-in (enum lm1c_md_field), and the hook's mode that
# holds without collecting, and keeps a hold for ever.
(MD_PHASE, MD_EPOCH, MD_HOLD, MD_MOVE, MD_LIVE_BYTES, MD_OLD_OBJECTS, MD_YOUNG_OBJECTS,
 MD_BLOCKS, MD_HEAP_BYTES, MD_FREE_BYTES, MD_NOTICES, MD_NOTICE_KIND, MD_NOTICE_ARG,
 MD_NOTICE_ADDR, MD_MOVED, MD_GROWS, MD_RETURNS, MD_WHY, MD_WHY_ADDR, MD_SHAPES,
 MD_CLOCK_NS, MD_WAKES) = range(22)
MD_HOLD_ONLY = 3
MD_HOLD_STALLS = 4
MD_FOR_EVER = (1 << 64) - 1


class ChipletBuildError(RuntimeError):
    """The library could not be built from the checkout's sources."""


# ---------------------------------------------------------------------------
# Finding, building and loading the library
# ---------------------------------------------------------------------------

def megapad_root(root: str | os.PathLike | None = None) -> Path:
    """The MegaPad checkout whose chiplet_module.h the library builds
    against: ROOT when given, else MEGAPAD_ROOT, as the link test bench
    names it, else the checkout beside this repository's."""
    if root is not None:
        return Path(root)
    named = os.environ.get("MEGAPAD_ROOT")
    return Path(named) if named else REPO.parent / "megapad"


def source_files() -> list[str]:
    """The files of this repository the library's digest covers, as the
    Makefile's DIGESTED lists them: the Makefile, the export list, then the
    headers and sources sorted together."""
    names: set[str] = set()
    for pattern in ("include/*.h", "src/*.h", "src/*.cpp"):
        for path in CHIPLET_DIR.glob(pattern):
            if not path.name.startswith("."):   # make's wildcard skips them
                names.add(path.relative_to(CHIPLET_DIR).as_posix())
    return ["Makefile", "exports.map"] + sorted(names)


def source_digest(megapad: str | os.PathLike | None = None) -> str:
    """The SHA-256 of what `sha256sum` prints for the source files and then
    for MegaPad's header, named by its path inside MegaPad."""
    header = megapad_root(megapad) / MP64_HEADER
    try:
        header_bytes = header.read_bytes()
    except OSError:
        raise ChipletBuildError(
            "the chiplet module builds against MegaPad's %s, which %s lacks; set "
            "MEGAPAD_ROOT to a MegaPad checkout" % (MP64_HEADER, header.parents[2])) from None
    listing = "".join(
        "%s  %s\n" % (hashlib.sha256((CHIPLET_DIR / name).read_bytes()).hexdigest(), name)
        for name in source_files())
    listing += "%s  %s\n" % (hashlib.sha256(header_bytes).hexdigest(), MP64_HEADER)
    return hashlib.sha256(listing.encode()).hexdigest()


def built_digest(path: Path = LIBRARY) -> str | None:
    """The digest a built library carries, read from its file, or None."""
    try:
        data = path.read_bytes()
    except OSError:
        return None
    at = data.find(_DIGEST_MARKER)
    if at < 0:
        return None
    at += len(_DIGEST_MARKER)
    return data[at:at + 64].decode("ascii", "replace")


def ensure_library(megapad: str | os.PathLike | None = None) -> Path:
    """The library, built from the checkout's sources and from the header
    of the MegaPad checkout that megapad_root(MEGAPAD) names, as they are
    now.  A test that attaches the library elsewhere calls this first, so
    that the files the library comes from are among the files it reads."""
    root = megapad_root(megapad)
    wanted = source_digest(root)
    if built_digest() == wanted:
        return LIBRARY
    build = LIBRARY.parent
    build.mkdir(parents=True, exist_ok=True)
    with open(build / ".lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)   # one build at a time
        for force in ([], ["-B"]):
            if built_digest() == wanted:
                break
            # make goes by times; -B rebuilds when the times mislead it.
            subprocess.run(["make", "--no-print-directory", "-C", str(CHIPLET_DIR), *force],
                           check=True, stdout=subprocess.DEVNULL,
                           env={**os.environ, "MEGAPAD_ROOT": str(root)})
    found = built_digest()
    if found != wanted:
        raise ChipletBuildError(
            "%s carries digest %s, but its sources' digest is %s" % (LIBRARY, found, wanted))
    return LIBRARY


class _Result(ctypes.Structure):
    _fields_ = [("stop", ctypes.c_int32), ("trap", ctypes.c_uint32),
                ("address", ctypes.c_uint64), ("executed", ctypes.c_uint64),
                ("tile", ctypes.c_uint32), ("thread", ctypes.c_uint32)]


_PUT_CHAR = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32)
_GET_CHAR = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32,
                             ctypes.POINTER(ctypes.c_int64))
_BLOCK_IO = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int,
                             ctypes.c_uint64, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint64)


class _Host(ctypes.Structure):
    _fields_ = [("context", ctypes.c_void_p), ("put_char", _PUT_CHAR),
                ("get_char", _GET_CHAR), ("block_io", _BLOCK_IO)]


# MegaPad's chiplet module interface as ctypes declares it, the two
# structures of chiplet_module.h for the version CHIPLET_ABI names.  The
# library reports the version it was built against, and library() refuses
# one built for another, so the two cannot drift apart unnoticed.
CHIPLET_ABI = 2
UNIT_BYTES = 64
LINK_OK, LINK_REFUSED, LINK_MEM_ERROR, LINK_CANCELLED = range(4)

_MEM = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint,
                        ctypes.c_uint, ctypes.c_int, ctypes.POINTER(ctypes.c_uint64),
                        ctypes.POINTER(ctypes.c_uint))
_DOORBELL = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint8)
_STATUS = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int)


class _ChipletHost(ctypes.Structure):
    _fields_ = [("ctx", ctypes.c_void_p), ("mem", _MEM), ("doorbell", _DOORBELL),
                ("status", _STATUS)]


class _ChipletModule(ctypes.Structure):
    _fields_ = [
        ("abi", ctypes.c_uint32),
        ("self", ctypes.c_void_p),
        ("bind", ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(_ChipletHost))),
        ("identity", ctypes.CFUNCTYPE(ctypes.c_uint64, ctypes.c_void_p, ctypes.c_uint8)),
        ("reset", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
        ("release", ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint64,
                                     ctypes.c_uint64)),
        ("stop", ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)),
        ("doorbell", ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint8)),
        ("step", ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32)),
        ("lent_write", ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint64,
                                        ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint,
                                        ctypes.c_uint)),
        ("save", ctypes.CFUNCTYPE(ctypes.c_size_t, ctypes.c_void_p,
                                  ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)),
        ("restore", ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p,
                                     ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)),
        ("destroy", ctypes.CFUNCTYPE(None, ctypes.c_void_p)),
    ]


# LM-1's end of the link (lm1_chiplet.h): its tile and doorbell queues,
# enum lm1c_boot and enum lm1c_link_field.
LINK_TILE, DOORBELL_IN_QUEUE, DOORBELL_OUT_QUEUE = 0, 0, 1
BOOT_RESET, BOOT_FETCH, BOOT_RUNNING, BOOT_FAILED = range(4)
(LINK_BOOT, LINK_BOOT_ADDR, LINK_BOOT_LEN, LINK_BOOT_ARG, LINK_FETCHED, LINK_STOP_REQ,
 LINK_READY, LINK_STOPPED, LINK_RANGES, LINK_FAULTS, LINK_DOORBELLS_IN, LINK_DOORBELLS_OUT,
 LINK_DOORBELLS_DROPPED) = range(13)


_lib = None
_lib_lock = threading.Lock()


def library() -> ctypes.CDLL:
    """The loaded library, built first if its sources changed."""
    global _lib
    with _lib_lock:
        if _lib is None:
            lib = ctypes.CDLL(str(ensure_library()))
            _declare(lib)
            loaded = lib.lm1c_source_digest().decode("ascii")
            if loaded != source_digest():
                raise ChipletBuildError("the loaded library's digest %s is not its "
                                        "sources' %s" % (loaded, source_digest()))
            if lib.lm1c_chiplet_abi() != CHIPLET_ABI:
                raise ChipletBuildError(
                    "the library implements version %d of MegaPad's chiplet_module.h, and "
                    "this binding mirrors version %d" % (lib.lm1c_chiplet_abi(), CHIPLET_ABI))
            _lib = lib
        return _lib


def _declare(lib: ctypes.CDLL) -> None:
    c = ctypes
    u32, u64, i32 = c.c_uint32, c.c_uint64, c.c_int
    p = c.c_void_p
    pu64 = c.POINTER(c.c_uint64)
    sig = {
        "lm1c_source_digest": (c.c_char_p, []),
        "lm1c_interface_version": (u32, []),
        "lm1c_create": (p, [c.c_char_p, c.c_char_p, c.c_size_t]),
        "lm1c_destroy": (None, [p]),
        "lm1c_set_host": (None, [p, c.POINTER(_Host)]),
        "lm1c_tiles": (u32, [p]),
        "lm1c_tile_get": (i32, [p, u32, i32, pu64]),
        "lm1c_tile_set": (i32, [p, u32, i32, u64]),
        "lm1c_tile_perf": (i32, [p, u32, u32, pu64]),
        "lm1c_thread_get": (i32, [p, u32, u32, i32, pu64]),
        "lm1c_thread_set": (i32, [p, u32, u32, i32, u64]),
        "lm1c_reg_get": (i32, [p, u32, u32, u32, pu64]),
        "lm1c_reg_set": (i32, [p, u32, u32, u32, u64]),
        "lm1c_regs_get": (i32, [p, u32, u32, pu64]),
        "lm1c_regs_set": (i32, [p, u32, u32, pu64]),
        "lm1c_mem_read": (i32, [p, u32, u64, p, u64]),
        "lm1c_mem_write": (i32, [p, u32, u64, p, u64]),
        "lm1c_template_count": (u32, []),
        "lm1c_template_get": (i32, [p, u32, u32, pu64]),
        "lm1c_template_set": (i32, [p, u32, u32, u64]),
        "lm1c_cards_read": (i32, [p, u32, u64, p, u64]),
        "lm1c_ic_install": (i32, [p, u32, u64, u64, u64]),
        "lm1c_ic_entries": (i32, [p, u32, pu64, u64, pu64]),
        "lm1c_queue_items": (i32, [p, u32, u32, pu64, u64, pu64]),
        "lm1c_queue_push": (i32, [p, u32, u32, u64]),
        "lm1c_run": (i32, [p, u32, u64, c.POINTER(_Result)]),
        "lm1c_run_round_robin": (i32, [p, u32, u64, u64, c.POINTER(_Result)]),
        "lm1c_run_cluster": (i32, [p, u64, u64, c.POINTER(_Result)]),
        "lm1c_collect": (i32, [p, u32, c.POINTER(_Result)]),
        "lm1c_chiplet_abi": (u32, []),
        "lm1c_link_get": (i32, [p, i32, pu64]),
        "lm1c_link_range": (i32, [p, u32, pu64, pu64, c.POINTER(c.c_int)]),
        "lm1c_link_lend": (i32, [p, u32, u64, u64, i32]),
        "lm1c_link_fault": (i32, [p, c.POINTER(_Result)]),
        "mp64_chiplet_create": (i32, [c.c_char_p, c.POINTER(_ChipletModule)]),
        "lm1c_chiplet_find": (p, [u64]),
        "lm1c_md_hold": (i32, [p, u32, u64]),
        "lm1c_md_write": (i32, [p, u64, u64]),
        "lm1c_md_churn": (i32, [p, u64, u64, u64, u64]),
        "lm1c_md_get": (i32, [p, i32, pu64]),
        "lm1c_md_trace": (i32, [p, i32]),
        "lm1c_md_drain": (i32, [p, i32]),
        "lm1c_md_trace_take": (i32, [p, pu64, u64, pu64]),
    }
    for name, (restype, argtypes) in sig.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes


def _check(code: int, what: str) -> None:
    if code != 0:
        raise ValueError("%s failed with %d" % (what, code))


# ---------------------------------------------------------------------------
# The model
# ---------------------------------------------------------------------------

class Model:
    """One model made from a configuration string (lm1c_create), or a view
    of the model inside a chiplet module, which the module owns."""

    def __init__(self, config: str = "", *, _view_of: int | None = None):
        self._lib = lib = library()
        self._owned = _view_of is None
        if self._owned:
            error = ctypes.create_string_buffer(256)
            handle = lib.lm1c_create(config.encode(), error, len(error))
            if not handle:
                raise ValueError(error.value.decode())
        else:
            handle = _view_of
        self._handle = handle
        self._host = None        # the callbacks, kept alive while installed
        self.host_error: BaseException | None = None
        self._views = weakref.WeakValueDictionary()   # Emulator views by tile

    def __del__(self):
        handle, self._handle = getattr(self, "_handle", None), None
        if handle and getattr(self, "_owned", False):
            self._lib.lm1c_destroy(handle)

    # -- LM-1's end of the link --

    def link(self, field: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_link_get(self._handle, field, ctypes.byref(out)), "link_get")
        return out.value

    def lend_range(self, index: int) -> tuple[int, int, bool]:
        base, limit, write = ctypes.c_uint64(), ctypes.c_uint64(), ctypes.c_int()
        _check(self._lib.lm1c_link_range(self._handle, index, ctypes.byref(base),
                                         ctypes.byref(limit), ctypes.byref(write)),
               "link_range")
        return base.value, limit.value, bool(write.value)

    def lend(self, index: int, base: int, limit: int, write: bool) -> int:
        """The lend port; returns the C interface's code, 0 when taken."""
        return self._lib.lm1c_link_lend(self._handle, index, base & WORD_MASK,
                                        limit & WORD_MASK, int(write))

    # -- Managed data's stand-in and its test hooks --

    def md(self, field: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_md_get(self._handle, field, ctypes.byref(out)), "md_get")
        return out.value

    def md_hold(self, mode: int, keep_ns: int = 0) -> None:
        """A hold as soon as HOLD_GAP allows, kept ACTIVE KEEP_NS more."""
        _check(self._lib.lm1c_md_hold(self._handle, mode, keep_ns & WORD_MASK), "md_hold")

    def md_write(self, addr: int, value: int) -> None:
        """A word written as LM-1's store would, at the next step."""
        _check(self._lib.lm1c_md_write(self._handle, addr, value & WORD_MASK), "md_write")

    def md_drain(self, on: bool) -> None:
        """Whether LM-1 drains a store log when its core rings."""
        _check(self._lib.lm1c_md_drain(self._handle, int(on)), "md_drain")

    def md_trace(self, on: bool) -> None:
        """Record every word the stand-in writes across the link."""
        _check(self._lib.lm1c_md_trace(self._handle, int(on)), "md_trace")

    def md_trace_take(self) -> list[tuple[int, int]]:
        """The (address, value) pairs recorded since the last take."""
        count = ctypes.c_uint64()
        _check(self._lib.lm1c_md_trace_take(self._handle, None, 0, ctypes.byref(count)),
               "md_trace_take")
        pairs = (ctypes.c_uint64 * (2 * count.value + 2))()
        _check(self._lib.lm1c_md_trace_take(self._handle, pairs, count.value, ctypes.byref(count)),
               "md_trace_take")
        return [(pairs[2 * i], pairs[2 * i + 1]) for i in range(count.value)]

    def md_churn(self, addr: int, a: int, b: int, times: int) -> None:
        """A and B written in turn into ADDR, one write a step."""
        _check(self._lib.lm1c_md_churn(self._handle, addr, a & WORD_MASK, b & WORD_MASK, times),
               "md_churn")

    def first_fault(self) -> _Result:
        result = _Result()
        _check(self._lib.lm1c_link_fault(self._handle, ctypes.byref(result)), "link_fault")
        return result

    @property
    def tiles(self) -> int:
        return self._lib.lm1c_tiles(self._handle)

    def set_host(self, put_char=None, get_char=None, block_io=None) -> None:
        """Install host callbacks, each a Python function or None.  A
        callback that raises fails its trap; the exception waits in
        host_error for the caller of the run."""
        model = weakref.ref(self)

        def guard(fn, failed):
            def call(*args):
                try:
                    return fn(*args)
                except BaseException as exc:   # never unwind through C
                    owner = model()
                    if owner is not None:
                        owner.host_error = exc
                    return failed
            return call

        host = _Host()
        host.context = None
        if put_char is not None:
            host.put_char = _PUT_CHAR(guard(lambda _c, tile, cp: put_char(tile, cp), HOST_FAIL))
        if get_char is not None:
            def get(_c, tile, out):
                out[0] = get_char(tile)
                return HOST_OK
            host.get_char = _GET_CHAR(guard(get, HOST_FAIL))
        if block_io is not None:
            def blocks(_c, tile, write, block, data, size):
                return block_io(tile, bool(write), block, data, size)
            host.block_io = _BLOCK_IO(guard(blocks, HOST_FAIL))
        self._host = host
        self._lib.lm1c_set_host(self._handle, ctypes.byref(host))

    # -- fields --

    def tile_get(self, tile: int, field: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_tile_get(self._handle, tile, field, ctypes.byref(out)), "tile_get")
        return out.value

    def tile_set(self, tile: int, field: int, value: int) -> None:
        _check(self._lib.lm1c_tile_set(self._handle, tile, field, value & WORD_MASK), "tile_set")

    def tile_perf(self, tile: int, index: int) -> int:
        """Performance counter INDEX of the tile's core, as SYS_INFO 10 reads it."""
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_tile_perf(self._handle, tile, index, ctypes.byref(out)), "tile_perf")
        return out.value

    def thread_get(self, tile: int, thread: int, field: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_thread_get(self._handle, tile, thread, field, ctypes.byref(out)),
               "thread_get")
        return out.value

    def thread_set(self, tile: int, thread: int, field: int, value: int) -> None:
        _check(self._lib.lm1c_thread_set(self._handle, tile, thread, field, value & WORD_MASK),
               "thread_set")

    def reg(self, tile: int, thread: int, index: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_reg_get(self._handle, tile, thread, index, ctypes.byref(out)),
               "reg_get")
        return out.value

    def set_reg(self, tile: int, thread: int, index: int, value: int) -> None:
        _check(self._lib.lm1c_reg_set(self._handle, tile, thread, index, value & WORD_MASK),
               "reg_set")

    def regs(self, tile: int, thread: int) -> list[int]:
        out = (ctypes.c_uint64 * 32)()
        _check(self._lib.lm1c_regs_get(self._handle, tile, thread, out), "regs_get")
        return list(out)

    # -- memory --

    def read(self, tile: int, address: int, size: int) -> bytes:
        buf = ctypes.create_string_buffer(size)
        _check(self._lib.lm1c_mem_read(self._handle, tile, address, buf, size), "mem_read")
        return buf.raw

    def write(self, tile: int, address: int, data: bytes) -> None:
        _check(self._lib.lm1c_mem_write(self._handle, tile, address, data, len(data)),
               "mem_write")

    def template(self, tile: int, index: int) -> int:
        out = ctypes.c_uint64()
        _check(self._lib.lm1c_template_get(self._handle, tile, index, ctypes.byref(out)),
               "template_get")
        return out.value

    def set_template(self, tile: int, index: int, value: int) -> None:
        _check(self._lib.lm1c_template_set(self._handle, tile, index, value & WORD_MASK),
               "template_set")

    def cards(self, tile: int, first: int = 0, count: int | None = None) -> bytes:
        if count is None:
            count = self.tile_get(tile, TILE_CARD_TABLE_SIZE) - first
        buf = ctypes.create_string_buffer(max(count, 1))
        _check(self._lib.lm1c_cards_read(self._handle, tile, first, buf, count), "cards_read")
        return buf.raw[:count]

    def ic_install(self, tile: int, callsite: int, shape: int, target: int) -> None:
        _check(self._lib.lm1c_ic_install(self._handle, tile, callsite & WORD_MASK,
                                         shape & WORD_MASK, target & WORD_MASK), "ic_install")

    def ic_entries(self, tile: int) -> dict[tuple[int, int], int]:
        count = ctypes.c_uint64()
        _check(self._lib.lm1c_ic_entries(self._handle, tile, None, 0, ctypes.byref(count)),
               "ic_entries")
        out = (ctypes.c_uint64 * (3 * count.value + 1))()
        _check(self._lib.lm1c_ic_entries(self._handle, tile, out, count.value,
                                         ctypes.byref(count)), "ic_entries")
        return {(out[3 * i], out[3 * i + 1]): out[3 * i + 2] for i in range(count.value)}

    def queue(self, tile: int, queue: int) -> list[int]:
        count = ctypes.c_uint64()
        _check(self._lib.lm1c_queue_items(self._handle, tile, queue, None, 0,
                                          ctypes.byref(count)), "queue_items")
        out = (ctypes.c_uint64 * (count.value + 1))()
        _check(self._lib.lm1c_queue_items(self._handle, tile, queue, out, count.value,
                                          ctypes.byref(count)), "queue_items")
        return list(out[:count.value])

    def queue_push(self, tile: int, queue: int, value: int) -> None:
        _check(self._lib.lm1c_queue_push(self._handle, tile, queue, value & WORD_MASK),
               "queue_push")

    # -- runs --

    def run(self, tile: int, max_instructions: int = 0) -> _Result:
        result = _Result()
        _check(self._lib.lm1c_run(self._handle, tile, max_instructions, ctypes.byref(result)),
               "run")
        return result

    def run_round_robin(self, tile: int, max_instructions: int = 0, quanta: int = 1) -> _Result:
        result = _Result()
        _check(self._lib.lm1c_run_round_robin(self._handle, tile, max_instructions, quanta,
                                              ctypes.byref(result)), "run_round_robin")
        return result

    def run_cluster(self, max_instructions: int = 0, quanta: int = 1) -> _Result:
        result = _Result()
        _check(self._lib.lm1c_run_cluster(self._handle, max_instructions, quanta,
                                          ctypes.byref(result)), "run_cluster")
        return result

    def collect(self, tile: int) -> _Result:
        result = _Result()
        _check(self._lib.lm1c_collect(self._handle, tile, ctypes.byref(result)), "collect")
        return result

    def raise_for(self, result: _Result) -> None:
        """Raise what the reference emulator raises where the run stopped."""
        if result.stop in (STOP_BUDGET, STOP_IDLE):
            return
        if result.stop == STOP_TRAP:
            raise LM1Trap(result.trap)
        if result.stop == STOP_MEMORY:
            raise IndexError("address %#x lies outside the tile's memory" % result.address)
        if result.stop == STOP_OLDGEN:
            raise RuntimeError("Old-gen overflow during GC: need %d bytes" % result.address)
        error, self.host_error = self.host_error, None
        if error is not None:
            raise error
        raise RuntimeError("a host callback failed")


# ---------------------------------------------------------------------------
# The chiplet module, as MegaPad's port bridge drives it
# ---------------------------------------------------------------------------

def attached_model(hook_id: int) -> "Model | None":
    """A view of the model inside the chiplet module MegaPad's emulator or
    simulator made with hook_id=HOOK_ID in its configuration, the most
    recent of them, or None.  The view lasts while the module does."""
    handle = library().lm1c_chiplet_find(hook_id)
    return Model(_view_of=handle) if handle else None


class Module:
    """A chiplet module made by mp64_chiplet_create from a configuration
    string, bound to a host whose services are Python callables:

    - ``mem(addr, size, len, wen, words)`` answers one memory request with a
      response code and the count of words completed; WORDS holds len + 1
      words, which a read fills in place;
    - ``doorbell(dst)`` takes a doorbell the chiplet rings;
    - ``status(ready, stopped)`` takes the sideband's signals.

    The calls follow chiplet_module.h, and ``model`` views the model inside
    through the library's own interface.  A host callable that raises fails
    its request with MEM_ERROR, or is dropped, and the exception waits in
    ``host_error``.
    """

    def __init__(self, config: str = "", *, mem=None, doorbell=None, status=None):
        self._lib = lib = library()
        module = _ChipletModule()
        if lib.mp64_chiplet_create(config.encode(), ctypes.byref(module)) != 0:
            # lm1c_create reads the same configuration and says what is wrong.
            Model(config)
            raise ValueError("mp64_chiplet_create refused %r" % config)
        if module.abi != CHIPLET_ABI:
            module.destroy(module.self)
            raise ChipletBuildError("the module reports interface version %d" % module.abi)
        self._module = module
        self.model = Model(_view_of=module.self)
        self.host_error: BaseException | None = None
        owner = weakref.ref(self)

        def guarded(fn, failed):
            def call(*args):
                try:
                    return fn(*args)
                except BaseException as exc:   # never unwind through C
                    me = owner()
                    if me is not None:
                        me.host_error = exc
                    return failed
            return call

        def host_mem(_ctx, addr, size, length, wen, data, done):
            words = (ctypes.c_uint64 * (length + 1)).from_address(ctypes.addressof(data.contents))
            code, completed = mem(addr, size, length, bool(wen), words)
            if done:
                done[0] = completed
            return code

        host = _ChipletHost()
        host.ctx = None
        host.mem = _MEM(guarded(host_mem, LINK_MEM_ERROR) if mem else
                        (lambda *_: LINK_REFUSED))
        host.doorbell = _DOORBELL(guarded(lambda _c, dst: doorbell(dst), None) if doorbell else
                                  (lambda *_: None))
        host.status = _STATUS(guarded(lambda _c, r, s: status(bool(r), bool(s)), None)
                              if status else (lambda *_: None))
        self._host = host    # kept alive while the module may call it
        module.bind(module.self, ctypes.byref(host))

    def close(self) -> None:
        module, self._module = getattr(self, "_module", None), None
        if module is not None:
            self.model = None
            module.destroy(module.self)

    def __del__(self):
        self.close()

    def identity(self, index: int) -> int:
        return self._module.identity(self._module.self, index)

    def reset(self) -> None:
        self._module.reset(self._module.self)

    def release(self, boot_addr: int, boot_len: int, boot_arg: int) -> None:
        self._module.release(self._module.self, boot_addr & WORD_MASK, boot_len & WORD_MASK,
                             boot_arg & WORD_MASK)

    def stop(self, stop_req: bool) -> None:
        self._module.stop(self._module.self, int(stop_req))

    def doorbell(self, src: int) -> bool:
        """Whether the module takes the doorbell."""
        return bool(self._module.doorbell(self._module.self, src))

    def step(self, budget: int) -> bool:
        """Whether more work waits."""
        return bool(self._module.step(self._module.self, budget))

    def lent_write(self, addr: int, data: bytes, zeros: int = 0) -> int:
        """One of MegaPad's writes into lent memory: DATA, whole units of
        64 bytes, then ZEROS units of zeros; returns the answer's code."""
        units, rest = divmod(len(data), UNIT_BYTES)
        if rest:
            raise ValueError("a write carries whole units of %d bytes" % UNIT_BYTES)
        buffer = (ctypes.c_uint8 * max(len(data), 1)).from_buffer_copy(data or b"\0")
        return self._module.lent_write(self._module.self, addr & WORD_MASK, buffer, units, zeros)

    def save(self) -> bytes:
        size = self._module.save(self._module.self, None, 0)
        buffer = (ctypes.c_uint8 * max(size, 1))()
        written = self._module.save(self._module.self, buffer, size)
        if written != size:
            raise RuntimeError("the state changed size between two saves")
        return bytes(buffer)[:size]

    def restore(self, state: bytes) -> bool:
        buffer = (ctypes.c_uint8 * max(len(state), 1)).from_buffer_copy(state or b"\0")
        return self._module.restore(self._module.self, buffer, len(state)) == 0


# ---------------------------------------------------------------------------
# The reference emulator's interface, backed by a model
# ---------------------------------------------------------------------------

class _Registers:
    """A thread's registers, indexed like the reference's list."""

    __slots__ = ("_m", "_tile", "_thread")

    def __init__(self, model: Model, tile: int, thread: int):
        self._m, self._tile, self._thread = model, tile, thread

    def __len__(self) -> int:
        return 32

    def __getitem__(self, index):
        if isinstance(index, slice):
            return self._m.regs(self._tile, self._thread)[index]
        if not -32 <= index < 32:
            raise IndexError("list index out of range")
        return self._m.reg(self._tile, self._thread, index % 32)

    def __setitem__(self, index: int, value: int) -> None:
        if not -32 <= index < 32:
            raise IndexError("list assignment index out of range")
        self._m.set_reg(self._tile, self._thread, index % 32, value)

    def __iter__(self):
        return iter(self._m.regs(self._tile, self._thread))

    def __eq__(self, other) -> bool:
        return list(self) == list(other)


class _Templates:
    """A tile's header-template table, which its threads share."""

    __slots__ = ("_m", "_tile")

    def __init__(self, model: Model, tile: int):
        self._m, self._tile = model, tile

    def __len__(self) -> int:
        return 256

    def __getitem__(self, index: int) -> int:
        if not 0 <= index < 256:
            raise IndexError("list index out of range")
        return self._m.template(self._tile, index)

    def __setitem__(self, index: int, value: int) -> None:
        if not 0 <= index < 256:
            raise IndexError("list assignment index out of range")
        self._m.set_template(self._tile, index, value)

    def __iter__(self):
        return (self._m.template(self._tile, i) for i in range(256))


def _thread_field(field: int, *, flag: bool = False, signed: bool = False):
    def get(self):
        value = self._m.thread_get(self._tile, self._thread, field)
        if flag:
            return bool(value)
        if signed and value & (1 << 63):
            return value - (1 << 64)
        return value

    def set(self, value):
        self._m.thread_set(self._tile, self._thread, field, int(value))

    return property(get, set)


def _reg_alias(index: int):
    def get(self):
        return self._m.reg(self._tile, self._thread, index)

    def set(self, value):
        self._m.set_reg(self._tile, self._thread, index, value)

    return property(get, set)


class Thread:
    """One hardware thread, with the fields of the reference's ThreadContext."""

    __slots__ = ("_m", "_tile", "_thread")

    def __init__(self, model: Model, tile: int, thread: int):
        self._m, self._tile, self._thread = model, tile, thread

    @property
    def regs(self) -> _Registers:
        return _Registers(self._m, self._tile, self._thread)

    @property
    def header_templates(self) -> _Templates:
        return _Templates(self._m, self._tile)

    pc = _thread_field(THREAD_PC)
    instret = _thread_field(THREAD_INSTRET)
    halted = _thread_field(THREAD_HALTED, flag=True)
    stalled = _thread_field(THREAD_STALLED, flag=True)
    stall_queue = _thread_field(THREAD_STALL_QUEUE, signed=True)
    trap_table_base = _thread_field(THREAD_TRAP_TABLE_BASE)
    trap_cause = _thread_field(THREAD_TRAP_CAUSE)
    trap_pc = _thread_field(THREAD_TRAP_PC)
    in_trap = _thread_field(THREAD_IN_TRAP, flag=True)
    tile_id = _thread_field(THREAD_TILE_ID)
    thread_id = _thread_field(THREAD_THREAD_ID)

    sp = _reg_alias(30)
    fp = _reg_alias(29)
    lr = _reg_alias(28)
    tp = _reg_alias(27)
    np = _reg_alias(26)
    nl = _reg_alias(25)

    def get_reg(self, idx: int) -> int:
        return self.regs[idx & 0x1F]

    def set_reg(self, idx: int, value: int) -> None:
        self.regs[idx & 0x1F] = value


class _Memory:
    """A tile's memory, with the methods of the reference's Memory."""

    __slots__ = ("_m", "_tile", "_size")

    def __init__(self, model: Model, tile: int):
        self._m, self._tile = model, tile
        self._size = model.tile_get(tile, TILE_MEM_SIZE)

    @property
    def size(self) -> int:
        return self._size

    def _word_address(self, addr: int) -> int:
        if addr < 0 or addr >> 3 >= self._size >> 3:
            raise IndexError("array index out of range")
        return (addr >> 3) << 3

    def _load(self, addr: int) -> int:
        return int.from_bytes(self._m.read(self._tile, self._word_address(addr), 8), "little")

    def _store(self, addr: int, value: int) -> None:
        if not 0 <= value <= WORD_MASK:
            raise OverflowError("unsigned long long is greater than maximum")
        self._m.write(self._tile, self._word_address(addr), value.to_bytes(8, "little"))

    def load_word(self, addr: int) -> int:
        return self._load(addr)

    def store_word(self, addr: int, value: int) -> None:
        self._store(addr, value & WORD_MASK)

    def load_u32(self, addr: int) -> int:
        word = self._load(addr)
        return (word >> 32) & 0xFFFF_FFFF if addr & 4 else word & 0xFFFF_FFFF

    def store_u32(self, addr: int, value: int) -> None:
        word, val32 = self._load(addr), value & 0xFFFF_FFFF
        if addr & 4:
            self._store(addr, (word & 0x0000_0000_FFFF_FFFF) | (val32 << 32))
        else:
            self._store(addr, (word & 0xFFFF_FFFF_0000_0000) | val32)

    def _load_bits(self, addr: int, mask: int) -> int:
        return (self._load(addr) >> ((addr & 7) * 8)) & mask

    def _store_bits(self, addr: int, mask: int, value: int) -> None:
        shift = (addr & 7) * 8
        word = self._load(addr)
        self._store(addr, (word & (~(mask << shift) & WORD_MASK)) | ((value & mask) << shift))

    def load_byte(self, addr: int) -> int:
        return self._load_bits(addr, 0xFF)

    def store_byte(self, addr: int, value: int) -> None:
        self._store_bits(addr, 0xFF, value)

    def load_u16(self, addr: int) -> int:
        return self._load_bits(addr, 0xFFFF)

    def store_u16(self, addr: int, value: int) -> None:
        self._store_bits(addr, 0xFFFF, value)

    def load_u32_data(self, addr: int) -> int:
        return self._load_bits(addr, 0xFFFF_FFFF)

    def store_u32_data(self, addr: int, value: int) -> None:
        self._store_bits(addr, 0xFFFF_FFFF, value)

    def _write_prefix(self, base: int, data: bytes) -> None:
        """Write DATA at BASE, as far as memory reaches, then raise as the
        reference does at the first byte past it."""
        if base < 0:
            raise IndexError("array index out of range")
        fits = max(0, min(len(data), self._size - base))
        if fits:
            self._m.write(self._tile, base, data[:fits])
        if fits < len(data):
            raise IndexError("array index out of range")

    def load_binary(self, base_addr: int, data: bytes) -> None:
        self._write_prefix(base_addr, bytes(data))

    def load_words(self, base_addr: int, words: list[int]) -> None:
        base = (base_addr >> 3) << 3
        self._write_prefix(base, b"".join((w & WORD_MASK).to_bytes(8, "little") for w in words))

    def load_instructions(self, base_addr: int, instructions: list[int]) -> None:
        if base_addr % 4 == 0 and base_addr >= 0:
            self._write_prefix(base_addr, b"".join(
                (i & 0xFFFF_FFFF).to_bytes(4, "little") for i in instructions))
        else:
            for i, inst in enumerate(instructions):
                self.store_u32(base_addr + i * 4, inst)

    def dump(self, addr: int, count: int) -> list[int]:
        return [self._load(addr + i * 8) for i in range(count)]

    def tobytes(self) -> bytes:
        return self._m.read(self._tile, 0, self._size)


class _Queue:
    """One of a tile's message queues, with the deque methods tests use."""

    __slots__ = ("_m", "_tile", "_queue")

    def __init__(self, model: Model, tile: int, queue: int):
        self._m, self._tile, self._queue = model, tile, queue

    def append(self, value: int) -> None:
        self._m.queue_push(self._tile, self._queue, value)

    def __len__(self) -> int:
        return len(self._m.queue(self._tile, self._queue))

    def __iter__(self):
        return iter(self._m.queue(self._tile, self._queue))

    def __getitem__(self, index: int) -> int:
        return self._m.queue(self._tile, self._queue)[index]


class _InlineCache:
    """A tile's inline-cache table, a mapping like the reference's dict."""

    __slots__ = ("_m", "_tile")

    def __init__(self, model: Model, tile: int):
        self._m, self._tile = model, tile

    def __setitem__(self, key: tuple[int, int], target: int) -> None:
        callsite, shape = key
        self._m.ic_install(self._tile, callsite, shape, target)

    def _all(self) -> dict[tuple[int, int], int]:
        return self._m.ic_entries(self._tile)

    def __getitem__(self, key):
        return self._all()[key]

    def get(self, key, default=None):
        return self._all().get(key, default)

    def __contains__(self, key) -> bool:
        return key in self._all()

    def __len__(self) -> int:
        return len(self._all())

    def items(self):
        return self._all().items()


class _Cards:
    """A tile's card table, indexed like the reference's bytearray."""

    __slots__ = ("_m", "_tile")

    def __init__(self, model: Model, tile: int):
        self._m, self._tile = model, tile

    def __len__(self) -> int:
        return self._m.tile_get(self._tile, TILE_CARD_TABLE_SIZE)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return bytes(self)[index]
        size = len(self)
        if not -size <= index < size:
            raise IndexError("bytearray index out of range")
        return self._m.cards(self._tile, index % size, 1)[0]

    def __iter__(self):
        return iter(bytes(self))

    def __bytes__(self) -> bytes:
        return self._m.cards(self._tile)


def _tile_field(field: int):
    def get(self):
        return self._model.tile_get(self._tile, field)

    def set(self, value):
        self._model.tile_set(self._tile, field, int(value))

    return property(get, set)


class Emulator:
    """A tile of a chiplet model, offering the reference Emulator's interface.

    Made on its own, it owns a model of one tile, configured from the same
    arguments as the reference's Emulator.  A Cluster makes one per tile.
    Runs end as the reference's end: a trap that stands raises LM1Trap, an
    access outside memory IndexError, and a collection without room in
    old-gen RuntimeError.
    """

    DEFAULT_NURSERY_BASE = 0x0003_0000
    DEFAULT_NURSERY_SIZE = 0x0001_0000
    DEFAULT_OLDGEN_BASE = 0x0008_0000
    DEFAULT_OLDGEN_SIZE = 0x0004_0000
    DEFAULT_CARD_SHIFT = 6
    DEFAULT_NURSERY_CHUNK = 0x1000
    QUEUE_DEPTH = 512
    NUM_QUEUES = 4

    def __init__(
        self,
        mem_size: int = 4 * 1024 * 1024,
        *,
        trace: bool = False,
        stdin: TextIO | None = None,
        stdout: TextIO | None = None,
        nursery_base: int | None = None,
        nursery_size: int | None = None,
        oldgen_base: int | None = None,
        oldgen_size: int | None = None,
        tile_id: int = 0,
        num_threads: int = 1,
        block_device: str | None = None,
        nursery_chunk: int | None = None,
        _model: Model | None = None,
        _tile: int = 0,
    ):
        if trace:
            raise NotImplementedError("the chiplet model does not trace")
        if _model is None:
            _model = Model(_config(mem_size, nursery_base, nursery_size, oldgen_base,
                                   oldgen_size, tiles=1, threads=num_threads,
                                   tile_id=tile_id, nursery_chunk=nursery_chunk))
            _attach_host(_model)
        self._model = _model
        self._tile = _tile
        self._stdin: TextIO = stdin or sys.stdin
        self._stdout: TextIO = stdout or sys.stdout
        self.trace = False
        self.block_device_path = block_device
        self.cluster: Cluster | None = None
        self.mem = _Memory(_model, _tile)
        _model._views[_tile] = self
        if block_device is not None:
            _attach_host(_model, blocks=True)

    # -- state --

    tile_id = _tile_field(TILE_ID)
    nursery_base = _tile_field(TILE_NURSERY_BASE)
    nursery_size = _tile_field(TILE_NURSERY_SIZE)
    oldgen_base = _tile_field(TILE_OLDGEN_BASE)
    oldgen_size = _tile_field(TILE_OLDGEN_SIZE)
    oldgen_ptr = _tile_field(TILE_OLDGEN_PTR)
    card_shift = _tile_field(TILE_CARD_SHIFT)
    card_table_base = _tile_field(TILE_CARD_TABLE_BASE)
    gen_boundary = _tile_field(TILE_GEN_BOUNDARY)
    card_table_size = _tile_field(TILE_CARD_TABLE_SIZE)
    gc_count = _tile_field(TILE_GC_COUNT)
    instruction_count = _tile_field(TILE_INSTRUCTION_COUNT)
    cycles = _tile_field(TILE_CYCLES)
    nursery_chunk = _tile_field(TILE_NURSERY_CHUNK)
    nursery_free = _tile_field(TILE_NURSERY_FREE)
    _current_thread_idx = _tile_field(TILE_CURRENT_THREAD)

    @property
    def perf(self) -> list[int]:
        """The core's performance counters, as SYS_INFO 10 reads them."""
        return [self._model.tile_perf(self._tile, i) for i in range(PERF_COUNTERS)]

    @property
    def threads(self) -> list[Thread]:
        n = self._model.tile_get(self._tile, TILE_THREADS)
        return [Thread(self._model, self._tile, i) for i in range(n)]

    @property
    def thread(self) -> Thread:
        return Thread(self._model, self._tile, self._current_thread_idx)

    @property
    def nursery_slices(self) -> list[int]:
        return [self._model.thread_get(self._tile, i, THREAD_SLICE_BASE)
                for i in range(len(self.threads))]

    @property
    def queues(self) -> list[_Queue]:
        return [_Queue(self._model, self._tile, q) for q in range(self.NUM_QUEUES)]

    @property
    def ic_table(self) -> _InlineCache:
        return _InlineCache(self._model, self._tile)

    @property
    def card_table(self) -> _Cards:
        return _Cards(self._model, self._tile)

    def reg(self, idx: int) -> int:
        return self.thread.regs[idx & 0x1F]

    def set_reg(self, idx: int, val: int) -> None:
        self.thread.regs[idx & 0x1F] = val

    def _is_nursery_addr(self, addr: int) -> bool:
        return self.nursery_base <= addr < self.nursery_base + self.nursery_size

    # -- runs --

    def run(self, max_instructions: int = 0) -> None:
        """Run the current thread until it halts or stalls, or for
        MAX_INSTRUCTIONS (0: no limit)."""
        self._model.raise_for(self._model.run(self._tile, max_instructions))

    def run_round_robin(self, max_instructions: int = 0, quanta: int = 1) -> None:
        self._model.raise_for(self._model.run_round_robin(self._tile, max_instructions, quanta))

    def _gc_collect(self) -> None:
        self._model.raise_for(self._model.collect(self._tile))


class Cluster:
    """A chiplet model of several tiles, offering the reference Cluster's
    interface.  Queue descriptors are fixnum-encoded: (tile_id << 2) | queue."""

    def __init__(
        self,
        n_tiles: int = 2,
        *,
        mem_size: int = 4 * 1024 * 1024,
        trace: bool = False,
        nursery_base: int | None = None,
        nursery_size: int | None = None,
        oldgen_base: int | None = None,
        oldgen_size: int | None = None,
        num_threads_per_tile: int = 1,
    ):
        model = Model(_config(mem_size, nursery_base, nursery_size, oldgen_base, oldgen_size,
                              tiles=n_tiles, threads=num_threads_per_tile, tile_id=0))
        _attach_host(model)
        self._model = model
        self.tiles: list[Emulator] = []
        for i in range(n_tiles):
            tile = Emulator(trace=trace, _model=model, _tile=i)
            tile.cluster = self
            self.tiles.append(tile)

    def run(self, max_instructions: int = 0, quanta: int = 1) -> None:
        self._model.raise_for(self._model.run_cluster(max_instructions, quanta))


def _config(mem_size, nursery_base, nursery_size, oldgen_base, oldgen_size, *,
            tiles: int, threads: int, tile_id: int, nursery_chunk: int | None = None) -> str:
    """The configuration of a model laid out as the reference's Emulator
    lays itself out from the same arguments, which it also checks."""
    assert mem_size % 8 == 0, "Memory size must be a multiple of 8"
    nb = Emulator.DEFAULT_NURSERY_BASE if nursery_base is None else nursery_base
    ns = Emulator.DEFAULT_NURSERY_SIZE if nursery_size is None else nursery_size
    ob = Emulator.DEFAULT_OLDGEN_BASE if oldgen_base is None else oldgen_base
    os_ = Emulator.DEFAULT_OLDGEN_SIZE if oldgen_size is None else oldgen_size
    if ob < nb + ns:
        raise ValueError(
            f"old-gen at {ob:#x} lies below the nursery's end {nb + ns:#x}; the "
            f"barrier's generation boundary needs old-gen above the nursery")
    chunk = Emulator.DEFAULT_NURSERY_CHUNK if nursery_chunk is None else nursery_chunk
    return (f"tiles={tiles},threads={threads},mem={mem_size},tile_id={tile_id},"
            f"nursery_base={nb},nursery_size={ns},oldgen_base={ob},oldgen_size={os_},"
            f"nursery_chunk={chunk}")


def _attach_host(model: Model, *, blocks: bool = False) -> None:
    """Route the model's console and block traps to the tiles' streams and
    devices, as the reference's emulator serves them."""
    views = model._views

    def put_char(tile, code_point):
        out = views[tile]._stdout
        out.write(chr(code_point))
        out.flush()
        return HOST_OK

    def get_char(tile):
        ch = views[tile]._stdin.read(1)
        return ord(ch) if ch else -1

    def block_io(tile, write, block, data, size):
        path = views[tile].block_device_path
        try:
            if not write:
                with open(path, "rb") as f:
                    f.seek(block * BLOCK_SIZE)
                    got = f.read(BLOCK_SIZE)
                ctypes.memmove(data, got.ljust(size, b"\x00"), size)
            else:
                payload = ctypes.string_at(data, size)
                mode = "r+b" if os.path.exists(path) else "wb"
                with open(path, mode) as f:
                    f.seek(block * BLOCK_SIZE)
                    f.write(payload)
        except (IOError, OSError):
            return HOST_IO_ERROR
        return HOST_OK

    model.set_host(put_char, get_char, block_io if blocks else None)
