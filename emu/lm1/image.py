"""LM-1 images that carry their symbol table.

An image is the words a program occupies from its load address on: its code
and data as the assembler lays them out, followed by the symbol table the
chiplet module's per-function profile reads (docs/profile.md).  The table
names the functions: each entry holds a function's address, its name's
length in bytes and its name, padded with zeros to a whole word; after the
entries come the table's length in bytes and the word "LM1SYMS1", which end
the image.  A chiplet module that boots the image finds the table there;
lm1c_profile_symbols reads it from an image's bytes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

from .asm import Assembler
from .compiler import Compiler, parse

MAGIC = int.from_bytes(b"LM1SYMS1", "little")


@dataclass
class Image:
    words: list[int]                  # the image, its symbol table at the end
    symbols: list[tuple[int, str]]    # (address, name), by address

    def tobytes(self) -> bytes:
        return b"".join(w.to_bytes(8, "little") for w in self.words)


def symbol_table(symbols: Iterable[tuple[int, str]]) -> bytes:
    """The table and its two closing words, as an image ends with them."""
    entries = b""
    for address, name in sorted(symbols):
        encoded = name.encode("utf-8")
        entries += address.to_bytes(8, "little") + len(encoded).to_bytes(8, "little")
        entries += encoded + b"\0" * (-len(encoded) % 8)
    return entries + len(entries).to_bytes(8, "little") + MAGIC.to_bytes(8, "little")


def build(source: str, names: Mapping[str, str] | None = None, *,
          base: int = 0, symbols: bool = True) -> Image:
    """Assemble SOURCE for loading at BASE into an image.

    The table names the functions NAMES maps, from their labels to the
    names a profile shows, as a compiler's symbols() gives them.  Without
    NAMES it names every label the assembler resolved, save the compiler's
    own, whose names begin with two underscores.  With SYMBOLS false the
    image carries no table, and a profile names its PCs by address.
    """
    asm = Assembler()
    binary = asm.assemble(source, base)
    binary += b"\0" * (-len(binary) % 8)
    if names is None:
        table = [(addr, label) for label, addr in asm.labels.items() if not label.startswith("__")]
    else:
        table = [(asm.labels[label], name) for label, name in names.items() if label in asm.labels]
    table.sort()
    if symbols:
        binary += symbol_table(table)
    words = [int.from_bytes(binary[i:i + 8], "little") for i in range(0, len(binary), 8)]
    return Image(words, table if symbols else [])


def compile_program(source: str, *, main: str = "main", stack_top: int = 0x3FF8,
                    symbols: bool = True) -> Image:
    """Compile Lisp SOURCE into an image that runs from its first word:
    _start sets up the stack and goes to _call_main, which calls MAIN and
    halts when it returns.  The table names _start, _call_main and every
    function the compiler emitted, by its Lisp name."""
    cc = Compiler()
    cc._emit_label("_start")
    cc._emit_instr(f"LI sp, {stack_top:#x}")
    cc._emit_instr("LI fp, 0")
    cc._emit_instr("BR _call_main")
    cc.compile_toplevel(parse(source))
    cc._emit_label("_call_main")
    cc.emit_call_main(main)
    names = {"_start": "_start", "_call_main": "_call_main", **cc.symbols()}
    return build(cc.get_output(), names, symbols=symbols)
