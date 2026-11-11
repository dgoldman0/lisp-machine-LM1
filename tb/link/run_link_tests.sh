#!/bin/bash
# Build and run the host-link test benches with Verilator.
#
# Usage: tb/link/run_link_tests.sh
#
# lm1_link_tb runs always: LM-1's link against a behavioural MegaPad.
#
# lm1_megapad_link_tb runs when MEGAPAD_ROOT names a MegaPad checkout: LM-1's
# link against MegaPad's own RTL (port bridge, mailbox, external-memory
# arbiter and controller).  It is the test that both sides of the link share.
#
# Build products go to tb/build/link, or to LM1_LINK_BUILD when it is set.
#
# Exit code 0 if every bench that ran passed, 1 otherwise.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${LM1_LINK_BUILD:-$ROOT/tb/build/link}"

WARN="-Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM -Wno-UNOPTFLAT \
      -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE \
      -Wno-BLKANDNBLK -Wno-INITIALDLY -Wno-SYNCASYNCNET \
      -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME -Wno-PINMISSING \
      -Wno-GENUNNAMED -Wno-VARHIDDEN -Wno-BLKSEQ"

LINK_SRC="$ROOT/rtl/link/lm1_link.sv $ROOT/rtl/link/lm1_link_boot.sv"

STATUS=0

run_bench() {
    local top="$1"; shift
    local dir="$BUILD_DIR/$top"
    mkdir -p "$dir"
    echo "=== Building $top ==="
    ( cd "$dir" && verilator --binary --timing -j 0 --timescale 1ns/1ps $WARN \
          --top-module "$top" "$@" > build.log 2>&1 ) || {
        echo "BUILD FAILED: $top (see $dir/build.log)"
        tail -20 "$dir/build.log"
        STATUS=1
        return
    }
    echo "=== Running $top ==="
    local out
    out="$("$dir/obj_dir/V$top" 2>&1)" || true
    echo "$out"
    if ! echo "$out" | grep -q "ALL TESTS PASSED"; then
        STATUS=1
    fi
}

run_bench lm1_link_tb $LINK_SRC "$ROOT/tb/link/lm1_link_tb.sv"

if [ -n "$MEGAPAD_ROOT" ]; then
    MP="$MEGAPAD_ROOT/rtl"
    # The mailbox includes MegaPad's package header at file scope, so it
    # comes first and the header's constants reach every MegaPad module.
    run_bench lm1_megapad_link_tb -I"$MP/pkg" \
        "$MP/periph/mp64_mailbox.v" \
        "$MP/soc/mp64_chiplet_port.v" \
        "$MP/mem/mp64_extmem_arb.v" \
        "$MP/mem/mp64_extmem.v" \
        $LINK_SRC "$ROOT/tb/link/lm1_megapad_link_tb.sv"
else
    echo "MEGAPAD_ROOT is unset: the bench against MegaPad's RTL was not run."
fi

exit $STATUS
