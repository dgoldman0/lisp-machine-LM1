#!/bin/bash
# Build and run the host-link test benches with Verilator.
#
# Usage: tb/link/run_link_tests.sh
#
# Both benches first run the shared link conformance cases of
# tb/link/link_cases.txt, read at run time through tb/link/link_cases.svh,
# the list that LM-1's chiplet module runs too (emu/tests/test_chiplet_link.py),
# and then the cases each alone can run.
#
# lm1_link_tb runs always: LM-1's link against a behavioural MegaPad.
#
# lm1_megapad_link_tb runs when MEGAPAD_ROOT names a MegaPad checkout: LM-1's
# link, with a cluster of LM-1's as the memory it writes, against MegaPad's
# own RTL (port bridge, mailbox, external-memory arbiter and controller, and
# the tile engines' image stage).  It is the test that both sides of the
# link share.
#
# Build products go to tb/build/link, or to LM1_LINK_BUILD when it is set.
# Verilator builds with LM1_LINK_JOBS jobs, every core when it is unset.
#
# Exit code 0 if every bench that ran passed, 1 otherwise.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${LM1_LINK_BUILD:-$ROOT/tb/build/link}"
JOBS="${LM1_LINK_JOBS:-0}"
CASES="$SCRIPT_DIR/link_cases.txt"

WARN="-Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM -Wno-UNOPTFLAT \
      -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE \
      -Wno-BLKANDNBLK -Wno-INITIALDLY -Wno-SYNCASYNCNET \
      -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME -Wno-PINMISSING \
      -Wno-GENUNNAMED -Wno-VARHIDDEN -Wno-BLKSEQ"

LINK_SRC="$ROOT/rtl/link/lm1_link.sv $ROOT/rtl/link/lm1_link_boot.sv"

# One cluster of LM-1's, package first.
CLUSTER_SRC="$ROOT/rtl/core/lm1_pkg.sv \
    $ROOT/rtl/tech/lm1_clock_gate.sv $ROOT/rtl/tech/lm1_sram_sp.sv \
    $ROOT/rtl/tech/lm1_sram_dp.sv \
    $ROOT/rtl/core/lm1_decoder.sv $ROOT/rtl/core/lm1_regfile.sv \
    $ROOT/rtl/core/lm1_alu.sv $ROOT/rtl/core/lm1_branch.sv \
    $ROOT/rtl/core/lm1_lsu.sv $ROOT/rtl/core/lm1_icache.sv \
    $ROOT/rtl/core/lm1_tmpl_table.sv $ROOT/rtl/core/lm1_ic_table.sv \
    $ROOT/rtl/core/lm1_msg_queue.sv $ROOT/rtl/core/lm1_perf_counters.sv \
    $ROOT/rtl/core/lm1_control.sv $ROOT/rtl/core/lm1_cpu.sv \
    $ROOT/rtl/gc/lm1_gc_scanner.sv $ROOT/rtl/gc/lm1_gc_copier.sv \
    $ROOT/rtl/gc/lm1_gc_fixup.sv $ROOT/rtl/gc/lm1_gc_engine_top.sv \
    $ROOT/rtl/tile/lm1_tile.sv \
    $ROOT/rtl/cluster/lm1_crossbar.sv $ROOT/rtl/cluster/lm1_cluster.sv"

STATUS=0

run_bench() {
    local top="$1"; shift
    local dir="$BUILD_DIR/$top"
    mkdir -p "$dir"
    echo "=== Building $top ==="
    ( cd "$dir" && verilator --binary --timing -j "$JOBS" --timescale 1ns/1ps $WARN \
          -I"$SCRIPT_DIR" --top-module "$top" "$@" > build.log 2>&1 ) || {
        echo "BUILD FAILED: $top (see $dir/build.log)"
        tail -20 "$dir/build.log"
        STATUS=1
        return
    }
    echo "=== Running $top ==="
    local out
    out="$("$dir/obj_dir/V$top" "+cases=$CASES" 2>&1)" || true
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
        "$MP/soc/mp64_tacc_transfer.v" \
        $CLUSTER_SRC $LINK_SRC "$ROOT/tb/link/lm1_megapad_link_tb.sv"
else
    echo "MEGAPAD_ROOT is unset: the bench against MegaPad's RTL was not run."
fi

exit $STATUS
