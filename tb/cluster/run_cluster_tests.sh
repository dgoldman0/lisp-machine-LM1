#!/bin/bash
# Build the two-tile cluster bench with Verilator and run every test in
# tb/cluster/tests, comparing both tiles' registers with <name>.expected.
#
# Usage: tb/cluster/run_cluster_tests.sh
#
# Build products go to tb/build/cluster, or to LM1_CLUSTER_BUILD when it is
# set.  Exit code 0 if every test passed, 1 otherwise.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${LM1_CLUSTER_BUILD:-$ROOT/tb/build/cluster}"
JOBS="${LM1_BUILD_JOBS:-4}"

WARN="-Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM -Wno-UNOPTFLAT \
      -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE \
      -Wno-BLKANDNBLK -Wno-INITIALDLY -Wno-SYNCASYNCNET \
      -Wno-PINCONNECTEMPTY -Wno-DECLFILENAME -Wno-PINMISSING \
      -Wno-GENUNNAMED -Wno-VARHIDDEN -Wno-BLKSEQ -Wno-MULTIDRIVEN"

SRC="$ROOT/rtl/core/lm1_pkg.sv \
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
    $ROOT/rtl/cluster/lm1_crossbar.sv $ROOT/rtl/cluster/lm1_cluster.sv \
    $SCRIPT_DIR/lm1_cluster_tb.sv"

mkdir -p "$BUILD_DIR"
echo "=== Building lm1_cluster_tb ==="
( cd "$BUILD_DIR" && verilator --binary --timing -j "$JOBS" --timescale 1ns/1ps $WARN \
      --top-module lm1_cluster_tb $SRC > build.log 2>&1 ) || {
    echo "BUILD FAILED (see $BUILD_DIR/build.log)"
    tail -20 "$BUILD_DIR/build.log"
    exit 1
}
SIM="$BUILD_DIR/obj_dir/Vlm1_cluster_tb"

TOTAL=0
PASSED=0
FAIL_LIST=""
for expected in "$SCRIPT_DIR/tests"/*.expected; do
    name=$(basename "$expected" .expected)
    TOTAL=$((TOTAL + 1))
    output=$(cd "$BUILD_DIR" && "$SIM" "+HEX0=$SCRIPT_DIR/tests/$name.t0.hex" \
                                       "+HEX1=$SCRIPT_DIR/tests/$name.t1.hex" 2>&1)
    if echo "$output" | grep -q "^TIMEOUT"; then
        echo "FAIL $name — $(echo "$output" | grep '^TIMEOUT')"
        FAIL_LIST="$FAIL_LIST $name"
        continue
    fi
    ok=1
    while IFS='=' read -r key val; do
        tile="${key%%.*}"; tile="${tile#t}"
        what="${key#*.}"
        case "$what" in
        write_collisions|read_collisions)
            kind="${what%_collisions}"
            line=$(echo "$output" | grep "^TILE $tile COLLISIONS ")
            if [ "$kind" = write ]; then got=$(echo "$line" | awk '{print $5}')
            else got=$(echo "$line" | awk '{print $7}'); fi
            if [ -z "$got" ] || [ "$got" -lt $((val)) ]; then
                echo "FAIL $name — tile $tile: $kind collisions $got, fewer than $((val))"
                ok=0
            fi
            ;;
        *)
            reg="${what#r}"
            want=$(printf '%016x' "$val")
            got=$(echo "$output" | grep "^TILE $tile REG r$reg = " | awk '{print $6}')
            if [ "$got" != "$want" ]; then
                echo "FAIL $name — tile $tile r$reg: expected=$want actual=$got"
                ok=0
            fi
            ;;
        esac
    done < "$expected"
    if [ "$ok" = 1 ]; then
        echo "PASS $name ($(echo "$output" | grep '^CLUSTER halted' | sed 's/.*after //');" \
             "$(echo "$output" | grep '^TILE 1 COLLISIONS' | sed 's/TILE 1 COLLISIONS/tile 1 collisions:/'))"
        PASSED=$((PASSED + 1))
    else
        FAIL_LIST="$FAIL_LIST $name"
    fi
done

echo ""
echo "Results: $PASSED/$TOTAL passed"
if [ "$PASSED" != "$TOTAL" ]; then
    echo "Failed tests:$FAIL_LIST"
    exit 1
fi
exit 0
