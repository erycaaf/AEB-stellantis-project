#!/usr/bin/env bash
# run_full_suite.sh — Full test suite execution with gcov coverage
#
# Compiles all 19 test suites with --coverage using the host gcc,
# runs each binary, accumulates gcda data, then generates per-source
# coverage via gcov and writes reports/full_suite/coverage_report.txt.
#
# Requirements: gcc >= 9, gcov (same version as gcc), make, bash
# Usage: bash scripts/run_full_suite.sh [--clean]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

REPORT_DIR="$ROOT/reports/full_suite"
COV_DIR="$REPORT_DIR/cov"
LOGS_DIR="$REPORT_DIR/logs"
BINS_DIR="$REPORT_DIR/bins"

CC="${CC:-gcc}"
GCOV="${GCOV:-gcov}"
CFLAGS_COV="-Wall -Wextra -Wpedantic -std=c99 -O0 -g --coverage -Iinclude -Istubs"
LDFLAGS="-lm"

DATE="$(date -Iseconds 2>/dev/null || date)"
GCC_VER="$($CC --version | head -1)"

# ── Helpers ───────────────────────────────────────────────────────────────

pass() { printf "  \033[32mPASS\033[0m  %s\n" "$1"; }
fail() { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; }
info() { printf "  \033[34m.....\033[0m  %s\n" "$1"; }

# ── Clean / init ──────────────────────────────────────────────────────────

if [[ "${1:-}" == "--clean" ]]; then
    rm -rf "$REPORT_DIR"
fi

mkdir -p "$COV_DIR" "$LOGS_DIR" "$BINS_DIR"

# Remove stale coverage data so runs don't bleed into each other
rm -f "$COV_DIR"/*.gcda "$COV_DIR"/*.gcno "$COV_DIR"/*.gcov "$COV_DIR"/*.o

echo ""
echo "======================================================================"
echo "  AEB Stellantis — Full Test Suite Execution"
echo "  Date   : $DATE"
echo "  Toolchain: $GCC_VER"
echo "  Report : $REPORT_DIR/coverage_report.txt"
echo "======================================================================"
echo ""

# ── Step 1: Compile all source modules once with --coverage ───────────────

echo "=== Step 1/4: Compiling source modules with coverage instrumentation ==="
info "aeb_can.c"
$CC $CFLAGS_COV -c src/communication/aeb_can.c      -o "$COV_DIR/aeb_can.o"
info "aeb_uds.c"
$CC $CFLAGS_COV -c src/communication/aeb_uds.c      -o "$COV_DIR/aeb_uds.o"
info "aeb_perception.c"
$CC $CFLAGS_COV -c src/perception/aeb_perception.c  -o "$COV_DIR/aeb_perception.o"
info "aeb_ttc.c"
$CC $CFLAGS_COV -c src/decision/aeb_ttc.c           -o "$COV_DIR/aeb_ttc.o"
info "aeb_fsm.c"
$CC $CFLAGS_COV -c src/decision/aeb_fsm.c           -o "$COV_DIR/aeb_fsm.o"
info "aeb_pid.c"
$CC $CFLAGS_COV -c src/execution/aeb_pid.c          -o "$COV_DIR/aeb_pid.o"
info "aeb_alert.c"
$CC $CFLAGS_COV -c src/execution/aeb_alert.c        -o "$COV_DIR/aeb_alert.o"
info "aeb_core.c"
$CC $CFLAGS_COV -c src/integration/aeb_core.c       -o "$COV_DIR/aeb_core.o"
info "can_hal.c (stub)"
$CC $CFLAGS_COV -c stubs/can_hal.c                  -o "$COV_DIR/can_hal.o"

echo "  All modules compiled OK."
echo ""

# Shorthand object groups
OBJ_CAN="$COV_DIR/aeb_can.o $COV_DIR/can_hal.o"
OBJ_UDS="$COV_DIR/aeb_uds.o"
OBJ_PERC="$COV_DIR/aeb_perception.o"
OBJ_DEC="$COV_DIR/aeb_ttc.o $COV_DIR/aeb_fsm.o"
OBJ_PID="$COV_DIR/aeb_pid.o"
OBJ_ALERT="$COV_DIR/aeb_alert.o"
OBJ_ALL="$COV_DIR/aeb_can.o $COV_DIR/aeb_uds.o $COV_DIR/aeb_perception.o \
         $COV_DIR/aeb_ttc.o $COV_DIR/aeb_fsm.o $COV_DIR/aeb_pid.o \
         $COV_DIR/aeb_alert.o $COV_DIR/aeb_core.o $COV_DIR/can_hal.o"

# ── Step 2: Link all 19 test suites ───────────────────────────────────────

echo "=== Step 2/4: Linking test binaries ==="

link_suite() {
    local name="$1"; shift
    info "Linking $name"
    $CC $CFLAGS_COV -o "$BINS_DIR/$name" "$@" $LDFLAGS
}

link_suite test_smoke            tests/test_smoke.c            $OBJ_CAN
link_suite test_can              tests/test_can.c              $OBJ_CAN
link_suite test_can_fault        tests/test_can_fault.c        $OBJ_CAN
link_suite test_can_struct       tests/test_can_struct.c       $OBJ_CAN
link_suite test_perception       tests/test_perception.c       $OBJ_PERC
link_suite test_perception_fault tests/test_perception_fault.c $OBJ_PERC
link_suite test_perception_mcdc  tests/test_perception_mcdc.c  $OBJ_PERC
link_suite test_decision         tests/test_decision.c         $OBJ_DEC
link_suite test_decision_fault   tests/test_decision_fault.c   $OBJ_DEC
link_suite test_decision_mcdc    tests/test_decision_mcdc.c    $OBJ_DEC
link_suite test_pid              tests/test_pid.c              $OBJ_PID
link_suite test_pid_fault        tests/test_pid_fault.c        $OBJ_PID
link_suite test_pid_mcdc         tests/test_pid_mcdc.c         $OBJ_PID
link_suite test_alert            tests/test_alert.c            $OBJ_ALERT
link_suite test_alert_fault      tests/test_alert_fault.c      $OBJ_ALERT
link_suite test_alert_mcdc       tests/test_alert_mcdc.c       $OBJ_ALERT
link_suite test_uds              tests/test_uds.c              $OBJ_UDS
link_suite test_uds_fault        tests/test_uds_fault.c        $OBJ_UDS
link_suite test_integration      tests/test_integration.c      $OBJ_ALL

echo "  All binaries linked OK."
echo ""

# ── Step 3: Run all test suites, capture results ──────────────────────────

echo "=== Step 3/4: Running all test suites ==="

TOTAL_SUITES=0
PASSED_SUITES=0
FAILED_SUITES=0
declare -A SUITE_STATUS
declare -A SUITE_RUN
declare -A SUITE_PASSED
declare -A SUITE_FAILED_COUNT

run_suite() {
    local name="$1"
    local log="$LOGS_DIR/${name}.log"
    TOTAL_SUITES=$((TOTAL_SUITES + 1))

    "$BINS_DIR/$name" > "$log" 2>&1
    local rc=$?

    # Parse test counts — handles the project's 6 distinct output formats:
    #  (1) "N run, N passed, N failed"        — test_smoke, test_can, test_decision_fault
    #  (2) "Assertions: N run  N passed  N"   — test_uds_fault
    #  (3) "N asserts, N passed, N failed"    — test_can_fault
    #  (4) "N tests, N passed"                — test_can_struct
    #  (5) "N/N passed[, N failed]"           — test_pid*, test_alert*, test_integration
    #  (6) "N passed[, N failed]"             — test_perception*, test_decision*, test_uds
    local run=0 passed=0 failed_count=0
    local summary_line
    summary_line=$(grep -iE "(Results|RESULTS|summary|Assertions):" "$log" 2>/dev/null | tail -1 || true)

    if [[ -n "$summary_line" ]]; then
        # Format (1/2): explicit "N run"
        if echo "$summary_line" | grep -qE "[0-9]+ run"; then
            run=$(echo "$summary_line"    | grep -oE "[0-9]+ run"    | grep -oE "[0-9]+" | head -1)
            passed=$(echo "$summary_line" | grep -oE "[0-9]+ passed" | grep -oE "[0-9]+" | head -1)
            failed_count=$(echo "$summary_line" | grep -oE "[0-9]+ failed" | grep -oE "[0-9]+" | head -1)
        # Format (5): "N/N passed" — denominator is total
        elif echo "$summary_line" | grep -qE "[0-9]+[[:space:]]*/[[:space:]]*[0-9]+[[:space:]]*passed"; then
            run=$(echo "$summary_line"    | grep -oE "[0-9]+[[:space:]]*/[[:space:]]*[0-9]+" | grep -oE "[0-9]+" | tail -1)
            passed=$(echo "$summary_line" | grep -oE "[0-9]+[[:space:]]*/[[:space:]]*[0-9]+" | grep -oE "[0-9]+" | head -1)
            failed_count=$(echo "$summary_line" | grep -oE "[0-9]+ failed" | grep -oE "[0-9]+" | head -1 || echo 0)
        # Format (3/4/6): "N passed" only — total = passed + failed
        else
            passed=$(echo "$summary_line"      | grep -oE "[0-9]+ passed" | grep -oE "[0-9]+" | head -1 || echo 0)
            failed_count=$(echo "$summary_line" | grep -oE "[0-9]+ failed" | grep -oE "[0-9]+" | head -1 || echo 0)
            run=$(( ${passed:-0} + ${failed_count:-0} ))
        fi
    fi

    SUITE_RUN[$name]="${run:-0}"
    SUITE_PASSED[$name]="${passed:-0}"
    SUITE_FAILED_COUNT[$name]="${failed_count:-0}"

    if [[ $rc -eq 0 ]]; then
        SUITE_STATUS[$name]="PASS"
        PASSED_SUITES=$((PASSED_SUITES + 1))
        pass "$name  (${run:-?} tests, ${passed:-?} passed)"
    else
        SUITE_STATUS[$name]="FAIL"
        FAILED_SUITES=$((FAILED_SUITES + 1))
        fail "$name  (exit $rc — see $log)"
    fi
}

# Smoke
run_suite test_smoke
echo ""
# CAN module
run_suite test_can
run_suite test_can_fault
run_suite test_can_struct
echo ""
# Perception module
run_suite test_perception
run_suite test_perception_fault
run_suite test_perception_mcdc
echo ""
# Decision module
run_suite test_decision
run_suite test_decision_fault
run_suite test_decision_mcdc
echo ""
# PID module
run_suite test_pid
run_suite test_pid_fault
run_suite test_pid_mcdc
echo ""
# Alert module
run_suite test_alert
run_suite test_alert_fault
run_suite test_alert_mcdc
echo ""
# UDS module
run_suite test_uds
run_suite test_uds_fault
echo ""
# Integration
run_suite test_integration
echo ""

echo "  Suites run: $TOTAL_SUITES  |  Passed: $PASSED_SUITES  |  Failed: $FAILED_SUITES"
echo ""

# ── Step 4: Generate gcov coverage ────────────────────────────────────────

echo "=== Step 4/4: Generating gcov coverage reports ==="

SOURCES=(
    "src/communication/aeb_can.c"
    "src/communication/aeb_uds.c"
    "src/perception/aeb_perception.c"
    "src/decision/aeb_ttc.c"
    "src/decision/aeb_fsm.c"
    "src/execution/aeb_pid.c"
    "src/execution/aeb_alert.c"
    "src/integration/aeb_core.c"
)

declare -A COV_LINES
declare -A COV_BRANCHES

for src in "${SOURCES[@]}"; do
    base="$(basename "$src" .c)"
    info "gcov $src"
    $GCOV -b -c --object-directory "$COV_DIR" "$src" \
        > "$LOGS_DIR/gcov_${base}.txt" 2>&1 || true

    line_pct=$(grep -E "Lines executed:" "$LOGS_DIR/gcov_${base}.txt" \
                 | head -1 | grep -oE "[0-9]+\.[0-9]+" | head -1 || echo "N/A")
    branch_pct=$(grep -E "Branches executed:" "$LOGS_DIR/gcov_${base}.txt" \
                   | head -1 | grep -oE "[0-9]+\.[0-9]+" | head -1 || echo "N/A")
    COV_LINES[$src]="$line_pct"
    COV_BRANCHES[$src]="$branch_pct"
    echo "    $src  lines=${line_pct}%  branches=${branch_pct}%"

    # Move annotated .gcov to logs
    mv -f "${base}.c.gcov" "$LOGS_DIR/" 2>/dev/null || true
done

echo ""

# ── Write consolidated coverage_report.txt ────────────────────────────────

REPORT="$REPORT_DIR/coverage_report.txt"

{
cat <<HEADER
======================================================================
  AEB Stellantis — Full Test Suite Consolidated Report
  Generated : $DATE
  Toolchain : $GCC_VER
  Branch    : $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
  Commit    : $(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
======================================================================

--- TEST SUITE RESULTS ---

Suite                    Status   Run   Passed  Failed
--------------------------------------------------------
HEADER

for suite in \
    test_smoke \
    test_can test_can_fault test_can_struct \
    test_perception test_perception_fault test_perception_mcdc \
    test_decision test_decision_fault test_decision_mcdc \
    test_pid test_pid_fault test_pid_mcdc \
    test_alert test_alert_fault test_alert_mcdc \
    test_uds test_uds_fault \
    test_integration; do
    printf "%-26s %-8s %-5s %-7s %s\n" \
        "$suite" \
        "${SUITE_STATUS[$suite]:-N/A}" \
        "${SUITE_RUN[$suite]:-?}" \
        "${SUITE_PASSED[$suite]:-?}" \
        "${SUITE_FAILED_COUNT[$suite]:-?}"
done

echo ""
echo "TOTAL: $TOTAL_SUITES suites | $PASSED_SUITES passed | $FAILED_SUITES failed"
echo ""
echo ""
cat <<COV_HEADER
--- COVERAGE SUMMARY (gcov — union of all suites) ---

Source File                           Lines Covered  Branches Covered
----------------------------------------------------------------------
COV_HEADER

for src in "${SOURCES[@]}"; do
    printf "%-38s %-14s %s\n" \
        "$src" \
        "${COV_LINES[$src]:-N/A}%" \
        "${COV_BRANCHES[$src]:-N/A}%"
done

echo ""
echo ""
cat <<NOTES
--- NOTES ---

Coverage is the UNION of all 19 test suites. All suites run against
coverage-instrumented objects compiled from the same translation units,
so gcda data accumulates across every run.

Module breakdown:
  communication : test_can, test_can_fault, test_can_struct (CAN)
                  test_uds, test_uds_fault               (UDS)
  perception    : test_perception, test_perception_fault,
                  test_perception_mcdc
  decision      : test_decision, test_decision_fault,
                  test_decision_mcdc
  execution     : test_pid, test_pid_fault, test_pid_mcdc (PID)
                  test_alert, test_alert_fault,
                  test_alert_mcdc                       (Alert)
  integration   : test_integration (end-to-end stack)
  smoke         : test_smoke (baseline sanity)

Per-suite logs   : reports/full_suite/logs/<suite>.log
gcov annotations : reports/full_suite/logs/<source>.c.gcov
Full gcov output : reports/full_suite/logs/gcov_<module>.txt
NOTES

echo ""
echo "======================================================================"
echo "  Report complete."
echo "======================================================================"
} > "$REPORT"

echo "Coverage report written to: $REPORT"
echo ""
cat "$REPORT"
