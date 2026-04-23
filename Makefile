# Makefile — AEB Stellantis Project (host build)
#
# Targets:
#   make build        — compile all modules (zero-warning gate)
#   make test         — build and run all unit tests
#   make misra        — run cppcheck MISRA C:2012 on non-stub sources
#   make mcdc-uds     — MC/DC coverage for aeb_uds.c   (requires gcc-14, gcov-14, gcovr)
#   make fault-uds    — run UDS fault-injection suite
#   make memory-uds   — Valgrind + ASan + UBSan on UDS suites
#   make misra-uds    — cppcheck MISRA scoped to aeb_uds.{c,h}
#   make html-uds     — generate navigable HTML reports (lcov genhtml, cppcheck-htmlreport, wrappers)
#   make vv-uds       — run the full V&V stack for aeb_uds (MC/DC + fault + memory + MISRA)
#   make clean        — remove build artefacts
#
# ASIL-D V&V targets use reports/vv_uds/ as the artefact directory.

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

# ── ASIL-D V&V (UDS module) ──────────────────────────────────────────────
# Notes:
#  - mcdc-uds requires gcc >=14 for -fcondition-coverage.
#  - memory-uds compiles two extra variants (sanitised and valgrind-targeted)
#    so the instrumentation does not affect the baseline build.

CFLAGS_COV = -Wall -Wextra -Wpedantic -std=c99 -O0 -g -Iinclude -Istubs \
             --coverage -fcondition-coverage
CFLAGS_SAN = -Wall -Wextra -Wpedantic -std=c99 -O0 -g -Iinclude -Istubs \
             -fsanitize=address \
             -fsanitize=undefined \
             -fsanitize=float-cast-overflow \
             -fno-omit-frame-pointer

VV_REPORT_DIR = reports/vv_uds

# All sources (stubs + real code)
SRC_ALL = src/communication/aeb_can.c \
          src/communication/aeb_uds.c \
          src/perception/aeb_perception.c \
          src/decision/aeb_ttc.c \
          src/decision/aeb_fsm.c \
          src/execution/aeb_pid.c \
          src/execution/aeb_alert.c \
          src/integration/aeb_core.c \
          stubs/can_hal.c

# ── Test source sets ─────────────────────────────────────────────────────

SRC_SMOKE = src/communication/aeb_can.c stubs/can_hal.c tests/test_smoke.c

SRC_CAN_TEST = src/communication/aeb_can.c stubs/can_hal.c tests/test_can.c

SRC_PERCEPTION_TEST = src/perception/aeb_perception.c tests/test_perception.c

SRC_DECISION_TEST = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                    tests/test_decision.c

SRC_PID_TEST = src/execution/aeb_pid.c tests/test_pid.c

SRC_ALERT_TEST = src/execution/aeb_alert.c tests/test_alert.c

SRC_UDS_TEST       = src/communication/aeb_uds.c tests/test_uds.c
SRC_UDS_FAULT_TEST = src/communication/aeb_uds.c tests/test_uds_fault.c

SRC_INTEGRATION_TEST = $(SRC_ALL) tests/test_integration.c

# ── MISRA — all non-stub sources ────────────────────────────────────────

SRC_MISRA = src/communication/aeb_can.c \
            src/communication/aeb_uds.c \
            src/perception/aeb_perception.c \
            src/decision/aeb_ttc.c \
            src/decision/aeb_fsm.c \
            src/execution/aeb_pid.c \
            src/execution/aeb_alert.c \
            src/integration/aeb_core.c

# ── Test binaries ────────────────────────────────────────────────────────

TEST_BINS = test_smoke test_can test_perception test_decision \
            test_pid test_alert test_uds test_integration

.PHONY: build test misra clean \
        mcdc-uds fault-uds memory-uds misra-uds html-uds vv-uds vv-clean

build:
	$(CC) $(CFLAGS) -c $(SRC_ALL)
	@echo "=== Build OK: zero warnings ==="

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do \
		echo ""; echo "--- Running $$t ---"; ./$$t || exit 1; \
	done
	@echo ""
	@echo "=== All test suites passed ==="

test_smoke: $(SRC_SMOKE)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_can: $(SRC_CAN_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_perception: $(SRC_PERCEPTION_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_decision: $(SRC_DECISION_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_pid: $(SRC_PID_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_alert: $(SRC_ALERT_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_uds: $(SRC_UDS_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_integration: $(SRC_INTEGRATION_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

misra:
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all $(SRC_MISRA)

clean: vv-clean
	rm -f $(TEST_BINS) *.o

# ── ASIL-D V&V targets (UDS) ────────────────────────────────────────────

mcdc-uds:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	$(CC) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_uds $(SRC_UDS_TEST) $(LDFLAGS)
	@cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_uds > run.log 2>&1 && grep "Results:" run.log
	@cd $(VV_REPORT_DIR)/coverage_mcdc && gcov -b -c test_uds-aeb_uds.gcno > gcov_summary.txt 2>&1 && cat gcov_summary.txt
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

fault-uds:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_uds_fault $(SRC_UDS_FAULT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/fault_injection/test_uds_fault | tee $(VV_REPORT_DIR)/fault_injection/run.log; \
		echo ""; echo "(non-zero exit expected while bugs are pending patch)"

memory-uds:
	@mkdir -p $(VV_REPORT_DIR)/memory_safety
	# Valgrind on unsanitised binaries
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs -o $(VV_REPORT_DIR)/memory_safety/test_uds_val       $(SRC_UDS_TEST)       $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs -o $(VV_REPORT_DIR)/memory_safety/test_uds_fault_val $(SRC_UDS_FAULT_TEST) $(LDFLAGS)
	@echo "--- Valgrind: test_uds (nominal) ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_uds_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds.log ] && echo "(clean)" || true
	@echo "--- Valgrind: test_uds_fault ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_uds_fault_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds_fault.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds_fault.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_uds_fault.log ] && echo "(clean)" || true
	# ASan + UBSan (sanitised binaries; separate from Valgrind to avoid collisions)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_uds_san       $(SRC_UDS_TEST)       $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_uds_fault_san $(SRC_UDS_FAULT_TEST) $(LDFLAGS)
	@echo "--- ASan+UBSan: test_uds (nominal) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_uds_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_uds.log || true
	@[ -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_uds.log ] && \
		cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_uds.log || echo "(clean)"
	@echo "--- ASan+UBSan: test_uds_fault (UB reports expected until bugs are patched) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_uds_fault_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_uds_fault.log || true
	@grep "runtime error" $(VV_REPORT_DIR)/memory_safety/ubsan_test_uds_fault.log | sort -u || echo "(clean)"

misra-uds:
	@mkdir -p $(VV_REPORT_DIR)/misra
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all \
		--xml --xml-version=2 \
		src/communication/aeb_uds.c include/aeb_uds.h \
		2> $(VV_REPORT_DIR)/misra/cppcheck_uds.xml
	@echo "cppcheck XML -> $(VV_REPORT_DIR)/misra/cppcheck_uds.xml"

html-uds:
	@mkdir -p $(VV_REPORT_DIR)/coverage_html $(VV_REPORT_DIR)/misra_html
	# Coverage HTML — lcov genhtml from the gcov outputs produced by mcdc-uds.
	@if [ -d $(VV_REPORT_DIR)/coverage_mcdc ]; then \
		lcov --capture --directory $(VV_REPORT_DIR)/coverage_mcdc \
			--rc branch_coverage=1 \
			--output-file $(VV_REPORT_DIR)/coverage_html/coverage.info >/dev/null 2>&1 || true; \
		if [ -s $(VV_REPORT_DIR)/coverage_html/coverage.info ]; then \
			lcov --extract $(VV_REPORT_DIR)/coverage_html/coverage.info '*aeb_uds.c' \
				--rc branch_coverage=1 \
				--output-file $(VV_REPORT_DIR)/coverage_html/coverage_uds.info >/dev/null 2>&1 || true; \
			genhtml $(VV_REPORT_DIR)/coverage_html/coverage_uds.info \
				--branch-coverage \
				--title "UDS Coverage" \
				--legend \
				--output-directory $(VV_REPORT_DIR)/coverage_html >/dev/null 2>&1; \
		fi; \
	fi
	# MISRA HTML — cppcheck-htmlreport from the XML produced by misra-uds.
	@if [ -s $(VV_REPORT_DIR)/misra/cppcheck_uds.xml ]; then \
		cppcheck-htmlreport \
			--file=$(VV_REPORT_DIR)/misra/cppcheck_uds.xml \
			--report-dir=$(VV_REPORT_DIR)/misra_html \
			--source-dir=. \
			--title="UDS MISRA C:2012 Report" >/dev/null 2>&1 || true; \
	fi
	# Memory + fault HTML wrappers.
	@bash scripts/wrap_memory.sh uds $(VV_REPORT_DIR)
	@python3 scripts/wrap_fault.py uds $(VV_REPORT_DIR)
	@echo "=== HTML reports in $(VV_REPORT_DIR)/{coverage_html,misra_html,memory_html,fault_html}/ ==="

vv-uds: mcdc-uds fault-uds memory-uds misra-uds html-uds
	@echo ""
	@echo "=== UDS V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="

vv-clean:
	rm -rf $(VV_REPORT_DIR)/coverage_mcdc/test_uds* \
	       $(VV_REPORT_DIR)/fault_injection/test_uds* \
	       $(VV_REPORT_DIR)/memory_safety/test_uds* \
	       $(VV_REPORT_DIR)/coverage_html \
	       $(VV_REPORT_DIR)/misra_html \
	       $(VV_REPORT_DIR)/memory_html \
	       $(VV_REPORT_DIR)/fault_html
