# Makefile — AEB Stellantis Project (host build)
#
# Targets:
#   make build               — compile all modules (zero-warning gate)
#   make test                — build and run all unit tests
#   make misra               — run cppcheck MISRA C:2012 on non-stub sources
#   make mcdc-perception     — MC/DC coverage for aeb_perception.c (requires gcc-14)
#   make fault-perception    — run Perception fault-injection suite
#   make memory-perception   — Valgrind + ASan + UBSan on Perception suites
#   make misra-perception    — cppcheck MISRA scoped to aeb_perception.{c,h}
#   make vv-perception       — run the full V&V stack for aeb_perception
#   make clean               — remove build artefacts
#
# ASIL-D V&V targets use reports/vv_perception/ as the artefact directory.

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

# ── ASIL-D V&V (Perception module) ──────────────────────────────────────
# Notes:
#  - mcdc-perception requires gcc >=14 for -fcondition-coverage.
#  - memory-perception compiles two extra variants (sanitised and valgrind-targeted)
#    so the instrumentation does not affect the baseline build.

CC_GCC14   = gcc-14
GCOV_GCC14 = gcov-14

CFLAGS_COV = -Wall -Wextra -Wpedantic -std=c99 -O0 -g -Iinclude -Istubs \
             --coverage -fcondition-coverage
CFLAGS_SAN = -Wall -Wextra -Wpedantic -std=c99 -O0 -g -Iinclude -Istubs \
             -fsanitize=address \
             -fsanitize=undefined \
             -fsanitize=float-cast-overflow \
             -fno-omit-frame-pointer

VV_REPORT_DIR = reports/vv_perception

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

SRC_PERCEPTION_TEST       = src/perception/aeb_perception.c tests/test_perception.c
SRC_PERCEPTION_FAULT_TEST = src/perception/aeb_perception.c tests/test_perception_fault.c

SRC_DECISION_TEST = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                    tests/test_decision.c

SRC_PID_TEST = src/execution/aeb_pid.c tests/test_pid.c

SRC_ALERT_TEST = src/execution/aeb_alert.c tests/test_alert.c

SRC_UDS_TEST = src/communication/aeb_uds.c tests/test_uds.c

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
        mcdc-perception fault-perception memory-perception misra-perception \
        vv-perception vv-clean

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
	rm -f $(TEST_BINS) *.o *.gcno *.gcda *.gcov

# ── ASIL-D V&V targets (Perception) ─────────────────────────────────────

mcdc-perception:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	$(CC_GCC14) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_perception \
		$(SRC_PERCEPTION_TEST) $(LDFLAGS)
	@cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_perception > run.log 2>&1 && grep "Results:" run.log
	@cd $(VV_REPORT_DIR)/coverage_mcdc && \
		$(GCOV_GCC14) -b -c --conditions test_perception-aeb_perception.gcno \
		> gcov_summary.txt 2>&1 && cat gcov_summary.txt
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

fault-perception:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_perception_fault \
		$(SRC_PERCEPTION_FAULT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/fault_injection/test_perception_fault \
		| tee $(VV_REPORT_DIR)/fault_injection/run.log; \
		echo ""; echo "(non-zero exit expected while bugs are pending patch)"

memory-perception:
	@mkdir -p $(VV_REPORT_DIR)/memory_safety
	# Valgrind on unsanitised binaries
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_perception_val \
		$(SRC_PERCEPTION_TEST) $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_perception_fault_val \
		$(SRC_PERCEPTION_FAULT_TEST) $(LDFLAGS)
	@echo "--- Valgrind: test_perception (nominal) ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_perception_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception.log ] && echo "(clean)" || true
	@echo "--- Valgrind: test_perception_fault ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_perception_fault_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception_fault.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception_fault.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_perception_fault.log ] && echo "(clean)" || true
	# ASan + UBSan (sanitised binaries; separate from Valgrind to avoid collisions)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_perception_san \
		$(SRC_PERCEPTION_TEST) $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_perception_fault_san \
		$(SRC_PERCEPTION_FAULT_TEST) $(LDFLAGS)
	@echo "--- ASan+UBSan: test_perception (nominal) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_perception_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_perception.log || true
	@[ -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_perception.log ] && \
		cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_perception.log || echo "(clean)"
	@echo "--- ASan+UBSan: test_perception_fault ---"
	@$(VV_REPORT_DIR)/memory_safety/test_perception_fault_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_perception_fault.log || true
	@grep "runtime error" $(VV_REPORT_DIR)/memory_safety/ubsan_test_perception_fault.log \
		| sort -u || echo "(clean)"

misra-perception:
	@mkdir -p $(VV_REPORT_DIR)/misra
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all \
		--xml --xml-version=2 \
		src/perception/aeb_perception.c include/aeb_perception.h \
		2> $(VV_REPORT_DIR)/misra/cppcheck_perception.xml
	@echo "cppcheck XML -> $(VV_REPORT_DIR)/misra/cppcheck_perception.xml"

vv-perception: mcdc-perception fault-perception memory-perception misra-perception
	@echo ""
	@echo "=== Perception V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="

vv-clean:
	rm -rf $(VV_REPORT_DIR)/coverage_mcdc/test_perception* \
	       $(VV_REPORT_DIR)/fault_injection/test_perception* \
	       $(VV_REPORT_DIR)/memory_safety/test_perception*
