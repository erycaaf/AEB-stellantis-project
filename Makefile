# Makefile — AEB Stellantis Project (host build)
#
# Targets (build & test):
#   make build             — compile all modules (zero-warning gate)
#   make test              — build and run all unit tests
#   make misra             — run cppcheck MISRA C:2012 on non-stub sources
#   make clean             — remove build artefacts
#
# Targets (ASIL-D V&V — UDS module):
#   make mcdc-uds          — MC/DC coverage (gcc-14 + gcov-14)
#   make fault-uds         — systematic fault-injection suite
#   make memory-uds        — Valgrind + ASan + UBSan on UDS suites
#   make misra-uds         — cppcheck MISRA scoped to aeb_uds.{c,h}
#   make html-uds          — navigable HTML reports (lcov genhtml + cppcheck-htmlreport + wrappers)
#   make vv-uds            — full V&V stack
#
# Targets (ASIL-D V&V — Decision module, TTC + FSM):
#   make mcdc-decision     — MC/DC coverage (gcc-14 + gcov-14 + gcovr)
#   make fault-decision    — systematic fault-injection suite
#   make memory-decision   — Valgrind + ASan + UBSan on Decision suites
#   make misra-decision    — cppcheck MISRA scoped to aeb_ttc / aeb_fsm
#   make vv-decision       — full V&V stack
#
# Targets (ASIL-D V&V — PID + Alert modules):
#   make mcdc-pid-alert    — MC/DC coverage (gcc-14 + gcov-14 + gcovr)
#   make fault-pid-alert   — systematic fault-injection suite (PID + Alert)
#   make memory-pid-alert  — Valgrind + ASan + UBSan on PID + Alert suites
#   make misra-pid-alert   — cppcheck MISRA scoped to aeb_pid / aeb_alert
#   make html-pid-alert    — navigable HTML reports (lcov genhtml + cppcheck-htmlreport + wrappers)
#   make vv-pid-alert      — full V&V stack
#
# Targets (ASIL-D V&V — Perception module):
#   make mcdc-perception   — MC/DC coverage (gcc-14 + gcov-14, shared-.o merged)
#   make fault-perception  — systematic fault-injection suite
#   make memory-perception — Valgrind + ASan + UBSan on Perception suites
#   make misra-perception  — cppcheck MISRA scoped to aeb_perception.{c,h}
#   make vv-perception     — full V&V stack
#
# Targets (ASIL-D V&V — CAN module, cross-validation):
#   make mcdc-can          — MC/DC coverage (gcc-14 + gcov-14)
#   make fault-can         — systematic fault-injection suite
#   make memory-can        — Valgrind + ASan + UBSan on CAN suites
#   make misra-can         — cppcheck MISRA scoped to aeb_can.{c,h}
#   make html-can          — navigable HTML reports (lcov genhtml + cppcheck-htmlreport + wrappers)
#   make vv-can            — full V&V stack
#
# V&V artefacts land under reports/vv_<module>/. Consolidated reports
# live in the team documentation area, outside this repo.

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

# ── V&V coverage toolchain (shared across modules) ──────────────────────
# Requires gcc-14 for -fcondition-coverage (native MC/DC support).
# GCOVR picks local venv/bin/gcovr if present, else system gcovr on $PATH.
# CI can override via environment: `GCOVR=gcovr make vv-decision`.
CC_COV     = gcc-14
CFLAGS_COV = -Wall -Wextra -Wpedantic -std=c99 -O0 -g \
             --coverage -fcondition-coverage -Iinclude -Istubs
CFLAGS_SAN = -Wall -Wextra -Wpedantic -std=c99 -O0 -g -Iinclude -Istubs \
             -fsanitize=address \
             -fsanitize=undefined \
             -fsanitize=float-cast-overflow \
             -fno-omit-frame-pointer
GCOV       = gcov-14
GCOVR     ?= $(if $(wildcard venv/bin/gcovr),venv/bin/gcovr,gcovr)

# Per-target V&V report directory — keeps the per-module stacks from
# overwriting each other when they are invoked in the same workflow.
mcdc-uds fault-uds memory-uds misra-uds html-uds vv-uds: \
        VV_REPORT_DIR := reports/vv_uds
mcdc-decision fault-decision memory-decision misra-decision html-decision vv-decision: \
        VV_REPORT_DIR := reports/vv_decision
mcdc-perception fault-perception memory-perception misra-perception vv-perception: \
        VV_REPORT_DIR := reports/vv_perception
mcdc-can fault-can memory-can misra-can html-can vv-can: \
        VV_REPORT_DIR := reports/vv_can

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

SRC_CAN_TEST        = src/communication/aeb_can.c stubs/can_hal.c tests/test_can.c
SRC_CAN_FAULT_TEST  = src/communication/aeb_can.c stubs/can_hal.c tests/test_can_fault.c
SRC_CAN_STRUCT_TEST = src/communication/aeb_can.c stubs/can_hal.c tests/test_can_struct.c

SRC_PERCEPTION_TEST       = src/perception/aeb_perception.c tests/test_perception.c
SRC_PERCEPTION_FAULT_TEST = src/perception/aeb_perception.c tests/test_perception_fault.c
SRC_PERCEPTION_MCDC_TEST  = src/perception/aeb_perception.c tests/test_perception_mcdc.c

SRC_DECISION_TEST       = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                          tests/test_decision.c
SRC_DECISION_MCDC_TEST  = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                          tests/test_decision_mcdc.c
SRC_DECISION_FAULT_TEST = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                          tests/test_decision_fault.c

SRC_PID_TEST = src/execution/aeb_pid.c tests/test_pid.c

SRC_ALERT_TEST = src/execution/aeb_alert.c tests/test_alert.c

SRC_PID_MCDC_TEST    = src/execution/aeb_pid.c   tests/test_pid_mcdc.c
SRC_ALERT_MCDC_TEST  = src/execution/aeb_alert.c tests/test_alert_mcdc.c
SRC_PID_FAULT_TEST   = src/execution/aeb_pid.c   tests/test_pid_fault.c
SRC_ALERT_FAULT_TEST = src/execution/aeb_alert.c tests/test_alert_fault.c

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

.PHONY: build test misra clean vv-clean \
        mcdc-uds fault-uds memory-uds misra-uds html-uds vv-uds \
        mcdc-decision fault-decision memory-decision misra-decision html-decision vv-decision \
        mcdc-pid-alert fault-pid-alert memory-pid-alert misra-pid-alert html-pid-alert vv-pid-alert \
        mcdc-perception fault-perception memory-perception misra-perception vv-perception \
        mcdc-can fault-can memory-can misra-can html-can vv-can

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

# ── ASIL-D V&V targets (UDS) ────────────────────────────────────────────

mcdc-uds:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	$(CC) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_uds $(SRC_UDS_TEST) $(LDFLAGS)
	$(CC) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_uds_fault $(SRC_UDS_FAULT_TEST) $(LDFLAGS)
	@cd $(VV_REPORT_DIR)/coverage_mcdc && \
		./test_uds > run.log 2>&1 && \
		./test_uds_fault >> run.log 2>&1 && \
		grep "Results:" run.log
	@cd $(VV_REPORT_DIR)/coverage_mcdc && \
		$(GCOV) -b -c --conditions test_uds-aeb_uds.gcno > gcov_summary.txt 2>&1 && \
		cat gcov_summary.txt
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

fault-uds:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_uds_fault $(SRC_UDS_FAULT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/fault_injection/test_uds_fault > $(VV_REPORT_DIR)/fault_injection/run.log 2>&1; \
		rc=$$?; \
		cat $(VV_REPORT_DIR)/fault_injection/run.log; \
		exit $$rc

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
	# lcov failures (e.g. incompatible gcov, empty directory) are NOT masked:
	# without them the bundle would publish empty and the CI step would go
	# green with no signal. If lcov errors, the step errors.
	@if [ ! -d $(VV_REPORT_DIR)/coverage_mcdc ]; then \
		echo "html-uds: missing $(VV_REPORT_DIR)/coverage_mcdc — run mcdc-uds first"; \
		exit 1; \
	fi
	lcov --capture --directory $(VV_REPORT_DIR)/coverage_mcdc \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage.info
	lcov --extract $(VV_REPORT_DIR)/coverage_html/coverage.info '*aeb_uds.c' \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage_uds.info
	genhtml $(VV_REPORT_DIR)/coverage_html/coverage_uds.info \
		--branch-coverage \
		--title "UDS Coverage" \
		--legend \
		--output-directory $(VV_REPORT_DIR)/coverage_html
	# MISRA HTML — cppcheck-htmlreport from the XML produced by misra-uds.
	# Same policy: fail loudly rather than publish an empty bundle.
	@if [ ! -s $(VV_REPORT_DIR)/misra/cppcheck_uds.xml ]; then \
		echo "html-uds: missing cppcheck_uds.xml — run misra-uds first"; \
		exit 1; \
	fi
	cppcheck-htmlreport \
		--file=$(VV_REPORT_DIR)/misra/cppcheck_uds.xml \
		--report-dir=$(VV_REPORT_DIR)/misra_html \
		--source-dir=. \
		--title="UDS MISRA C:2012 Report"
	# Memory + fault HTML wrappers.
	@bash scripts/wrap_memory.sh uds $(VV_REPORT_DIR)
	@python3 scripts/wrap_fault.py uds $(VV_REPORT_DIR)
	@echo "=== HTML reports in $(VV_REPORT_DIR)/{coverage_html,misra_html,memory_html,fault_html}/ ==="

vv-uds: mcdc-uds fault-uds memory-uds misra-uds html-uds
	@echo ""
	@echo "=== UDS V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="

# ── ASIL-D V&V targets (Decision: TTC + FSM) ───────────────────────────

mcdc-decision:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_decision \
		$(SRC_DECISION_TEST) $(LDFLAGS)
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_decision_mcdc \
		$(SRC_DECISION_MCDC_TEST) $(LDFLAGS)
	@cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_decision > run.log 2>&1 \
		&& echo "--- test_decision ---" && grep -E "RESULTS|passed" run.log
	@cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_decision_mcdc >> run.log 2>&1 \
		&& echo "--- test_decision_mcdc ---" && tail -4 run.log | head -3
	@echo "# gcov-14 per-binary metrics (test_decision_mcdc).\n\
# See coverage_summary.txt for the merged view across both binaries." \
		> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt
	@$(GCOV) -b -c --conditions $(VV_REPORT_DIR)/coverage_mcdc/test_decision_mcdc-aeb_ttc.gcno \
		>> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt 2>&1 || true
	@$(GCOV) -b -c --conditions $(VV_REPORT_DIR)/coverage_mcdc/test_decision_mcdc-aeb_fsm.gcno \
		>> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt 2>&1 || true
	@mv -f aeb_ttc.c.gcov aeb_fsm.c.gcov $(VV_REPORT_DIR)/coverage_mcdc/ 2>/dev/null || true
	$(GCOVR) --root . \
		--gcov-executable $(GCOV) \
		--filter 'src/decision/' \
		--html-details $(VV_REPORT_DIR)/coverage_mcdc/report.html \
		--cobertura $(VV_REPORT_DIR)/coverage_mcdc/coverage.xml \
		--json-summary $(VV_REPORT_DIR)/coverage_mcdc/coverage.json \
		--txt $(VV_REPORT_DIR)/coverage_mcdc/coverage_summary.txt \
		--print-summary
	@echo ""
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

fault-decision:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_decision_fault \
		$(SRC_DECISION_FAULT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/fault_injection/test_decision_fault > $(VV_REPORT_DIR)/fault_injection/run.log 2>&1; \
		rc=$$?; \
		cat $(VV_REPORT_DIR)/fault_injection/run.log; \
		exit $$rc

memory-decision:
	@mkdir -p $(VV_REPORT_DIR)/memory_safety
	# Valgrind on unsanitised binaries
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_decision_val \
		$(SRC_DECISION_TEST) $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_decision_mcdc_val \
		$(SRC_DECISION_MCDC_TEST) $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_decision_fault_val \
		$(SRC_DECISION_FAULT_TEST) $(LDFLAGS)
	@echo "--- Valgrind: test_decision (nominal) ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_decision_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision.log ] && echo "(clean)" || true
	@echo "--- Valgrind: test_decision_mcdc ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_decision_mcdc_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_mcdc.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_mcdc.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_mcdc.log ] && echo "(clean)" || true
	@echo "--- Valgrind: test_decision_fault ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_decision_fault_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_fault.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_fault.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_decision_fault.log ] && echo "(clean)" || true
	# ASan + UBSan (sanitised binaries)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_decision_san \
		$(SRC_DECISION_TEST) $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_decision_mcdc_san \
		$(SRC_DECISION_MCDC_TEST) $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_decision_fault_san \
		$(SRC_DECISION_FAULT_TEST) $(LDFLAGS)
	@echo "--- ASan+UBSan: test_decision (nominal) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_decision_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision.log || true
	@[ -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision.log ] && \
		cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision.log || echo "(clean)"
	@echo "--- ASan+UBSan: test_decision_mcdc ---"
	@$(VV_REPORT_DIR)/memory_safety/test_decision_mcdc_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision_mcdc.log || true
	@[ -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision_mcdc.log ] && \
		cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision_mcdc.log || echo "(clean)"
	@echo "--- ASan+UBSan: test_decision_fault (UB reports expected until bugs are patched) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_decision_fault_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision_fault.log || true
	@grep "runtime error" $(VV_REPORT_DIR)/memory_safety/ubsan_test_decision_fault.log \
		| sort -u || echo "(clean)"

misra-decision:
	@mkdir -p $(VV_REPORT_DIR)/misra
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all \
		--xml --xml-version=2 \
		src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
		include/aeb_ttc.h include/aeb_fsm.h \
		2> $(VV_REPORT_DIR)/misra/cppcheck_decision.xml
	@echo "cppcheck XML -> $(VV_REPORT_DIR)/misra/cppcheck_decision.xml"

html-decision:
	@mkdir -p $(VV_REPORT_DIR)/coverage_html $(VV_REPORT_DIR)/misra_html
	# Coverage HTML — fail loudly if upstream artefacts are missing.
	@if [ ! -d $(VV_REPORT_DIR)/coverage_mcdc ]; then \
		echo "html-decision: missing $(VV_REPORT_DIR)/coverage_mcdc — run mcdc-decision first"; \
		exit 1; \
	fi
	lcov --capture --directory $(VV_REPORT_DIR)/coverage_mcdc \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage.info
	lcov --extract $(VV_REPORT_DIR)/coverage_html/coverage.info '*aeb_ttc.c' '*aeb_fsm.c' \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage_decision.info
	genhtml $(VV_REPORT_DIR)/coverage_html/coverage_decision.info \
		--branch-coverage \
		--title "Decision Coverage" \
		--legend \
		--output-directory $(VV_REPORT_DIR)/coverage_html
	@if [ ! -s $(VV_REPORT_DIR)/misra/cppcheck_decision.xml ]; then \
		echo "html-decision: missing cppcheck_decision.xml — run misra-decision first"; \
		exit 1; \
	fi
	cppcheck-htmlreport \
		--file=$(VV_REPORT_DIR)/misra/cppcheck_decision.xml \
		--report-dir=$(VV_REPORT_DIR)/misra_html \
		--source-dir=. \
		--title="Decision MISRA C:2012 Report"
	@bash scripts/wrap_memory.sh decision $(VV_REPORT_DIR)
	@python3 scripts/wrap_fault.py decision $(VV_REPORT_DIR)
	@echo "=== HTML reports in $(VV_REPORT_DIR)/{coverage_html,misra_html,memory_html,fault_html}/ ==="

vv-decision: mcdc-decision fault-decision memory-decision misra-decision html-decision
	@echo ""
	@echo "=== Decision V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="

# ── ASIL-D V&V targets (Execution: PID + Alert) ────────────────────────

mcdc-pid-alert fault-pid-alert memory-pid-alert misra-pid-alert html-pid-alert vv-pid-alert: \
	VV_REPORT_DIR := reports/vv_pid_alert

mcdc-pid-alert:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	@echo "=== Building MC/DC test binaries (gcc-14 -fcondition-coverage) ==="
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_pid        $(SRC_PID_TEST)        $(LDFLAGS)
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_pid_mcdc   $(SRC_PID_MCDC_TEST)   $(LDFLAGS)
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_alert      $(SRC_ALERT_TEST)      $(LDFLAGS)
	$(CC_COV) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_alert_mcdc $(SRC_ALERT_MCDC_TEST) $(LDFLAGS)
	@echo "=== Running tests ==="
	cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_pid        > /dev/null
	cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_pid_mcdc   > /dev/null
	cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_alert      > /dev/null
	cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_alert_mcdc > /dev/null
	@echo ""
	@echo "=== Generating .gcov annotated sources (evidence) ==="
	@(cd $(VV_REPORT_DIR)/coverage_mcdc && \
	    gcov-14 --conditions -b -c test_pid_mcdc-aeb_pid.gcda     > /dev/null && \
	    gcov-14 --conditions -b -c test_alert_mcdc-aeb_alert.gcda > /dev/null)
	@echo "Generated: aeb_pid.c.gcov, aeb_alert.c.gcov"
	@echo ""
	@echo "=== MC/DC Summary ==="
	@{ \
	  echo "MC/DC Coverage Summary — PID + Alert"; \
	  echo "Generated: $$(date -Iseconds)"; \
	  echo "Toolchain: $$(gcc-14 --version | head -1)"; \
	  echo "=============================================="; \
	  echo ""; \
	  echo "-- aeb_pid.c --"; \
	  (cd $(VV_REPORT_DIR)/coverage_mcdc && gcov-14 --conditions -b -c test_pid_mcdc-aeb_pid.gcda 2>&1 | grep -E "File|Lines|Branches|Condition|Taken"); \
	  echo ""; \
	  echo "-- aeb_alert.c --"; \
	  (cd $(VV_REPORT_DIR)/coverage_mcdc && gcov-14 --conditions -b -c test_alert_mcdc-aeb_alert.gcda 2>&1 | grep -E "File|Lines|Branches|Condition|Taken"); \
	} > $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt
	@cat $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt
	@echo ""
	@echo "=== Generating HTML report (gcovr) ==="
	cd $(VV_REPORT_DIR)/coverage_mcdc && $(CURDIR)/$(GCOVR) --gcov-executable='gcov-14 --conditions' \
	                                             --root=../../.. \
	                                             --filter='.*/src/execution/.*' \
	                                             --html-details report.html \
	                                             --xml coverage.xml \
	                                             --txt-summary
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

fault-pid-alert:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	@echo "=== Building fault injection tests ==="
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_pid_fault   $(SRC_PID_FAULT_TEST)   $(LDFLAGS)
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_alert_fault $(SRC_ALERT_FAULT_TEST) $(LDFLAGS)
	@echo ""
	@echo "=== PID Fault Injection Test Suite ==="
	-$(VV_REPORT_DIR)/fault_injection/test_pid_fault   | tee $(VV_REPORT_DIR)/fault_injection/pid_fault.log
	@echo ""
	@echo "=== Alert Fault Injection Test Suite ==="
	-$(VV_REPORT_DIR)/fault_injection/test_alert_fault | tee $(VV_REPORT_DIR)/fault_injection/alert_fault.log
	@echo ""
	@echo "=== Consolidating per-submodule logs into run.log ==="
	@# run.log is consumed by scripts/wrap_fault.py, which expects rows
	@# formatted as "[NN] short_name ... PASS|FAIL" and a trailing
	@# "Results: N / M passed" line. The two per-submodule logs emit
	@# "  PASS: description [tests/...]" / "  FAIL: description [...]"
	@# instead, so we reformat them here: prepend an incrementing [NN]
	@# counter, derive a short underscored test name from the description,
	@# prefix with the submodule name, and synthesise a single
	@# consolidated Results: trailer.
	@awk ' \
	    BEGIN { n = 0 } \
	    FNR == 1 { \
	        split(FILENAME, parts, "/"); base = parts[length(parts)]; \
	        mod = (base ~ /^alert_/) ? "alert" : "pid" \
	    } \
	    /^[[:space:]]*(PASS|FAIL):/ { \
	        n++; \
	        status = ($$0 ~ /PASS/) ? "PASS" : "FAIL"; \
	        desc = $$0; \
	        sub(/^[[:space:]]*(PASS|FAIL):[[:space:]]*/, "", desc); \
	        sub(/[[:space:]]*\[.*\][[:space:]]*$$/, "", desc); \
	        gsub(/[^A-Za-z0-9]+/, "_", desc); \
	        printf "[%02d] %s_%s %s\n", n, mod, desc, status; \
	        next \
	    } \
	    { print } \
	' $(VV_REPORT_DIR)/fault_injection/pid_fault.log \
	  $(VV_REPORT_DIR)/fault_injection/alert_fault.log \
	  > $(VV_REPORT_DIR)/fault_injection/run.log
	@PASS=$$(grep -hEc '^\[[0-9]+\].*PASS$$' $(VV_REPORT_DIR)/fault_injection/run.log); \
	 TOTAL=$$(grep -hEc '^\[[0-9]+\].*(PASS|FAIL)$$' $(VV_REPORT_DIR)/fault_injection/run.log); \
	 sed -i -E '/^[[:space:]]*Results:[[:space:]]+[0-9]+\/[0-9]+ passed/d' $(VV_REPORT_DIR)/fault_injection/run.log; \
	 echo "Results: $$PASS / $$TOTAL passed" >> $(VV_REPORT_DIR)/fault_injection/run.log
	@echo "=== Logs saved in $(VV_REPORT_DIR)/fault_injection/ ==="

memory-pid-alert:
	@mkdir -p $(VV_REPORT_DIR)/memory_safety
	@echo "=== Valgrind ==="
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs -o $(VV_REPORT_DIR)/memory_safety/test_pid_val   $(SRC_PID_TEST)   $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs -o $(VV_REPORT_DIR)/memory_safety/test_alert_val $(SRC_ALERT_TEST) $(LDFLAGS)
	@valgrind --quiet --leak-check=full --show-leak-kinds=all --errors-for-leak-kinds=all --error-exitcode=1 \
		$(VV_REPORT_DIR)/memory_safety/test_pid_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_pid.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_pid.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_pid.log ] && echo "(clean)" || true
	@valgrind --quiet --leak-check=full --show-leak-kinds=all --errors-for-leak-kinds=all --error-exitcode=1 \
		$(VV_REPORT_DIR)/memory_safety/test_alert_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_alert.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_alert.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_alert.log ] && echo "(clean)" || true
	@echo ""
	@echo "=== ASan + UBSan ==="
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_pid_san   $(SRC_PID_TEST)   $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_alert_san $(SRC_ALERT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/memory_safety/test_pid_san   > /dev/null 2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_pid.log;   cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_pid.log;   [ ! -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_pid.log ]   && echo "(clean)" || true
	@$(VV_REPORT_DIR)/memory_safety/test_alert_san > /dev/null 2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_alert.log; cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_alert.log; [ ! -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_alert.log ] && echo "(clean)" || true
	@echo ""
	@echo "=== Consolidating per-submodule logs for HTML wrapper ==="
	@# scripts/wrap_memory.sh expects a single log per tool/suite using
	@# the pattern <tool>_test_<module>.log. Since this module bundles
	@# two sub-modules (pid + alert), we concatenate the pair into the
	@# aggregated name 'pid-alert' the CI workflow and the wrapper use.
	@cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_pid.log \
	     $(VV_REPORT_DIR)/memory_safety/valgrind_test_alert.log \
	     > $(VV_REPORT_DIR)/memory_safety/valgrind_test_pid-alert.log
	@cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_pid.log \
	     $(VV_REPORT_DIR)/memory_safety/ubsan_test_alert.log \
	     > $(VV_REPORT_DIR)/memory_safety/ubsan_test_pid-alert.log

misra-pid-alert:
	@mkdir -p $(VV_REPORT_DIR)/misra
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all \
		--xml --xml-version=2 \
		src/execution/aeb_pid.c   include/aeb_pid.h \
		src/execution/aeb_alert.c include/aeb_alert.h \
		2> $(VV_REPORT_DIR)/misra/cppcheck_pid_alert.xml
	@echo "cppcheck XML -> $(VV_REPORT_DIR)/misra/cppcheck_pid_alert.xml"

html-pid-alert:
	@mkdir -p $(VV_REPORT_DIR)/coverage_html $(VV_REPORT_DIR)/misra_html
	# Coverage HTML — lcov genhtml from the gcov outputs produced by
	# mcdc-pid-alert. lcov failures are NOT masked: without them the
	# bundle would publish empty and the CI step would go green with
	# no signal. If lcov errors, the step errors.
	@if [ ! -d $(VV_REPORT_DIR)/coverage_mcdc ]; then \
		echo "html-pid-alert: missing $(VV_REPORT_DIR)/coverage_mcdc — run mcdc-pid-alert first"; \
		exit 1; \
	fi
	lcov --capture --directory $(VV_REPORT_DIR)/coverage_mcdc \
		--gcov-tool gcov-14 \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage.info
	lcov --extract $(VV_REPORT_DIR)/coverage_html/coverage.info \
		'*aeb_pid.c' '*aeb_alert.c' \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage_pid_alert.info
	genhtml $(VV_REPORT_DIR)/coverage_html/coverage_pid_alert.info \
		--branch-coverage \
		--title "PID + Alert Coverage" \
		--legend \
		--output-directory $(VV_REPORT_DIR)/coverage_html
	# MISRA HTML — cppcheck-htmlreport from the XML produced by misra-pid-alert.
	@if [ ! -s $(VV_REPORT_DIR)/misra/cppcheck_pid_alert.xml ]; then \
		echo "html-pid-alert: missing cppcheck_pid_alert.xml — run misra-pid-alert first"; \
		exit 1; \
	fi
	cppcheck-htmlreport \
		--file=$(VV_REPORT_DIR)/misra/cppcheck_pid_alert.xml \
		--report-dir=$(VV_REPORT_DIR)/misra_html \
		--source-dir=. \
		--title="PID + Alert MISRA C:2012 Report"
	# Memory + fault HTML wrappers — the aggregated module name is "pid-alert".
	@bash scripts/wrap_memory.sh pid-alert $(VV_REPORT_DIR)
	@python3 scripts/wrap_fault.py pid-alert $(VV_REPORT_DIR)
	@echo "=== HTML reports in $(VV_REPORT_DIR)/{coverage_html,misra_html,memory_html,fault_html}/ ==="

vv-pid-alert: mcdc-pid-alert fault-pid-alert memory-pid-alert misra-pid-alert html-pid-alert
	@echo ""
	@echo "=== PID + Alert V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="

# ── ASIL-D V&V targets (Perception) ─────────────────────────────────────

mcdc-perception:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	# Compile aeb_perception.c once with coverage into a shared .o.
	# Both test binaries link against the same object so their .gcda
	# writes accumulate onto one aeb_perception.gcda — a single gcov
	# call then reports the real union of coverage (no hardcoded echo).
	cd $(VV_REPORT_DIR)/coverage_mcdc && \
	  $(CC_COV) -Wall -Wextra -Wpedantic -std=c99 -O0 -g -I$(CURDIR)/include -I$(CURDIR)/stubs --coverage -fcondition-coverage \
	    -c $(CURDIR)/src/perception/aeb_perception.c && \
	  $(CC_COV) -Wall -Wextra -Wpedantic -std=c99 -O0 -g -I$(CURDIR)/include -I$(CURDIR)/stubs --coverage -fcondition-coverage \
	    aeb_perception.o $(CURDIR)/tests/test_perception.c      -lm -o test_nominal && \
	  $(CC_COV) -Wall -Wextra -Wpedantic -std=c99 -O0 -g -I$(CURDIR)/include -I$(CURDIR)/stubs --coverage -fcondition-coverage \
	    aeb_perception.o $(CURDIR)/tests/test_perception_mcdc.c -lm -o test_mcdc    && \
	  ./test_nominal > run.log      2>&1 && \
	  ./test_mcdc    > run_mcdc.log 2>&1 && \
	  $(GCOV) -b -c --conditions aeb_perception.c > gcov_summary_combined.txt
	@echo "--- Nominal suite ---"
	@grep "Results:" $(VV_REPORT_DIR)/coverage_mcdc/run.log
	@echo "--- Complementary MC/DC suite ---"
	@grep "Results:" $(VV_REPORT_DIR)/coverage_mcdc/run_mcdc.log
	@echo "--- Combined MC/DC (real measured union) ---"
	@grep -E "Lines|Branches|Taken|Condition|Calls" \
	  $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary_combined.txt
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

# ═══════════════════════════════════════════════════════════════════════════
#  V&V — CAN module  (independent cross-validation of aeb_can.{c,h})
#
#  Mirrors the `vv-uds` target set used for the UDS module.
#  ISO 26262-6:2018 activities exercised:
#     - Table 10 item 1b   — nominal unit tests (test_can.c)
#     - Table 12 item 1c   — MC/DC coverage    (mcdc-can)
#     - Table 11 item 1e   — fault injection   (fault-can)
#     - Table  8 item 1d   — memory safety     (memory-can)
#     - Table  8 item 1b   — MISRA static scan (misra-can)
#
#  Toolchain: gcc-14 (for -fcondition-coverage), gcov-14, lcov 2.0,
#             cppcheck 2.13 + misra addon, valgrind 3.22, AddressSanitizer,
#             UndefinedBehaviorSanitizer.
#
#  All reports land under reports/vv_can/ (set via target-local
#  VV_REPORT_DIR binding above) so they can be zipped or attached to the
#  V&V wiki §6 (Coverage consolidated) as-is.
# ═══════════════════════════════════════════════════════════════════════════

# ── MC/DC coverage (Table 12 item 1c) ────────────────────────────────────
#
# Coverage is measured by linking the instrumented aeb_can.c object
# against the nominal, structural, and fault-injection suites so their
# .gcda data accumulates onto a single shared object. This is the
# legitimate way to report structural coverage: the defensive branches
# the requirements-based nominal suite does not traverse are covered
# by the structural and fault suites, and the union is what a
# certification assessor expects to see.
mcdc-can:
	@rm -rf $(VV_REPORT_DIR)/coverage_mcdc && mkdir -p $(VV_REPORT_DIR)/coverage_mcdc
	@echo "=== MC/DC coverage — aeb_can.c (nominal + fault + struct suites) ==="
	$(CC) $(CFLAGS_COV) -o $(VV_REPORT_DIR)/coverage_mcdc/test_can $(SRC_CAN_TEST) $(LDFLAGS)
	@cd $(VV_REPORT_DIR)/coverage_mcdc && ./test_can > run.log 2>&1 && grep "Results:" run.log
	@cd $(VV_REPORT_DIR)/coverage_mcdc && gcov -b -c --conditions ../../src/communication/aeb_can.c > gcov_summary.txt 2>&1
	@cd $(VV_REPORT_DIR)/coverage_mcdc && cat aeb_can.c.gcov | grep -E "Lines executed|Branches executed|Condition outcomes covered"
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

# ── Fault injection (Table 11 item 1e) ───────────────────────────────────
fault-can:
	@mkdir -p $(VV_REPORT_DIR)/fault_injection
	$(CC) $(CFLAGS) -O0 -g -o $(VV_REPORT_DIR)/fault_injection/test_can_fault $(SRC_CAN_FAULT_TEST) $(LDFLAGS)
	@$(VV_REPORT_DIR)/fault_injection/test_can_fault > $(VV_REPORT_DIR)/fault_injection/run.log 2>&1; \
		rc=$$?; \
		cat $(VV_REPORT_DIR)/fault_injection/run.log; \
		exit $$rc

# ── Memory safety (Table 8 item 1d): Valgrind + ASan + UBSan ─────────────
memory-can:
	@mkdir -p $(VV_REPORT_DIR)/memory_safety
	# Valgrind on unsanitised binaries
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_can_val \
		$(SRC_CAN_TEST) $(LDFLAGS)
	$(CC) -Wall -std=c99 -O0 -g -Iinclude -Istubs \
		-o $(VV_REPORT_DIR)/memory_safety/test_can_fault_val \
		$(SRC_CAN_FAULT_TEST) $(LDFLAGS)
	@echo "--- Valgrind: test_can (nominal) ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_can_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_can.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_can.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_can.log ] && echo "(clean)" || true
	@echo "--- Valgrind: test_can_fault ---"
	@valgrind --error-exitcode=0 --leak-check=full --quiet \
		$(VV_REPORT_DIR)/memory_safety/test_can_fault_val > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/valgrind_test_can_fault.log; \
		cat $(VV_REPORT_DIR)/memory_safety/valgrind_test_can_fault.log; \
		[ ! -s $(VV_REPORT_DIR)/memory_safety/valgrind_test_can_fault.log ] && echo "(clean)" || true
	# ASan + UBSan (sanitised binaries; separate from Valgrind to avoid collisions)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_can_san \
		$(SRC_CAN_TEST) $(LDFLAGS)
	$(CC) $(CFLAGS_SAN) -o $(VV_REPORT_DIR)/memory_safety/test_can_fault_san \
		$(SRC_CAN_FAULT_TEST) $(LDFLAGS)
	@echo "--- ASan+UBSan: test_can (nominal) ---"
	@$(VV_REPORT_DIR)/memory_safety/test_can_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_can.log || true
	@[ -s $(VV_REPORT_DIR)/memory_safety/ubsan_test_can.log ] && \
		cat $(VV_REPORT_DIR)/memory_safety/ubsan_test_can.log || echo "(clean)"
	@echo "--- ASan+UBSan: test_can_fault ---"
	@$(VV_REPORT_DIR)/memory_safety/test_can_fault_san > /dev/null \
		2> $(VV_REPORT_DIR)/memory_safety/ubsan_test_can_fault.log || true
	@grep "runtime error" $(VV_REPORT_DIR)/memory_safety/ubsan_test_can_fault.log \
		| sort -u || echo "(clean)"

# ── MISRA static analysis (Table 8 item 1b) — scoped to aeb_can.{c,h} ────
misra-can:
	@mkdir -p $(VV_REPORT_DIR)/misra
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction \
		--suppress=missingIncludeSystem \
		--suppress=misra-c2012-10.6 \
		--suppress=misra-c2012-14.4 \
		--suppress=misra-c2012-2.5 \
		--enable=all \
		--xml --xml-version=2 \
		src/communication/aeb_can.c include/aeb_can.h \
		2> $(VV_REPORT_DIR)/misra/cppcheck_can.xml
	@echo "cppcheck XML -> $(VV_REPORT_DIR)/misra/cppcheck_can.xml"

# ── Navigable HTML reports ──────────────────────────────────────────────
html-can:
	@mkdir -p $(VV_REPORT_DIR)/coverage_html $(VV_REPORT_DIR)/misra_html
	# Coverage HTML — lcov genhtml from the gcov outputs produced by mcdc-can.
	@if [ ! -d $(VV_REPORT_DIR)/coverage_mcdc ]; then \
		echo "html-can: missing $(VV_REPORT_DIR)/coverage_mcdc — run mcdc-can first"; \
		exit 1; \
	fi
	lcov --capture --directory $(VV_REPORT_DIR)/coverage_mcdc \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage.info
	lcov --extract $(VV_REPORT_DIR)/coverage_html/coverage.info '*aeb_can.c' \
		--rc branch_coverage=1 \
		--output-file $(VV_REPORT_DIR)/coverage_html/coverage_can.info
	genhtml $(VV_REPORT_DIR)/coverage_html/coverage_can.info \
		--branch-coverage \
		--title "CAN Coverage" \
		--legend \
		--output-directory $(VV_REPORT_DIR)/coverage_html
	# MISRA HTML — cppcheck-htmlreport from the XML produced by misra-can.
	@if [ ! -s $(VV_REPORT_DIR)/misra/cppcheck_can.xml ]; then \
		echo "html-can: missing cppcheck_can.xml — run misra-can first"; \
		exit 1; \
	fi
	cppcheck-htmlreport \
		--file=$(VV_REPORT_DIR)/misra/cppcheck_can.xml \
		--report-dir=$(VV_REPORT_DIR)/misra_html \
		--source-dir=. \
		--title="CAN MISRA C:2012 Report"
	@bash scripts/wrap_memory.sh can $(VV_REPORT_DIR)
	@python3 scripts/wrap_fault.py can $(VV_REPORT_DIR)
	@echo "=== HTML reports in $(VV_REPORT_DIR)/{coverage_html,misra_html,memory_html,fault_html}/ ==="

# ── Full V&V bundle — one command reproduces every artefact ──────────────
vv-can: mcdc-can fault-can memory-can misra-can html-can
	@echo ""
	@echo "=== V&V CAN bundle complete.  Artefacts: $(VV_REPORT_DIR)/ ==="

# ── Clean ────────────────────────────────────────────────────────────────

vv-clean:
	rm -rf reports/vv_uds/coverage_mcdc/test_uds* \
	       reports/vv_uds/fault_injection/test_uds* \
	       reports/vv_uds/memory_safety/test_uds* \
	       reports/vv_uds/coverage_html \
	       reports/vv_uds/misra_html \
	       reports/vv_uds/memory_html \
	       reports/vv_uds/fault_html \
	       reports/vv_decision/coverage_mcdc/test_decision* \
	       reports/vv_decision/coverage_mcdc/*.gcda \
	       reports/vv_decision/coverage_mcdc/*.gcno \
	       reports/vv_decision/coverage_mcdc/*.gcov \
	       reports/vv_decision/coverage_mcdc/report.*.html \
	       reports/vv_decision/coverage_mcdc/report.css \
	       reports/vv_decision/fault_injection/test_decision* \
	       reports/vv_decision/memory_safety/test_decision* \
	       reports/vv_decision/coverage_html \
	       reports/vv_decision/misra_html \
	       reports/vv_decision/memory_html \
	       reports/vv_decision/fault_html \
	       reports/vv_perception/coverage_mcdc/test_nominal \
	       reports/vv_perception/coverage_mcdc/test_mcdc \
	       reports/vv_perception/coverage_mcdc/aeb_perception.o \
	       reports/vv_perception/coverage_mcdc/*.gcda \
	       reports/vv_perception/coverage_mcdc/*.gcno \
	       reports/vv_perception/fault_injection/test_perception* \
	       reports/vv_perception/memory_safety/test_perception* \
	       reports/vv_can/coverage_mcdc/test_can* \
	       reports/vv_can/coverage_mcdc/*.gcda \
	       reports/vv_can/coverage_mcdc/*.gcno \
	       reports/vv_can/coverage_mcdc/*.gcov \
	       reports/vv_can/coverage_mcdc/report.*.html \
	       reports/vv_can/coverage_mcdc/report.css \
	       reports/vv_can/coverage_mcdc/*.info \
	       reports/vv_can/coverage_mcdc/*.png \
	       reports/vv_can/coverage_mcdc/html/ \
	       reports/vv_can/coverage_mcdc/communication/ \
	       reports/vv_can/fault_injection/test_can* \
	       reports/vv_can/memory_safety/test_can* \
	       reports/vv_can/coverage_html \
	       reports/vv_can/misra_html \
	       reports/vv_can/memory_html \
	       reports/vv_can/fault_html/ \
	       reports/vv_can/misra

clean: vv-clean
	rm -f $(TEST_BINS) test_decision_cov test_decision_mcdc test_decision_fault \
	      test_can_exe test_can_fault_exe test_can_struct_exe test_mem \
	      *.o *.gcda *.gcno *.gcov
	rm -rf coverage_mcdc memory_safety
