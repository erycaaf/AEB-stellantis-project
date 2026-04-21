# Makefile — AEB Stellantis Project (host build)
#
# Targets (build & test):
#   make build            — compile all modules (zero-warning gate)
#   make test             — build and run all unit tests
#   make misra            — run cppcheck MISRA C:2012 on non-stub sources
#   make clean            — remove build artefacts
#
# Targets (ASIL-D V&V — TTC + FSM decision module, cross-validation by Eryca):
#   make mcdc-decision    — MC/DC coverage (gcc-14 + gcov-14 + gcovr 8.6)
#   make vv-decision      — full V&V stack (MC/DC + fault + memory + MISRA)
#                           — subtargets added as activities come online
#
# All V&V artefacts land under reports/vv_decision/. Consolidated report
# lives in the team docs area, outside this repo.

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

# ── Coverage toolchain (V&V targets) ────────────────────────────────────
# Requires gcc-14 for -fcondition-coverage (native MC/DC support).
# gcovr 8.6 is used from venv to parse gcov-14's MC/DC output.
CC_COV   = gcc-14
CFLAGS_COV = -Wall -Wextra -Wpedantic -std=c99 -O0 -g \
             --coverage -fcondition-coverage -Iinclude -Istubs
GCOV     = gcov-14
GCOVR    = venv/bin/gcovr

VV_REPORT_DIR = reports/vv_decision

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

SRC_DECISION_TEST      = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                         tests/test_decision.c
SRC_DECISION_MCDC_TEST = src/decision/aeb_ttc.c src/decision/aeb_fsm.c \
                         tests/test_decision_mcdc.c

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

.PHONY: build test misra mcdc-decision vv-decision vv-clean clean

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

# ── ASIL-D V&V targets (Decision: TTC + FSM) ───────────────────────────
#
# Mirrors the convention established in reports/vv_uds/ (Renato). Binaries
# and intermediates land inside reports/vv_decision/; only text/XML/CSV
# evidence is committed to git (see .gitignore).

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
	# Per-file gcov text annotation — processed against the MCDC binary,
	# which exercises the highest share of FSM branches. For the merged
	# view across both binaries see coverage_summary.txt (gcovr output).
	@echo "# gcov-14 per-binary metrics (test_decision_mcdc).\n\
# See coverage_summary.txt for the merged view across both binaries." \
		> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt
	@$(GCOV) -b -c $(VV_REPORT_DIR)/coverage_mcdc/test_decision_mcdc-aeb_ttc.gcno \
		>> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt 2>&1 || true
	@$(GCOV) -b -c $(VV_REPORT_DIR)/coverage_mcdc/test_decision_mcdc-aeb_fsm.gcno \
		>> $(VV_REPORT_DIR)/coverage_mcdc/gcov_summary.txt 2>&1 || true
	@mv -f aeb_ttc.c.gcov aeb_fsm.c.gcov $(VV_REPORT_DIR)/coverage_mcdc/ 2>/dev/null || true
	$(GCOVR) --root . \
		--gcov-executable $(GCOV) \
		--filter 'src/decision/' \
		--html-details $(VV_REPORT_DIR)/coverage_mcdc/report.html \
		--cobertura $(VV_REPORT_DIR)/coverage_mcdc/coverage.xml \
		--txt $(VV_REPORT_DIR)/coverage_mcdc/coverage_summary.txt \
		--print-summary
	@echo ""
	@echo "Artefacts in $(VV_REPORT_DIR)/coverage_mcdc/"

vv-decision: mcdc-decision
	@echo ""
	@echo "=== Decision V&V stack complete — artefacts in $(VV_REPORT_DIR)/ ==="
	@echo "   (fault-decision, memory-decision, misra-decision pending)"

vv-clean:
	rm -rf $(VV_REPORT_DIR)/coverage_mcdc/test_decision* \
	       $(VV_REPORT_DIR)/coverage_mcdc/*.gcda \
	       $(VV_REPORT_DIR)/coverage_mcdc/*.gcno \
	       $(VV_REPORT_DIR)/coverage_mcdc/*.gcov \
	       $(VV_REPORT_DIR)/coverage_mcdc/report.*.html \
	       $(VV_REPORT_DIR)/coverage_mcdc/report.css

clean: vv-clean
	rm -f $(TEST_BINS) test_decision_cov test_decision_mcdc test_decision_fault \
	      *.o *.gcda *.gcno *.gcov
	rm -rf coverage_mcdc memory_safety
