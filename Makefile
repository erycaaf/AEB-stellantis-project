# Makefile — AEB Stellantis Project (host build)
#
# Targets:
#   make build     — compile all modules (zero-warning gate)
#   make test      — build and run all unit tests
#   make misra     — run cppcheck MISRA C:2012 on non-stub sources
#   make clean     — remove build artefacts

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

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

.PHONY: build test misra clean

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

# ═══════════════════════════════════════════════════════════════════════════
#  V&V — CAN module  (cross-validation by Lourenço; original author: Renato)
#
#  Mirrors the `vv-uds` target set produced by Renato for the UDS module.
#  ISO 26262-6:2018 activities exercised:
#     - Table 10 item 1b   — nominal unit tests (test_can.c, Lourenço)
#     - Table 12 item 1c   — MC/DC coverage    (mcdc-can)
#     - Table 11 item 1e   — fault injection   (fault-can)
#     - Table  8 item 1d   — memory safety     (memory-can)
#     - Table  8 item 1b   — MISRA static scan (misra-can)
#
#  Toolchain: gcc-14 (for -fcondition-coverage), gcov-14, lcov 2.0,
#             cppcheck 2.13 + misra addon, valgrind 3.22, AddressSanitizer,
#             UndefinedBehaviorSanitizer.
#
#  All reports land under reports/vv_can/ so they can be zipped or
#  attached to the V&V wiki §6 (Coverage consolidated) as-is.
# ═══════════════════════════════════════════════════════════════════════════

GCC14        ?= gcc-14
GCOV14       ?= gcov-14

VV_CAN_DIR    = reports/vv_can
CAN_SRC       = src/communication/aeb_can.c stubs/can_hal.c
CAN_TEST_SRC  = tests/test_can.c
CAN_FAULT_SRC = tests/test_can_fault.c
CAN_STRUCT_SRC = tests/test_can_struct.c

.PHONY: mcdc-can fault-can memory-can misra-can vv-can vv-can-clean

# ── MC/DC coverage (Table 12 item 1c) ────────────────────────────────────
#
# Coverage is measured with BOTH the nominal suite (Lourenço) and the
# fault injection suite (Eryca) running against the same instrumented
# binary.  This mirrors Renato's approach for UDS and is the legitimate
# way to report structural coverage: a requirement-based test that
# traverses the defensive-branch can only be the fault test, so the
# two suites must be merged for a fair number.
mcdc-can:
	@mkdir -p $(VV_CAN_DIR)/coverage_mcdc
	@echo "=== MC/DC coverage — aeb_can.c (nominal + fault suites) ==="
	$(GCC14) -Wall -Wextra -std=c99 -O0 -g \
		-fprofile-arcs -ftest-coverage -fcondition-coverage \
		-Iinclude -Istubs \
		$(CAN_SRC) $(CAN_TEST_SRC) \
		-o $(VV_CAN_DIR)/coverage_mcdc/test_can_cov $(LDFLAGS)
	$(GCC14) -Wall -Wextra -std=c99 -O0 -g \
		-fprofile-arcs -ftest-coverage -fcondition-coverage \
		-Iinclude -Istubs \
		$(CAN_SRC) $(CAN_FAULT_SRC) \
		-o $(VV_CAN_DIR)/coverage_mcdc/test_can_fault_cov $(LDFLAGS)
	$(GCC14) -Wall -Wextra -std=c99 -O0 -g \
		-fprofile-arcs -ftest-coverage -fcondition-coverage \
		-Iinclude -Istubs \
		$(CAN_SRC) $(CAN_STRUCT_SRC) \
		-o $(VV_CAN_DIR)/coverage_mcdc/test_can_struct_cov $(LDFLAGS)
	cd $(VV_CAN_DIR)/coverage_mcdc && \
		./test_can_cov        > run_nominal.log 2>&1 || true ; \
		./test_can_fault_cov  > run_fault.log   2>&1 || true ; \
		./test_can_struct_cov > run_struct.log  2>&1 || true
	cd $(VV_CAN_DIR)/coverage_mcdc && \
		$(GCOV14) --conditions --branch-probabilities --branch-counts \
		          test_can_cov-aeb_can.gcno \
		          > gcov_summary.txt 2>&1 || true
	cd $(VV_CAN_DIR)/coverage_mcdc && \
		lcov --capture --directory . --rc geninfo_unexecuted_blocks=1 \
		     --gcov-tool $(GCOV14) --output-file can.info 2>/dev/null || true
	cd $(VV_CAN_DIR)/coverage_mcdc && \
		genhtml can.info --output-directory html 2>/dev/null || true
	@echo "Artefacts: $(VV_CAN_DIR)/coverage_mcdc/"

# ── Fault injection (Table 11 item 1e) ───────────────────────────────────
fault-can:
	@mkdir -p $(VV_CAN_DIR)/fault_injection
	@echo "=== Fault injection — aeb_can.c ==="
	$(CC) $(CFLAGS) -o $(VV_CAN_DIR)/fault_injection/test_can_fault \
		$(CAN_SRC) $(CAN_FAULT_SRC) $(LDFLAGS)
	cd $(VV_CAN_DIR)/fault_injection && ./test_can_fault > run.log 2>&1 || true
	@echo "Artefacts: $(VV_CAN_DIR)/fault_injection/run.log"

# ── Memory safety (Table 8 item 1d): Valgrind + ASan + UBSan ─────────────
memory-can:
	@mkdir -p $(VV_CAN_DIR)/memory_safety
	@echo "=== Memory safety — Valgrind on test_can ==="
	$(CC) -g -O0 -Wall -Wextra -std=c99 -Iinclude -Istubs \
		$(CAN_SRC) $(CAN_TEST_SRC) -o $(VV_CAN_DIR)/memory_safety/test_can_dbg $(LDFLAGS)
	valgrind --leak-check=full --error-exitcode=0 --track-origins=yes \
		$(VV_CAN_DIR)/memory_safety/test_can_dbg \
		> $(VV_CAN_DIR)/memory_safety/valgrind_nominal.log 2>&1 || true
	@echo "=== Memory safety — Valgrind on test_can_fault ==="
	$(CC) -g -O0 -Wall -Wextra -std=c99 -Iinclude -Istubs \
		$(CAN_SRC) $(CAN_FAULT_SRC) -o $(VV_CAN_DIR)/memory_safety/test_can_fault_dbg $(LDFLAGS)
	valgrind --leak-check=full --error-exitcode=0 --track-origins=yes \
		$(VV_CAN_DIR)/memory_safety/test_can_fault_dbg \
		> $(VV_CAN_DIR)/memory_safety/valgrind_fault.log 2>&1 || true
	@echo "=== Memory safety — ASan + UBSan on test_can ==="
	$(CC) -g -O0 -Wall -Wextra -std=c99 -Iinclude -Istubs \
		-fsanitize=address,undefined -fsanitize=float-cast-overflow \
		-fno-sanitize-recover=float-cast-overflow \
		$(CAN_SRC) $(CAN_TEST_SRC) -o $(VV_CAN_DIR)/memory_safety/test_can_san $(LDFLAGS)
	$(VV_CAN_DIR)/memory_safety/test_can_san \
		> $(VV_CAN_DIR)/memory_safety/ubsan_nominal.log 2>&1 || true
	@echo "=== Memory safety — ASan + UBSan on test_can_fault ==="
	$(CC) -g -O0 -Wall -Wextra -std=c99 -Iinclude -Istubs \
		-fsanitize=address,undefined -fsanitize=float-cast-overflow \
		$(CAN_SRC) $(CAN_FAULT_SRC) -o $(VV_CAN_DIR)/memory_safety/test_can_fault_san $(LDFLAGS)
	$(VV_CAN_DIR)/memory_safety/test_can_fault_san \
		> $(VV_CAN_DIR)/memory_safety/ubsan_fault.log 2>&1 || true
	@echo "Artefacts: $(VV_CAN_DIR)/memory_safety/*.log"

# ── MISRA static analysis (Table 8 item 1b) — scoped to aeb_can.{c,h} ────
misra-can:
	@mkdir -p $(VV_CAN_DIR)/misra
	@echo "=== MISRA static scan — aeb_can.{c,h} (scoped) ==="
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all --xml --xml-version=2 \
		src/communication/aeb_can.c include/aeb_can.h \
		2> $(VV_CAN_DIR)/misra/cppcheck.xml || true
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
		--suppress=unusedFunction --suppress=missingIncludeSystem \
		--enable=all \
		src/communication/aeb_can.c include/aeb_can.h \
		> $(VV_CAN_DIR)/misra/cppcheck.log 2>&1 || true
	@echo "Artefacts: $(VV_CAN_DIR)/misra/cppcheck.{xml,log}"

# ── Full V&V bundle — one command reproduces every artefact ──────────────
vv-can: mcdc-can fault-can memory-can misra-can
	@echo ""
	@echo "=== V&V CAN bundle complete.  Artefacts: $(VV_CAN_DIR)/ ==="

# vv-can-clean removes generated artefacts but KEEPS hand-written files
# (the .tex/.pdf report, the CSV, the README).  Use `rm -rf reports/vv_can/`
# if you want a full wipe.
vv-can-clean:
	rm -rf $(VV_CAN_DIR)/coverage_mcdc
	rm -rf $(VV_CAN_DIR)/fault_injection
	rm -rf $(VV_CAN_DIR)/memory_safety
	rm -rf $(VV_CAN_DIR)/misra

clean:
	rm -f $(TEST_BINS) *.o *.gcda *.gcno *.gcov
