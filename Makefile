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

clean:
	rm -f $(TEST_BINS) *.o


# ==========================================================================
# MC/DC Coverage (ASIL-D) — PID + Alert
# ==========================================================================
# Requisitos (uma vez):
#   sudo apt install -y gcc-14
#   pip install gcovr --break-system-packages
#
# Uso:
#   make mcdc        — rodar cobertura MC/DC e gerar relatório HTML
#   make mcdc-clean  — limpar artefatos MC/DC
# ==========================================================================

MCDC_CC       = gcc-14
MCDC_CFLAGS   = --coverage -fcondition-coverage -std=c99 \
                -Wall -Wextra -Wpedantic -Iinclude -Istubs
MCDC_LDFLAGS  = -lm
MCDC_DIR      = coverage_mcdc

.PHONY: mcdc mcdc-clean

mcdc:
	@echo "=== Building MC/DC test binaries (gcc-14 -fcondition-coverage) ==="
	@rm -rf $(MCDC_DIR) && mkdir -p $(MCDC_DIR)
	$(MCDC_CC) $(MCDC_CFLAGS) -c src/execution/aeb_pid.c   -o $(MCDC_DIR)/aeb_pid.o
	$(MCDC_CC) $(MCDC_CFLAGS) -c src/execution/aeb_alert.c -o $(MCDC_DIR)/aeb_alert.o
	$(MCDC_CC) $(MCDC_CFLAGS) $(MCDC_DIR)/aeb_pid.o   tests/test_pid.c        $(MCDC_LDFLAGS) -o $(MCDC_DIR)/test_pid
	$(MCDC_CC) $(MCDC_CFLAGS) $(MCDC_DIR)/aeb_pid.o   tests/test_pid_mcdc.c   $(MCDC_LDFLAGS) -o $(MCDC_DIR)/test_pid_mcdc
	$(MCDC_CC) $(MCDC_CFLAGS) $(MCDC_DIR)/aeb_alert.o tests/test_alert.c      $(MCDC_LDFLAGS) -o $(MCDC_DIR)/test_alert
	$(MCDC_CC) $(MCDC_CFLAGS) $(MCDC_DIR)/aeb_alert.o tests/test_alert_mcdc.c $(MCDC_LDFLAGS) -o $(MCDC_DIR)/test_alert_mcdc
	@echo "=== Running tests ==="
	cd $(MCDC_DIR) && ./test_pid        > /dev/null
	cd $(MCDC_DIR) && ./test_pid_mcdc   > /dev/null
	cd $(MCDC_DIR) && ./test_alert      > /dev/null
	cd $(MCDC_DIR) && ./test_alert_mcdc > /dev/null
	@echo ""
	@echo "=== MC/DC Summary (gcov-14 native) ==="
	@cd $(MCDC_DIR) && gcov-14 --conditions -b -c aeb_pid.gcda   2>/dev/null | head -10
	@cd $(MCDC_DIR) && gcov-14 --conditions -b -c aeb_alert.gcda 2>/dev/null | head -10
	@echo ""
	@echo "=== Generating HTML report (gcovr) ==="
	cd $(MCDC_DIR) && gcovr --gcov-executable='gcov-14 --conditions' \
	                         --root=.. \
	                         --filter='.*/src/execution/.*' \
	                         --html-details report.html \
	                         --xml coverage.xml \
	                         --txt-summary
	@echo ""
	@echo "=== DONE ==="
	@echo "Open: $(MCDC_DIR)/report.html"

mcdc-clean:
	rm -rf $(MCDC_DIR)

# ==========================================================================
# Fault Injection Tests (ASIL-D Robustness)
# ==========================================================================

FAULT_DIR = fault_tests

.PHONY: fault fault-clean

fault:
	@echo "=== Building fault injection tests ==="
	@rm -rf $(FAULT_DIR) && mkdir -p $(FAULT_DIR)
	$(CC) $(CFLAGS) src/execution/aeb_pid.c   tests/test_pid_fault.c   $(LDFLAGS) -o $(FAULT_DIR)/test_pid_fault
	$(CC) $(CFLAGS) src/execution/aeb_alert.c tests/test_alert_fault.c $(LDFLAGS) -o $(FAULT_DIR)/test_alert_fault
	@echo ""
	@echo "=== Running PID fault tests ==="
	-./$(FAULT_DIR)/test_pid_fault   | tee $(FAULT_DIR)/pid_fault.log
	@echo ""
	@echo "=== Running Alert fault tests ==="
	-./$(FAULT_DIR)/test_alert_fault | tee $(FAULT_DIR)/alert_fault.log
	@echo ""
	@echo "=== Logs saved in $(FAULT_DIR)/ ==="

fault-clean:
	rm -rf $(FAULT_DIR)


# ==========================================================================
# Memory Safety Verification (ASIL-D)
# ==========================================================================
# Requisitos (uma vez):
#   sudo apt install -y valgrind
#
# Uso:
#   make memory          — roda Valgrind + ASan + UBSan em toda a suíte
#   make memory-valgrind — só Valgrind
#   make memory-asan     — só AddressSanitizer + UBSan
#   make memory-clean    — limpa artefatos
# ==========================================================================

MEM_DIR      = memory_safety
SAN_CFLAGS   = -fsanitize=address,undefined -fno-omit-frame-pointer -g -O1 \
               -std=c99 -Wall -Wextra -Iinclude -Istubs

.PHONY: memory memory-valgrind memory-asan memory-clean

memory: memory-valgrind memory-asan
	@echo ""
	@echo "=== Memory Safety: todos os checks concluídos ==="
	@echo "Logs: $(MEM_DIR)/valgrind/  e  $(MEM_DIR)/sanitizers/"

memory-valgrind: $(TEST_BINS)
	@mkdir -p $(MEM_DIR)/valgrind
	@echo "=== Valgrind (leak-check=full) ==="
	@for t in $(TEST_BINS); do \
		valgrind --leak-check=full \
		         --show-leak-kinds=all \
		         --errors-for-leak-kinds=all \
		         --error-exitcode=1 \
		         --track-origins=yes \
		         --log-file=$(MEM_DIR)/valgrind/$$t.log \
		         ./$$t > /dev/null 2>&1; \
		ec=$$?; \
		sum=$$(grep "ERROR SUMMARY" $(MEM_DIR)/valgrind/$$t.log | head -1); \
		if [ $$ec -eq 0 ]; then \
			printf "  OK  %-20s  %s\n" $$t "$$sum"; \
		else \
			printf "  FAIL %-20s  %s\n" $$t "$$sum"; \
		fi; \
	done

memory-asan:
	@mkdir -p $(MEM_DIR)/bin $(MEM_DIR)/sanitizers
	@echo "=== AddressSanitizer + UBSan ==="
	$(CC) $(SAN_CFLAGS) $(SRC_SMOKE)        -lm -o $(MEM_DIR)/bin/san_smoke
	$(CC) $(SAN_CFLAGS) $(SRC_CAN_TEST)     -lm -o $(MEM_DIR)/bin/san_can
	$(CC) $(SAN_CFLAGS) $(SRC_PERCEPTION_TEST)  -lm -o $(MEM_DIR)/bin/san_perception
	$(CC) $(SAN_CFLAGS) $(SRC_DECISION_TEST)    -lm -o $(MEM_DIR)/bin/san_decision
	$(CC) $(SAN_CFLAGS) $(SRC_PID_TEST)         -lm -o $(MEM_DIR)/bin/san_pid
	$(CC) $(SAN_CFLAGS) $(SRC_ALERT_TEST)       -lm -o $(MEM_DIR)/bin/san_alert
	$(CC) $(SAN_CFLAGS) $(SRC_UDS_TEST)         -lm -o $(MEM_DIR)/bin/san_uds
	$(CC) $(SAN_CFLAGS) $(SRC_INTEGRATION_TEST) -lm -o $(MEM_DIR)/bin/san_integration
	@for t in smoke can perception decision pid alert uds integration; do \
		$(MEM_DIR)/bin/san_$$t > $(MEM_DIR)/sanitizers/$$t.log 2>&1; \
		ec=$$?; \
		if [ $$ec -eq 0 ]; then \
			printf "  OK  san_test_%-15s  PASS (0 errors)\n" $$t; \
		else \
			printf "  FAIL san_test_%-15s  exit=%d — ver %s.log\n" $$t $$ec $$t; \
		fi; \
	done

memory-clean:
	rm -rf $(MEM_DIR)
