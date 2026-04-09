# Makefile — AEB Stellantis Project (host build)
#
# Targets:
#   make build     — compile all modules (zero-warning gate)
#   make test      — build and run unit tests
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
          stubs/can_hal.c

# Smoke test (baseline — always present)
SRC_SMOKE = src/communication/aeb_can.c stubs/can_hal.c tests/test_smoke.c

# CAN module tests (Task D)
SRC_CAN_TEST = src/communication/aeb_can.c stubs/can_hal.c tests/test_can.c

# Real module sources for MISRA check (add files here as stubs are replaced)
SRC_MISRA = src/communication/aeb_can.c

.PHONY: build test misra clean

build:
	$(CC) $(CFLAGS) -c $(SRC_ALL)
	@echo "=== Build OK: zero warnings ==="

test: test_smoke test_can
	./test_smoke
	./test_can

test_smoke: $(SRC_SMOKE)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_can: $(SRC_CAN_TEST)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

misra:
ifneq ($(SRC_MISRA),)
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs --suppress=unusedFunction --suppress=missingIncludeSystem --enable=all $(SRC_MISRA)
else
	@echo "=== No real modules to check yet (all stubs) ==="
endif

clean:
	rm -f test_smoke test_can *.o

.PHONY: test_can
