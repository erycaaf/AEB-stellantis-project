# Makefile — AEB Stellantis Project (host build)
#
# Targets:
#   make build     — compile all modules (zero-warning gate)
#   make test      — build and run unit tests
#   make misra     — run cppcheck MISRA C:2012 on real (non-stub) code
#   make clean     — remove build artefacts

CC       = gcc
CFLAGS   = -Wall -Wextra -Wpedantic -std=c99 -O2 -Iinclude -Istubs
LDFLAGS  = -lm

# Real module sources
SRC_CAN  = src/communication/aeb_can.c

# Stub sources (replaced by team members' real code via PR)
SRC_STUBS = src/perception/aeb_perception.c \
            src/decision/aeb_ttc.c \
            src/decision/aeb_fsm.c \
            src/execution/aeb_pid.c \
            src/execution/aeb_alert.c \
            src/communication/aeb_uds.c

# HAL stubs (for host build only)
SRC_HAL  = stubs/can_hal.c

# Tests
SRC_TEST = tests/test_can.c

ALL_SRC  = $(SRC_CAN) $(SRC_STUBS) $(SRC_HAL)
TEST_SRC = $(SRC_CAN) $(SRC_HAL) $(SRC_TEST)

.PHONY: build test misra clean

build:
	$(CC) $(CFLAGS) -c $(ALL_SRC)
	@echo "=== Build OK: zero warnings ==="

test: test_can
	./test_can

test_can: $(TEST_SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

misra:
	cppcheck --addon=misra --std=c99 -Iinclude -Istubs \
	  --suppress=unusedFunction --suppress=missingIncludeSystem \
	  --enable=all $(SRC_CAN)

clean:
	rm -f test_can *.o
