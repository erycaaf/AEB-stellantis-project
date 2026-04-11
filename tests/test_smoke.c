/**
 * @file  test_smoke.c
 * @brief Smoke tests — verify all stubs compile and basic init works.
 *
 * This is the baseline CI test. Each team member adds their own
 * test file (test_perception.c, test_fsm.c, etc.) via PR.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_can.h"
#include <stdio.h>
#include <string.h>

static int32_t tests_run    = 0;
static int32_t tests_passed = 0;
static int32_t tests_failed = 0;

#define ASSERT_EQ(a, b) do { \
    if ((a) == (b)) { tests_passed++; } \
    else { printf("  FAIL: %s:%d  %s != %s\n", __FILE__, __LINE__, #a, #b); tests_failed++; } \
    tests_run++; \
} while (0)

#define TEST(name) static void name(void)
#define RUN(name) do { printf("  [TEST] %s\n", #name); name(); } while (0)

/* Verify aeb_types.h structs can be instantiated */
TEST(test_types_instantiation)
{
    perception_output_t perc = {0};
    ttc_output_t        ttc  = {0};
    fsm_output_t        fsm  = {0};
    pid_output_t        pid  = {0};
    alert_output_t      alrt = {0};
    driver_input_t      drv  = {0};

    ASSERT_EQ(perc.fault_flag, 0U);
    ASSERT_EQ(ttc.is_closing, 0U);
    ASSERT_EQ(fsm.fsm_state, 0U);
    ASSERT_EQ(pid.brake_pct, 0.0F);
    ASSERT_EQ(alrt.alert_type, 0U);
    ASSERT_EQ(drv.aeb_enabled, 0U);
}

/* Verify CAN init stub returns OK */
TEST(test_can_init_stub)
{
    can_state_t state;
    int32_t rc = can_init(&state);
    ASSERT_EQ(rc, CAN_OK);
}

/* Verify FSM enum values match specification */
TEST(test_fsm_enum_values)
{
    ASSERT_EQ((int)FSM_OFF, 0);
    ASSERT_EQ((int)FSM_STANDBY, 1);
    ASSERT_EQ((int)FSM_WARNING, 2);
    ASSERT_EQ((int)FSM_BRAKE_L1, 3);
    ASSERT_EQ((int)FSM_BRAKE_L2, 4);
    ASSERT_EQ((int)FSM_BRAKE_L3, 5);
    ASSERT_EQ((int)FSM_POST_BRAKE, 6);
}

int main(void)
{
    printf("=== AEB Smoke Tests ===\n\n");

    RUN(test_types_instantiation);
    RUN(test_can_init_stub);
    RUN(test_fsm_enum_values);

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}
