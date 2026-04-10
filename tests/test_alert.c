/**
 * @file    test_alert.c
 * @brief   Unit tests for aeb_alert module.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-09
 *
 * @details Tests alert mapping and override detection against
 *          FR-ALR-001 to FR-ALR-004, FR-DEC-006, FR-DEC-007.
 *
 * @requirements FR-ALR-001 to FR-ALR-004, FR-DEC-006, FR-DEC-007
 */

#include <stdio.h>
#include "aeb_alert.h"
#include "aeb_config.h"

/* ========================================================================= */
/*  Minimal test framework                                                   */
/* ========================================================================= */

static int g_test_count  = 0;
static int g_pass_count  = 0;
static int g_fail_count  = 0;

#define TEST_ASSERT(cond, msg)                                              \
    do {                                                                     \
        g_test_count++;                                                      \
        if (cond) {                                                          \
            g_pass_count++;                                                  \
            printf("  PASS: %s\n", (msg));                                   \
        } else {                                                             \
            g_fail_count++;                                                  \
            printf("  FAIL: %s [%s:%d]\n", (msg), __FILE__, __LINE__);       \
        }                                                                    \
    } while (0)

#define TEST_ASSERT_EQ(val, expected, msg)                                  \
    TEST_ASSERT((val) == (expected), (msg))

/* ========================================================================= */
/*  Alert mapping tests                                                      */
/* ========================================================================= */

/**
 * @brief FR-ALR-004: OFF state — no alert.
 */
static void test_alert_off(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: OFF state\n");
    alert_map((uint8_t)FSM_OFF, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_NONE,
        "alert_type == NONE in OFF");
    TEST_ASSERT_EQ(out.alert_active, 0U,
        "alert_active == 0 in OFF");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_SILENT,
        "buzzer == SILENT in OFF");
}

/**
 * @brief FR-ALR-004: STANDBY state — no alert.
 */
static void test_alert_standby(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: STANDBY state\n");
    alert_map((uint8_t)FSM_STANDBY, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_NONE,
        "alert_type == NONE in STANDBY");
    TEST_ASSERT_EQ(out.alert_active, 0U,
        "alert_active == 0 in STANDBY");
}

/**
 * @brief FR-ALR-001, FR-ALR-002: WARNING — visual + audible alert.
 */
static void test_alert_warning(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: WARNING state\n");
    alert_map((uint8_t)FSM_WARNING, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_BOTH,
        "alert_type == BOTH in WARNING (visual + audible)");
    TEST_ASSERT_EQ(out.alert_active, 1U,
        "alert_active == 1 in WARNING");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_SINGLE,
        "buzzer == SINGLE in WARNING");
}

/**
 * @brief BRAKE_L1 — both alerts, double beep.
 */
static void test_alert_brake_l1(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: BRAKE_L1 state\n");
    alert_map((uint8_t)FSM_BRAKE_L1, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_BOTH,
        "alert_type == BOTH in BRAKE_L1");
    TEST_ASSERT_EQ(out.alert_active, 1U,
        "alert_active == 1 in BRAKE_L1");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_DOUBLE,
        "buzzer == DOUBLE in BRAKE_L1");
}

/**
 * @brief BRAKE_L2 — both alerts, fast pulse.
 */
static void test_alert_brake_l2(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: BRAKE_L2 state\n");
    alert_map((uint8_t)FSM_BRAKE_L2, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_BOTH,
        "alert_type == BOTH in BRAKE_L2");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_FAST_PULSE,
        "buzzer == FAST_PULSE in BRAKE_L2");
}

/**
 * @brief BRAKE_L3 — both alerts, continuous beep.
 */
static void test_alert_brake_l3(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: BRAKE_L3 state\n");
    alert_map((uint8_t)FSM_BRAKE_L3, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_BOTH,
        "alert_type == BOTH in BRAKE_L3");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_CONTINUOUS,
        "buzzer == CONTINUOUS in BRAKE_L3");
}

/**
 * @brief FR-ALR-004: POST_BRAKE — no alert.
 */
static void test_alert_post_brake(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: POST_BRAKE state\n");
    alert_map((uint8_t)FSM_POST_BRAKE, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_NONE,
        "alert_type == NONE in POST_BRAKE");
    TEST_ASSERT_EQ(out.alert_active, 0U,
        "alert_active == 0 in POST_BRAKE");
    TEST_ASSERT_EQ(out.buzzer_cmd, (uint8_t)BUZZER_SILENT,
        "buzzer == SILENT in POST_BRAKE");
}

/**
 * @brief Invalid state — defaults to no alert (MISRA default case).
 */
static void test_alert_invalid_state(void)
{
    alert_output_t out = {0U, 0U, 0U};

    printf("\n[TEST] alert: invalid state (99)\n");
    alert_map(99U, &out);

    TEST_ASSERT_EQ(out.alert_type, (uint8_t)ALERT_NONE,
        "alert_type == NONE for invalid state");
    TEST_ASSERT_EQ(out.alert_active, 0U,
        "alert_active == 0 for invalid state");
}

/* ========================================================================= */
/*  Override detection tests                                                 */
/* ========================================================================= */

/**
 * @brief No override: no pedal, no steering.
 */
static void test_override_none(void)
{
    printf("\n[TEST] override: no inputs\n");
    uint8_t result = override_detect(0.0F, 0U);
    TEST_ASSERT_EQ(result, 0U, "no override when all inputs inactive");
}

/**
 * @brief FR-DEC-006: Brake pedal override.
 */
static void test_override_brake_pedal(void)
{
    printf("\n[TEST] override: brake pedal\n");
    uint8_t result = override_detect(0.0F, 1U);
    TEST_ASSERT_EQ(result, 1U, "override active on brake pedal");
}

/**
 * @brief FR-DEC-007: Steering override > 5 degrees positive.
 */
static void test_override_steering_positive(void)
{
    printf("\n[TEST] override: steering > +5 deg\n");
    uint8_t result = override_detect(6.0F, 0U);
    TEST_ASSERT_EQ(result, 1U, "override active at +6 degrees");
}

/**
 * @brief FR-DEC-007: Steering override > 5 degrees negative.
 */
static void test_override_steering_negative(void)
{
    printf("\n[TEST] override: steering < -5 deg\n");
    uint8_t result = override_detect(-7.0F, 0U);
    TEST_ASSERT_EQ(result, 1U, "override active at -7 degrees");
}

/**
 * @brief No override at exactly 5 degrees (boundary — not exceeded).
 */
static void test_override_steering_boundary(void)
{
    printf("\n[TEST] override: steering exactly 5 deg (boundary)\n");
    uint8_t result = override_detect(5.0F, 0U);
    TEST_ASSERT_EQ(result, 0U, "no override at exactly 5 degrees (not >5)");
}

/**
 * @brief No override at 4.9 degrees.
 */
static void test_override_steering_below(void)
{
    printf("\n[TEST] override: steering 4.9 deg (below threshold)\n");
    uint8_t result = override_detect(4.9F, 0U);
    TEST_ASSERT_EQ(result, 0U, "no override at 4.9 degrees");
}

/**
 * @brief Both brake and steering active — still override.
 */
static void test_override_both(void)
{
    printf("\n[TEST] override: both brake + steering\n");
    uint8_t result = override_detect(10.0F, 1U);
    TEST_ASSERT_EQ(result, 1U, "override active with both inputs");
}

/* ========================================================================= */
/*  Main                                                                     */
/* ========================================================================= */

int main(void)
{
    printf("========================================\n");
    printf("  AEB Alert & Override — Unit Tests\n");
    printf("========================================\n");

    /* Alert mapping */
    test_alert_off();
    test_alert_standby();
    test_alert_warning();
    test_alert_brake_l1();
    test_alert_brake_l2();
    test_alert_brake_l3();
    test_alert_post_brake();
    test_alert_invalid_state();

    /* Override detection */
    test_override_none();
    test_override_brake_pedal();
    test_override_steering_positive();
    test_override_steering_negative();
    test_override_steering_boundary();
    test_override_steering_below();
    test_override_both();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}
