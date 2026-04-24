/**
 * @file    test_alert_fault.c
 * @brief   Systematic fault injection tests for aeb_alert module.
 * @author  Rian Ithalo (cross-validation — ISO 26262 independence)
 * @iso26262 Part 6, §9.4.2 Table 11 item 1e (fault injection, ++ for ASIL-D)
 */

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "aeb_alert.h"
#include "aeb_config.h"
#include "aeb_types.h"

static int g_test_count = 0;
static int g_pass_count = 0;
static int g_fail_count = 0;

#define TEST_ASSERT(cond, msg)                                               \
    do {                                                                      \
        g_test_count++;                                                       \
        if (cond) {                                                           \
            g_pass_count++;                                                   \
            printf("  PASS: %s\n", (msg));                                    \
        } else {                                                              \
            g_fail_count++;                                                   \
            printf("  FAIL: %s [%s:%d]\n", (msg), __FILE__, __LINE__);        \
        }                                                                     \
    } while (0)

/* ============ CATEGORY A: Corrupted FSM state ============ */

static void test_fault_alert_invalid_state_7(void)
{
    printf("\n[FAULT-A1] alert_map(7) — one past valid range\n");
    alert_output_t out = {0};
    out.alert_type = 99U; out.alert_active = 99U; out.buzzer_cmd = 99U;
    alert_map(7U, &out);
    TEST_ASSERT(out.alert_type   == (uint8_t)ALERT_NONE,
        "fsm_state=7 -> ALERT_NONE (silent default)");
    TEST_ASSERT(out.alert_active == 0U,
        "fsm_state=7 -> alert_active=0");
    TEST_ASSERT(out.buzzer_cmd   == (uint8_t)BUZZER_SILENT,
        "fsm_state=7 -> BUZZER_SILENT");
}

static void test_fault_alert_invalid_state_99(void)
{
    printf("\n[FAULT-A2] alert_map(99) — grossly out of range\n");
    alert_output_t out = {0};
    alert_map(99U, &out);
    TEST_ASSERT(out.alert_type == (uint8_t)ALERT_NONE,
        "fsm_state=99 -> ALERT_NONE (silent default)");
    TEST_ASSERT(out.alert_active == 0U,
        "fsm_state=99 -> alert_active=0");
}

static void test_fault_alert_invalid_state_uint8_max(void)
{
    printf("\n[FAULT-A3] alert_map(255) — UINT8_MAX (bit-flip scenario)\n");
    alert_output_t out = {0};
    alert_map(UINT8_MAX, &out);
    TEST_ASSERT(out.alert_type == (uint8_t)ALERT_NONE,
        "fsm_state=255 -> ALERT_NONE (silent default)");
    TEST_ASSERT(out.alert_active == 0U,
        "fsm_state=255 -> alert_active=0");
    TEST_ASSERT(out.buzzer_cmd == (uint8_t)BUZZER_SILENT,
        "fsm_state=255 -> BUZZER_SILENT");
}

/* ============ CATEGORY B: Non-finite steering inputs ============ */

static void test_fault_override_steering_nan(void)
{
    printf("\n[FAULT-B1] override_detect(NaN, 0)\n");
    uint8_t result = override_detect(NAN, 0U);
    TEST_ASSERT(result == 0U || result == 1U,
        "override_detect returns valid 0/1 even with NaN input");
}

static void test_fault_override_steering_positive_inf(void)
{
    printf("\n[FAULT-B2] override_detect(+Inf, 0)\n");
    uint8_t result = override_detect(INFINITY, 0U);
    TEST_ASSERT(result == 1U,
        "override_detect(+Inf) treats as override");
}

static void test_fault_override_steering_negative_inf(void)
{
    printf("\n[FAULT-B3] override_detect(-Inf, 0)\n");
    uint8_t result = override_detect(-INFINITY, 0U);
    TEST_ASSERT(result == 1U,
        "override_detect(-Inf) treats as override");
}

/* ============ CATEGORY C: Extreme finite steering values ============ */

static void test_fault_override_steering_huge_positive(void)
{
    printf("\n[FAULT-C1] override_detect(1e20, 0)\n");
    uint8_t result = override_detect(1.0e20F, 0U);
    TEST_ASSERT(result == 1U,
        "huge positive angle = override");
}

static void test_fault_override_steering_huge_negative(void)
{
    printf("\n[FAULT-C2] override_detect(-1e20, 0)\n");
    uint8_t result = override_detect(-1.0e20F, 0U);
    TEST_ASSERT(result == 1U,
        "huge negative angle = override");
}

static void test_fault_override_boundary_exact(void)
{
    printf("\n[FAULT-C3] override_detect at exact 5.0 deg boundary\n");
    uint8_t r_pos = override_detect( 5.0F, 0U);
    uint8_t r_neg = override_detect(-5.0F, 0U);
    TEST_ASSERT(r_pos == 0U,
        "steering = +5.0 (exact threshold): no override");
    TEST_ASSERT(r_neg == 0U,
        "steering = -5.0 (exact threshold): no override");
    uint8_t r_above = override_detect(5.01F, 0U);
    TEST_ASSERT(r_above == 1U,
        "steering = 5.01 (just above): override active");
}

/* ============ CATEGORY D: Abnormal brake_pedal encoding ============ */

static void test_fault_brake_pedal_value_2(void)
{
    printf("\n[FAULT-D1] override_detect(0, 2) — unexpected pedal value\n");
    uint8_t result = override_detect(0.0F, 2U);
    TEST_ASSERT(result == 1U,
        "brake_pedal=2 triggers override (safe default)");
}

static void test_fault_brake_pedal_uint8_max(void)
{
    printf("\n[FAULT-D2] override_detect(0, 255) — corrupted pedal signal\n");
    uint8_t result = override_detect(0.0F, UINT8_MAX);
    TEST_ASSERT(result == 1U,
        "brake_pedal=255 triggers override (safe default)");
}

int main(void)
{
    printf("========================================\n");
    printf("  AEB Alert — Fault Injection Tests\n");
    printf("  (ASIL-D Robustness — ISO 26262-6 §9.4.2)\n");
    printf("========================================\n");

    test_fault_alert_invalid_state_7();
    test_fault_alert_invalid_state_99();
    test_fault_alert_invalid_state_uint8_max();
    test_fault_override_steering_nan();
    test_fault_override_steering_positive_inf();
    test_fault_override_steering_negative_inf();
    test_fault_override_steering_huge_positive();
    test_fault_override_steering_huge_negative();
    test_fault_override_boundary_exact();
    test_fault_brake_pedal_value_2();
    test_fault_brake_pedal_uint8_max();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}