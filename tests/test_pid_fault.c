/**
 * @file    test_pid_fault.c
 * @brief   Systematic fault injection tests for aeb_pid module.
 * @author  Rian Ithalo (cross-validation — ISO 26262 independence)
 * @iso26262 Part 6, §9.4.2 Table 11 item 1e (fault injection, ++ for ASIL-D)
 */

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "aeb_pid.h"
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

static int is_safe_output(float32_t value, float32_t lo, float32_t hi)
{
    if (isnan(value)) { return 0; }
    if (isinf(value)) { return 0; }
    if (value < lo)   { return 0; }
    if (value > hi)   { return 0; }
    return 1;
}

/* ============ CATEGORY A: Non-finite float inputs ============ */

static void test_fault_a_ego_nan(void)
{
    printf("\n[FAULT-A1] a_ego = NaN (faulty accelerometer)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, NAN, (uint8_t)FSM_BRAKE_L2, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct stays in [0,100] (no NaN propagation)");
    TEST_ASSERT(is_safe_output(out.brake_bar, 0.0F, 10.0F),
        "brake_bar stays in [0,10] (no NaN propagation)");
}

static void test_fault_a_ego_positive_infinity(void)
{
    printf("\n[FAULT-A2] a_ego = +Infinity (saturated sensor)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, INFINITY, (uint8_t)FSM_BRAKE_L2, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct stays in [0,100] with a_ego=+Inf");
    TEST_ASSERT(is_safe_output(out.brake_bar, 0.0F, 10.0F),
        "brake_bar stays in [0,10] with a_ego=+Inf");
}

static void test_fault_a_ego_negative_infinity(void)
{
    printf("\n[FAULT-A3] a_ego = -Infinity (sensor stuck low)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, -INFINITY, (uint8_t)FSM_BRAKE_L2, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct stays in [0,100] with a_ego=-Inf");
    TEST_ASSERT(is_safe_output(out.brake_bar, 0.0F, 10.0F),
        "brake_bar stays in [0,10] with a_ego=-Inf");
}

static void test_fault_decel_target_nan(void)
{
    printf("\n[FAULT-A4] decel_target = NaN (corrupted FSM output)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(NAN, 0.0F, (uint8_t)FSM_BRAKE_L1, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct stays in [0,100] with decel_target=NaN");
}

/* ============ CATEGORY B: Boundary extremes ============ */

static void test_fault_a_ego_extreme_positive(void)
{
    printf("\n[FAULT-B1] a_ego = 100 m/s^2 (impossible but finite)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, 100.0F, (uint8_t)FSM_BRAKE_L2, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct clamped to [0,100] under extreme a_ego");
}

static void test_fault_decel_target_extreme(void)
{
    printf("\n[FAULT-B2] decel_target = 1000 m/s^2 (impossible command)\n");
    pid_init();
    pid_output_t out = {0};
    for (int i = 0; i < 100; i++)
        pid_brake_step(1000.0F, 0.0F, (uint8_t)FSM_BRAKE_L3, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct saturates at 100 under extreme demand");
    TEST_ASSERT(out.brake_pct <= 100.0F + 0.001F,
        "brake_pct never exceeds BRAKE_OUT_MAX");
}

static void test_fault_float_max(void)
{
    printf("\n[FAULT-B3] a_ego = FLT_MAX (numerical overflow test)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, FLT_MAX, (uint8_t)FSM_BRAKE_L2, &out);
    TEST_ASSERT(is_safe_output(out.brake_pct, 0.0F, 100.0F),
        "brake_pct stays in [0,100] with a_ego=FLT_MAX");
}

/* ============ CATEGORY C: Corrupted FSM state ============ */

static void test_fault_fsm_state_out_of_range(void)
{
    printf("\n[FAULT-C1] fsm_state = 99 (invalid state)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, 0.0F, 99U, &out);
    TEST_ASSERT(out.brake_pct == 0.0F,
        "invalid fsm_state produces zero braking (fail-safe)");
    TEST_ASSERT(out.brake_bar == 0.0F,
        "invalid fsm_state produces zero pressure");
}

static void test_fault_fsm_state_uint8_max(void)
{
    printf("\n[FAULT-C2] fsm_state = 255 (UINT8_MAX, bit-flip scenario)\n");
    pid_init();
    pid_output_t out = {0};
    pid_brake_step(4.0F, 0.0F, UINT8_MAX, &out);
    TEST_ASSERT(out.brake_pct == 0.0F,
        "fsm_state=255 produces zero braking (fail-safe)");
}

/* ============ CATEGORY D: State persistence / recovery ============ */

static void test_fault_integrator_reset_through_invalid_state(void)
{
    printf("\n[FAULT-D1] integrator reset after invalid-state excursion\n");
    pid_init();
    pid_output_t out = {0};

    for (int i = 0; i < 200; i++)
        pid_brake_step(6.0F, 0.0F, (uint8_t)FSM_BRAKE_L3, &out);
    float32_t brake_accumulated = out.brake_pct;
    TEST_ASSERT(brake_accumulated > 10.0F,
        "precondition: integrator accumulated braking output");

    pid_brake_step(0.0F, 0.0F, 42U, &out);
    TEST_ASSERT(out.brake_pct == 0.0F,
        "invalid state forces reset: brake_pct drops to 0");

    pid_brake_step(6.0F, 0.0F, (uint8_t)FSM_BRAKE_L3, &out);
    TEST_ASSERT(out.brake_pct < brake_accumulated,
        "no stale integrator carry-over after invalid-state excursion");
}

static void test_fault_jerk_limit_holds_under_fault(void)
{
    printf("\n[FAULT-D2] jerk limit holds even with extreme target change\n");
    pid_init();
    pid_output_t out = {0};

    for (int i = 0; i < 5; i++)
        pid_brake_step(2.0F, 0.0F, (uint8_t)FSM_BRAKE_L1, &out);
    float32_t pct_before = out.brake_pct;

    pid_brake_step(100.0F, 0.0F, (uint8_t)FSM_BRAKE_L3, &out);
    float32_t delta = out.brake_pct - pct_before;

    float32_t max_allowed = MAX_JERK * (BRAKE_OUT_MAX / DECEL_L3) * AEB_DT;
    TEST_ASSERT(delta <= max_allowed + 0.01F,
        "jerk limiter holds even under spike input (FR-BRK-004)");
}

int main(void)
{
    printf("========================================\n");
    printf("  AEB PID — Fault Injection Tests\n");
    printf("  (ASIL-D Robustness — ISO 26262-6 §9.4.2)\n");
    printf("========================================\n");

    test_fault_a_ego_nan();
    test_fault_a_ego_positive_infinity();
    test_fault_a_ego_negative_infinity();
    test_fault_decel_target_nan();
    test_fault_a_ego_extreme_positive();
    test_fault_decel_target_extreme();
    test_fault_float_max();
    test_fault_fsm_state_out_of_range();
    test_fault_fsm_state_uint8_max();
    test_fault_integrator_reset_through_invalid_state();
    test_fault_jerk_limit_holds_under_fault();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}