/**
 * @file test_decision.c
 * @brief Unit tests for decision logic (TTC + FSM)
 * @author Lourenço Jamba Mphili
 * @requirements FR-DEC-001 to FR-DEC-011, FR-FSM-001 to FR-FSM-006
 */

#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include <stdio.h>
#include <math.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(condition, msg) \
    do { \
        if (condition) { \
            printf("  ✅ PASS: %s\n", msg); \
            tests_passed++; \
        } else { \
            printf("  ❌ FAIL: %s\n", msg); \
            tests_failed++; \
        } \
    } while(0)

/* ============================================================ */
/* TTC Tests (FR-DEC-001, FR-DEC-002, FR-DEC-003)               */
/* ============================================================ */

static void test_ttc_normal(void)
{
    perception_output_t perc = {
        .distance = 50.0f,
        .v_ego = 20.0f,
        .v_rel = 10.0f
    };
    ttc_output_t ttc;
    ttc_process(&perc, &ttc);
    TEST_ASSERT(fabsf(ttc.ttc - 5.0f) < 0.05f, "TTC normal closing (5.0s)");
}

static void test_ttc_critical(void)
{
    perception_output_t perc = {
        .distance = 10.0f,
        .v_ego = 20.0f,
        .v_rel = 20.0f
    };
    ttc_output_t ttc;
    ttc_process(&perc, &ttc);
    TEST_ASSERT(fabsf(ttc.ttc - 0.5f) < 0.05f, "TTC critical (0.5s)");
}

static void test_ttc_not_closing(void)
{
    perception_output_t perc = {
        .distance = 50.0f,
        .v_ego = 10.0f,
        .v_rel = 0.4f
    };
    ttc_output_t ttc;
    ttc_process(&perc, &ttc);
    TEST_ASSERT(ttc.ttc == 10.0f, "TTC not closing (max 10.0s)");
}

static void test_dbrake_normal(void)
{
    float32_t d = d_brake_calc(13.89f);
    float32_t expected = (13.89f * 13.89f) / 12.0f;
    TEST_ASSERT(fabsf(d - expected) < 0.5f, "Braking distance at 50 km/h");
}

static void test_dbrake_zero(void)
{
    float32_t d = d_brake_calc(0.0f);
    TEST_ASSERT(d == 0.0f, "Braking distance at zero speed");
}

/* ============================================================ */
/* FSM Tests (FR-FSM-001 to FR-FSM-006)                         */
/* ============================================================ */

static void test_fsm_initial(void)
{
    fsm_output_t fsm_out;
    fsm_init(&fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY, "FSM initial state = STANDBY");
}

static void test_fsm_fault(void)
{
    perception_output_t perc = {
        .distance = 50.0f,
        .v_ego = 15.0f,
        .v_rel = 5.0f,
        .fault_flag = 0U
    };
    driver_input_t driver = {
        .brake_pedal = 0U,
        .aeb_enabled = 1U
    };
    ttc_output_t ttc = {
        .ttc = 2.0f,
        .is_closing = 1U
    };
    fsm_output_t fsm_out;
    
    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    
    /* Inject fault */
    perc.fault_flag = 1U;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    
    TEST_ASSERT(fsm_out.fsm_state == FSM_OFF, "FSM goes to OFF on fault");
}

static void test_fsm_override(void)
{
    perception_output_t perc = {
        .distance = 30.0f,
        .v_ego = 15.0f,
        .v_rel = 15.0f,
        .fault_flag = 0U
    };
    driver_input_t driver = {
        .brake_pedal = 0U,
        .accel_pedal = 0U,
        .steering_angle = 0.0f,
        .aeb_enabled = 1U
    };
    ttc_output_t ttc = {
        .ttc = 1.5f,
        .d_brake = 18.75f,
        .is_closing = 1U
    };
    fsm_output_t fsm_out;
    int i;
    
    fsm_init(&fsm_out);
    
    /* Run cycles to enter braking state */
    for (i = 0; i < 100; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    
    /* Apply brake pedal */
    driver.brake_pedal = 1U;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY, "Driver override goes to STANDBY");
}

/* ============================================================ */
/* Regression test for issue #95                                */
/* "Continue braking when v_ego drops below V_EGO_MIN mid-run"  */
/* ============================================================ */

/* Drive the FSM into BRAKE_L2 with a credible CCRs trajectory, then drop
 * v_ego below V_EGO_MIN while the target is still close (distance under
 * the 20 m floor). The pre-#95 code forced FSM_STANDBY here and zeroed
 * decel_target, leaving the ego coasting the last metres. The fix lets
 * Priority 4 run instead, so the distance-floor logic in
 * evaluate_desired_state keeps a BRAKE_Lx state and decel_target > 0. */
static void test_fsm_continue_braking_below_vmin(void)
{
    perception_output_t perc = {
        .distance = 8.0f,        /* under the 10 m floor → desired = BRAKE_L2 */
        .v_ego = 6.0f,           /* above V_EGO_MIN to start */
        .v_rel = 6.0f,
        .fault_flag = 0U
    };
    driver_input_t driver = {
        .brake_pedal = 0U,
        .accel_pedal = 0U,
        .steering_angle = 0.0f,
        .aeb_enabled = 1U
    };
    ttc_output_t ttc = {
        .ttc = 1.3f,             /* below TTC_BRAKE_L2 */
        .d_brake = 7.0f,
        .is_closing = 1U
    };
    fsm_output_t fsm_out;
    int i;

    fsm_init(&fsm_out);

    /* Run a few cycles to enter and confirm a BRAKE_Lx state. */
    for (i = 0; i < 100; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state >= (uint8_t)FSM_BRAKE_L1 &&
                fsm_out.fsm_state <= (uint8_t)FSM_BRAKE_L3,
                "FSM enters a BRAKE_Lx state under TTC + distance floor");

    /* Now ego drops below V_EGO_MIN (10 km/h = 2.78 m/s) but is still
     * above V_STOP_THRESHOLD. Target is still close (distance in floor). */
    perc.v_ego = 1.5f;            /* < V_EGO_MIN, > V_STOP_THRESHOLD */
    perc.distance = 6.5f;         /* still under the 10 m floor */
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);

    TEST_ASSERT(fsm_out.fsm_state >= (uint8_t)FSM_BRAKE_L1 &&
                fsm_out.fsm_state <= (uint8_t)FSM_BRAKE_L3,
                "FSM stays in BRAKE_Lx when v_ego dips below V_EGO_MIN "
                "while distance floor still applies (#95)");
    TEST_ASSERT(fsm_out.brake_active != 0U,
                "Brake remains active when actively braking below V_EGO_MIN");
    TEST_ASSERT(fsm_out.decel_target > 0.0f,
                "decel_target stays > 0 when actively braking below V_EGO_MIN");

    /* Once ego truly stops (v_ego < V_STOP_THRESHOLD) the existing branch
     * hands off to POST_BRAKE — same behaviour as before #95. */
    perc.v_ego = V_STOP_THRESHOLD * 0.5f;   /* guaranteed below the threshold */
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == (uint8_t)FSM_POST_BRAKE,
                "FSM enters POST_BRAKE once v_ego < V_STOP_THRESHOLD");
}

/* When the ego is below V_EGO_MIN and *not* in a braking state (e.g. the
 * vehicle is rolling along at low speed with no threat), the FSM still
 * collapses cleanly to STANDBY — no regression on the existing path. */
static void test_fsm_low_speed_no_brake_goes_standby(void)
{
    perception_output_t perc = {
        .distance = 80.0f,       /* far from any distance floor */
        .v_ego = 1.0f,           /* below V_EGO_MIN */
        .v_rel = 0.0f,           /* not closing */
        .fault_flag = 0U
    };
    driver_input_t driver = {
        .brake_pedal = 0U,
        .accel_pedal = 0U,
        .steering_angle = 0.0f,
        .aeb_enabled = 1U
    };
    ttc_output_t ttc = {
        .ttc = 99.0f,
        .d_brake = 0.5f,
        .is_closing = 0U
    };
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);

    TEST_ASSERT(fsm_out.fsm_state == (uint8_t)FSM_STANDBY,
                "Low-speed idle (not braking) falls to STANDBY");
    TEST_ASSERT(fsm_out.brake_active == 0U,
                "Brake is released in STANDBY at low speed");
}

/* ============================================================ */
/* Parameter Validation (SRS Table 10)                          */
/* ============================================================ */

static void test_deceleration_values(void)
{
    /* Test deceleration values - values are defined in aeb_config.h */
    /* DECEL_L1 = 2.0f, DECEL_L2 = 4.0f, DECEL_L3 = 6.0f */
    
    int all_pass = 1;
    
    /* Verify that constants exist and have correct values */
    if (DECEL_L1 != 2.0f) all_pass = 0;
    if (DECEL_L2 != 4.0f) all_pass = 0;
    if (DECEL_L3 != 6.0f) all_pass = 0;
    
    TEST_ASSERT(all_pass == 1, "Deceleration values (2/4/6 m/s² per Table 10)");
}

/* ============================================================ */
/* Main Function                                                */
/* ============================================================ */

int main(void)
{
    printf("\n========================================\n");
    printf("Decision Logic Tests (TTC + FSM)\n");
    printf("Author: Lourenço Jamba Mphili\n");
    printf("Requirements: FR-DEC-001 to FR-DEC-011\n");
    printf("              FR-FSM-001 to FR-FSM-006\n");
    printf("========================================\n\n");
    
    printf("--- TTC Tests (FR-DEC-001, FR-DEC-002, FR-DEC-003) ---\n");
    test_ttc_normal();
    test_ttc_critical();
    test_ttc_not_closing();
    test_dbrake_normal();
    test_dbrake_zero();
    
    printf("\n--- FSM Tests (FR-FSM-001 to FR-FSM-006) ---\n");
    test_fsm_initial();
    test_fsm_fault();
    test_fsm_override();

    printf("\n--- Issue #95: brake-hold below V_EGO_MIN ---\n");
    test_fsm_continue_braking_below_vmin();
    test_fsm_low_speed_no_brake_goes_standby();

    printf("\n--- Parameter Validation (SRS Table 10) ---\n");
    test_deceleration_values();
    
    printf("\n========================================\n");
    printf("RESULTS: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");
    
    if (tests_failed == 0)
    {
        printf("\n✅ ALL TESTS PASSED - Decision Logic ready for integration\n");
    }
    else
    {
        printf("\n❌ %d TEST(S) FAILED - Review implementation\n", tests_failed);
    }
    
    return (tests_failed == 0) ? 0 : 1;
}