/**
 * @file  test_decision_mcdc.c
 * @brief Complementary requirement-based tests for TTC + FSM (ASIL-D).
 *
 * @author Eryca — cross-validation of module originally authored by Lourenço
 *
 * @note  Complements tests/test_decision.c. Each case traces to a specific
 *        functional requirement from SRS v2.0 or to the NFR-SAF-ROB
 *        robustness family (NULL-guards and invalid-input handling),
 *        raising MC/DC coverage toward the ≥95% threshold required by
 *        ISO 26262-6 Table 12 item 1c for ASIL-D.
 *
 * @requirements  FR-DEC-001, FR-DEC-003, FR-DEC-004, FR-DEC-007,
 *                FR-DEC-008, FR-DEC-010, FR-DEC-011,
 *                FR-FSM-002, FR-FSM-004, FR-FSM-006,
 *                FR-BRK-005, FR-BRK-006, FR-PER-005, FR-PER-006,
 *                NFR-SAF-ROB (defensive guards)
 *
 * Coverage groups:
 *   A — TTC edge cases and defensive NULL checks
 *   B — FSM defensive entry checks (delta_t, NULL inputs)
 *   C — evaluate_desired_state threshold bands (FR-DEC-001, -003, -004)
 *   D — Speed out-of-range handling (FR-DEC-008)
 *   E — POST_BRAKE state transitions (FR-FSM-006, FR-BRK-005, FR-BRK-006)
 *   F — WARNING state de-escalation debounce (FR-DEC-010, FR-FSM-004)
 *   G — BRAKE state transitions (escalate, de-escalate — FR-FSM-004)
 *   H — STANDBY → WARNING forced pass-through (FR-DEC-011)
 *   I — Steering override (FR-DEC-007, FR-PER-005)
 *   J — AEB disabled via driver input (FR-FSM-002)
 *   K — Complementary requirement-based cases for MC/DC gap closure
 */

#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_config.h"
#include <stdio.h>
#include <math.h>
#include <stddef.h>

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

/* ================================================================== */
/* Helper                                                             */
/* ================================================================== */

/**
 * Drives the FSM from STANDBY into WARNING and accumulates the internal
 * warn_timer to at least `warn_time_s` using a scenario that resolves to
 * desired_state == FSM_WARNING (TTC inside (TTC_BRAKE_L1, TTC_WARNING]).
 *
 * After this call, a single fsm_step() with modified inputs is enough to
 * exercise a specific transition out of WARNING.
 */
static void setup_warning(fsm_output_t * const fsm_out, float32_t warn_time_s)
{
    perception_output_t perc = {
        .distance   = 50.0f,
        .v_ego      = 15.0f,
        .v_rel      = 5.0f,
        .fault_flag = 0U
    };
    driver_input_t driver = {
        .brake_pedal    = 0U,
        .accel_pedal    = 0U,
        .steering_angle = 0.0f,
        .aeb_enabled    = 1U
    };
    ttc_output_t ttc = {
        .ttc        = 3.5f,    /* inside WARNING band (3.0, 4.0] */
        .d_brake    = 5.0f,
        .is_closing = 1U
    };
    int cycles = (int)(warn_time_s / 0.01f) + 1;
    int i;

    fsm_init(fsm_out);
    /* First step: STANDBY → WARNING (warn_timer reset to 0 in STANDBY case) */
    fsm_step(0.01f, &perc, &driver, &ttc, fsm_out);
    /* Accumulate warn_timer */
    for (i = 0; i < cycles; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, fsm_out);
    }
}

/* ================================================================== */
/* Group A — TTC edge cases                                           */
/* ================================================================== */

static void test_A1_ttc_clamp_max(void)
{
    /* 100 m / 1 m/s = 100 s, clamp to TTC_MAX = 10.0 s (covers line 28) */
    float32_t ttc = ttc_calc(100.0f, 1.0f);
    TEST_ASSERT(ttc == TTC_MAX, "A1: TTC > TTC_MAX clamped to TTC_MAX");
}

static void test_A2_ttc_distance_zero(void)
{
    /* distance == 0 → "distance > 0" is false → returns TTC_MAX */
    float32_t ttc = ttc_calc(0.0f, 5.0f);
    TEST_ASSERT(ttc == TTC_MAX, "A2: TTC with distance=0 returns TTC_MAX");
}

static void test_A3_ttc_distance_negative(void)
{
    /* Defensive path for corrupted distance */
    float32_t ttc = ttc_calc(-5.0f, 5.0f);
    TEST_ASSERT(ttc == TTC_MAX, "A3: TTC with negative distance returns TTC_MAX");
}

static void test_A4_ttc_process_null_perception(void)
{
    ttc_output_t out;
    out.ttc = 999.0f;   /* sentinel */
    out.d_brake = 888.0f;
    out.is_closing = 77U;
    ttc_process(NULL, &out);
    /* Defensive return must not touch output */
    TEST_ASSERT((out.ttc == 999.0f) && (out.d_brake == 888.0f) && (out.is_closing == 77U),
                "A4: ttc_process(NULL perception) is a no-op");
}

static void test_A5_ttc_process_null_output(void)
{
    perception_output_t perc = {
        .distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U
    };
    ttc_process(&perc, NULL);
    /* Surviving without crash is the assertion */
    TEST_ASSERT(1, "A5: ttc_process(NULL output) does not crash");
}

/* ================================================================== */
/* Group B — FSM defensive entry checks                               */
/* ================================================================== */

static void test_B1_fsm_delta_zero(void)
{
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;
    uint8_t before;

    fsm_init(&fsm_out);
    before = fsm_out.fsm_state;
    fsm_step(0.0f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == before, "B1: fsm_step(delta_t=0) is a no-op");
}

static void test_B2_fsm_delta_negative(void)
{
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;
    uint8_t before;

    fsm_init(&fsm_out);
    before = fsm_out.fsm_state;
    fsm_step(-0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == before, "B2: fsm_step(delta_t<0) is a no-op");
}

static void test_B3_fsm_null_perception(void)
{
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;
    uint8_t before;

    fsm_init(&fsm_out);
    before = fsm_out.fsm_state;
    fsm_step(0.01f, NULL, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == before, "B3: fsm_step(NULL perception) is a no-op");
}

static void test_B4_fsm_null_driver(void)
{
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;
    uint8_t before;

    fsm_init(&fsm_out);
    before = fsm_out.fsm_state;
    fsm_step(0.01f, &perc, NULL, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == before, "B4: fsm_step(NULL driver) is a no-op");
}

static void test_B5_fsm_null_ttc(void)
{
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    fsm_output_t fsm_out;
    uint8_t before;

    fsm_init(&fsm_out);
    before = fsm_out.fsm_state;
    fsm_step(0.01f, &perc, &driver, NULL, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == before, "B5: fsm_step(NULL ttc_in) is a no-op");
}

static void test_B6_fsm_null_output(void)
{
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};

    fsm_step(0.01f, &perc, &driver, &ttc, NULL);
    TEST_ASSERT(1, "B6: fsm_step(NULL fsm_out) does not crash");
}

/* ================================================================== */
/* Group C — evaluate_desired_state threshold bands                   */
/* Each test primes FSM to WARNING with warn_timer ≥ 0.8 s, then       */
/* exercises a single step with inputs that force a specific branch of */
/* evaluate_desired_state.                                             */
/* ================================================================== */

static void test_C1_desired_not_closing(void)
{
    /* FR-DEC-004: is_closing = 0 → desired = STANDBY.
       From WARNING, desired=STANDBY triggers 0.2 s debounce. */
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=5.0f, .is_closing=0U};
    fsm_output_t fsm_out;
    int i;

    setup_warning(&fsm_out, 0.9f);
    /* Debounce: 0.2 s → 20 cycles */
    for (i = 0; i < 25; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "C1: WARNING → STANDBY when is_closing=0 (FR-DEC-004)");
}

static void test_C2_desired_dbrake_ge_distance(void)
{
    /* FR-DEC-005: d_brake ≥ distance → desired = BRAKE_L3 */
    perception_output_t perc = {.distance=15.0f, .v_ego=15.0f, .v_rel=10.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=20.0f, .is_closing=1U};  /* d_brake > distance */
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L3,
                "C2: WARNING → BRAKE_L3 when d_brake ≥ distance (FR-DEC-003)");
}

static void test_C3_distance_floor_L3(void)
{
    /* distance ≤ 5 m and d_brake < distance → desired = BRAKE_L3 */
    perception_output_t perc = {.distance=4.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L3,
                "C3: distance ≤ 5 m → BRAKE_L3 (FR-DEC-004 + Table 10)");
}

static void test_C4_distance_floor_L2(void)
{
    /* 5 < distance ≤ 10 m → desired = BRAKE_L2 */
    perception_output_t perc = {.distance=8.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L2,
                "C4: 5 < distance ≤ 10 m → BRAKE_L2 (FR-DEC-004 + Table 10)");
}

static void test_C5_distance_floor_L1(void)
{
    /* 10 < distance ≤ 20 m → desired = BRAKE_L1 */
    perception_output_t perc = {.distance=15.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L1,
                "C5: 10 < distance ≤ 20 m → BRAKE_L1 (FR-DEC-004 + Table 10)");
}

static void test_C6_ttc_band_L3(void)
{
    /* distance > 20 m, ttc ≤ TTC_BRAKE_L3 (1.8) → desired = BRAKE_L3 */
    perception_output_t perc = {.distance=25.0f, .v_ego=15.0f, .v_rel=15.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.5f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L3,
                "C6: ttc ≤ TTC_BRAKE_L3 → BRAKE_L3 (FR-DEC-004 + Table 10)");
}

static void test_C7_ttc_band_L2(void)
{
    /* TTC_BRAKE_L3 < ttc ≤ TTC_BRAKE_L2 (2.2) → desired = BRAKE_L2 */
    perception_output_t perc = {.distance=25.0f, .v_ego=15.0f, .v_rel=12.5f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    /* With d_brake > distance this would hit FR-DEC-005 path.
       Using distance=25, d_brake=18.75: d_brake < distance ✔ */
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L2,
                "C7: TTC_BRAKE_L3 < ttc ≤ TTC_BRAKE_L2 → BRAKE_L2 (FR-DEC-004 + Table 10)");
}

static void test_C8_ttc_band_L1(void)
{
    /* TTC_BRAKE_L2 < ttc ≤ TTC_BRAKE_L1 (3.0) → desired = BRAKE_L1 */
    perception_output_t perc = {.distance=30.0f, .v_ego=15.0f, .v_rel=12.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.5f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L1,
                "C8: TTC_BRAKE_L2 < ttc ≤ TTC_BRAKE_L1 → BRAKE_L1 (FR-DEC-004 + Table 10)");
}

static void test_C9_ttc_band_warning(void)
{
    /* TTC_BRAKE_L1 < ttc ≤ TTC_WARNING (4.0) → desired = WARNING (stays) */
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=12.5f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=4.0f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_WARNING,
                "C9: TTC_BRAKE_L1 < ttc ≤ TTC_WARNING → stays WARNING");
}

static void test_C10_ttc_above_warning(void)
{
    /* ttc > TTC_WARNING, is_closing=1, distance > 20 → else branch → STANDBY */
    perception_output_t perc = {.distance=60.0f, .v_ego=10.0f, .v_rel=6.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=5.0f, .d_brake=8.33f, .is_closing=1U};
    fsm_output_t fsm_out;
    int i;

    setup_warning(&fsm_out, 0.9f);
    /* Debounce to STANDBY */
    for (i = 0; i < 25; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "C10: ttc > TTC_WARNING → STANDBY (else branch in evaluate_desired_state)");
}

/* ================================================================== */
/* Group D — Speed out-of-range handling (FR-DEC-008)                 */
/* ================================================================== */

static void test_D1_speed_below_min(void)
{
    /* v_ego < V_EGO_MIN (2.78 m/s) → forces STANDBY output */
    perception_output_t perc = {.distance=50.0f, .v_ego=2.0f, .v_rel=1.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT((fsm_out.fsm_state == FSM_STANDBY) && (fsm_out.brake_active == 0U),
                "D1: v_ego < V_EGO_MIN forces STANDBY with brake_active=0");
}

static void test_D2_speed_above_max(void)
{
    /* v_ego > V_EGO_MAX (16.67 m/s) → forces STANDBY output */
    perception_output_t perc = {.distance=50.0f, .v_ego=20.0f, .v_rel=1.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=2.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "D2: v_ego > V_EGO_MAX forces STANDBY");
}

static void test_D3_stop_during_braking(void)
{
    /* From BRAKE_L3 with v_ego < V_STOP_THRESHOLD → POST_BRAKE.
       This path is in the speed-out-of-range block (line 213).
       Note: FSM reaches POST_BRAKE also from the in-range BRAKE case
       (line 271), exercised in G1 below. */
    perception_output_t perc = {.distance=3.0f, .v_ego=0.005f, .v_rel=0.001f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.0f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;

    /* Pre-position FSM in BRAKE_L3 via public fsm_out field */
    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_BRAKE_L3;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_POST_BRAKE,
                "D3: BRAKE_L3 with v_ego≈0 transitions to POST_BRAKE (speed-guard branch)");
}

/* ================================================================== */
/* Group E — POST_BRAKE transitions (FR-FSM-005)                      */
/* ================================================================== */

static void test_E1_postbrake_timeout(void)
{
    /* POST_BRAKE stays for 2 s then releases to STANDBY */
    perception_output_t perc = {.distance=50.0f, .v_ego=5.0f, .v_rel=0.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=2.08f, .is_closing=0U};
    fsm_output_t fsm_out;
    int i;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_POST_BRAKE;
    /* Run 2.01 s */
    for (i = 0; i < 202; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "E1: POST_BRAKE → STANDBY after 2 s timeout (FR-FSM-006 + FR-BRK-005)");
}

static void test_E2_postbrake_accel_release(void)
{
    /* Accelerator pedal releases POST_BRAKE immediately */
    perception_output_t perc = {.distance=50.0f, .v_ego=5.0f, .v_rel=0.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=1U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=2.08f, .is_closing=0U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_POST_BRAKE;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "E2: POST_BRAKE → STANDBY on accel_pedal (FR-FSM-006 + FR-BRK-006)");
}

static void test_E3_postbrake_holds(void)
{
    /* Before 2 s elapses POST_BRAKE must hold */
    perception_output_t perc = {.distance=50.0f, .v_ego=5.0f, .v_rel=0.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=2.08f, .is_closing=0U};
    fsm_output_t fsm_out;
    int i;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_POST_BRAKE;
    /* Run 1 s (< 2 s hold) */
    for (i = 0; i < 100; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_POST_BRAKE,
                "E3: POST_BRAKE holds for < 2 s without accel input");
}

/* ================================================================== */
/* Group F — WARNING state de-escalation                              */
/* ================================================================== */

static void test_F1_warning_debounce_progress(void)
{
    /* WARNING + desired=STANDBY for less than 0.2 s must not transition yet */
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=5.0f, .is_closing=0U};
    fsm_output_t fsm_out;
    int i;

    setup_warning(&fsm_out, 0.5f);
    /* Run 10 cycles (0.1 s < 0.2 s debounce) */
    for (i = 0; i < 10; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_WARNING,
                "F1: WARNING holds during debounce (< 0.2 s)");
}

/* ================================================================== */
/* Group G — BRAKE transitions (escalate / de-escalate / stop)        */
/* ================================================================== */

static void test_G2_brake_escalate(void)
{
    /* BRAKE_L1 + desired = BRAKE_L3 → immediate escalation (no debounce) */
    perception_output_t perc = {.distance=4.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_BRAKE_L1;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L3,
                "G2: BRAKE_L1 → BRAKE_L3 on escalation (immediate, no debounce)");
}

static void test_G3_brake_deescalate(void)
{
    /* BRAKE_L3 + desired = BRAKE_L1 → de-escalation requires 0.2 s debounce */
    perception_output_t perc = {.distance=15.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;
    int i;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_BRAKE_L3;
    /* 25 cycles = 0.25 s > 0.2 s debounce */
    for (i = 0; i < 25; i++)
    {
        fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    }
    TEST_ASSERT(fsm_out.fsm_state == FSM_BRAKE_L1,
                "G3: BRAKE_L3 → BRAKE_L1 after 0.2 s debounce (de-escalation)");
}

/* ================================================================== */
/* Group H — STANDBY → WARNING forced pass-through (FR-DEC-011)       */
/* ================================================================== */

static void test_H1_standby_to_warning(void)
{
    /* STANDBY + desired = WARNING → WARNING */
    perception_output_t perc = {.distance=50.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_WARNING,
                "H1: STANDBY → WARNING on desired=WARNING");
}

static void test_H2_standby_forces_warning_before_brake(void)
{
    /* FR-DEC-011: STANDBY with desired=BRAKE_L3 must route through WARNING first */
    perception_output_t perc = {.distance=4.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_WARNING,
                "H2: STANDBY with desired=BRAKE forced to WARNING first (FR-DEC-011)");
}

/* ================================================================== */
/* Group I — Steering override (FR-FSM-006)                           */
/* ================================================================== */

static void test_I1_steering_override_positive(void)
{
    /* Steering > +5° → override → STANDBY */
    perception_output_t perc = {.distance=30.0f, .v_ego=15.0f, .v_rel=15.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=6.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.5f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    /* Apply steering override */
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "I1: steering > +5° forces STANDBY (FR-DEC-007)");
}

static void test_I2_steering_override_negative(void)
{
    /* Steering < -5° → override → STANDBY */
    perception_output_t perc = {.distance=30.0f, .v_ego=15.0f, .v_rel=15.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=-6.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=1.5f, .d_brake=18.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.9f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_STANDBY,
                "I2: steering < -5° forces STANDBY (FR-DEC-007)");
}

/* ================================================================== */
/* Group J — AEB disabled via driver input (FR-FSM-003)               */
/* ================================================================== */

static void test_J1_aeb_disabled_to_off(void)
{
    /* aeb_enabled = 0 (without fault_flag) → FSM_OFF */
    perception_output_t perc = {.distance=30.0f, .v_ego=15.0f, .v_rel=5.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=0U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=5.0f, .is_closing=1U};
    fsm_output_t fsm_out;

    setup_warning(&fsm_out, 0.5f);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_OFF,
                "J1: aeb_enabled=0 forces FSM_OFF (FR-FSM-002)");
}

/* ================================================================== */
/* Group K — Complementary requirement-based cases                    */
/* These target requirement scenarios whose branches were partially   */
/* covered by the original suite, closing remaining MC/DC outcomes.    */
/* ================================================================== */

static void test_K1_post_brake_ignores_brake_pedal(void)
{
    /* FR-FSM-006: in POST_BRAKE, the brake pedal is NOT an override.
       FSM must remain in POST_BRAKE when only the brake pedal is applied. */
    perception_output_t perc = {.distance=50.0f, .v_ego=5.0f, .v_rel=0.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=1U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=2.08f, .is_closing=0U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_POST_BRAKE;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_POST_BRAKE,
                "K1: POST_BRAKE ignores brake pedal (FR-FSM-006)");
}

static void test_K2_post_brake_ignores_steering(void)
{
    /* FR-FSM-006: in POST_BRAKE, steering > 5° is NOT an override.
       FSM must remain in POST_BRAKE when only steering is applied. */
    perception_output_t perc = {.distance=50.0f, .v_ego=5.0f, .v_rel=0.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=10.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=2.08f, .is_closing=0U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_out.fsm_state = (uint8_t)FSM_POST_BRAKE;
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_POST_BRAKE,
                "K2: POST_BRAKE ignores steering override (FR-FSM-006)");
}

static void test_K3_speed_below_stop_in_standby(void)
{
    /* FR-DEC-008: system stays in STANDBY outside the 10–60 km/h range.
       Complements D1: exercises v_ego below V_STOP_THRESHOLD (0.01 m/s)
       with current state < BRAKE_L1, hitting the (T,F) combo of the
       speed-guard inner decision. */
    perception_output_t perc = {.distance=50.0f, .v_ego=0.005f, .v_rel=0.001f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=TTC_MAX, .d_brake=0.0f, .is_closing=0U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT((fsm_out.fsm_state == FSM_STANDBY) && (fsm_out.brake_active == 0U),
                "K3: v_ego < V_STOP_THRESHOLD in STANDBY stays STANDBY (FR-DEC-008)");
}

static void test_K4_standby_to_brake_l1_via_warning(void)
{
    /* FR-DEC-011: every threat from STANDBY must pass through WARNING.
       Complements H2 (BRAKE_L3 path): this exercises the BRAKE_L1-triggered
       arm of the pass-through OR chain, using the distance-floor band
       (10 < distance ≤ 20 m). */
    perception_output_t perc = {.distance=15.0f, .v_ego=3.0f, .v_rel=2.0f, .fault_flag=0U};
    driver_input_t driver = {.brake_pedal=0U, .accel_pedal=0U, .steering_angle=0.0f, .aeb_enabled=1U};
    ttc_output_t ttc = {.ttc=3.5f, .d_brake=0.75f, .is_closing=1U};
    fsm_output_t fsm_out;

    fsm_init(&fsm_out);
    fsm_step(0.01f, &perc, &driver, &ttc, &fsm_out);
    TEST_ASSERT(fsm_out.fsm_state == FSM_WARNING,
                "K4: STANDBY with desired=BRAKE_L1 forced to WARNING (FR-DEC-011)");
}

/* ================================================================== */
/* Main                                                               */
/* ================================================================== */

int main(void)
{
    printf("\n========================================\n");
    printf("Decision MC/DC Gap-Closure Tests\n");
    printf("Module: TTC + FSM  (author Lourenço, validator Eryca)\n");
    printf("Standard: ISO 26262-6 Table 12 item 1c (ASIL-D)\n");
    printf("========================================\n\n");

    printf("--- Group A: TTC edge cases ---\n");
    test_A1_ttc_clamp_max();
    test_A2_ttc_distance_zero();
    test_A3_ttc_distance_negative();
    test_A4_ttc_process_null_perception();
    test_A5_ttc_process_null_output();

    printf("\n--- Group B: FSM defensive entry ---\n");
    test_B1_fsm_delta_zero();
    test_B2_fsm_delta_negative();
    test_B3_fsm_null_perception();
    test_B4_fsm_null_driver();
    test_B5_fsm_null_ttc();
    test_B6_fsm_null_output();

    printf("\n--- Group C: evaluate_desired_state bands ---\n");
    test_C1_desired_not_closing();
    test_C2_desired_dbrake_ge_distance();
    test_C3_distance_floor_L3();
    test_C4_distance_floor_L2();
    test_C5_distance_floor_L1();
    test_C6_ttc_band_L3();
    test_C7_ttc_band_L2();
    test_C8_ttc_band_L1();
    test_C9_ttc_band_warning();
    test_C10_ttc_above_warning();

    printf("\n--- Group D: speed out-of-range ---\n");
    test_D1_speed_below_min();
    test_D2_speed_above_max();
    test_D3_stop_during_braking();

    printf("\n--- Group E: POST_BRAKE transitions (FR-FSM-006 + FR-BRK-005/006) ---\n");
    test_E1_postbrake_timeout();
    test_E2_postbrake_accel_release();
    test_E3_postbrake_holds();

    printf("\n--- Group F: WARNING debounce ---\n");
    test_F1_warning_debounce_progress();

    printf("\n--- Group G: BRAKE transitions ---\n");
    test_G2_brake_escalate();
    test_G3_brake_deescalate();

    printf("\n--- Group H: STANDBY → WARNING (FR-DEC-011) ---\n");
    test_H1_standby_to_warning();
    test_H2_standby_forces_warning_before_brake();

    printf("\n--- Group I: steering override (FR-DEC-007) ---\n");
    test_I1_steering_override_positive();
    test_I2_steering_override_negative();

    printf("\n--- Group J: AEB disabled (FR-FSM-002) ---\n");
    test_J1_aeb_disabled_to_off();

    printf("\n--- Group K: complementary requirement-based cases ---\n");
    test_K1_post_brake_ignores_brake_pedal();
    test_K2_post_brake_ignores_steering();
    test_K3_speed_below_stop_in_standby();
    test_K4_standby_to_brake_l1_via_warning();

    printf("\n========================================\n");
    printf("RESULTS: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return (tests_failed == 0) ? 0 : 1;
}
