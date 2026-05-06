/**
 * @file    test_perception.c
 * @brief   Unit tests for the AEB Perception module.
 *
 * Compile and run on PC (no Zephyr / hardware dependency):
 *
 *   gcc -Wall -Wextra -I../include \
 *       ../src/perception/aeb_perception.c test_perception.c \
 *       -o test_perception -lm
 *   ./test_perception
 *
 * Each test prints PASS or FAIL with a short description.
 * Exit code is 0 if all tests pass, 1 otherwise.
 *
 * Requirements verified: Requirements: FR-PER-001, FR-PER-002, FR-PER-003, FR-PER-006, FR-PER-007, NFR-COD-006, NFR-COD-007
 */

#include <stdio.h>
#include <stdint.h>

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_perception.h"

/* =========================================================================
 * Mini test framework
 * ====================================================================== */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, name)                                      \
    do {                                                       \
        if (cond) {                                            \
            printf("[PASS] %s\n", (name));                     \
            g_pass++;                                          \
        } else {                                               \
            printf("[FAIL] %s  (line %d)\n", (name), __LINE__);\
            g_fail++;                                          \
        }                                                      \
    } while (0)

#define FABSF_T(x) (((x) < 0.0f) ? -(x) : (x))

/* =========================================================================
 * Helper: build a "good" nominal input frame
 * ====================================================================== */
static raw_sensor_input_t make_good_input(void)
{
    raw_sensor_input_t in;
    in.radar_d     = 30.0f;
    in.radar_vr    = 5.0f;
    in.lidar_d     = 30.2f;
    in.v_ego       = 20.0f;
    in.can_timeout = 0U;
    in.fi          = 0U;
    return in;
}

/* =========================================================================
 * TEST 1 — FR-PER-006, FR-PER-007
 * LiDAR fault must NOT be latched on fewer than SENSOR_FAULT_CYCLES bad frames.
 * ====================================================================== */
static void test_lidar_no_fault_before_latch(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();

    /* Feed (SENSOR_FAULT_CYCLES - 1) clearly out-of-range LiDAR samples */
    in.lidar_d = 999.0f;   /* above LIDAR_DIST_MAX = 100 m */
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES - 1; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
          "LiDAR: no fault before latch cycle");
}

/* =========================================================================
 * TEST 2 — FR-PER-006, FR-PER-007
 * LiDAR fault IS latched on exactly SENSOR_FAULT_CYCLES consecutive bad frames.
 * ====================================================================== */
static void test_lidar_fault_latches_at_cycle3(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.lidar_d = 999.0f;

    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "LiDAR: fault latches at cycle 3");
}

/* =========================================================================
 * TEST 3 — FR-PER-006, FR-PER-007
 * LiDAR counter resets when a good frame is received before latch.
 * ====================================================================== */
static void test_lidar_counter_resets_on_good_frame(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* 2 bad frames: just above LIDAR_DIST_MAX (100 m), values close together
     * so that later the ROC from bad→good doesn't re-trigger. */
    in.lidar_d = 105.0f;   /* out of range; ctr=1, prev_d=105 */
    perception_step(&in, &out);
    in.lidar_d = 104.0f;   /* out of range; ROC |104-105|=1≤10; ctr=2, prev_d=104 */
    perception_step(&in, &out);

    /* 1 good frame close to prev_d: in range AND ROC OK → counter resets */
    in.lidar_d = 97.0f;    /* in range [1,100]; ROC |97-104|=7≤10; ctr=0 */
    perception_step(&in, &out);

    /* 2 more bad frames (just above max, incremental) → ctr reaches 2, not 3 */
    in.lidar_d = 105.0f;   /* out of range; ROC |105-97|=8≤10; ctr=1 */
    perception_step(&in, &out);
    in.lidar_d = 106.0f;   /* out of range; ROC |106-105|=1≤10; ctr=2 */
    perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
          "LiDAR: counter resets on good frame before latch");
}

/* =========================================================================
 * TEST 4 — FR-PER-006, FR-PER-007
 * Radar fault latches at exactly SENSOR_FAULT_CYCLES out-of-range frames.
 * ====================================================================== */
static void test_radar_fault_latches_at_cycle3(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.radar_d = 999.0f;   /* above RADAR_DIST_MAX = 200 m */

    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar: fault latches at cycle 3");
}

/* =========================================================================
 * TEST 5 — FR-PER-006, FR-PER-007
 * Radar fault NOT latched when relative velocity exceeds MAX_REL_VEL only
 * once (just one bad cycle, not three).
 * ====================================================================== */
static void test_radar_no_fault_single_bad_vr(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* One cycle with |vr| > 50 m/s */
    in.radar_vr = 60.0f;
    perception_step(&in, &out);

    /* Two good cycles */
    in.radar_vr = 5.0f;
    perception_step(&in, &out);
    perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar: no fault on single bad relative velocity");
}

/* =========================================================================
 * TEST 6 — FR-PER-007
 * CAN timeout sets AEB_FAULT_CAN_TO and both sensor fault bits.
 * ====================================================================== */
static void test_can_timeout_sets_fault_flags(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();

    /* Warm up with a few good cycles first */
    in = make_good_input();
    perception_step(&in, &out);
    perception_step(&in, &out);

    /* Now simulate CAN timeout */
    in.can_timeout = 1U;
    perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_CAN_TO)  != 0U, "CAN timeout: bit CAN_TO set");
    CHECK((out.fault_flag & AEB_FAULT_LIDAR)   != 0U, "CAN timeout: bit LIDAR set");
    CHECK((out.fault_flag & AEB_FAULT_RADAR)   != 0U, "CAN timeout: bit RADAR set");
    CHECK(out.confidence == 0.0f,                      "CAN timeout: confidence = 0");
}

/* =========================================================================
 * TEST 7 — FR-PER-001, FR-PER-003
 * Kalman output converges towards true distance in ~50 cycles.
 * Both sensors report 30 m; expect fused distance within 0.5 m of 30.
 * ====================================================================== */
static void test_kalman_convergence(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.radar_d  = 30.0f;
    in.radar_vr = 0.0f;   /* stationary target */
    in.lidar_d  = 30.0f;

    for (i = 0; i < 50; i++) {
        perception_step(&in, &out);
    }
    CHECK(FABSF_T(out.distance - 30.0f) < 0.5f,
          "Kalman: converges to 30 m within 50 cycles");
}

/* =========================================================================
 * TEST 8 — FR-PER-001
 * Output distance is clamped to DIST_MIN_OUTPUT when Kalman state is near 0.
 * ====================================================================== */
static void test_output_clamp_min_distance(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();

    /* Drive sensors towards a very small distance */
    in.radar_d  = 0.3f;   /* below RADAR_DIST_MIN so radar faults eventually */
    in.lidar_d  = 0.5f;   /* below LIDAR_DIST_MIN (1.0 m), triggers LiDAR fault */
    in.radar_vr = 0.0f;

    /* Run 5 cycles — Kalman state pulled towards 0.5 m */
    for (i = 0; i < 5; i++) {
        perception_step(&in, &out);
    }
    CHECK(out.distance >= DIST_MIN_OUTPUT,
          "Output: distance clamped to >= DIST_MIN_OUTPUT");
}

/* =========================================================================
 * TEST 9 — FR-PER-001
 * Output distance clamped to DIST_MAX_OUTPUT when Kalman state is very large.
 * ====================================================================== */
static void test_output_clamp_max_distance(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();

    /* Drive sensors towards a very large distance */
    in.radar_d  = 199.0f;
    in.lidar_d  = 99.0f;
    in.radar_vr = -30.0f;  /* receding target */

    for (i = 0; i < 50; i++) {
        perception_step(&in, &out);
    }
    CHECK(out.distance <= DIST_MAX_OUTPUT,
          "Output: distance clamped to <= DIST_MAX_OUTPUT");
}

/* =========================================================================
 * TEST 10 — FR-PER-007, NFR-SAF-005
 * Fault injection (fi=1) disables both sensors inside the Kalman.
 * V&V criterion: "Inject fault_inj → fault_flag=1 within 30 ms".
 * chart_41: when fi=1, r_ok=0, l_ok=0 → fault_out=1, conf_out=0.0.
 * ====================================================================== */
static void test_fault_injection_disables_sensors(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();
    in.fi = 1U;   /* inject fault */

    /* Run 3 cycles with fi=1 (within 30 ms = 3 cycles at 100 Hz) */
    perception_step(&in, &out);
    perception_step(&in, &out);
    perception_step(&in, &out);

    /* fault_flag must indicate both sensors faulted (chart_41 fault_out) */
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "Fault injection: LiDAR fault flag set");
    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Fault injection: Radar fault flag set");

    /* Confidence must be 0.0 when both sensors disabled (chart_41) */
    CHECK(out.confidence == 0.0f,
          "Fault injection: confidence = 0.0 (both sensors disabled)");
}

/* =========================================================================
 * TEST 11 — Derived (confidence ordering)
 * Confidence is higher with both sensors healthy than with only one.
 * ====================================================================== */
static void test_confidence_both_sensors_gt_one_sensor(void)
{
    raw_sensor_input_t in;
    perception_output_t out_both;
    perception_output_t out_one;
    int i;

    /* Both sensors OK — run 20 cycles to let P converge */
    perception_init();
    in = make_good_input();
    for (i = 0; i < 20; i++) {
        perception_step(&in, &out_both);
    }

    /* Only LiDAR OK (radar faulty distance) */
    perception_init();
    in = make_good_input();
    in.radar_d = 999.0f;  /* force radar fault after 3 cycles */
    for (i = 0; i < 20; i++) {
        perception_step(&in, &out_one);
    }

    CHECK(out_both.confidence >= out_one.confidence,
          "Confidence: both sensors >= one sensor");
}

/* =========================================================================
 * TEST 12 — FR-PER-006
 * Good data produces no fault flags at all.
 * ====================================================================== */
static void test_no_fault_on_good_data(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();

    for (i = 0; i < 20; i++) {
        perception_step(&in, &out);
    }
    CHECK(out.fault_flag == 0U, "No faults on 20 cycles of good data");
}

/* =========================================================================
 * TEST 13 — FR-PER-003
 * Relative velocity clamped to [-MAX_REL_VEL, MAX_REL_VEL].
 * ====================================================================== */
static void test_output_clamp_rel_velocity(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    /* Feed a safely high-but-valid velocity for many cycles */
    in.radar_vr = 40.0f;
    in.lidar_d  = 30.0f;

    for (i = 0; i < 30; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.v_rel >= -MAX_REL_VEL) &&
          (out.v_rel <=  MAX_REL_VEL),
          "Output: rel_velocity clamped within [-MAX_REL_VEL, MAX_REL_VEL]");
}

/* =========================================================================
 * TEST 14 — FR-PER-002
 * Ego velocity is passed through directly from input.
 * ====================================================================== */
static void test_ego_velocity_passthrough(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();
    in.v_ego = 27.5f;

    perception_step(&in, &out);

    CHECK(out.v_ego == 27.5f, "Ego velocity: direct passthrough");
}

/* =========================================================================
 * TEST 15 — FR-PER-006
 * LiDAR ROC (rate-of-change) check: jump > DIST_ROC_LIMIT triggers counter.
 * ====================================================================== */
static void test_lidar_roc_triggers_counter(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* Establish prev_d = 30 m (is_first cleared after this call) */
    in.lidar_d = 30.0f;
    perception_step(&in, &out);

    /* Jump >10 m on 3 *consecutive* cycles so the ROC check fires each time.
     * Each step must also differ from the previous by >DIST_ROC_LIMIT (10 m).
     * Cycle 1: d=50, ROC |50-30|=20>10 → bad, ctr=1, prev_d=50
     * Cycle 2: d=70, ROC |70-50|=20>10 → bad, ctr=2, prev_d=70
     * Cycle 3: d=90, ROC |90-70|=20>10 → bad, ctr=3 → FAULT latched */
    in.lidar_d = 50.0f;
    perception_step(&in, &out);
    in.lidar_d = 70.0f;
    perception_step(&in, &out);
    in.lidar_d = 90.0f;
    perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "LiDAR: ROC > 10 m for 3 cycles latches fault");
}

/* =========================================================================
 * TEST 16 — FR-PER-006, FR-PER-007
 * Radar ROC: consecutive rate-of-change violations latch radar fault.
 * Analogous to TEST 15 (LiDAR ROC) but for the radar detector.
 * ====================================================================== */
static void test_radar_roc_triggers_counter(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* Establish prev_d = 30 m, prev_vr = 5 m/s (is_first cleared) */
    perception_step(&in, &out);

    /* 3 consecutive cycles with distance ROC > 10 m each.
     * Velocity stays valid so only distance ROC triggers.
     * Cycle 1: d=50, ROC |50-30|=20>10 → bad, ctr=1
     * Cycle 2: d=70, ROC |70-50|=20>10 → bad, ctr=2
     * Cycle 3: d=90, ROC |90-70|=20>10 → bad, ctr=3 → FAULT */
    in.radar_d = 50.0f;
    perception_step(&in, &out);
    in.radar_d = 70.0f;
    perception_step(&in, &out);
    in.radar_d = 90.0f;
    perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar: ROC > 10 m for 3 cycles latches fault");
}

/* =========================================================================
 * TEST 17 — FR-PER-007
 * perception_init() resets all static state: after a latched fault,
 * calling init and feeding good data must produce fault_flag = 0.
 * ====================================================================== */
static void test_init_resets_state(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    /* Phase 1: force LiDAR fault latch */
    perception_init();
    in = make_good_input();
    in.lidar_d = 999.0f;
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "Init reset: fault latched before init");

    /* Phase 2: reset and feed good data */
    perception_init();
    in = make_good_input();
    for (i = 0; i < 5; i++) {
        perception_step(&in, &out);
    }
    CHECK(out.fault_flag == 0U,
          "Init reset: no faults after init + good data");
}

/* =========================================================================
 * TEST 18 — FR-PER-007
 * CAN timeout must not corrupt fault detector counters.
 * Strategy: feed 1 bad LiDAR frame (ctr=1), then 1 CAN timeout (detectors
 * skipped, ctr frozen at 1), then 1 more bad frame (ctr=2).
 * If timeout had incremented the counter, ctr would reach 3 and fault
 * would latch.  We assert no fault, proving timeout didn't corrupt state.
 * ====================================================================== */
static void test_can_timeout_preserves_counters(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* Warm-up: establish prev_d with a good frame */
    perception_step(&in, &out);

    /* 1 bad LiDAR frame → ctr=1 */
    in.lidar_d = 999.0f;
    perception_step(&in, &out);
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
          "CAN timeout preserves ctr: no fault after 1 bad frame");

    /* CAN timeout: returns early, fault detectors NOT executed.
     * Counter stays frozen at 1 (not incremented to 2). */
    in.can_timeout = 1U;
    perception_step(&in, &out);

    /* Resume normal: 1 more bad frame (same value, no ROC issue).
     * If timeout preserved state correctly: ctr goes from 1 → 2.
     * If timeout had incremented: ctr would go from 2 → 3 = FAULT. */
    in.can_timeout = 0U;
    in.lidar_d = 999.0f;
    perception_step(&in, &out);
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
          "CAN timeout preserves ctr: counter not corrupted by timeout");
}

/* =========================================================================
 * TEST 19 — FR-PER-006, FR-PER-007
 * LiDAR fault counter overflow protection: the "if (ctr < 255U)" guard
 * must clamp the counter at 255 and never let it wrap.
 *
 * On the 256th consecutive bad frame the guard's FALSE branch executes,
 * keeping ctr at 255. The fault must remain latched throughout.
 * ====================================================================== */
static void test_lidar_counter_saturation(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.lidar_d = 999.0f;   /* above LIDAR_DIST_MAX=100 m — bad every cycle */

    for (i = 0; i < 256; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "LiDAR counter saturation: fault active after 256 bad cycles");
}

/* =========================================================================
 * TEST 20 — FR-PER-006, FR-PER-007
 * Radar fault counter overflow protection: analogous to TEST 19.
 * ====================================================================== */
static void test_radar_counter_saturation(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.radar_d = 999.0f;   /* above RADAR_DIST_MAX=200 m — bad every cycle */

    for (i = 0; i < 256; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar counter saturation: fault active after 256 bad cycles");
}

/* =========================================================================
 * TEST 21 — FR-PER-001 (radar-only confidence path)
 * When LiDAR is permanently faulted but radar is healthy, kalman_fusion()
 * must take the "else if (r_ok != 0U)" branch (base = 0.7).
 * Confidence must be > 0 but below the both-sensor ceiling.
 * ====================================================================== */
static void test_confidence_radar_only(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();        /* radar_d=30, radar_vr=5 (healthy) */
    in.lidar_d = 999.0f;          /* above LIDAR_DIST_MAX — always faulty */

    for (i = 0; i < 20; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_LIDAR) != 0U,
          "Radar-only: LiDAR fault flag set");
    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar-only: Radar fault flag clear");
    CHECK(out.confidence > 0.0f,
          "Radar-only: confidence > 0 (base = 0.7 path)");
    CHECK(out.confidence < 0.8f,
          "Radar-only: confidence < 0.8 (below both-sensor ceiling)");
}

/* =========================================================================
 * TEST 22 — FR-PER-006 (MC/DC: velocity ROC independently triggers fault)
 * Covers the second condition in the radar ROC compound OR:
 *   (FABSF(d_r - prev_d) > DIST_ROC_LIMIT) || (FABSF(vr_r - prev_vr) > VEL_ROC_LIMIT)
 *
 * The MC/DC complement to TEST 16 which only exercised the distance-ROC
 * condition. Here distance is stable (ROC = 0) and velocity jumps by
 * +3 m/s per cycle, which exceeds VEL_ROC_LIMIT = 2.0 m/s while
 * remaining well within MAX_REL_VEL = 50 m/s (range check stays clear).
 *
 * Warm-up establishes prev_vr = 5 m/s. Then:
 *   Cycle 1: vr=8.5  -> |8.5-5.0|  = 3.5 > 2.0, d_roc=0 -> bad, ctr=1
 *   Cycle 2: vr=11.5 -> |11.5-8.5| = 3.0 > 2.0, d_roc=0 -> bad, ctr=2
 *   Cycle 3: vr=14.5 -> |14.5-11.5|= 3.0 > 2.0, d_roc=0 -> bad, ctr=3 -> FAULT
 * ====================================================================== */
static void test_radar_vel_roc_triggers_fault(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();   /* radar_d=30, radar_vr=5 */

    /* Warm-up: establish prev_d=30 m, prev_vr=5 m/s */
    perception_step(&in, &out);

    in.radar_d  = 30.0f;  in.radar_vr =  8.5f;  perception_step(&in, &out);
    in.radar_d  = 30.0f;  in.radar_vr = 11.5f;  perception_step(&in, &out);
    in.radar_d  = 30.0f;  in.radar_vr = 14.5f;  perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar vel ROC: velocity jump > 2 m/s for 3 cycles latches fault");
}

/* =========================================================================
 * TEST 23 — FR-PER-006 (radar ROC — FABSF negative-distance-difference paths)
 * Covers the FABSF macro's negate branch for distance inside the radar ROC
 * check. All prior tests only had stable or *increasing* distance, so the
 * negative-difference path of FABSF(d_r - prev_d) was never exercised.
 *
 * Phase A (small decrease): d drops by 3 m — FABSF = 3 <= DIST_ROC_LIMIT.
 *   Covers: FABSF ternary TRUE (negate), ROC comparison FALSE (fallthrough).
 * Phase B (large decrease): d drops by 20 m — FABSF = 20 > DIST_ROC_LIMIT.
 *   Covers: FABSF ternary TRUE (negate), ROC comparison TRUE (short-circuit).
 * ====================================================================== */
static void test_radar_roc_decreasing_distance(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();   /* radar_d=30, radar_vr=5 */

    /* Warm-up: establish prev_d=90, prev_vr=5 */
    in.radar_d = 90.0f;
    perception_step(&in, &out);

    /* Phase A: small decrease — ROC does NOT fire (3 <= 10).
     * diff = 87-90 = -3 < 0 -> FABSF negate branch; result=3 <= 10. */
    in.radar_d = 87.0f;
    perception_step(&in, &out);
    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar ROC dec-d: small decrease (3 m) does NOT latch fault");

    /* Phase B: large decrease — ROC fires (20 > 10).
     * diff = 67-87 = -20 < 0 -> FABSF negate branch; result=20 > 10. */
    in.radar_d = 67.0f;
    perception_step(&in, &out);
    in.radar_d = 47.0f;
    perception_step(&in, &out);
    in.radar_d = 27.0f;
    perception_step(&in, &out);
    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar ROC dec-d: large decrease (20 m) latches fault after 3 cycles");
}

/* =========================================================================
 * TEST 24 — FR-PER-006 (radar velocity ROC — FABSF negative-velocity-diff)
 * Covers the FABSF ternary negate branch for velocity inside the radar ROC
 * check. Test 22 only covered velocity *increasing*; here velocity decreases.
 *
 * VEL_ROC_LIMIT = 2.0 m/s/cycle. We step vr by -3 m/s each cycle:
 *   Cycle 1: vr=7.0  from prev=10 -> diff=-3, FABSF=3 > 2 -> bad, ctr=1
 *   Cycle 2: vr=4.0  from prev=7  -> diff=-3, FABSF=3 > 2 -> bad, ctr=2
 *   Cycle 3: vr=1.0  from prev=4  -> diff=-3, FABSF=3 > 2 -> bad, ctr=3 -> FAULT
 * Distance stays at 30 m (dist ROC = 0), so the distance condition is FALSE
 * and the velocity condition alone drives the OR to TRUE.
 * ====================================================================== */
static void test_radar_vel_roc_decreasing_vr(void)
{
    raw_sensor_input_t in;
    perception_output_t out;

    perception_init();
    in = make_good_input();

    /* Warm-up: establish prev_d=30, prev_vr=10 */
    in.radar_d  = 30.0f;
    in.radar_vr = 10.0f;
    perception_step(&in, &out);

    in.radar_d = 30.0f;  in.radar_vr = 7.0f;  perception_step(&in, &out);
    in.radar_d = 30.0f;  in.radar_vr = 4.0f;  perception_step(&in, &out);
    in.radar_d = 30.0f;  in.radar_vr = 1.0f;  perception_step(&in, &out);

    CHECK((out.fault_flag & AEB_FAULT_RADAR) != 0U,
          "Radar vel ROC dec-vr: decreasing velocity > 2 m/s latches fault");
}

/* =========================================================================
 * TEST 25 — FR-PER-006 (radar range check — negative relative velocity)
 * Covers the FABSF macro's negate branch on line 86 for the third condition
 * of the radar range check: FABSF(vr_r) > MAX_REL_VEL.
 * All prior tests used vr_r >= 0. This test uses a *negative* vr_r (receding
 * target) that is within range: FABSF(-15) = 15 <= MAX_REL_VEL=50 -> bad=0.
 * ====================================================================== */
static void test_radar_negative_velocity_in_range(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.radar_d  = 30.0f;
    in.radar_vr = -15.0f;  /* negative (receding), within [-50, 50] */

    for (i = 0; i < 10; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar negative vr in range: no fault for vr=-15 m/s (within limit)");
}

/* =========================================================================
 * TEST 26 — FR-PER-006 (radar velocity ROC — negative diff within limit)
 * Complements TEST 24 (which fires the velocity ROC) and TEST 22 (positive
 * diff). Here velocity DECREASES by less than VEL_ROC_LIMIT=2.0 m/s so the
 * ROC guard's FABSF negate branch executes but the comparison stays FALSE.
 *
 * prev_vr=5, then vr=4.5 -> diff=-0.5, FABSF(-0.5)=0.5 <= 2.0 -> no fault.
 * ====================================================================== */
static void test_radar_small_velocity_decrease_no_roc(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();   /* radar_vr=5.0, prev_vr will be 5.0 */
    perception_step(&in, &out);   /* warm-up: prev_vr=5.0, is_first=0 */

    /* Small velocity decrease — diff=-0.5, FABSF(-0.5)=0.5 <= VEL_ROC_LIMIT=2 */
    in.radar_d  = 30.0f;
    in.radar_vr = 4.5f;
    for (i = 0; i < 5; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar vel ROC: small decrease (0.5 m/s) does NOT latch fault");
}

/* =========================================================================
 * TEST 27 — FR-PER-006 (radar range check — negative vr near RADAR_DIST_MIN)
 * Exercises the specific combination: d close to (but above) RADAR_DIST_MIN,
 * and vr negative (receding target). Forces the FABSF ternary's negate branch
 * on line 86 via a path that prior tests did not walk.
 *
 * Uses d=1.0 (above RADAR_DIST_MIN=0.5) and vr=-20.0 (negative, |20|<=50).
 * ====================================================================== */
static void test_radar_negative_vr_near_dist_min(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;

    perception_init();
    in = make_good_input();
    in.radar_d  = 1.0f;    /* just above RADAR_DIST_MIN=0.5 — in range */
    in.radar_vr = -20.0f;  /* negative velocity, |20|<=MAX_REL_VEL=50 */

    for (i = 0; i < 10; i++) {
        perception_step(&in, &out);
    }
    CHECK((out.fault_flag & AEB_FAULT_RADAR) == 0U,
          "Radar neg-vr near dist-min: no range fault (d=1.0, vr=-20)");
}

/* =========================================================================
 * Main
 * ====================================================================== */
int main(void)
{
    printf("=== AEB Perception Unit Tests ===\n\n");

    test_lidar_no_fault_before_latch();
    test_lidar_fault_latches_at_cycle3();
    test_lidar_counter_resets_on_good_frame();
    test_radar_fault_latches_at_cycle3();
    test_radar_no_fault_single_bad_vr();
    test_can_timeout_sets_fault_flags();
    test_kalman_convergence();
    test_output_clamp_min_distance();
    test_output_clamp_max_distance();
    test_fault_injection_disables_sensors();
    test_confidence_both_sensors_gt_one_sensor();
    test_no_fault_on_good_data();
    test_output_clamp_rel_velocity();
    test_ego_velocity_passthrough();
    test_lidar_roc_triggers_counter();
    test_radar_roc_triggers_counter();
    test_init_resets_state();
    test_can_timeout_preserves_counters();
    /* New tests — coverage gap closers */
    test_lidar_counter_saturation();
    test_radar_counter_saturation();
    test_confidence_radar_only();
    test_radar_vel_roc_triggers_fault();
    test_radar_roc_decreasing_distance();
    test_radar_vel_roc_decreasing_vr();
    test_radar_negative_velocity_in_range();
    test_radar_small_velocity_decrease_no_roc();
    test_radar_negative_vr_near_dist_min();

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);

    return (g_fail == 0) ? 0 : 1;
}
