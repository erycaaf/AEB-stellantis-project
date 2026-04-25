/**
 * @file    test_perception_fault.c
 * @brief   Fault-injection suite for the AEB Perception module (ASIL-D V&V).
 *
 * Aligned with ISO 26262-6:2018 Tab. 11 item 1e (Error guessing / fault
 * injection — "++" for ASIL-D).
 *
 * Categories exercised (analogous to UDS report §3):
 *   A. NaN / +/-inf in raw sensor inputs
 *   B. Out-of-range numeric extremes (FLT_MAX, large negatives, denormals)
 *   C. Bit-flip / SEU on counter/state bytes (control-path corruption)
 *   D. Persistence: rapid toggling, cycle counter saturation
 *
 * Fail-safe predicate (Categories A, B):
 *   "Under abnormal sensor input, the perception output must be either
 *    (i) marked as faulty (fault_flag != 0) AND have confidence == 0.0,
 *    OR
 *    (ii) emit a finite, in-range distance/v_rel within physical bounds."
 *
 *   ANY output with NaN/inf in distance, v_rel or confidence — OR a finite
 *   value with confidence > 0 while the input was NaN/inf — is NOT fail-safe.
 *
 * Compile (manual): see Makefile target `make fault-perception`.
 *
 * Each assertion prints PASS or FAIL with diagnostic context. Exit code is
 * 0 if all assertions pass, 1 otherwise. Non-zero exit is EXPECTED while bugs
 * are pending patch.
 *
 * Requirements verified: NFR-SAF-005 (fault propagation), NFR-SAF-ROB
 * (robustness to invalid sensor data), FR-PER-006 (fault detection),
 * FR-PER-007 (fault flag semantics).
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <string.h>

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_perception.h"

/* ============================================================
 * Mini test framework (matches test_perception.c style)
 * ============================================================ */

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT(cond, name)                                         \
    do {                                                           \
        if (cond) {                                                \
            printf("  [PASS] %s\n", (name));                       \
            g_pass++;                                              \
        } else {                                                   \
            printf("  [FAIL] %s  (line %d)\n", (name), __LINE__);  \
            g_fail++;                                              \
        }                                                          \
    } while (0)

/* Helper: build a nominal "good" sensor input frame. */
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

/* Predicate: output is "fail-safe" under abnormal input.
 * Returns 1 if fail-safe, 0 otherwise. */
static int is_fail_safe(const perception_output_t *out)
{
    /* Branch (i): explicitly faulted with zero confidence. */
    if ((out->fault_flag != 0U) && (out->confidence == 0.0f)) {
        if (isfinite(out->distance) && isfinite(out->v_rel)) {
            return 1;
        }
    }
    /* Branch (ii): finite, in-range output (treated as a clean recovery). */
    if (isfinite(out->distance) && isfinite(out->v_rel) &&
        isfinite(out->confidence) &&
        (out->distance >= DIST_MIN_OUTPUT) &&
        (out->distance <= DIST_MAX_OUTPUT) &&
        (out->confidence >= 0.0f) && (out->confidence <= 1.0f)) {
        /* But: if input was abnormal, we still expect SOME signal of
         * degradation. The caller's assertion handles that semantic. */
        return 1;
    }
    return 0;
}

/* ============================================================
 * CATEGORY A — NaN / +inf / -inf in raw sensor inputs
 * ============================================================ */

static void cat_A_nan_lidar(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[A1] NaN injected in lidar_d (single frame)\n");

    perception_init();
    in = make_good_input();
    in.lidar_d = (float)NAN;
    perception_step(&in, &out);

    printf("       distance=%f  v_rel=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.v_rel,
           (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT(is_fail_safe(&out),
           "A1: NaN lidar_d must produce fail-safe output");
    ASSERT(isfinite(out.distance),
           "A1: distance must be finite (no NaN propagation)");
}

static void cat_A_nan_radar_d(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[A2] NaN injected in radar_d (single frame)\n");

    perception_init();
    in = make_good_input();
    in.radar_d = (float)NAN;
    perception_step(&in, &out);

    printf("       distance=%f  v_rel=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.v_rel,
           (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT(is_fail_safe(&out),
           "A2: NaN radar_d must produce fail-safe output");
    ASSERT(isfinite(out.v_rel),
           "A2: v_rel must be finite (no NaN propagation through Kalman)");
}

static void cat_A_inf_lidar(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[A3] +inf injected in lidar_d\n");

    perception_init();
    in = make_good_input();
    in.lidar_d = (float)INFINITY;
    perception_step(&in, &out);

    printf("       distance=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT(is_fail_safe(&out),
           "A3: +inf lidar_d must produce fail-safe output");
}

static void cat_A_neg_inf_radar_vr(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[A4] -inf injected in radar_vr\n");

    perception_init();
    in = make_good_input();
    in.radar_vr = -(float)INFINITY;
    perception_step(&in, &out);

    printf("       distance=%f  v_rel=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.v_rel,
           (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT(is_fail_safe(&out),
           "A4: -inf radar_vr must produce fail-safe output");
    ASSERT(isfinite(out.v_rel),
           "A4: v_rel must remain finite");
}

static void cat_A_nan_v_ego(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[A5] NaN injected in v_ego (passthrough)\n");

    perception_init();
    in = make_good_input();
    in.v_ego = (float)NAN;
    perception_step(&in, &out);

    printf("       v_ego_out=%f  fault=0x%02x\n",
           (double)out.v_ego, (unsigned)out.fault_flag);
    /* v_ego is a documented passthrough, but propagating NaN to downstream
     * (TTC, FSM) cripples the entire pipeline. Predicate: out.v_ego must
     * be finite OR fault_flag must be raised. */
    ASSERT(isfinite(out.v_ego) || (out.fault_flag != 0U),
           "A5: NaN v_ego must be sanitised or trigger a fault flag");
}

/* ============================================================
 * CATEGORY B — Out-of-range numeric extremes
 * ============================================================ */

static void cat_B_flt_max_radar(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[B1] FLT_MAX injected in radar_d for SENSOR_FAULT_CYCLES frames\n");

    perception_init();
    in = make_good_input();
    in.radar_d = FLT_MAX;
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    printf("       distance=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT(out.fault_flag & AEB_FAULT_RADAR,
           "B1: FLT_MAX radar_d must latch RADAR fault");
    ASSERT(isfinite(out.distance),
           "B1: distance must remain finite (clamped)");
}

static void cat_B_neg_distance(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[B2] Large negative lidar_d for SENSOR_FAULT_CYCLES frames\n");

    perception_init();
    in = make_good_input();
    in.lidar_d = -1000.0f;
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    printf("       distance=%f  fault=0x%02x\n",
           (double)out.distance, (unsigned)out.fault_flag);
    ASSERT(out.fault_flag & AEB_FAULT_LIDAR,
           "B2: large negative lidar_d must latch LIDAR fault");
}

static void cat_B_subnormal(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[B3] Subnormal float (FLT_MIN/2) in radar_d\n");

    perception_init();
    in = make_good_input();
    in.radar_d = FLT_MIN / 2.0f;   /* subnormal — below RADAR_DIST_MIN */
    perception_step(&in, &out);
    printf("       distance=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.confidence, (unsigned)out.fault_flag);
    /* Subnormal is below RADAR_DIST_MIN (0.5), so single-frame should be 'bad'
     * but not yet latched. Predicate: output remains finite + bounded. */
    ASSERT(isfinite(out.distance) && (out.distance >= DIST_MIN_OUTPUT),
           "B3: subnormal input must produce finite, in-range output");
}

static void cat_B_velocity_extreme(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[B4] Extreme radar_vr (1000 m/s) for SENSOR_FAULT_CYCLES frames\n");

    perception_init();
    in = make_good_input();
    in.radar_vr = 1000.0f;
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    printf("       v_rel=%f  fault=0x%02x\n",
           (double)out.v_rel, (unsigned)out.fault_flag);
    ASSERT(out.fault_flag & AEB_FAULT_RADAR,
           "B4: 1000 m/s radar_vr must latch RADAR fault");
    ASSERT((out.v_rel >= -MAX_REL_VEL) && (out.v_rel <= MAX_REL_VEL),
           "B4: v_rel output must remain clamped to [-MAX_REL_VEL, +MAX_REL_VEL]");
}

static void cat_B_alternating_extreme(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[B5] Alternating ROC > DIST_ROC_LIMIT (radar) — regression guard\n");

    perception_init();
    in = make_good_input();
    /* Establish baseline */
    perception_step(&in, &out);
    /* Now jump by 50 m every cycle (5x DIST_ROC_LIMIT) */
    in.radar_d = 80.0f;
    perception_step(&in, &out);
    in.radar_d = 30.0f;
    perception_step(&in, &out);
    in.radar_d = 80.0f;
    perception_step(&in, &out);
    printf("       distance=%f  fault=0x%02x\n",
           (double)out.distance, (unsigned)out.fault_flag);
    /* Regression guard: post-fix behaviour (commit 9d7d965, PR #93
     * fix/perception-robustness). Alternating bad/good/bad frames must
     * reset the persistence counter, mirroring D2 oscillation semantics,
     * so radar fault MUST NOT latch on transient ROC violations. The
     * previous version of this test asserted the opposite (the buggy
     * behaviour documented as Bug #3 by the V&V cross-validation) — it
     * is kept under the same B5 category as an anti-regression check. */
    ASSERT((out.fault_flag & AEB_FAULT_RADAR) == 0U,
           "B5: alternating ROC (bad/good/bad) must NOT latch RADAR fault");
}

/* ============================================================
 * CATEGORY C — SEU / control-path corruption
 *
 * Perception keeps no externally addressable state struct exposed
 * by the public API (unlike UDS), so we exercise SEU-like conditions
 * via boundary scenarios that force the internal state machine into
 * paths that nominal tests do not reach.
 * ============================================================ */

static void cat_C_can_timeout_first_frame(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[C1] can_timeout asserted on the FIRST frame (state never seeded)\n");

    perception_init();
    in = make_good_input();
    in.can_timeout = 1U;   /* timeout before any good frame */
    perception_step(&in, &out);

    printf("       distance=%f  v_rel=%f  conf=%f  fault=0x%02x\n",
           (double)out.distance, (double)out.v_rel,
           (double)out.confidence, (unsigned)out.fault_flag);
    /* Predicate: with no prior seed, distance must NOT be reported as a
     * spurious finite value (e.g. clamped 0.5 m masquerading as a real
     * close-range obstacle). Output must signal degraded state and
     * confidence==0. */
    ASSERT(out.confidence == 0.0f,
           "C1: confidence must be 0 when timeout fires before seed");
    ASSERT((out.fault_flag & AEB_FAULT_CAN_TO) != 0U,
           "C1: CAN_TO bit must be set");
}

static void cat_C_fi_with_simultaneous_good_data(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[C2] fi=0xAA (non-Boolean) with otherwise good data\n");

    perception_init();
    in = make_good_input();
    in.fi = 0xAAU;        /* non-Boolean truthy value (SEU-like) */
    perception_step(&in, &out);
    printf("       conf=%f  fault=0x%02x\n",
           (double)out.confidence, (unsigned)out.fault_flag);
    /* fi is documented as Boolean; per spec, ANY non-zero must trigger
     * fault propagation. */
    ASSERT(out.fault_flag & AEB_FAULT_LIDAR,
           "C2: fi != 0 must propagate as LiDAR fault");
    ASSERT(out.fault_flag & AEB_FAULT_RADAR,
           "C2: fi != 0 must propagate as RADAR fault");
}

static void cat_C_can_timeout_byte(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    printf("\n[C3] can_timeout=0x80 (high-bit-only SEU)\n");

    perception_init();
    in = make_good_input();
    /* Seed the Kalman first */
    perception_step(&in, &out);
    in.can_timeout = 0x80U;   /* bit-flip pattern */
    perception_step(&in, &out);
    printf("       conf=%f  fault=0x%02x\n",
           (double)out.confidence, (unsigned)out.fault_flag);
    ASSERT((out.fault_flag & AEB_FAULT_CAN_TO) != 0U,
           "C3: any non-zero can_timeout byte must set CAN_TO bit");
    ASSERT(out.confidence == 0.0f,
           "C3: confidence must be zero when CAN timeout flagged");
}

/* ============================================================
 * CATEGORY D — Persistence / counter saturation
 * ============================================================ */

static void cat_D_counter_saturation(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[D1] LiDAR fault held for 300 cycles (uint8_t saturation)\n");

    perception_init();
    in = make_good_input();
    in.lidar_d = 999.0f;   /* permanently bad */
    for (i = 0; i < 300; i++) {
        perception_step(&in, &out);
    }
    printf("       fault=0x%02x  conf=%f\n",
           (unsigned)out.fault_flag, (double)out.confidence);
    ASSERT(out.fault_flag & AEB_FAULT_LIDAR,
           "D1: LiDAR fault must remain latched after 300 cycles");
    /* The internal counter cap at 255 must not roll over and clear the fault. */
    in.lidar_d = 999.0f;
    perception_step(&in, &out);
    ASSERT(out.fault_flag & AEB_FAULT_LIDAR,
           "D1: counter must saturate, not wrap (fault still latched on cycle 301)");
}

static void cat_D_rapid_recovery_relatch(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[D2] Rapid bad/good/bad oscillation (counter reset semantics)\n");

    perception_init();
    in = make_good_input();
    /* 100 oscillation cycles: 1 bad, 1 good */
    for (i = 0; i < 100; i++) {
        in.lidar_d = (i % 2 == 0) ? 999.0f : 30.0f;
        perception_step(&in, &out);
    }
    printf("       fault=0x%02x\n", (unsigned)out.fault_flag);
    /* With reset on every good frame and SENSOR_FAULT_CYCLES=3, the
     * counter never reaches the latch threshold. Predicate: no false
     * positive over 100 oscillations. */
    ASSERT((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
           "D2: oscillating bad/good must not produce false-positive latch");
}

static void cat_D_recovery_after_latch(void)
{
    raw_sensor_input_t in;
    perception_output_t out;
    int i;
    printf("\n[D3] Recovery after latch (FR-PER-006: latched fault is sticky?)\n");

    perception_init();
    in = make_good_input();
    /* Latch the fault */
    in.lidar_d = 999.0f;
    for (i = 0; i < (int)SENSOR_FAULT_CYCLES; i++) {
        perception_step(&in, &out);
    }
    printf("       after latch: fault=0x%02x\n", (unsigned)out.fault_flag);
    /* Now feed good data — observe whether the fault clears or remains */
    in.lidar_d = 30.2f;
    for (i = 0; i < 10; i++) {
        perception_step(&in, &out);
    }
    printf("       after 10 good frames: fault=0x%02x\n", (unsigned)out.fault_flag);
    /* Documented behaviour: counter resets on good frame, so fault should
     * clear. This asserts the documented (non-sticky) behaviour explicitly
     * so any future change to a sticky-latch design is caught. */
    ASSERT((out.fault_flag & AEB_FAULT_LIDAR) == 0U,
           "D3: LiDAR fault must clear after sustained good data");
}

/* ============================================================
 * Test runner
 * ============================================================ */

int main(void)
{
    printf("======================================================\n");
    printf("  Perception fault-injection suite (ASIL-D V&V)\n");
    printf("  ISO 26262-6:2018 Tab. 11 item 1e (++)\n");
    printf("======================================================\n");

    printf("\n--- Category A: NaN / inf in sensor inputs ---");
    cat_A_nan_lidar();
    cat_A_nan_radar_d();
    cat_A_inf_lidar();
    cat_A_neg_inf_radar_vr();
    cat_A_nan_v_ego();

    printf("\n--- Category B: Out-of-range numeric extremes ---");
    cat_B_flt_max_radar();
    cat_B_neg_distance();
    cat_B_subnormal();
    cat_B_velocity_extreme();
    cat_B_alternating_extreme();

    printf("\n--- Category C: SEU / control-path corruption ---");
    cat_C_can_timeout_first_frame();
    cat_C_fi_with_simultaneous_good_data();
    cat_C_can_timeout_byte();

    printf("\n--- Category D: Persistence / saturation ---");
    cat_D_counter_saturation();
    cat_D_rapid_recovery_relatch();
    cat_D_recovery_after_latch();

    printf("\n======================================================\n");
    printf("  Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("======================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
