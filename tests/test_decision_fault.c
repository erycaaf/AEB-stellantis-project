/**
 * @file test_decision_fault.c
 * @brief Systematic fault-injection tests for the Decision module (TTC + FSM).
 *
 * Exercises aeb_ttc.c and aeb_fsm.c with invalid, corrupted and extreme
 * inputs to verify that the implementation remains fail-safe under ASIL-D.
 * Independent verification of code authored by Lourenco J. Mphili;
 * cross-validated by Eryca.
 *
 * Categories exercised (ISO 26262-6 Tab. 11 item 1e — "++ for ASIL-D"):
 *   A  Invalid float values (NaN, +/-Inf) on live-data inputs.
 *   B  Numeric extremes (negative distance, overflow of v^2 cast).
 *   C  Corrupted state (simulated single-event upsets on fsm_state,
 *      aeb_enabled, fault_flag, is_closing).
 *   D  Persistence of state under abusive sequences (rapid toggling,
 *      long-hold timers, debounce oscillation).
 *   E  Timing / concurrency — OUT OF SCOPE for unit level,
 *      deferred to SIL (no scheduler on host bench).
 *
 * Each assertion is non-fatal: failures are counted, not aborted, so the
 * full robustness picture is visible in a single run.
 *
 * @req FR-DEC-001..011, FR-FSM-001..006, NFR-SAF-ROB
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_types.h"
#include "aeb_config.h"

/* ===================================================================
 * Non-fatal test harness (counters only, no abort)
 * =================================================================== */

static int32_t asserts_run    = 0;
static int32_t asserts_passed = 0;
static int32_t asserts_failed = 0;
static const char *current_case = "";

#define FAULT_ASSERT(cond, tag)                                        \
    do {                                                               \
        asserts_run++;                                                 \
        if (cond) {                                                    \
            asserts_passed++;                                          \
        } else {                                                       \
            asserts_failed++;                                          \
            printf("    FAIL  %-10s  %s\n", current_case, tag);        \
        }                                                              \
    } while (0)

#define RUN_CASE(fn)                                                   \
    do {                                                               \
        current_case = #fn;                                            \
        printf("  [%s]\n", #fn);                                       \
        (fn)();                                                        \
    } while (0)

/* ===================================================================
 * Shared test fixtures
 * =================================================================== */

static perception_output_t perc;
static driver_input_t      driver;
static ttc_output_t        ttc;
static fsm_output_t        fsm;

/**
 * @brief Reset fixtures to a known-good baseline.
 *
 * Baseline: ego at 15 m/s, no threat, AEB enabled, no override, no fault,
 * FSM initialised to STANDBY via fsm_init().
 */
static void fault_reset(void)
{
    (void)memset(&perc,   0, sizeof(perc));
    (void)memset(&driver, 0, sizeof(driver));
    (void)memset(&ttc,    0, sizeof(ttc));
    (void)memset(&fsm,    0, sizeof(fsm));

    perc.distance      = 50.0f;
    perc.v_ego         = 15.0f;
    perc.v_rel         = 0.0f;
    perc.fault_flag    = 0U;

    driver.aeb_enabled = 1U;

    ttc.ttc            = TTC_MAX;
    ttc.d_brake        = 10.0f;
    ttc.is_closing     = 0U;

    fsm_init(&fsm);
}

/* ===================================================================
 * Fail-safe predicates
 * =================================================================== */

/**
 * @brief FSM output passes fail-safe checks for ASIL-D.
 *
 * All scalar fields must be finite, non-negative where applicable, and
 * the enumerated fields must be inside their declared range.
 */
static int fsm_output_is_valid(const fsm_output_t *f)
{
    if (f->fsm_state > (uint8_t)FSM_POST_BRAKE)       { return 0; }
    if ((f->brake_active != 0U) && (f->brake_active != 1U)) { return 0; }
    if (f->alert_level > 2U)                          { return 0; }
    if (!isfinite(f->decel_target) || (f->decel_target < 0.0f)) { return 0; }
    if (!isfinite(f->warn_timer)   || (f->warn_timer   < 0.0f)) { return 0; }
    if (!isfinite(f->state_timer)  || (f->state_timer  < 0.0f)) { return 0; }
    return 1;
}

/**
 * @brief TTC output passes fail-safe checks for ASIL-D.
 */
static int ttc_output_is_valid(const ttc_output_t *t)
{
    if (!isfinite(t->ttc) || (t->ttc < 0.0f) || (t->ttc > (TTC_MAX + 1e-3f))) {
        return 0;
    }
    if (!isfinite(t->d_brake) || (t->d_brake < 0.0f)) { return 0; }
    if ((t->is_closing != 0U) && (t->is_closing != 1U)) { return 0; }
    return 1;
}

/* ===================================================================
 * CATEGORY A — NaN / +/- Infinity on live-data inputs
 * =================================================================== */

/** A1 — ttc_calc with NaN distance must remain in [0, TTC_MAX]. */
static void fault_a1_ttc_calc_nan_distance(void)
{
    float32_t r = ttc_calc((float32_t)NAN, 5.0f);
    FAULT_ASSERT(isfinite(r),                       "A1: finite");
    FAULT_ASSERT((r >= 0.0f) && (r <= TTC_MAX),     "A1: in [0, TTC_MAX]");
}

/** A2 — ttc_calc with NaN v_rel must remain in [0, TTC_MAX]. */
static void fault_a2_ttc_calc_nan_vrel(void)
{
    float32_t r = ttc_calc(10.0f, (float32_t)NAN);
    FAULT_ASSERT(isfinite(r),                       "A2: finite");
    FAULT_ASSERT((r >= 0.0f) && (r <= TTC_MAX),     "A2: in [0, TTC_MAX]");
}

/** A3 — ttc_calc with +Inf v_rel (division produces +0; must stay in range). */
static void fault_a3_ttc_calc_inf_vrel(void)
{
    float32_t r = ttc_calc(10.0f, (float32_t)INFINITY);
    FAULT_ASSERT(isfinite(r),                       "A3: finite");
    FAULT_ASSERT((r >= 0.0f) && (r <= TTC_MAX),     "A3: in [0, TTC_MAX]");
}

/** A4 — d_brake_calc with NaN v_ego.
 *  NaN * NaN = NaN; clamp (d_brake < 0.0f) evaluates false for NaN in
 *  IEEE 754, so NaN propagates to the output. Expected to FAIL until
 *  the module adds isfinite() gating (Bug candidate #1). */
static void fault_a4_dbrake_calc_nan(void)
{
    float32_t r = d_brake_calc((float32_t)NAN);
    FAULT_ASSERT(isfinite(r),     "A4: d_brake_calc(NaN) finite");
    FAULT_ASSERT(r >= 0.0f,       "A4: d_brake_calc(NaN) non-negative");
}

/** A5 — d_brake_calc with +Inf v_ego yields +Inf; same failure mode as A4. */
static void fault_a5_dbrake_calc_inf(void)
{
    float32_t r = d_brake_calc((float32_t)INFINITY);
    FAULT_ASSERT(isfinite(r),     "A5: d_brake_calc(+Inf) finite");
}

/** A6 — fsm_step with delta_t_s = NaN.
 *  Defensive guard is (delta_t_s <= 0.0f); NaN comparisons are false
 *  in IEEE 754, so NaN bypasses the guard and poisons warn_timer via
 *  += NaN. Expected to FAIL (Bug candidate #3). */
static void fault_a6_fsm_delta_t_nan(void)
{
    fault_reset();
    perc.v_rel     = 5.0f;
    ttc.ttc        = 3.5f;
    ttc.is_closing = 1U;
    fsm.fsm_state  = (uint8_t)FSM_WARNING;

    fsm_step((float32_t)NAN, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(isfinite(fsm.warn_timer),    "A6: warn_timer finite");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "A6: fsm output valid");
}

/** A7 — ttc_process with NaN distance must produce a valid ttc_output. */
static void fault_a7_ttc_process_nan_distance(void)
{
    fault_reset();
    perc.distance = (float32_t)NAN;
    ttc_process(&perc, &ttc);
    FAULT_ASSERT(ttc_output_is_valid(&ttc),   "A7: ttc_output valid");
}

/* ===================================================================
 * CATEGORY B — Numeric extremes
 * =================================================================== */

/** B1 — Negative distance must return saturated TTC_MAX (no division). */
static void fault_b1_ttc_calc_neg_distance(void)
{
    float32_t r = ttc_calc(-50.0f, 5.0f);
    FAULT_ASSERT(r >= (TTC_MAX - 1e-3f),      "B1: saturates to TTC_MAX");
}

/** B2 — d_brake_calc with FLT_MAX overflows v^2 to +Inf (Bug candidate #1). */
static void fault_b2_dbrake_calc_fltmax(void)
{
    float32_t r = d_brake_calc(FLT_MAX);
    FAULT_ASSERT(isfinite(r),     "B2: d_brake_calc(FLT_MAX) finite");
}

/** B3 — Large negative v_ego: v^2 is large positive, result must be finite. */
static void fault_b3_dbrake_calc_large_neg(void)
{
    float32_t r = d_brake_calc(-100.0f);
    FAULT_ASSERT(isfinite(r) && (r >= 0.0f),  "B3: finite and non-negative");
}

/** B4 — Huge delta_t must not leave timers non-finite.
 *  Even if the code does not saturate, the invariant is that warn_timer
 *  remains a finite float (no +Inf), because +=huge_positive is still finite. */
static void fault_b4_fsm_delta_t_huge(void)
{
    fault_reset();
    fsm.fsm_state = (uint8_t)FSM_WARNING;
    ttc.ttc = 3.5f;
    ttc.is_closing = 1U;
    perc.v_rel = 5.0f;

    fsm_step(1.0e20f, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(isfinite(fsm.warn_timer),    "B4: warn_timer finite");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "B4: fsm output valid");
}

/* ===================================================================
 * CATEGORY C — SEU / corrupted state
 * =================================================================== */

/** C1 — fsm_state corrupted to 42 (outside enum range).
 *  The switch default branch handles unknown current_state as STANDBY,
 *  but new_state is initialised to current_state and only overwritten
 *  when desired_state in {WARNING, BRAKE_L1..L3}. If desired is STANDBY,
 *  the corrupted value propagates to the output. Expected to FAIL until
 *  fsm_step validates fsm_state on entry (Bug candidate #4). */
static void fault_c1_fsm_state_42(void)
{
    fault_reset();
    fsm.fsm_state = 42U;
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state <= (uint8_t)FSM_POST_BRAKE,
                 "C1: fsm_state normalized");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "C1: fsm output valid");
}

/** C2 — fsm_state corrupted to 255 (UINT8_MAX). Same failure mode as C1. */
static void fault_c2_fsm_state_255(void)
{
    fault_reset();
    fsm.fsm_state = 255U;
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state <= (uint8_t)FSM_POST_BRAKE,
                 "C2: fsm_state normalized");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "C2: fsm output valid");
}

/** C3 — driver->aeb_enabled corrupted to 0xAA.
 *  Code checks (driver->aeb_enabled == 0U); any non-zero bit pattern —
 *  including a SEU-corrupted 0xAA — is interpreted as "enabled". A fail-safe
 *  design should treat any non-canonical Boolean as disabled. Expected to
 *  FAIL until driver->aeb_enabled is normalized to {0,1} on entry
 *  (Bug candidate #2, analogous to Rian's UDS Bug #2). */
static void fault_c3_aeb_enabled_0xAA(void)
{
    fault_reset();
    driver.aeb_enabled = 0xAAU;
    perc.distance      = 5.0f;
    perc.v_rel         = 10.0f;
    ttc.ttc            = 1.0f;
    ttc.d_brake        = 8.0f;
    ttc.is_closing     = 1U;

    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_OFF,
                 "C3: 0xAA treated as disabled");
}

/** C4 — perception->fault_flag corrupted to 0xFF must force FSM_OFF. */
static void fault_c4_fault_flag_0xFF(void)
{
    fault_reset();
    perc.fault_flag = 0xFFU;
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_OFF, "C4: fault -> FSM_OFF");
}

/** C5 — ttc_in->is_closing corrupted to 0x80. Current code uses
 *  (is_closing == 0U); any non-zero value is treated as closing. This is
 *  conservative (false positive is safer than false negative), so the test
 *  asserts the expected conservative behaviour. */
static void fault_c5_is_closing_0x80(void)
{
    fault_reset();
    ttc.is_closing = 0x80U;
    ttc.ttc        = 3.5f;
    perc.distance  = 50.0f;

    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_WARNING,
                 "C5: non-zero is_closing -> WARNING");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "C5: fsm output valid");
}

/* ===================================================================
 * CATEGORY D — Persistence / abusive sequences
 * =================================================================== */

/** D1 — 100x rapid toggle of aeb_enabled; end disabled -> OFF. */
static void fault_d1_rapid_toggle(void)
{
    int i;
    fault_reset();
    for (i = 0; i < 100; i++) {
        driver.aeb_enabled = ((uint32_t)i & 1U) ? 1U : 0U;
        fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    }
    /* Loop ends with i=100 (even) -> last write set aeb_enabled=0 for i=99
       (odd was 1, even was 0). i=99 is odd -> last enabled=1. Make it
       explicit to avoid off-by-one ambiguity. */
    driver.aeb_enabled = 0U;
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_OFF,
                 "D1: ends in FSM_OFF");
    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "D1: fsm output valid");
}

/** D2 — WARNING held for 2s, then BRAKE_L2 threat arrives.
 *  warn_timer should be >= 0.8s so the escalation triggers immediately. */
static void fault_d2_warning_persist_long(void)
{
    int i;
    fault_reset();
    /* Drive STANDBY -> WARNING through the normal path. */
    perc.v_rel     = 5.0f;
    ttc.ttc        = 3.5f;
    ttc.is_closing = 1U;
    for (i = 0; i < 200; i++) {   /* 2 seconds */
        fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    }
    /* Now BRAKE_L2 threat */
    ttc.ttc = 2.0f;
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);

    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_BRAKE_L2,
                 "D2: escalates to BRAKE_L2");
    FAULT_ASSERT(isfinite(fsm.warn_timer),    "D2: warn_timer finite");
}

/** D3 — POST_BRAKE auto-exits to STANDBY after 2s. */
static void fault_d3_post_brake_exit(void)
{
    int i;
    fault_reset();
    /* Reach POST_BRAKE via BRAKE_L3 then v_ego < V_STOP_THRESHOLD. */
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L3;
    perc.v_ego    = 0.0f;                 /* triggers POST_BRAKE transition */
    fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    /* Restore a plausible speed range for subsequent steps. */
    perc.v_ego    = 5.0f;

    for (i = 0; i < 210; i++) {           /* hold > 2 seconds */
        fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    }

    FAULT_ASSERT(fsm.fsm_state == (uint8_t)FSM_STANDBY,
                 "D3: auto-exit to STANDBY");
}

/** D4 — Oscillating BRAKE band faster than debounce must leave FSM in a
 *  valid braking state. */
static void fault_d4_debounce_oscillation(void)
{
    int i;
    fault_reset();
    fsm.fsm_state  = (uint8_t)FSM_BRAKE_L3;
    perc.v_rel     = 5.0f;
    ttc.is_closing = 1U;

    for (i = 0; i < 50; i++) {
        ttc.ttc = ((uint32_t)i & 1U) ? 2.0f : 1.5f;  /* L2 vs L3 band */
        fsm_step(AEB_DT, &perc, &driver, &ttc, &fsm);
    }

    FAULT_ASSERT(fsm_output_is_valid(&fsm),   "D4: fsm output valid");
    FAULT_ASSERT((fsm.fsm_state >= (uint8_t)FSM_BRAKE_L1) &&
                 (fsm.fsm_state <= (uint8_t)FSM_BRAKE_L3),
                 "D4: still braking");
}

/* ===================================================================
 * Main
 * =================================================================== */

int main(void)
{
    printf("\n");
    printf("==========================================================\n");
    printf(" Decision (TTC+FSM) - Fault Injection Suite (ASIL-D)\n");
    printf("==========================================================\n");

    printf("\n[Category A] NaN / +-Infinity on live inputs\n");
    RUN_CASE(fault_a1_ttc_calc_nan_distance);
    RUN_CASE(fault_a2_ttc_calc_nan_vrel);
    RUN_CASE(fault_a3_ttc_calc_inf_vrel);
    RUN_CASE(fault_a4_dbrake_calc_nan);
    RUN_CASE(fault_a5_dbrake_calc_inf);
    RUN_CASE(fault_a6_fsm_delta_t_nan);
    RUN_CASE(fault_a7_ttc_process_nan_distance);

    printf("\n[Category B] Numeric extremes\n");
    RUN_CASE(fault_b1_ttc_calc_neg_distance);
    RUN_CASE(fault_b2_dbrake_calc_fltmax);
    RUN_CASE(fault_b3_dbrake_calc_large_neg);
    RUN_CASE(fault_b4_fsm_delta_t_huge);

    printf("\n[Category C] SEU / corrupted state\n");
    RUN_CASE(fault_c1_fsm_state_42);
    RUN_CASE(fault_c2_fsm_state_255);
    RUN_CASE(fault_c3_aeb_enabled_0xAA);
    RUN_CASE(fault_c4_fault_flag_0xFF);
    RUN_CASE(fault_c5_is_closing_0x80);

    printf("\n[Category D] Persistence / abusive sequences\n");
    RUN_CASE(fault_d1_rapid_toggle);
    RUN_CASE(fault_d2_warning_persist_long);
    RUN_CASE(fault_d3_post_brake_exit);
    RUN_CASE(fault_d4_debounce_oscillation);

    printf("\n==========================================================\n");
    printf(" Results: %d run, %d passed, %d failed\n",
           asserts_run, asserts_passed, asserts_failed);
    printf("==========================================================\n\n");

    return (asserts_failed == 0) ? 0 : 1;
}
