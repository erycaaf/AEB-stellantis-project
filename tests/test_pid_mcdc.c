/**
 * @file    test_pid_mcdc.c
 * @brief   Additional MC/DC coverage tests for aeb_pid module.
 * @module  Execution — PID braking controller (defensive paths)
 * @version 1.0
 * @date    2026-04-20
 * @author  Rian Ithalo (cross-validation — ISO 26262 independence)
 *
 * @details Tests targeting defensive/boundary conditions required for
 *          ASIL-D MC/DC coverage. These complement the nominal tests
 *          in test_pid.c (authored by Jéssica) with:
 *            - NULL pointer injection (FR-BRK safety)
 *            - Boundary values in clamp_f32 (lower bound)
 *            - Invalid decel_target in active braking states
 *
 *          Following ISO 26262-6 §9.4.2, these tests exercise the
 *          "false" outcome of each atomic condition that nominal
 *          requirements-based tests cannot reach in normal operation.
 *
 * @requirements FR-BRK-001, FR-BRK-002 (defensive), non-functional NFR-SAF-NULL
 * @traceability aeb_pid.c:61 (clamp lower bound),
 *               aeb_pid.c:98 (NULL guard),
 *               aeb_pid.c:119 (decel_target boundary)
 */

#include <stdio.h>
#include <math.h>
#include "aeb_pid.h"
#include "aeb_config.h"
#include "aeb_types.h"

/* ========================================================================= */
/*  Minimal test framework (mirrors test_pid.c style)                        */
/* ========================================================================= */

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

/* ========================================================================= */
/*  MC/DC Test 1 — clamp_f32 lower bound                                     */
/*                                                                           */
/*  Target: aeb_pid.c:61  -> if (result < lo)                                */
/*  Atomic condition "result < lo" must evaluate TRUE at least once.         */
/*  Nominal tests never produce negative/below-minimum values because the    */
/*  PI controller's integrator saturates upward — but MISRA + ASIL-D         */
/*  require the defensive path to be exercised.                              */
/*                                                                           */
/*  Strategy: call pid_brake_step in BRAKE_L1 with extreme a_ego >> target   */
/*  so the PI produces a NEGATIVE intermediate value that must be clamped    */
/*  up to 0.0F by the lower-bound branch.                                    */
/* ========================================================================= */
static void test_mcdc_clamp_lower_bound(void)
{
    printf("\n[MC/DC-01] clamp_f32 lower bound (result < lo)\n");

    pid_init();
    pid_output_t out = {0};

    /* Inject a situation where the error is extremely NEGATIVE:
     * decel_target is small positive, a_ego is much larger (over-braking).
     * The PI error = (decel_target - a_ego) will be very negative,
     * pushing the proportional term below 0 and forcing the lower-bound
     * clamp to activate.                                                   */
    pid_brake_step(1.0F, 50.0F, (uint8_t)FSM_BRAKE_L1, &out);

    TEST_ASSERT(out.brake_pct >= 0.0F,
        "brake_pct clamped to >= 0 (lower bound exercised)");
    TEST_ASSERT(out.brake_pct <= 100.0F,
        "brake_pct still within [0, 100] range");
}

/* ========================================================================= */
/*  MC/DC Test 2 — NULL output pointer                                       */
/*                                                                           */
/*  Target: aeb_pid.c:98  -> if (output == NULL)                             */
/*  Atomic condition "output == NULL" must evaluate TRUE at least once.      */
/*                                                                           */
/*  This is a SAFETY requirement (MISRA C:2012 Dir 4.7 + ISO 26262-6 §9.3)  */
/*  the function MUST NOT crash when called with NULL. It must return       */
/*  gracefully with no side effects.                                         */
/* ========================================================================= */
static void test_mcdc_null_output_pid_step(void)
{
    printf("\n[MC/DC-02] pid_brake_step NULL guard\n");

    pid_init();

    /* If the NULL guard is missing, this call segfaults.
     * If the guard is correct, it returns without writing anything.        */
    pid_brake_step(4.0F, 0.0F, (uint8_t)FSM_BRAKE_L2, NULL);

    TEST_ASSERT(1, "pid_brake_step survived NULL output (no crash)");
}

/* ========================================================================= */
/*  MC/DC Test 3 — decel_target <= 0 in active braking state                 */
/*                                                                           */
/*  Target: aeb_pid.c:119 ->                                                 */
/*     if ((fsm_state < FSM_BRAKE_L1) || (decel_target <= 0.0F))             */
/*                                                                           */
/*  Atomic condition "decel_target <= 0.0F" must evaluate TRUE               */
/*  INDEPENDENTLY (while fsm_state >= FSM_BRAKE_L1 is FALSE, i.e. we ARE     */
/*  in an active braking state). This proves the second condition can       */
/*  affect the outcome on its own — the MC/DC requirement (SC-MCDC-03).     */
/*                                                                           */
/*  Real-world relevance: a corrupted FSM output with decel_target=0 in     */
/*  BRAKE state must still be treated as "no braking demand" to fail-safe.  */
/* ========================================================================= */
static void test_mcdc_decel_target_zero_in_brake_state(void)
{
    printf("\n[MC/DC-03] decel_target <= 0 while in BRAKE state\n");

    pid_init();
    pid_output_t out = {0};

    /* Active braking state + zero decel target (corrupted/fault scenario) */
    pid_brake_step(0.0F, 0.0F, (uint8_t)FSM_BRAKE_L1, &out);

    TEST_ASSERT(out.brake_pct == 0.0F,
        "no braking produced when decel_target=0 in BRAKE_L1 (fail-safe)");

    /* Same check with negative decel_target (shouldn't produce acceleration) */
    pid_brake_step(-2.0F, 0.0F, (uint8_t)FSM_BRAKE_L2, &out);

    TEST_ASSERT(out.brake_pct == 0.0F,
        "no braking produced when decel_target<0 in BRAKE_L2 (fail-safe)");
}

/* ========================================================================= */
/*  Main                                                                     */
/* ========================================================================= */
int main(void)
{
    printf("========================================\n");
    printf("  AEB PID — MC/DC Cross-Validation\n");
    printf("  (Independence — Rian, ISO 26262-6)\n");
    printf("========================================\n");

    test_mcdc_clamp_lower_bound();
    test_mcdc_null_output_pid_step();
    test_mcdc_decel_target_zero_in_brake_state();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}