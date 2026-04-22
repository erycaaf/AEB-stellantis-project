/**
 * @file    test_alert_mcdc.c
 * @brief   Additional MC/DC coverage tests for aeb_alert module.
 * @module  Execution — Alert mapping (defensive paths)
 * @version 1.0
 * @date    2026-04-20
 * @author  Rian Ithalo (cross-validation — ISO 26262 independence)
 *
 * @details Tests targeting the defensive NULL pointer guard in
 *          alert_map() that nominal requirements-based tests do not
 *          exercise. Required for ASIL-D MC/DC coverage compliance.
 *
 * @requirements FR-ALR-001 (defensive), NFR-SAF-NULL
 * @traceability aeb_alert.c:64 (NULL guard in alert_map)
 */

#include <stdio.h>
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

/* ========================================================================= */
/*  MC/DC Test — alert_map NULL output guard                                 */
/*                                                                           */
/*  Target: aeb_alert.c:64  -> if (output == NULL)                           */
/*  Atomic condition "output == NULL" must evaluate TRUE at least once.      */
/*                                                                           */
/*  Safety requirement (MISRA C:2012 Dir 4.7 + ISO 26262-6 §9.3):            */
/*  alert_map MUST NOT crash or produce side effects when called with NULL. */
/*                                                                           */
/*  Note: override_detect does not accept pointers — no NULL path exists    */
/*  there, so it is already 100% MC/DC from the nominal tests.              */
/* ========================================================================= */
static void test_mcdc_alert_map_null_output(void)
{
    printf("\n[MC/DC-01] alert_map NULL guard\n");

    /* If the NULL guard is missing, this call segfaults.
     * If correct, it returns without writing anything.                     */
    alert_map((uint8_t)FSM_BRAKE_L2, NULL);

    TEST_ASSERT(1, "alert_map survived NULL output (no crash)");

    /* Also test with every possible FSM state to be exhaustive:
     * ensures the NULL check happens BEFORE the switch on state.           */
    alert_map((uint8_t)FSM_OFF,        NULL);
    alert_map((uint8_t)FSM_STANDBY,    NULL);
    alert_map((uint8_t)FSM_WARNING,    NULL);
    alert_map((uint8_t)FSM_BRAKE_L1,   NULL);
    alert_map((uint8_t)FSM_BRAKE_L3,   NULL);
    alert_map((uint8_t)FSM_POST_BRAKE, NULL);

    TEST_ASSERT(1, "alert_map NULL guard dominates FSM-state switch");
}

/* ========================================================================= */
int main(void)
{
    printf("========================================\n");
    printf("  AEB Alert — MC/DC Cross-Validation\n");
    printf("  (Independence — Rian, ISO 26262-6)\n");
    printf("========================================\n");

    test_mcdc_alert_map_null_output();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}