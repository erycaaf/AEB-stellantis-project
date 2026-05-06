/**
 * @file aeb_ttc.c
 * @brief Implementation of TTC and braking distance calculations.
 * @author Lourenço Jamba Mphili
 * @requirements FR-DEC-001, FR-DEC-002, FR-DEC-003
 * @misra Fully compliant with MISRA C:2012
 */

#include "aeb_ttc.h"
#include "aeb_config.h"
#include <math.h>    /* isfinite() — Bug #1 guard in d_brake_calc */
#include <stddef.h>

float32_t ttc_calc(float32_t distance, float32_t v_rel)
{
    float32_t ttc_result = TTC_MAX;

    /* Safety check for invalid distance */
    if (distance > 0.0f)
    {
        /* FR-DEC-001: Only calculate when approaching (v_rel > V_REL_MIN) */
        if (v_rel > V_REL_MIN)
        {
            ttc_result = distance / v_rel;

            /* Clamp to [0, TTC_MAX] seconds per SRS.
             * D1: only the upper bound is reachable — the guard above
             * ensures distance > 0 AND v_rel > V_REL_MIN > 0, so the
             * quotient is strictly positive. The defensive
             * `else if (ttc_result < 0.0f)` branch was unreachable
             * (MISRA C:2012 Rule 2.1) and was removed as catalogued in
             * the V&V report Sec. 7.5 desvio D1. */
            if (ttc_result > TTC_MAX)
            {
                ttc_result = TTC_MAX;
            }
        }
    }

    return ttc_result;
}

float32_t d_brake_calc(float32_t v_ego)
{
    float32_t d_brake;

    /* Bug #1 fail-safe gate: reject non-finite input (NaN, +Inf, -Inf).
     * IEEE 754 comparisons with NaN are false, so `d_brake < 0.0f` below
     * cannot be relied on to catch this. */
    if (!isfinite(v_ego))
    {
        return 0.0f;
    }

    /* FR-DEC-002: d_brake = v² / (2 * a) with a = DECEL_L3 = 6.0 m/s² */
    d_brake = (v_ego * v_ego) / (2.0f * DECEL_L3);

    /* Overflow guard: a very large but finite v_ego (e.g. FLT_MAX)
     * squares to +Inf, which would slip past the isfinite() gate above.
     * The original `d_brake < 0.0f` clamp that used to be here is
     * unreachable (IEEE 754: v*v >= 0 for all finite v, including
     * subnormals) and was removed per V&V report Sec. 7.5 desvio D2
     * (MISRA C:2012 Rule 2.1). */
    if (!isfinite(d_brake))
    {
        d_brake = 0.0f;
    }

    return d_brake;
}

void ttc_process(const perception_output_t * const perception,
                 ttc_output_t * const ttc_out)
{
    /* Defensive checks */
    if ((perception == NULL) || (ttc_out == NULL))
    {
        return;
    }

    /* FR-DEC-003: Use v_rel directly from perception (the Perception module already computed it) */
    /* Set closing flag (v_rel > V_REL_MIN means approaching) */
    ttc_out->is_closing = (perception->v_rel > V_REL_MIN) ? 1U : 0U;

    /* Calculate TTC and braking distance */
    ttc_out->ttc = ttc_calc(perception->distance, perception->v_rel);
    ttc_out->d_brake = d_brake_calc(perception->v_ego);
}