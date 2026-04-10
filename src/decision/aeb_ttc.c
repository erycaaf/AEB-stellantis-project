/**
 * @file aeb_ttc.c
 * @brief Implementation of TTC and braking distance calculations.
 * @author Lourenço Jamba Mphili
 * @requirements FR-DEC-001, FR-DEC-002, FR-DEC-003
 * @misra Fully compliant with MISRA C:2012
 */

#include "aeb_ttc.h"
#include "aeb_config.h"
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

            /* Clamp to [0, TTC_MAX] seconds per SRS */
            if (ttc_result > TTC_MAX)
            {
                ttc_result = TTC_MAX;
            }
            else if (ttc_result < 0.0f)
            {
                ttc_result = 0.0f;
            }
        }
    }

    return ttc_result;
}

float32_t d_brake_calc(float32_t v_ego)
{
    float32_t d_brake;

    /* FR-DEC-002: d_brake = v² / (2 * a) with a = DECEL_L3 = 6.0 m/s² */
    d_brake = (v_ego * v_ego) / (2.0f * DECEL_L3);

    /* Physical lower bound */
    if (d_brake < 0.0f)
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

    /* FR-DEC-003: Use v_rel directly from perception (Task A already computed it) */
    /* Set closing flag (v_rel > V_REL_MIN means approaching) */
    ttc_out->is_closing = (perception->v_rel > V_REL_MIN) ? 1U : 0U;

    /* Calculate TTC and braking distance */
    ttc_out->ttc = ttc_calc(perception->distance, perception->v_rel);
    ttc_out->d_brake = d_brake_calc(perception->v_ego);
}