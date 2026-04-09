/**
 * @file  aeb_ttc.c
 * @brief TTC Calculation — STUB (Task B placeholder).
 *
 * This file will be replaced by Lourenco's implementation.
 */

#include "aeb_types.h"

void ttc_calc(const float32_t distance,
              const float32_t v_ego,
              const float32_t v_target,
              ttc_output_t *out)
{
    (void)distance;
    (void)v_ego;
    (void)v_target;

    if (out != (void *)0)
    {
        out->ttc        = 10.0F;
        out->d_brake    = 0.0F;
        out->is_closing = 0U;
    }
}
