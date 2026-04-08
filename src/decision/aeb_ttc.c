/**
 * @file    aeb_ttc.c
 * @brief   TTC calculation module — STUB implementation (Sprint 0).
 * @owner   Task B (Lourenço Jamba Mphili)
 *
 * @todo    Implement ttc_calc with full TTC and d_brake logic.
 */

#include "aeb_ttc.h"
#include "aeb_config.h"

void ttc_calc(float32_t distance, float32_t v_ego,
              float32_t v_target, ttc_output_t *output)
{
    (void)distance;
    (void)v_ego;
    (void)v_target;

    /* STUB: safe defaults — no threat. */
    output->ttc        = TTC_MAX;
    output->d_brake    = 0.0f;
    output->is_closing = 0U;
}
