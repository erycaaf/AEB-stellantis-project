/**
 * @file  aeb_pid.c
 * @brief PID Brake Controller — STUB (Task C placeholder).
 *
 * This file will be replaced by Jessica's implementation.
 */

#include "aeb_types.h"

void pid_init(void)
{
    /* STUB — Task C (Jessica) */
}

void pid_brake_step(const float32_t decel_target,
                    const float32_t a_ego,
                    const uint8_t   fsm_state,
                    pid_output_t *out)
{
    (void)decel_target;
    (void)a_ego;
    (void)fsm_state;

    if (out != (void *)0)
    {
        out->brake_pct = 0.0F;
        out->brake_bar = 0.0F;
    }
}
