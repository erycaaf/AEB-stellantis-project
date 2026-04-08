/**
 * @file    aeb_pid.c
 * @brief   PI brake controller — STUB implementation (Sprint 0).
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 *
 * @todo    Implement pid_brake_step with PI control, anti-windup,
 *          and jerk limiting.
 */

#include "aeb_pid.h"
#include "aeb_config.h"

void pid_init(void)
{
    /* STUB: no internal state to initialise yet. */
}

void pid_brake_step(float32_t decel_target, float32_t a_ego,
                    uint8_t fsm_state, pid_output_t *output)
{
    (void)decel_target;
    (void)a_ego;
    (void)fsm_state;

    /* STUB: safe default — no braking. */
    output->brake_pct = 0.0f;
    output->brake_bar = 0.0f;
}
