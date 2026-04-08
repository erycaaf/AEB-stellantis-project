/**
 * @file    aeb_fsm.c
 * @brief   FSM module — STUB implementation (Sprint 0).
 * @owner   Task B (Lourenço Jamba Mphili)
 *
 * @todo    Implement fsm_step with full 7-state FSM logic.
 */

#include "aeb_fsm.h"
#include "aeb_config.h"

void fsm_init(void)
{
    /* STUB: no internal state to initialise yet. */
}

void fsm_step(const ttc_output_t *ttc_in,
              const perception_output_t *percep_in,
              const driver_input_t *driver_in,
              fsm_output_t *output)
{
    (void)ttc_in;
    (void)percep_in;
    (void)driver_in;

    /* STUB: safe default — system in STANDBY, no braking. */
    output->fsm_state    = (uint8_t)FSM_STANDBY;
    output->decel_target = 0.0f;
    output->brake_active = 0U;
    output->alert_level  = 0U;
    output->warn_timer   = 0.0f;
    output->state_timer  = 0.0f;
}
