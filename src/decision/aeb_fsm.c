/**
 * @file  aeb_fsm.c
 * @brief Finite State Machine — STUB (Task B placeholder).
 *
 * This file will be replaced by Lourenco's implementation.
 */

#include "aeb_types.h"

void fsm_init(fsm_output_t *out)
{
    if (out != (void *)0)
    {
        out->fsm_state    = (uint8_t)FSM_OFF;
        out->decel_target = 0.0F;
        out->brake_active = 0U;
        out->alert_level  = 0U;
        out->warn_timer   = 0.0F;
        out->state_timer  = 0.0F;
    }
}

void fsm_step(const perception_output_t *perc,
              const driver_input_t *driver,
              fsm_output_t *out)
{
    (void)perc;
    (void)driver;
    (void)out;
    /* STUB — Task B (Lourenco) */
}
