/**
 * @file  aeb_alert.c
 * @brief Alert Mapping — STUB (Task C placeholder).
 *
 * This file will be replaced by Jessica's implementation.
 */

#include "aeb_types.h"

void alert_map(const uint8_t fsm_state, alert_output_t *out)
{
    (void)fsm_state;

    if (out != (void *)0)
    {
        out->alert_type   = 0U;
        out->alert_active = 0U;
        out->buzzer_cmd   = 0U;
    }
}
