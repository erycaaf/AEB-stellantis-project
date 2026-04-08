/**
 * @file    aeb_alert.c
 * @brief   Alert mapping and override detection — STUB implementation (Sprint 0).
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 *
 * @todo    Implement alert_map and override_detect with full logic.
 */

#include "aeb_alert.h"
#include "aeb_config.h"

void alert_map(uint8_t fsm_state, alert_output_t *output)
{
    (void)fsm_state;

    /* STUB: safe default — no alert. */
    output->alert_type   = (uint8_t)ALERT_NONE;
    output->alert_active = 0U;
    output->buzzer_cmd   = (uint8_t)BUZZER_SILENT;
}

uint8_t override_detect(float32_t steering_angle, uint8_t brake_pedal)
{
    (void)steering_angle;
    (void)brake_pedal;

    /* STUB: no override detected. */
    return 0U;
}
