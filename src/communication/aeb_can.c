/**
 * @file  aeb_can.c
 * @brief CAN Bus Communication — STUB (Task D placeholder).
 *
 * This file will be replaced by Renato's implementation.
 * Provides empty function bodies so the project compiles and CI passes.
 */

#include "aeb_can.h"
#include "can_hal.h"
#include <string.h>

void can_pack_signal(uint8_t *data,
                     uint8_t  start_bit,
                     uint8_t  length,
                     uint32_t raw_value)
{
    (void)data;
    (void)start_bit;
    (void)length;
    (void)raw_value;
}

uint32_t can_unpack_signal(const uint8_t *data,
                           uint8_t        start_bit,
                           uint8_t        length)
{
    (void)data;
    (void)start_bit;
    (void)length;
    return 0U;
}

int32_t can_init(can_state_t *state)
{
    if (state != (void *)0)
    {
        (void)memset(state, 0, sizeof(can_state_t));
    }
    return CAN_OK;
}

void can_rx_process(can_state_t *state,
                    uint32_t id,
                    const uint8_t *data,
                    uint8_t dlc)
{
    (void)state;
    (void)id;
    (void)data;
    (void)dlc;
}

void can_check_timeout(can_state_t *state)
{
    (void)state;
}

int32_t can_tx_brake_cmd(can_state_t *state,
                         const pid_output_t *pid_out,
                         const fsm_output_t *fsm_out)
{
    (void)state;
    (void)pid_out;
    (void)fsm_out;
    return CAN_OK;
}

int32_t can_tx_fsm_state(can_state_t *state,
                         const fsm_output_t *fsm_out)
{
    (void)state;
    (void)fsm_out;
    return 1;
}

int32_t can_tx_alert(const alert_output_t *alert_out)
{
    (void)alert_out;
    return CAN_OK;
}

void can_get_rx_data(const can_state_t *state,
                     can_rx_data_t *out)
{
    if ((state != (void *)0) && (out != (void *)0))
    {
        (void)memcpy(out, &state->last_rx, sizeof(can_rx_data_t));
    }
}
