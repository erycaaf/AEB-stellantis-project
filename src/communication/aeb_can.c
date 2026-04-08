/**
 * @file    aeb_can.c
 * @brief   CAN bus communication — STUB implementation (Sprint 0).
 * @owner   Task D (Renato Silva Fagundes)
 *
 * @todo    Implement CAN init, TX, RX, timeout, encode/decode.
 */

#include "aeb_can.h"
#include "aeb_config.h"

int32_t can_init(void)
{
    /* STUB: success. */
    return 0;
}

void can_tx_ego_dynamics(const fsm_output_t *fsm_out,
                         const pid_output_t *pid_out)
{
    (void)fsm_out;
    (void)pid_out;
    /* STUB: no transmission. */
}

void can_rx_radar_callback(const uint8_t *frame_data, uint8_t dlc)
{
    (void)frame_data;
    (void)dlc;
    /* STUB: no processing. */
}

uint8_t can_check_timeout(void)
{
    /* STUB: no timeout. */
    return 0U;
}

void can_encode_signal(uint8_t *frame_data, float32_t raw_value,
                       uint8_t start_bit, uint8_t length,
                       float32_t scale, float32_t offset)
{
    (void)frame_data;
    (void)raw_value;
    (void)start_bit;
    (void)length;
    (void)scale;
    (void)offset;
    /* STUB: no encoding. */
}

float32_t can_decode_signal(const uint8_t *frame_data,
                            uint8_t start_bit, uint8_t length,
                            float32_t scale, float32_t offset)
{
    (void)frame_data;
    (void)start_bit;
    (void)length;
    (void)scale;
    (void)offset;
    /* STUB: return zero. */
    return 0.0f;
}
