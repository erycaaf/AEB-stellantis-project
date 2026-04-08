/**
 * @file    aeb_can.h
 * @brief   CAN bus communication module — public API.
 * @owner   Task D (Renato Silva Fagundes)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Handles CAN frame transmission, reception, encoding/decoding,
 *          and timeout detection for the AEB system.
 *
 * @requirements FR-CAN-001 to FR-CAN-004
 * @simulink     system_177 (CAN_Bus subsystem)
 */

#ifndef AEB_CAN_H
#define AEB_CAN_H

#include "aeb_types.h"

/**
 * @brief Initialise CAN driver and register RX filters.
 *
 * Configures Zephyr CAN peripheral at CAN_BAUD_RATE (500 kbit/s),
 * registers RX filters for radar target messages, and sets up TX work queue.
 *
 * @return 0 on success, negative error code on failure.
 *
 * @requirement FR-CAN-004
 */
int32_t can_init(void);

/**
 * @brief Transmit ego vehicle dynamics CAN frame.
 *
 * Encodes and transmits ego speed, FSM state, and brake command
 * every CAN_TX_PERIOD_MS (50 ms).
 *
 * @param[in] fsm_out  Pointer to current FSM output.
 * @param[in] pid_out  Pointer to current PID output.
 *
 * @requirement FR-CAN-001
 */
void can_tx_ego_dynamics(const fsm_output_t *fsm_out,
                         const pid_output_t *pid_out);

/**
 * @brief Process received radar CAN frame (ISR-safe callback).
 *
 * Decodes relative distance and relative speed from incoming
 * radar frame and pushes to ring buffer.
 *
 * @param[in] frame_data  Pointer to raw CAN frame data (8 bytes).
 * @param[in] dlc         Data Length Code of received frame.
 *
 * @requirement FR-CAN-002, FR-CAN-003
 */
void can_rx_radar_callback(const uint8_t *frame_data, uint8_t dlc);

/**
 * @brief Check for CAN RX timeout.
 *
 * If 3 consecutive radar frames are missed (>= CAN_RX_TIMEOUT_MS),
 * sets timeout fault flag.
 *
 * @return 1 if timeout detected, 0 otherwise.
 *
 * @requirement FR-CAN-002 (implicit timeout)
 */
uint8_t can_check_timeout(void);

/**
 * @brief Encode a signal into CAN frame data per DBC layout.
 *
 * @param[out] frame_data   Pointer to 8-byte frame buffer.
 * @param[in]  raw_value    Signal value to encode.
 * @param[in]  start_bit    Start bit position in frame.
 * @param[in]  length       Signal bit length.
 * @param[in]  scale        Signal scaling factor.
 * @param[in]  offset       Signal offset.
 *
 * @requirement FR-CAN-003
 */
void can_encode_signal(uint8_t *frame_data, float32_t raw_value,
                       uint8_t start_bit, uint8_t length,
                       float32_t scale, float32_t offset);

/**
 * @brief Decode a signal from CAN frame data per DBC layout.
 *
 * @param[in]  frame_data  Pointer to 8-byte frame buffer.
 * @param[in]  start_bit   Start bit position in frame.
 * @param[in]  length      Signal bit length.
 * @param[in]  scale       Signal scaling factor.
 * @param[in]  offset      Signal offset.
 * @return     Decoded physical signal value.
 *
 * @requirement FR-CAN-003
 */
float32_t can_decode_signal(const uint8_t *frame_data,
                            uint8_t start_bit, uint8_t length,
                            float32_t scale, float32_t offset);

#endif /* AEB_CAN_H */
