/**
 * @file  aeb_can.h
 * @brief CAN Bus Communication module — public API.
 *
 * Implements FR-CAN-001..004 from the AEB SRS v2.0.
 * Designed for Zephyr RTOS CAN driver; MISRA C:2012 compliant.
 *
 * @author  Renato Fagundes
 * @date    2026-04-07
 */

#ifndef AEB_CAN_H
#define AEB_CAN_H

#include "aeb_types.h"
#include "aeb_config.h"
#include <stdint.h>

/* ── CAN Message IDs (from aeb_system.dbc) ───────────────────────────── */
#define CAN_ID_BRAKE_CMD      (0x080U)   /**< 128 — AEB_BrakeCmd  TX     */
#define CAN_ID_EGO_VEHICLE    (0x100U)   /**< 256 — AEB_EgoVehicle RX    */
#define CAN_ID_DRIVER_INPUT   (0x101U)   /**< 257 — AEB_DriverInput RX   */
#define CAN_ID_RADAR_TARGET   (0x120U)   /**< 288 — AEB_RadarTarget RX   */
#define CAN_ID_FSM_STATE      (0x200U)   /**< 512 — AEB_FSMState TX      */
#define CAN_ID_ALERT          (0x300U)   /**< 768 — AEB_Alert TX         */

/* ── CAN Frame Lengths (bytes) ───────────────────────────────────────── */
#define CAN_DLC_BRAKE_CMD     (4U)
#define CAN_DLC_EGO_VEHICLE   (8U)
#define CAN_DLC_DRIVER_INPUT  (4U)
#define CAN_DLC_RADAR_TARGET  (8U)
#define CAN_DLC_FSM_STATE     (4U)
#define CAN_DLC_ALERT         (2U)

/* ── RX Timeout ──────────────────────────────────────────────────────── */
#define CAN_RX_TIMEOUT_CYCLES (3U)  /**< 3 missed frames → fault        */

/* ── Alive counter ───────────────────────────────────────────────────── */
#define ALIVE_COUNTER_MAX     (15U) /**< 4-bit rolling counter 0..15     */

/* ── Return codes ────────────────────────────────────────────────────── */
#define CAN_OK                (0)
#define CAN_ERR_INIT          (-1)
#define CAN_ERR_TX            (-2)
#define CAN_ERR_TIMEOUT       (-3)

/**
 * @brief Raw sensor data decoded from CAN RX frames.
 *
 * Populated by can_rx_process() from AEB_EgoVehicle (0x100) and
 * AEB_RadarTarget (0x120).  Consumed by the Perception module.
 */
typedef struct
{
    /* From AEB_RadarTarget (0x120) */
    float32_t target_distance;   /**< [m]                                */
    float32_t relative_speed;    /**< [m/s]                              */
    float32_t ttc_radar;         /**< [s]   (info only)                  */
    uint8_t   confidence_raw;    /**< 0..15                              */

    /* From AEB_EgoVehicle (0x100) */
    float32_t vehicle_speed;     /**< [m/s]                              */
    float32_t long_accel;        /**< [m/s^2]                            */
    float32_t yaw_rate;          /**< [deg/s]                            */
    float32_t steering_angle;    /**< [deg]                              */

    /* From AEB_DriverInput (0x101) */
    uint8_t   brake_pedal;       /**< [%] 0..100                         */
    uint8_t   accel_pedal;       /**< [%] 0..100                         */
    uint8_t   aeb_enable;        /**< 0/1                                */
    uint8_t   driver_override;   /**< 0/1                                */

    /* Status */
    uint8_t   rx_timeout_flag;   /**< 1 if radar RX timed out            */
} can_rx_data_t;

/**
 * @brief Internal state of the CAN module (opaque to other modules).
 */
typedef struct
{
    uint8_t   alive_counter;       /**< 4-bit TX rolling counter         */
    uint8_t   rx_miss_count;       /**< Consecutive missed RX frames     */
    uint32_t  tx_cycle_counter;    /**< Counts 10 ms ticks for 50 ms TX  */
    uint8_t   initialised;         /**< 1 after successful can_init()    */
    can_rx_data_t  last_rx;        /**< Most recent decoded RX data      */
} can_state_t;

/* ═══════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise CAN peripheral at 500 kbit/s and register RX filters.
 *
 * @param[out] state  Module state, zeroed before first call.
 * @return CAN_OK on success, CAN_ERR_INIT on failure.
 *
 * @req FR-CAN-004  Baud rate 500 kbit/s.
 */
int32_t can_init(can_state_t *state);

/**
 * @brief Process a single received CAN frame (called from RX callback).
 *
 * Decodes EgoVehicle (0x100) or RadarTarget (0x120) per DBC layout.
 * Resets the RX miss counter on valid frame.
 *
 * @param[in,out] state  Module state.
 * @param[in]     id     CAN message ID.
 * @param[in]     data   Pointer to frame payload (up to 8 bytes).
 * @param[in]     dlc    Data Length Code.
 *
 * @req FR-CAN-002  Receive radar target data.
 * @req FR-CAN-003  DBC signal encoding.
 */
void can_rx_process(can_state_t *state,
                    uint32_t     id,
                    const uint8_t *data,
                    uint8_t      dlc);

/**
 * @brief Check RX timeout — call once per 10 ms cycle.
 *
 * Increments miss counter; sets rx_timeout_flag after 3 consecutive
 * misses (≥ 60 ms with 20 ms radar period).
 *
 * @param[in,out] state  Module state.
 *
 * @req FR-CAN-002  Timeout detection (acceptance: 3 × 20 ms = 60 ms).
 */
void can_check_timeout(can_state_t *state);

/**
 * @brief Transmit AEB_BrakeCmd (0x080) — every 10 ms cycle.
 *
 * @param[in,out] state    Module state (alive counter incremented).
 * @param[in]     pid_out  PID brake output from the PID module.
 * @param[in]     fsm_out  FSM output from the FSM module.
 * @return CAN_OK on success, CAN_ERR_TX on failure.
 *
 * @req FR-CAN-003  DBC signal encoding.
 */
int32_t can_tx_brake_cmd(can_state_t       *state,
                         const pid_output_t *pid_out,
                         const fsm_output_t *fsm_out);

/**
 * @brief Transmit AEB_FSMState (0x200) — every 50 ms (5 × 10 ms ticks).
 *
 * @param[in,out] state    Module state (cycle counter checked).
 * @param[in]     fsm_out  FSM output from the FSM module.
 * @return CAN_OK if transmitted, 1 if not yet due, CAN_ERR_TX on failure.
 *
 * @req FR-CAN-001  Transmit ego dynamics at fixed cycle.
 */
int32_t can_tx_fsm_state(can_state_t       *state,
                         const fsm_output_t *fsm_out);

/**
 * @brief Transmit AEB_Alert (0x300) — event-driven, call when alert changes.
 *
 * @param[in] alert_out  Alert output from the Alert module.
 * @return CAN_OK on success, CAN_ERR_TX on failure.
 *
 * @req FR-CAN-003  DBC signal encoding.
 */
int32_t can_tx_alert(const alert_output_t *alert_out);

/**
 * @brief Get a copy of the latest decoded RX data.
 *
 * @param[in]  state  Module state.
 * @param[out] out    Destination struct.
 */
void can_get_rx_data(const can_state_t *state,
                     can_rx_data_t     *out);

/* ── Signal encode/decode helpers (FR-CAN-003) ───────────────────────── */

/**
 * @brief Pack an unsigned integer signal into a CAN frame payload.
 *
 * @param[out] data       Frame payload buffer.
 * @param[in]  start_bit  LSB bit position (Intel / little-endian).
 * @param[in]  length     Signal length in bits (1..32).
 * @param[in]  raw_value  Unsigned raw value to pack.
 */
void can_pack_signal(uint8_t *data,
                     uint8_t  start_bit,
                     uint8_t  length,
                     uint32_t raw_value);

/**
 * @brief Unpack an unsigned integer signal from a CAN frame payload.
 *
 * @param[in]  data       Frame payload buffer.
 * @param[in]  start_bit  LSB bit position (Intel / little-endian).
 * @param[in]  length     Signal length in bits (1..32).
 * @return Unsigned raw value.
 */
uint32_t can_unpack_signal(const uint8_t *data,
                           uint8_t        start_bit,
                           uint8_t        length);

#endif /* AEB_CAN_H */
