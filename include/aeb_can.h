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
#include "aeb_uds.h"
#include <stdint.h>

/* ── CAN Message IDs (from aeb_system.dbc) ───────────────────────────── */
#define CAN_ID_BRAKE_CMD      (0x080U)   /**< 128  — AEB_BrakeCmd   TX    */
#define CAN_ID_EGO_VEHICLE    (0x100U)   /**< 256  — AEB_EgoVehicle RX    */
#define CAN_ID_DRIVER_INPUT   (0x101U)   /**< 257  — AEB_DriverInput RX   */
#define CAN_ID_RADAR_TARGET   (0x120U)   /**< 288  — AEB_RadarTarget RX   */
#define CAN_ID_FSM_STATE      (0x200U)   /**< 512  — AEB_FSMState   TX    */
#define CAN_ID_ALERT          (0x300U)   /**< 768  — AEB_Alert      TX    */
#define CAN_ID_UDS_REQUEST    (0x7DFU)   /**< 2015 — UDS_Request    RX    */
#define CAN_ID_UDS_RESPONSE   (0x7E8U)   /**< 2024 — UDS_Response   TX    */

/* ── CAN Frame Lengths (bytes) ───────────────────────────────────────── */
#define CAN_DLC_BRAKE_CMD     (4U)
#define CAN_DLC_EGO_VEHICLE   (8U)
#define CAN_DLC_DRIVER_INPUT  (4U)
#define CAN_DLC_RADAR_TARGET  (8U)
#define CAN_DLC_FSM_STATE     (4U)
#define CAN_DLC_ALERT         (2U)
#define CAN_DLC_UDS_REQUEST   (4U)
#define CAN_DLC_UDS_RESPONSE  (8U)

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

    /* From UDS_Request (0x7DF) — FR-UDS-005
     *
     * Concurrency contract (IMPORTANT on Zephyr target):
     *   can_rx_process()           writes uds_request_pending = 1 from the
     *                              CAN RX callback context.
     *   aeb_core_step()            reads-and-clears it from the 10 ms cycle
     *                              thread.
     *
     *   On the host stub the two are called sequentially from the same
     *   thread, so there is no race.  On the real Zephyr driver the RX
     *   callback runs in a separate context; the driver integration layer
     *   MUST serialise access — either by marshalling RX frames into a
     *   message queue consumed by the core tick, by protecting this flag
     *   with an atomic operation / brief interrupt disable around the
     *   read-and-clear pair, or equivalent.
     *
     *   Without such synchronisation, a request arriving between the
     *   read of uds_request_pending and the call to
     *   can_clear_uds_request_pending() is silently dropped.  This is
     *   not catchable at unit-test level because the host stub never
     *   exercises the race — hence the documentation here. */
    uds_request_t uds_request;         /**< Decoded UDS request frame    */
    uint8_t       uds_request_pending; /**< 1 when a new request arrived;
                                            cleared by can_clear_uds_request_pending()
                                            after the response transmits. */

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
    uint32_t  tx_cycle_counter;    /**< Counts 10 ms ticks for FSM TX    */
    uint32_t  tx_ego_counter;      /**< Counts 10 ms ticks for ego TX    */
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
 * Decodes EgoVehicle (0x100), RadarTarget (0x120), DriverInput (0x101)
 * or UDS_Request (0x7DF) per DBC layout.  Resets the RX miss counter
 * on valid radar frame.  A valid 0x7DF frame sets uds_request_pending.
 *
 * @param[in,out] state  Module state.
 * @param[in]     id     CAN message ID.
 * @param[in]     data   Pointer to frame payload (up to 8 bytes).
 * @param[in]     dlc    Data Length Code.
 *
 * @req FR-CAN-002  Receive radar target data.
 * @req FR-CAN-003  DBC signal encoding.
 * @req FR-UDS-005  Receive UDS request on 0x7DF.
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
 * @brief Transmit AEB_EgoVehicle (0x100) every 50 ms.
 *
 * Broadcasts ego dynamics (processed speed from Perception, plus
 * longitudinal acceleration, yaw rate and steering angle received
 * from the vehicle bus) so that other nodes can consume them.
 *
 * DBC layout — AEB_EgoVehicle (0x100), DLC 8:
 *   VehicleSpeed   bits  0..15  factor 0.01   offset    0
 *   LongAccel      bits 16..31  factor 0.001  offset  -32
 *   YawRate        bits 32..47  factor 0.01   offset -327.68
 *   SteeringAngle  bits 48..63  factor 0.1    offset -3276.8
 *
 * @param[in,out] state  Module state (ego TX counter incremented).
 * @param[in]     perc   Perception output — provides processed v_ego.
 * @return CAN_OK if transmitted, 1 if not yet due, CAN_ERR_TX on failure.
 *
 * @req FR-CAN-001  Transmit ego dynamics at 50 ms cadence.
 */
int32_t can_tx_ego_vehicle(can_state_t               *state,
                           const perception_output_t *perc);

/**
 * @brief Get a copy of the latest decoded RX data.
 *
 * @param[in]  state  Module state.
 * @param[out] out    Destination struct.
 */
void can_get_rx_data(const can_state_t *state,
                     can_rx_data_t     *out);

/**
 * @brief Transmit a UDS response frame on CAN_ID_UDS_RESPONSE (0x7E8).
 *
 * Packs the 8-byte uds_response_t into a CAN frame and sends via HAL.
 * Called from aeb_core_step() after uds_process_request() produces a
 * response for a pending 0x7DF request.
 *
 * @param[in] resp  UDS response to transmit.
 * @return CAN_OK on success, CAN_ERR_TX on failure.
 *
 * @req FR-UDS-005  UDS request/response via CAN within one cycle.
 */
int32_t can_tx_uds_response(const uds_response_t *resp);

/**
 * @brief Mark the current UDS request as serviced and clear the pending flag.
 *
 * The caller (aeb_core_step) is responsible for deciding *when* to call
 * this — typically only after can_tx_uds_response() has confirmed the
 * response transmitted successfully. Clearing the flag on TX failure
 * silently drops the response and forces the requester into a P2_server
 * timeout (ISO 14229-2 §6.3) instead of a best-effort retry at the ECU
 * level. See aeb_core.c step 10 for the retry policy.
 *
 * Must be called exactly once per successfully-serviced request to
 * prevent double-processing on the next cycle.
 *
 * @param[in,out] state  Module state.
 */
void can_clear_uds_request_pending(can_state_t *state);

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
