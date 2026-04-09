/**
 * @file aeb_uds.h
 * @brief UDS Diagnostics module API for the AEB system.
 *
 * Implements the UDS server: live data access (SID 0x22), DTC management
 * (SID 0x14), fault monitoring, and AEB enable/disable (SID 0x31).
 *
 * Structural correspondence: Simulink chart_117 (UDS_Server).
 *
 * @req FR-UDS-001 ReadDataByIdentifier (SID 0x22).
 * @req FR-UDS-002 DTC storage (C1001, C1004, C1006).
 * @req FR-UDS-003 ClearDiagnosticInformation (SID 0x14).
 * @req FR-UDS-004 RoutineControl (SID 0x31, routine 0x0301).
 * @req FR-UDS-005 UDS request/response via CAN within one cycle.
 * @req FR-COD-001 Structural correspondence with Simulink model.
 *
 * @version 1.0
 * @date 2026-04
 */

#ifndef AEB_UDS_H
#define AEB_UDS_H

#include <stdint.h>
#include "aeb_types.h"

/* ===================================================================
 * UDS Service Identifiers (ISO 14229)
 * =================================================================== */

#define UDS_SID_READ_DID            ((uint8_t)0x22U)
#define UDS_SID_READ_DID_RESP       ((uint8_t)0x62U)
#define UDS_SID_CLEAR_DTC           ((uint8_t)0x14U)
#define UDS_SID_CLEAR_DTC_RESP      ((uint8_t)0x54U)
#define UDS_SID_ROUTINE_CTRL        ((uint8_t)0x31U)
#define UDS_SID_ROUTINE_CTRL_RESP   ((uint8_t)0x71U)
#define UDS_SID_NEGATIVE_RESP       ((uint8_t)0x7FU)

/* ===================================================================
 * Negative Response Codes (NRC)
 * =================================================================== */

#define UDS_NRC_SERVICE_NOT_SUPP    ((uint8_t)0x11U)
#define UDS_NRC_REQUEST_OOR         ((uint8_t)0x31U)

/* ===================================================================
 * DID Identifiers
 * =================================================================== */

#define UDS_DID_TTC_VAL             ((uint16_t)0xF100U)
#define UDS_DID_FSM_STATE_VAL       ((uint16_t)0xF101U)
#define UDS_DID_BRAKE_PRESS_VAL     ((uint16_t)0xF102U)

/* ===================================================================
 * Routine Identifiers
 * =================================================================== */

#define UDS_RID_AEB_CTRL            ((uint16_t)0x0301U)

/* ===================================================================
 * Data Structures
 * =================================================================== */

/**
 * @brief UDS request — maps to UDS_Request CAN msg (ID 0x7DF / 2015).
 */
typedef struct
{
    uint8_t sid;        /**< Service Identifier                       */
    uint8_t did_high;   /**< DID high byte                            */
    uint8_t did_low;    /**< DID low byte                             */
    uint8_t value;      /**< Value field (RoutineControl)             */
} uds_request_t;

/**
 * @brief UDS response — maps to UDS_Response CAN msg (ID 0x7E8 / 2024).
 */
typedef struct
{
    uint8_t response_sid;   /**< Response SID                         */
    uint8_t did_high_resp;  /**< DID high byte (echo or SID for NRC)  */
    uint8_t did_low_resp;   /**< DID low byte (or NRC code)           */
    uint8_t data1;          /**< Response data byte 1                 */
    uint8_t data2;          /**< Response data byte 2                 */
    uint8_t data3;          /**< Response data byte 3                 */
    uint8_t data4;          /**< Response data byte 4                 */
    uint8_t data5;          /**< Response data byte 5                 */
} uds_response_t;

/**
 * @brief UDS server internal state (persistent variables).
 *
 * Maps to MATLAB Function persistent variables:
 *   aeb_enabled_mem, dtc_sensor_mem, dtc_crc_mem, dtc_actuator_mem.
 */
typedef struct
{
    uint8_t aeb_enabled;    /**< AEB enable flag (persistent)          */
    uint8_t dtc_sensor;     /**< DTC C1001: sensor fault (latched)     */
    uint8_t dtc_crc;        /**< DTC C1004: CRC error (latched)        */
    uint8_t dtc_actuator;   /**< DTC C1006: actuator fault (latched)   */
} uds_state_t;

/**
 * @brief UDS server output signals.
 */
typedef struct
{
    uint8_t aeb_enabled;    /**< Current AEB enable state              */
    uint8_t dtc_count;      /**< Number of active DTCs (0..3)          */
    uint8_t fault_lamp;     /**< MIL lamp: 1 if dtc_count > 0         */
} uds_output_t;

/* ===================================================================
 * Public API
 * =================================================================== */

/**
 * @brief Initialise the UDS server state.
 * @param[out] state  Pointer to UDS state to initialise.
 * @req NFR-SAF-004 Power-on self-test.
 */
void uds_init(uds_state_t *state);

/**
 * @brief Monitor fault inputs and latch DTCs (called every 10 ms).
 *
 * DTCs stored immediately on first active fault (DD-UDS-01, no debounce).
 *
 * @param[in,out] state          UDS server state.
 * @param[in]     sensor_fault   Sensor fault flag (0/1).
 * @param[in]     crc_error      CRC error flag (0/1).
 * @param[in]     actuator_fault Actuator fault flag (0/1).
 * @req FR-UDS-002
 */
void uds_monitor_faults(uds_state_t *state,
                        uint8_t sensor_fault,
                        uint8_t crc_error,
                        uint8_t actuator_fault);

/**
 * @brief Process a UDS request and generate the response.
 *
 * @param[in,out] state    UDS server state.
 * @param[in]     request  Incoming UDS request.
 * @param[out]    response UDS response to populate.
 * @param[in]     fsm_out  Current FSM output (live data).
 * @param[in]     pid_out  Current PID output (live data).
 * @param[in]     ttc_out  Current TTC output (live data).
 * @req FR-UDS-001, FR-UDS-003, FR-UDS-004, FR-UDS-005
 */
void uds_process_request(uds_state_t *state,
                         const uds_request_t *request,
                         uds_response_t *response,
                         const fsm_output_t *fsm_out,
                         const pid_output_t *pid_out,
                         const ttc_output_t *ttc_out);

/**
 * @brief Get current UDS output signals.
 * @param[in]  state  UDS server state.
 * @param[out] output Output structure to populate.
 */
void uds_get_output(const uds_state_t *state, uds_output_t *output);

#endif /* AEB_UDS_H */
