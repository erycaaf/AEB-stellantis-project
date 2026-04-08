/**
 * @file    aeb_uds.h
 * @brief   UDS diagnostics services module — public API.
 * @owner   Task E (Rian Ithalo da Costa Linhares)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Implements ISO 14229 UDS services: ReadDataByIdentifier,
 *          ClearDiagnosticInformation, RoutineControl, and DTC monitoring.
 *
 * @requirements FR-UDS-001 to FR-UDS-005
 * @simulink     chart_117 (UDS_Server)
 */

#ifndef AEB_UDS_H
#define AEB_UDS_H

#include "aeb_types.h"

/**
 * @brief Initialise UDS module.
 *
 * Clears all stored DTCs and resets internal state.
 */
void uds_init(void);

/**
 * @brief Process an incoming UDS diagnostic request.
 *
 * Dispatches based on SID:
 *   SID 0x22: ReadDataByIdentifier (TTC, FSM state, brake pressure)
 *   SID 0x14: ClearDiagnosticInformation (reset all DTCs)
 *   SID 0x31: RoutineControl (enable/disable AEB, routine 0x0301)
 *   Other:    Negative response 0x7F
 *
 * @param[in]  sid       Service Identifier.
 * @param[in]  did_high  DID high byte (for SID 0x22).
 * @param[in]  did_low   DID low byte (for SID 0x22).
 * @param[in]  value     Input value (for SID 0x31 routine parameter).
 * @param[out] response  Pointer to response buffer (8 bytes minimum).
 * @param[out] resp_len  Pointer to response length.
 *
 * @requirement FR-UDS-001 to FR-UDS-005
 */
void uds_process_request(uint8_t sid, uint8_t did_high, uint8_t did_low,
                         uint8_t value, uint8_t *response, uint8_t *resp_len);

/**
 * @brief Monitor fault conditions and update DTC storage.
 *
 * Checks for:
 *   C1001: sensor fault
 *   C1004: CRC error
 *   C1006: actuator fault
 *
 * DTCs are latched until cleared by SID 0x14.
 * Fault lamp activated when dtc_count > 0.
 *
 * @param[in] sensor_fault    Sensor fault flag (from perception).
 * @param[in] crc_error       CRC error flag (from CAN).
 * @param[in] actuator_fault  Actuator fault flag.
 * @return    Number of active DTCs.
 *
 * @requirement FR-UDS-002
 */
uint8_t uds_monitor_faults(uint8_t sensor_fault, uint8_t crc_error,
                           uint8_t actuator_fault);

/**
 * @brief Get the current AEB enable state set via UDS.
 *
 * @return 1 if AEB is enabled, 0 if disabled via RoutineControl.
 *
 * @requirement FR-UDS-004
 */
uint8_t uds_get_aeb_enabled(void);

/**
 * @brief Update live diagnostic data for ReadDataByIdentifier.
 *
 * Called each cycle to refresh the values returned by SID 0x22.
 *
 * @param[in] fsm_out  Pointer to current FSM output.
 * @param[in] ttc_out  Pointer to current TTC output.
 * @param[in] pid_out  Pointer to current PID output.
 */
void uds_update_live_data(const fsm_output_t *fsm_out,
                          const ttc_output_t *ttc_out,
                          const pid_output_t *pid_out);

#endif /* AEB_UDS_H */
