/**
 * @file aeb_uds.c
 * @brief UDS Diagnostics server — C translation of Simulink UDS_Server.
 *
 * Direct translation of the MATLAB Function block (chart_117):
 *   - Persistent variables  ->  uds_state_t struct members.
 *   - Immediate DTC latch (DD-UDS-01).
 *   - Scaled integer encoding for DIDs (DD-UDS-03 encoding).
 *   - Persistent aeb_enabled across cycles (DD-UDS-02).
 *   - Negative response 0x7F for unsupported SID/DID (DD-UDS-03).
 *
 * @req FR-UDS-001..005, FR-COD-001, FR-COD-003
 *
 * @version 1.0
 * @date 2026-04
 */

#include "aeb_uds.h"
#include "aeb_config.h"
#include <math.h>   /* isfinite() — Bug #1 defensive clamping */

/* ===================================================================
 * Static helpers
 * =================================================================== */

/**
 * @brief Zero-initialise a UDS response.
 */
static void uds_clear_response(uds_response_t *response)
{
    response->response_sid  = 0U;
    response->did_high_resp = 0U;
    response->did_low_resp  = 0U;
    response->data1         = 0U;
    response->data2         = 0U;
    response->data3         = 0U;
    response->data4         = 0U;
    response->data5         = 0U;
}

/**
 * @brief Build a negative response (SID 0x7F).
 */
static void uds_send_negative(uds_response_t *response,
                              uint8_t original_sid,
                              uint8_t nrc)
{
    response->response_sid  = UDS_SID_NEGATIVE_RESP;
    response->did_high_resp = original_sid;
    response->did_low_resp  = nrc;
}

/* ===================================================================
 * SID 0x22 — ReadDataByIdentifier
 * =================================================================== */

/**
 * @brief Handle SID 0x22.
 *
 * DIDs supported:
 *   0xF100 — TTC  (scaled x100, little-endian uint16).
 *   0xF101 — FSM state (raw uint8 in data1).
 *   0xF102 — Brake pressure (scaled x10, little-endian uint16).
 *
 * @req FR-UDS-001
 */
static void uds_handle_read_did(const uds_request_t *request,
                                uds_response_t *response,
                                const fsm_output_t *fsm_out,
                                const pid_output_t *pid_out,
                                const ttc_output_t *ttc_out)
{
    uint16_t requested_id;
    uint16_t scaled_val = 0U;

    requested_id = ((uint16_t)request->did_high * 256U)
                 + (uint16_t)request->did_low;

    switch (requested_id)
    {
        case UDS_DID_TTC_VAL:           /* 0xF100 */
        {
            /* Bug #1 fix: validate float before cast to uint16_t.
             * C11 §6.3.1.4: float->int cast is UB if out of range.
             * Guards NaN, +/-Inf, negatives, and values exceeding
             * UINT16_MAX after the x100 scaling (i.e. > 655.35). */
            if (!isfinite(ttc_out->ttc) ||
                (ttc_out->ttc < 0.0f)   ||
                (ttc_out->ttc > 655.35f))
            {
                uds_send_negative(response,
                                  UDS_SID_READ_DID,
                                  UDS_NRC_REQUEST_OOR);
                break;
            }

            scaled_val = (uint16_t)(ttc_out->ttc * 100.0f);

            response->response_sid  = UDS_SID_READ_DID_RESP;
            response->did_high_resp = request->did_high;
            response->did_low_resp  = request->did_low;
            response->data1         = (uint8_t)(scaled_val & 0x00FFU);
            response->data2         = (uint8_t)((scaled_val >> 8U) & 0x00FFU);
            break;
        }

        case UDS_DID_FSM_STATE_VAL:     /* 0xF101 */
        {
            response->response_sid  = UDS_SID_READ_DID_RESP;
            response->did_high_resp = request->did_high;
            response->did_low_resp  = request->did_low;
            response->data1         = fsm_out->fsm_state;
            break;
        }

        case UDS_DID_BRAKE_PRESS_VAL:   /* 0xF102 */
        {
            /* Bug #1 fix: validate float before cast to uint16_t.
             * Guards NaN, +/-Inf, negatives, and values exceeding
             * UINT16_MAX after the x10 scaling (i.e. > 6553.5). */
            if (!isfinite(pid_out->brake_pct) ||
                (pid_out->brake_pct < 0.0f)   ||
                (pid_out->brake_pct > 6553.5f))
            {
                uds_send_negative(response,
                                  UDS_SID_READ_DID,
                                  UDS_NRC_REQUEST_OOR);
                break;
            }

            scaled_val = (uint16_t)(pid_out->brake_pct * 10.0f);

            response->response_sid  = UDS_SID_READ_DID_RESP;
            response->did_high_resp = request->did_high;
            response->did_low_resp  = request->did_low;
            response->data1         = (uint8_t)(scaled_val & 0x00FFU);
            response->data2         = (uint8_t)((scaled_val >> 8U) & 0x00FFU);
            break;
        }

        default:    /* DID not supported — NRC 0x31 */
        {
            uds_send_negative(response,
                              UDS_SID_READ_DID,
                              UDS_NRC_REQUEST_OOR);
            break;
        }
    }
}

/* ===================================================================
 * SID 0x14 — ClearDiagnosticInformation
 * =================================================================== */

/**
 * @brief Handle SID 0x14 — reset all stored DTCs.
 * @req FR-UDS-003
 */
static void uds_handle_clear_dtc(uds_state_t *state,
                                 uds_response_t *response)
{
    state->dtc_sensor   = 0U;
    state->dtc_crc      = 0U;
    state->dtc_actuator = 0U;

    response->response_sid = UDS_SID_CLEAR_DTC_RESP;
}

/* ===================================================================
 * SID 0x31 — RoutineControl
 * =================================================================== */

/**
 * @brief Handle SID 0x31, routine 0x0301 — AEB enable/disable.
 * @req FR-UDS-004
 */
static void uds_handle_routine_ctrl(uds_state_t *state,
                                    const uds_request_t *request,
                                    uds_response_t *response)
{
    uint16_t routine_id;

    routine_id = ((uint16_t)request->did_high * 256U)
               + (uint16_t)request->did_low;

    if (routine_id == UDS_RID_AEB_CTRL)     /* 0x0301 */
    {
        if (request->value == 0U)           /* Disable */
        {
            state->aeb_enabled = 0U;

            response->response_sid  = UDS_SID_ROUTINE_CTRL_RESP;
            response->did_high_resp = request->did_high;
            response->did_low_resp  = request->did_low;
            response->data1         = 0U;
        }
        else if (request->value == 1U)      /* Enable */
        {
            state->aeb_enabled = 1U;

            response->response_sid  = UDS_SID_ROUTINE_CTRL_RESP;
            response->did_high_resp = request->did_high;
            response->did_low_resp  = request->did_low;
            response->data1         = 1U;
        }
        else                                /* Invalid value */
        {
            uds_send_negative(response,
                              UDS_SID_ROUTINE_CTRL,
                              UDS_NRC_REQUEST_OOR);
        }
    }
    else                                    /* Routine not supported */
    {
        uds_send_negative(response,
                          UDS_SID_ROUTINE_CTRL,
                          UDS_NRC_REQUEST_OOR);
    }
}

/* ===================================================================
 * Public API
 * =================================================================== */

void uds_init(uds_state_t *state)
{
    state->aeb_enabled  = 1U;
    state->dtc_sensor   = 0U;
    state->dtc_crc      = 0U;
    state->dtc_actuator = 0U;
}

void uds_monitor_faults(uds_state_t *state,
                        uint8_t sensor_fault,
                        uint8_t crc_error,
                        uint8_t actuator_fault)
{
    /* DD-UDS-01: immediate latch, no debounce. */
    if (sensor_fault != 0U)
    {
        state->dtc_sensor = 1U;
    }

    if (crc_error != 0U)
    {
        state->dtc_crc = 1U;
    }

    if (actuator_fault != 0U)
    {
        state->dtc_actuator = 1U;
    }
}

void uds_process_request(uds_state_t *state,
                         const uds_request_t *request,
                         uds_response_t *response,
                         const fsm_output_t *fsm_out,
                         const pid_output_t *pid_out,
                         const ttc_output_t *ttc_out)
{
    uds_clear_response(response);

    /* Only process if sid is valid (sid != 0) — single exit point (Rule 15.5) */
    if (request->sid != 0U)
    {
        switch (request->sid)
        {
            case UDS_SID_READ_DID:          /* 0x22 */
            {
                uds_handle_read_did(request, response,
                                    fsm_out, pid_out, ttc_out);
                break;
            }

            case UDS_SID_CLEAR_DTC:         /* 0x14 */
            {
                uds_handle_clear_dtc(state, response);
                break;
            }

            case UDS_SID_ROUTINE_CTRL:      /* 0x31 */
            {
                uds_handle_routine_ctrl(state, request, response);
                break;
            }

            default:                        /* NRC 0x11 */
            {
                uds_send_negative(response,
                                  request->sid,
                                  UDS_NRC_SERVICE_NOT_SUPP);
                break;
            }
        }
    }
}

void uds_get_output(const uds_state_t *state, uds_output_t *output)
{
    /* Bugs #2 and #3 fix: Boolean normalisation against SEU (single-event upset).
     * Each flag is specified as Boolean (0 or 1), but a bit-flip from EMI,
     * voltage glitch, or cosmic radiation may corrupt the raw byte to values
     * like 0xAA or 0xFF. Without normalisation:
     *   - aeb_enabled could be propagated as a non-Boolean, mis-read by
     *     consumers using "== 1U" vs implicit truthiness tests.
     *   - dtc_count (uint8_t sum of three flags) could overflow/wrap
     *     (e.g. 3*0xFF = 0x2FD -> 0xFD), producing a nonsensical count.
     * Normalising each flag via (x != 0U) ? 1U : 0U bounds the sum to [0..3]
     * and delivers deterministic Boolean semantics to downstream consumers. */
    uint8_t s = (state->dtc_sensor   != 0U) ? 1U : 0U;
    uint8_t c = (state->dtc_crc      != 0U) ? 1U : 0U;
    uint8_t a = (state->dtc_actuator != 0U) ? 1U : 0U;

    output->aeb_enabled = (state->aeb_enabled != 0U) ? 1U : 0U;
    output->dtc_count   = (uint8_t)(s + c + a);                /* <= 3 */
    output->fault_lamp  = (output->dtc_count > 0U) ? 1U : 0U;
}
