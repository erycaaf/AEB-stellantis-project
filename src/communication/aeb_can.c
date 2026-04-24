/**
 * @file  aeb_can.c
 * @brief CAN Bus Communication module — implementation.
 *
 * Covers FR-CAN-001..004 from SRS v2.0.
 * All signal layouts match aeb_system.dbc.
 * MISRA C:2012 compliant: no malloc, no recursion, fixed-width types,
 * all variables initialised, single return per function (preferred),
 * bounded loops, default in every switch.
 *
 * @author  Renato Fagundes
 * @date    2026-04-07
 */

#include "aeb_can.h"
#include "can_hal.h"   /* Zephyr CAN HAL stub / real driver */
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  PRIVATE HELPERS — Signal pack / unpack  (FR-CAN-003)
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Pack an unsigned raw value into a little-endian CAN payload.
 */
void can_pack_signal(uint8_t *data,
                     uint8_t  start_bit,
                     uint8_t  length,
                     uint32_t raw_value)
{
    uint8_t bit_idx = 0U;

    for (bit_idx = 0U; bit_idx < length; bit_idx++)
    {
        uint8_t  abs_bit  = start_bit + bit_idx;
        uint8_t  byte_pos = abs_bit / 8U;
        uint8_t  bit_pos  = abs_bit % 8U;
        uint8_t  bit_val  = (uint8_t)((raw_value >> bit_idx) & 1U);

        /* Clear then set the target bit */
        data[byte_pos] = (uint8_t)(data[byte_pos] & (uint8_t)(~(1U << bit_pos)));
        data[byte_pos] = (uint8_t)(data[byte_pos] | (uint8_t)(bit_val << bit_pos));
    }
}

/**
 * @brief Unpack an unsigned raw value from a little-endian CAN payload.
 */
uint32_t can_unpack_signal(const uint8_t *data,
                           uint8_t        start_bit,
                           uint8_t        length)
{
    uint32_t result  = 0U;
    uint8_t  bit_idx = 0U;

    for (bit_idx = 0U; bit_idx < length; bit_idx++)
    {
        uint8_t  abs_bit  = start_bit + bit_idx;
        uint8_t  byte_pos = abs_bit / 8U;
        uint8_t  bit_pos  = abs_bit % 8U;
        uint8_t  bit_val  = (uint8_t)((data[byte_pos] >> bit_pos) & 1U);

        result |= ((uint32_t)bit_val << bit_idx);
    }

    return result;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PRIVATE — Physical-to-raw / raw-to-physical conversions
 *
 *  DBC formula: physical = raw * factor + offset
 *  Encode:      raw = (physical - offset) / factor
 * ═══════════════════════════════════════════════════════════════════════ */

static uint32_t encode_unsigned(float32_t physical,
                                float32_t factor,
                                float32_t offset)
{
    float32_t raw_f = (physical - offset) / factor;
    uint32_t  raw   = 0U;

    if (raw_f < 0.0F)
    {
        raw = 0U;
    }
    else
    {
        raw = (uint32_t)(raw_f + 0.5F);  /* round to nearest */
    }

    return raw;
}

static float32_t decode_unsigned(uint32_t  raw,
                                 float32_t factor,
                                 float32_t offset)
{
    return ((float32_t)raw * factor) + offset;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PRIVATE — CRC-4 computation for BrakeCmd alive monitoring
 * ═══════════════════════════════════════════════════════════════════════ */

static uint8_t compute_crc4(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x0FU;   /* Initial value, all ones for 4-bit CRC */
    uint8_t i   = 0U;

    for (i = 0U; i < len; i++)
    {
        uint8_t j = 0U;

        crc ^= (data[i] & 0x0FU);
        for (j = 0U; j < 4U; j++)
        {
            if ((crc & 0x08U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x03U);  /* Poly x^4+x+1 */
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
            crc &= 0x0FU;
        }

        crc ^= ((data[i] >> 4U) & 0x0FU);
        for (j = 0U; j < 4U; j++)
        {
            if ((crc & 0x08U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x03U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
            crc &= 0x0FU;
        }
    }

    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise CAN driver at 500 kbit/s, register RX filters.
 * @req FR-CAN-004
 */
int32_t can_init(can_state_t *state)
{
    int32_t result = CAN_OK;

    /* Zero the entire state */
    (void)memset(state, 0, sizeof(can_state_t));

    /* Initialise CAN peripheral via HAL */
    if (can_hal_init(CAN_BAUD_500K) != 0)
    {
        result = CAN_ERR_INIT;
    }
    else
    {
        /* Register RX filters for the messages we receive */
        (void)can_hal_add_rx_filter(CAN_ID_EGO_VEHICLE);
        (void)can_hal_add_rx_filter(CAN_ID_DRIVER_INPUT);
        (void)can_hal_add_rx_filter(CAN_ID_RADAR_TARGET);
        (void)can_hal_add_rx_filter(CAN_ID_UDS_REQUEST);

        state->initialised = 1U;
    }

    return result;
}

/**
 * @brief Decode a received CAN frame into the internal RX struct.
 * @req FR-CAN-002, FR-CAN-003
 */
void can_rx_process(can_state_t   *state,
                    uint32_t       id,
                    const uint8_t *data,
                    uint8_t        dlc)
{
    if ((state != NULL) && (data != NULL))
    {
    if (id == CAN_ID_EGO_VEHICLE)
    {
        if (dlc >= CAN_DLC_EGO_VEHICLE)
        {
            /* VehicleSpeed:  bits 0..15, factor 0.01, offset 0 */
            uint32_t raw_spd = can_unpack_signal(data, 0U, 16U);
            state->last_rx.vehicle_speed = decode_unsigned(raw_spd, 0.01F, 0.0F);

            /* LongAccel:  bits 16..31, factor 0.001, offset -32
             * DBC @1+ = unsigned raw with offset */
            uint32_t raw_acc = can_unpack_signal(data, 16U, 16U);
            state->last_rx.long_accel = decode_unsigned(raw_acc, 0.001F, -32.0F);

            /* YawRate:  bits 32..47, factor 0.01, offset -327.68 */
            uint32_t raw_yaw = can_unpack_signal(data, 32U, 16U);
            state->last_rx.yaw_rate = decode_unsigned(raw_yaw, 0.01F, -327.68F);

            /* SteeringAngle:  bits 48..63, factor 0.1, offset -3276.8 */
            uint32_t raw_str = can_unpack_signal(data, 48U, 16U);
            state->last_rx.steering_angle = decode_unsigned(raw_str, 0.1F, -3276.8F);
        }
    }
    else if (id == CAN_ID_DRIVER_INPUT)
    {
        if (dlc >= CAN_DLC_DRIVER_INPUT)
        {
            /* BrakePedal:  bits 0..7, factor 1, offset 0 */
            uint32_t raw_bp = can_unpack_signal(data, 0U, 8U);
            state->last_rx.brake_pedal = (uint8_t)raw_bp;

            /* AccelPedal:  bits 8..15, factor 1, offset 0 */
            uint32_t raw_ap = can_unpack_signal(data, 8U, 8U);
            state->last_rx.accel_pedal = (uint8_t)raw_ap;

            /* AEB_Enable:  bit 16, 1 bit */
            uint32_t raw_en = can_unpack_signal(data, 16U, 1U);
            state->last_rx.aeb_enable = (uint8_t)raw_en;

            /* DriverOverride:  bit 17, 1 bit */
            uint32_t raw_ov = can_unpack_signal(data, 17U, 1U);
            state->last_rx.driver_override = (uint8_t)raw_ov;
        }
    }
    else if (id == CAN_ID_RADAR_TARGET)
    {
        if (dlc >= CAN_DLC_RADAR_TARGET)
        {
            /* TargetDistance:  bits 0..15, factor 0.01, offset 0 */
            uint32_t raw_dist = can_unpack_signal(data, 0U, 16U);
            state->last_rx.target_distance = decode_unsigned(raw_dist, 0.01F, 0.0F);

            /* RelativeSpeed:  bits 16..31, factor 0.01, offset -327.68
             * DBC @1+ = unsigned raw with offset */
            uint32_t raw_vrel = can_unpack_signal(data, 16U, 16U);
            state->last_rx.relative_speed = decode_unsigned(raw_vrel, 0.01F, -327.68F);

            /* TTC:  bits 32..47, factor 0.001, offset 0 */
            uint32_t raw_ttc = can_unpack_signal(data, 32U, 16U);
            state->last_rx.ttc_radar = decode_unsigned(raw_ttc, 0.001F, 0.0F);

            /* Confidence:  bits 48..55, factor 1, offset 0 */
            uint32_t raw_conf = can_unpack_signal(data, 48U, 8U);
            state->last_rx.confidence_raw = (uint8_t)raw_conf;

            /* Valid frame received — reset miss counter */
            state->rx_miss_count = 0U;
            state->last_rx.rx_timeout_flag = 0U;
        }
    }
    else if (id == CAN_ID_UDS_REQUEST)
    {
        /* UDS_Request (0x7DF) — ISO 14229 functional broadcast.
         * 4-byte payload mapped directly to uds_request_t.
         * FR-UDS-005: request serviced within same 10 ms cycle. */
        if (dlc >= CAN_DLC_UDS_REQUEST)
        {
            state->last_rx.uds_request.sid      = data[0];
            state->last_rx.uds_request.did_high = data[1];
            state->last_rx.uds_request.did_low  = data[2];
            state->last_rx.uds_request.value    = data[3];
            state->last_rx.uds_request_pending  = 1U;
        }
    }
    else
    {
        /* Unknown ID — ignore (MISRA: all paths handled) */
    }
    } /* end null guard */
}

/**
 * @brief Check for RX timeout.  Call once per 10 ms tick.
 * @req FR-CAN-002 (timeout: 3 × 20 ms = 60 ms)
 */
void can_check_timeout(can_state_t *state)
{
    if (state != NULL)
    {
    if (state->rx_miss_count < 255U)
    {
        state->rx_miss_count++;
    }

    /*
     * Radar period = 20 ms, our tick = 10 ms.
     * 3 missed frames ≈ 6 ticks of 10 ms = 60 ms.
     * We use CAN_RX_TIMEOUT_CYCLES (3) as a count of our 20-ms-equivalent
     * intervals, so threshold = 3 * 2 = 6 ticks.
     */
    if (state->rx_miss_count >= (CAN_RX_TIMEOUT_CYCLES * 2U))
    {
        state->last_rx.rx_timeout_flag = 1U;
    }
    } /* end null guard */
}

/**
 * @brief Transmit AEB_BrakeCmd (0x080) with alive counter and CRC.
 * @req FR-CAN-003
 */
int32_t can_tx_brake_cmd(can_state_t       *state,
                         const pid_output_t *pid_out,
                         const fsm_output_t *fsm_out)
{
    int32_t result = CAN_OK;

    if ((state == NULL) || (pid_out == NULL) || (fsm_out == NULL))
    {
        result = CAN_ERR_TX;
    }
    else
    {
        uint8_t frame[8]   = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t brake_mode = 0U;

        /* BrakeRequest: bit 0, 1 bit */
        uint32_t brake_req = (pid_out->brake_pct > 0.0F) ? 1U : 0U;
        can_pack_signal(frame, 0U, 1U, brake_req);

        /* BrakePressure: bits 1..15, factor 0.1, offset 0 */
        uint32_t raw_press = encode_unsigned(pid_out->brake_bar, 0.1F, 0.0F);
        can_pack_signal(frame, 1U, 15U, raw_press);

        /* BrakeMode: bits 16..18, 3 bits — map FSM state to brake mode */
        switch (fsm_out->fsm_state)
        {
            case (uint8_t)FSM_OFF:        brake_mode = 0U; break;
            case (uint8_t)FSM_STANDBY:    brake_mode = 0U; break;
            case (uint8_t)FSM_WARNING:    brake_mode = 1U; break;
            case (uint8_t)FSM_BRAKE_L1:   brake_mode = 2U; break;
            case (uint8_t)FSM_BRAKE_L2:   brake_mode = 3U; break;
            case (uint8_t)FSM_BRAKE_L3:   brake_mode = 4U; break;
            case (uint8_t)FSM_POST_BRAKE: brake_mode = 5U; break;
            default:                      brake_mode = 0U; break;
        }
        can_pack_signal(frame, 16U, 3U, (uint32_t)brake_mode);

        /* AliveCounter: bits 24..27, 4 bits */
        can_pack_signal(frame, 24U, 4U, (uint32_t)state->alive_counter);

        /* CRC: bits 28..31, 4 bits — computed over bytes 0..2 */
        uint8_t crc = compute_crc4(frame, 3U);
        can_pack_signal(frame, 28U, 4U, (uint32_t)crc);

        /* Increment alive counter (wraps at 15) */
        state->alive_counter++;
        if (state->alive_counter > ALIVE_COUNTER_MAX)
        {
            state->alive_counter = 0U;
        }

        /* Send via HAL */
        if (can_hal_send(CAN_ID_BRAKE_CMD, frame, CAN_DLC_BRAKE_CMD) != 0)
        {
            result = CAN_ERR_TX;
        }
    }

    return result;
}

/**
 * @brief Transmit AEB_FSMState (0x200) every 50 ms.
 * @req FR-CAN-001
 */
int32_t can_tx_fsm_state(can_state_t       *state,
                         const fsm_output_t *fsm_out)
{
    int32_t result = 1;   /* 1 = not yet due */

    if ((state == NULL) || (fsm_out == NULL))
    {
        result = CAN_ERR_TX;
    }
    else
    {
        state->tx_cycle_counter++;

        /* 50 ms / 10 ms = every 5 ticks */
        if (state->tx_cycle_counter >= 5U)
        {
            uint8_t frame[4] = {0U, 0U, 0U, 0U};

            state->tx_cycle_counter = 0U;

            /* FSMState: bits 0..7 */
            can_pack_signal(frame, 0U, 8U, (uint32_t)fsm_out->fsm_state);

            /* AlertLevel: bits 8..15 */
            can_pack_signal(frame, 8U, 8U, (uint32_t)fsm_out->alert_level);

            /* BrakeActive: bits 16..23 */
            can_pack_signal(frame, 16U, 8U, (uint32_t)fsm_out->brake_active);

            /* TTCThreshold: bits 24..31, factor 0.1, offset 0 */
            float32_t ttc_thresh = 0.0F;
            switch (fsm_out->fsm_state)
            {
                case (uint8_t)FSM_WARNING:    ttc_thresh = TTC_WARNING;  break;
                case (uint8_t)FSM_BRAKE_L1:   ttc_thresh = TTC_BRAKE_L1; break;
                case (uint8_t)FSM_BRAKE_L2:   ttc_thresh = TTC_BRAKE_L2; break;
                case (uint8_t)FSM_BRAKE_L3:   ttc_thresh = TTC_BRAKE_L3; break;
                default:                      ttc_thresh = 0.0F;         break;
            }
            uint32_t raw_ttc = encode_unsigned(ttc_thresh, 0.1F, 0.0F);
            can_pack_signal(frame, 24U, 8U, raw_ttc);

            if (can_hal_send(CAN_ID_FSM_STATE, frame, CAN_DLC_FSM_STATE) != 0)
            {
                result = CAN_ERR_TX;
            }
            else
            {
                result = CAN_OK;
            }
        }
    }

    return result;
}

/**
 * @brief Transmit AEB_Alert (0x300).
 * @req FR-CAN-003
 */
int32_t can_tx_alert(const alert_output_t *alert_out)
{
    int32_t result = CAN_OK;

    if (alert_out == NULL)
    {
        result = CAN_ERR_TX;
    }
    else
    {
        uint8_t frame[2] = {0U, 0U};
        /* AlertType: bits 0..7, 8 bits, factor 1, offset 0 */
        can_pack_signal(frame, 0U, 8U, (uint32_t)alert_out->alert_type);

        /* AlertActive: bit 8, 1 bit */
        can_pack_signal(frame, 8U, 1U, (uint32_t)alert_out->alert_active);

        /* BuzzerCmd: bits 9..11, 3 bits */
        can_pack_signal(frame, 9U, 3U, (uint32_t)alert_out->buzzer_cmd);

        if (can_hal_send(CAN_ID_ALERT, frame, CAN_DLC_ALERT) != 0)
        {
            result = CAN_ERR_TX;
        }
    }

    return result;
}

/**
 * @brief Copy latest RX data to caller's struct.
 */
void can_get_rx_data(const can_state_t *state,
                     can_rx_data_t     *out)
{
    if ((state != NULL) && (out != NULL))
    {
        (void)memcpy(out, &state->last_rx, sizeof(can_rx_data_t));
    }
}

/**
 * @brief Transmit UDS_Response (0x7E8).
 * @req FR-UDS-005
 */
int32_t can_tx_uds_response(const uds_response_t *resp)
{
    int32_t result = CAN_OK;

    if (resp == NULL)
    {
        result = CAN_ERR_TX;
    }
    else
    {
        uint8_t frame[8];

        /* uds_response_t is an 8-byte wire-format struct — map 1:1. */
        frame[0] = resp->response_sid;
        frame[1] = resp->did_high_resp;
        frame[2] = resp->did_low_resp;
        frame[3] = resp->data1;
        frame[4] = resp->data2;
        frame[5] = resp->data3;
        frame[6] = resp->data4;
        frame[7] = resp->data5;

        if (can_hal_send(CAN_ID_UDS_RESPONSE, frame, CAN_DLC_UDS_RESPONSE) != 0)
        {
            result = CAN_ERR_TX;
        }
    }

    return result;
}

/**
 * @brief Clear the pending UDS request flag after it has been serviced.
 *
 * Must only be called after can_tx_uds_response() reported CAN_OK.
 * See aeb_can.h for the retry semantics rationale.
 */
void can_clear_uds_request_pending(can_state_t *state)
{
    if (state != NULL)
    {
        state->last_rx.uds_request_pending = 0U;
    }
}
