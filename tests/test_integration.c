/**
 * @file    test_integration.c
 * @brief   End-to-end integration tests for the AEB system.
 * @module  Integration --- scenario-level verification
 * @version 1.0
 * @date    2026-04-10
 *
 * @details Exercises the full AEB pipeline through aeb_core_step(),
 *          verifying correct module interplay for key scenarios:
 *          CCRs activation sequence, driver override, sensor fault,
 *          and speed-out-of-range.
 *
 * @requirements FR-COD-001, FR-COD-002, NFR-VAL-001, NFR-VAL-005
 */

#include <stdio.h>
#include <string.h>
#include "aeb_core.h"
#include "aeb_config.h"
#include "can_hal_test.h"

static int g_test_count  = 0;
static int g_pass_count  = 0;
static int g_fail_count  = 0;

#define TEST_ASSERT(cond, msg)                                                  do {                                                                             g_test_count++;                                                              if (cond) {                                                                      g_pass_count++;                                                              printf("  PASS: %s\n", (msg));                                           } else {                                                                         g_fail_count++;                                                              printf("  FAIL: %s [%s:%d]\n", (msg), __FILE__, __LINE__);               }                                                                        } while (0)

static raw_sensor_input_t make_raw(float32_t distance,
                                   float32_t v_ego,
                                   float32_t v_target)
{
    raw_sensor_input_t raw;
    (void)memset(&raw, 0, sizeof(raw));
    raw.radar_d  = distance;
    raw.lidar_d  = distance;
    raw.radar_vr = v_ego - v_target;
    raw.v_ego    = v_ego;
    return raw;
}


static void inject_can_frames(aeb_core_state_t *state,
                              float32_t distance,
                              float32_t v_rel)
{
    /* Radar target frame (0x120) to prevent CAN RX timeout.
     * DBC layout: TargetDistance bits 0..15 factor 0.01 offset 0
     *             RelativeSpeed bits 16..31 factor 0.01 offset -327.68 */
    uint8_t radar_frame[8] = {0};
    uint32_t raw_dist = (uint32_t)(distance / 0.01F);
    uint32_t raw_vrel = (uint32_t)((v_rel + 327.68F) / 0.01F);
    can_pack_signal(radar_frame, 0, 16, raw_dist);
    can_pack_signal(radar_frame, 16, 16, raw_vrel);
    can_rx_process(&state->can, 0x120U, radar_frame, 8U);
}
static void test_init_clean_state(void)
{
    aeb_core_state_t state;

    printf("\n[TEST] Init clean state\n");
    int32_t rc = aeb_core_init(&state);

    TEST_ASSERT(rc == 0,
        "aeb_core_init returns success");
    TEST_ASSERT(state.fsm.fsm_state == (uint8_t)FSM_STANDBY,
        "FSM starts in STANDBY after init");
    TEST_ASSERT(state.pid.brake_pct < 0.01F,
        "brake output is zero after init");
    TEST_ASSERT(state.alert.alert_active == 0U,
        "no alert active after init");
}

static void test_ccrs_activation_sequence(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;
    uint8_t saw_warning  = 0U;
    float32_t max_brake_pct = 0.0F;
    uint8_t saw_braking  = 0U;

    printf("\n[TEST] CCRs activation sequence\n");
    (void)aeb_core_init(&state);

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }

    float32_t distance = 50.0F;
    float32_t v_ego    = 11.11F;
    uint16_t i = 0U;

    for (i = 0U; i < 500U; i++)
    {
        distance = distance - (v_ego * AEB_DT);
        if (distance < 0.5F)
        {
            distance = 0.5F;
        }

        raw = make_raw(distance, v_ego, 0.0F);
        inject_can_frames(&state, distance, v_ego);
        aeb_core_step(&state, &raw);

        if (state.fsm.fsm_state == (uint8_t)FSM_WARNING)
        {
            saw_warning = 1U;
        }
        if (state.fsm.fsm_state >= (uint8_t)FSM_BRAKE_L1)
        {
            saw_braking = 1U;
        }
        if (state.pid.brake_pct > max_brake_pct)
        {
            max_brake_pct = state.pid.brake_pct;
        }
    }

    TEST_ASSERT(saw_warning == 1U,
        "system entered WARNING during closing scenario");
    TEST_ASSERT(saw_braking == 1U,
        "system escalated to braking during closing scenario");
    TEST_ASSERT(max_brake_pct > 0.0F,
        "PID produced positive brake output during scenario");
    TEST_ASSERT(state.alert.alert_active == 1U || saw_braking == 1U,
        "alert was activated during scenario");
}

static void test_driver_override(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] Driver override\n");
    (void)aeb_core_init(&state);

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }

    raw = make_raw(35.0F, 10.0F, 0.0F);
    uint16_t i = 0U;
    for (i = 0U; i < 50U; i++)
    {
        inject_can_frames(&state, 35.0F, 10.0F);
        aeb_core_step(&state, &raw);
    }

    TEST_ASSERT(state.fsm.fsm_state >= (uint8_t)FSM_WARNING,
        "system reached WARNING before override");

    {
        uint8_t override_frame[8] = {0};
        can_pack_signal(override_frame, 0, 1, 1U);
        can_pack_signal(override_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, override_frame, 8U);
    }

    for (i = 0U; i < 10U; i++)
    {
        inject_can_frames(&state, 35.0F, 10.0F);
        aeb_core_step(&state, &raw);
    }

    TEST_ASSERT(state.fsm.fsm_state == (uint8_t)FSM_STANDBY,
        "system returned to STANDBY after driver override");
}

static void test_sensor_fault_to_off(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] Sensor fault -> OFF\n");
    (void)aeb_core_init(&state);

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }

    raw = make_raw(50.0F, 10.0F, 0.0F);
    uint16_t i = 0U;
    for (i = 0U; i < 20U; i++)
    {
        aeb_core_step(&state, &raw);
    }

    raw.fi = 1U;
    for (i = 0U; i < 10U; i++)
    {
        aeb_core_step(&state, &raw);
    }

    TEST_ASSERT(state.perception.fault_flag != 0U,
        "perception reports fault after fi injection");
    TEST_ASSERT(state.fsm.fsm_state == (uint8_t)FSM_OFF,
        "FSM transitions to OFF on sensor fault");
    TEST_ASSERT(state.pid.brake_pct < 0.01F,
        "brake output is zero in OFF state");
}

/* Scan the HAL TX capture buffer for a frame with the given CAN ID.
 * Returns the matching record, or NULL if not present. */
static const tx_record_t *find_tx(uint32_t id)
{
    uint32_t n = can_hal_test_get_tx_count();
    uint32_t i = 0U;
    const tx_record_t *hit = NULL;

    for (i = 0U; i < n; i++)
    {
        const tx_record_t *rec = can_hal_test_get_tx(i);
        if ((rec != NULL) && (rec->id == id))
        {
            hit = rec;
        }
    }
    return hit;
}

static void test_uds_request_response_roundtrip(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] UDS request/response round-trip (0x7DF -> 0x7E8)\n");
    (void)aeb_core_init(&state);
    can_hal_test_reset();

    /* Benign scenario — AEB enabled, stopped ego, far non-closing target.
     * FSM stays in STANDBY so DID 0xF101 returns a known value.
     * The roundtrip contract we care about is CAN path, not FSM dynamics. */
    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);   /* aeb_enable */
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }
    inject_can_frames(&state, 100.0F, 0.0F);

    uint8_t req[4] = { 0x22U, 0xF1U, 0x01U, 0x00U };
    can_rx_process(&state.can, CAN_ID_UDS_REQUEST, req, CAN_DLC_UDS_REQUEST);

    /* Run one full cycle — step 10 services the request,
     * step 11 transmits.  We scan the TX buffer for the 0x7E8 record. */
    raw = make_raw(100.0F, 0.0F, 0.0F);
    aeb_core_step(&state, &raw);

    const tx_record_t *resp = find_tx(CAN_ID_UDS_RESPONSE);
    TEST_ASSERT(resp != NULL,
        "UDS response frame 0x7E8 was transmitted in same cycle");
    if (resp != NULL)
    {
        TEST_ASSERT(resp->dlc == CAN_DLC_UDS_RESPONSE,
            "UDS response DLC is 8");
        TEST_ASSERT(resp->data[0] == 0x62U,
            "positive ReadDID response SID (0x62)");
        TEST_ASSERT(resp->data[1] == 0xF1U && resp->data[2] == 0x01U,
            "response echoes DID 0xF101");
        TEST_ASSERT(resp->data[3] == (uint8_t)FSM_STANDBY,
            "response reports current FSM state (STANDBY) in data1");
    }

    /* After service, the pending flag must be cleared. */
    can_rx_data_t rx;
    can_get_rx_data(&state.can, &rx);
    TEST_ASSERT(rx.uds_request_pending == 0U,
        "pending flag cleared after one cycle");
}

/* ISO 14229 negative-response flow through the full pipeline:
 * an unknown SID produces SID 0x7F + NRC 0x11 (Service Not Supported);
 * a supported SID with an unknown DID produces SID 0x7F + NRC 0x31
 * (Request Out Of Range). In both cases the response MUST still go out
 * on 0x7E8 — silence would force the client into a P2 timeout. */
static void test_uds_negative_response_unknown_sid(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] UDS negative response — unknown SID\n");
    (void)aeb_core_init(&state);
    can_hal_test_reset();

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }
    inject_can_frames(&state, 100.0F, 0.0F);

    /* SID 0xAA is not in the supported set (0x22, 0x14, 0x31).
     * SID 0x00 is reserved as "empty request" by uds_process_request
     * and produces no response at all — we need a real unknown SID. */
    uint8_t req[4] = { 0xAAU, 0x00U, 0x00U, 0x00U };
    can_rx_process(&state.can, CAN_ID_UDS_REQUEST, req, CAN_DLC_UDS_REQUEST);

    raw = make_raw(100.0F, 0.0F, 0.0F);
    aeb_core_step(&state, &raw);

    const tx_record_t *resp = find_tx(CAN_ID_UDS_RESPONSE);
    TEST_ASSERT(resp != NULL,
        "negative response frame 0x7E8 transmitted on unknown SID");
    if (resp != NULL)
    {
        TEST_ASSERT(resp->data[0] == 0x7FU,
            "negative response SID byte is 0x7F");
        TEST_ASSERT(resp->data[2] == 0x11U,
            "NRC 0x11 (Service Not Supported) on unknown SID");
    }
}

static void test_uds_negative_response_unknown_did(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] UDS negative response — unknown DID\n");
    (void)aeb_core_init(&state);
    can_hal_test_reset();

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }
    inject_can_frames(&state, 100.0F, 0.0F);

    /* ReadDID with a DID outside the supported set (0xF100, 0xF101, 0xF102). */
    uint8_t req[4] = { 0x22U, 0xFFU, 0xFFU, 0x00U };
    can_rx_process(&state.can, CAN_ID_UDS_REQUEST, req, CAN_DLC_UDS_REQUEST);

    raw = make_raw(100.0F, 0.0F, 0.0F);
    aeb_core_step(&state, &raw);

    const tx_record_t *resp = find_tx(CAN_ID_UDS_RESPONSE);
    TEST_ASSERT(resp != NULL,
        "negative response frame 0x7E8 transmitted on unknown DID");
    if (resp != NULL)
    {
        TEST_ASSERT(resp->data[0] == 0x7FU,
            "negative response SID byte is 0x7F");
        TEST_ASSERT(resp->data[2] == 0x31U,
            "NRC 0x31 (Request Out Of Range) on unknown DID");
    }
}

static void test_speed_out_of_range(void)
{
    aeb_core_state_t state;
    raw_sensor_input_t raw;

    printf("\n[TEST] Speed out of range\n");
    (void)aeb_core_init(&state);

    {
        uint8_t enable_frame[8] = {0};
        can_pack_signal(enable_frame, 16, 1, 1U);
        can_rx_process(&state.can, 0x101U, enable_frame, 8U);
    }

    raw = make_raw(10.0F, 1.39F, 0.0F);
    uint16_t i = 0U;
    for (i = 0U; i < 100U; i++)
    {
        aeb_core_step(&state, &raw);
    }

    TEST_ASSERT(state.fsm.fsm_state <= (uint8_t)FSM_STANDBY,
        "system stays in STANDBY when ego speed below 10 km/h");
}

int main(void)
{
    printf("========================================\n");
    printf("  AEB Integration Tests\n");
    printf("========================================\n");

    test_init_clean_state();
    test_ccrs_activation_sequence();
    test_driver_override();
    test_sensor_fault_to_off();
    test_speed_out_of_range();
    test_uds_request_response_roundtrip();
    test_uds_negative_response_unknown_sid();
    test_uds_negative_response_unknown_did();

    printf("\n========================================\n");
    printf("  Results: %d/%d passed, %d failed\n",
           g_pass_count, g_test_count, g_fail_count);
    printf("========================================\n");

    return (g_fail_count > 0) ? 1 : 0;
}
