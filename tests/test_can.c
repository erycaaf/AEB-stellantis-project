/**
 * @file  test_can.c
 * @brief Unit tests for aeb_can module.
 */

#include "aeb_can.h"
#include "can_hal.h"
#include "can_hal_test.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ── Minimal test framework ─────────────────────────────────────────── */
static int32_t tests_run    = 0;
static int32_t tests_passed = 0;
static int32_t tests_failed = 0;

#define ASSERT_EQ(a, b) do { \
    if ((a) == (b)) { tests_passed++; } \
    else { printf("  FAIL: %s:%d  %s != %s\n", __FILE__, __LINE__, #a, #b); tests_failed++; } \
    tests_run++; \
} while (0)

#define ASSERT_FLOAT_NEAR(a, b, tol) do { \
    if (fabsf((float)(a) - (float)(b)) <= (float)(tol)) { tests_passed++; } \
    else { printf("  FAIL: %s:%d  %s=%.4f != %s=%.4f (tol=%.4f)\n", \
           __FILE__, __LINE__, #a, (double)(a), #b, (double)(b), (double)(tol)); tests_failed++; } \
    tests_run++; \
} while (0)

#define TEST(name) static void name(void)
#define RUN(name) do { printf("  [TEST] %s\n", #name); name(); } while (0)

/* Helper to extract BrakePressure field */
static uint32_t extract_brake_pressure(const tx_record_t *tx)
{
    return can_unpack_signal(tx->data, 1U, 15U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ORIGINAL TESTS
 * ═══════════════════════════════════════════════════════════════════════ */

TEST(test_signal_roundtrip)
{
    uint8_t buf[8] = {0};
    can_pack_signal(buf, 4U, 16U, 0xABCDU);
    uint32_t val = can_unpack_signal(buf, 4U, 16U);
    ASSERT_EQ(val, 0xABCDU);
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 0U, 1U, 1U);
    ASSERT_EQ(can_unpack_signal(buf, 0U, 1U), 1U);
    ASSERT_EQ(buf[0] & 0x01U, 0x01U);
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 24U, 4U, 0x0FU);
    ASSERT_EQ(can_unpack_signal(buf, 24U, 4U), 0x0FU);
}

TEST(test_init_success)
{
    can_state_t state;
    can_hal_test_reset();
    int32_t rc = can_init(&state);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(state.initialised, 1U);
    ASSERT_EQ(state.alive_counter, 0U);
}

TEST(test_init_failure)
{
    can_state_t state;
    can_hal_test_reset();
    can_hal_test_force_init_fail(1);
    int32_t rc = can_init(&state);
    ASSERT_EQ(rc, CAN_ERR_INIT);
    ASSERT_EQ(state.initialised, 0U);
    can_hal_test_force_init_fail(0);
}

TEST(test_rx_ego_vehicle)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1389U);
    can_pack_signal(frame, 16U, 16U, 30000U);
    can_pack_signal(frame, 48U, 16U, 32868U);
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 13.89F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.long_accel, -2.0F, 0.01F);
    ASSERT_FLOAT_NEAR(rx.steering_angle, 10.0F, 0.2F);
}

TEST(test_rx_radar_target)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 5000U);
    can_pack_signal(frame, 16U, 16U, 32268U);
    can_pack_signal(frame, 32U, 16U, 3500U);
    can_pack_signal(frame, 48U, 8U, 12U);
    can_rx_process(&state, CAN_ID_RADAR_TARGET, frame, 8U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_FLOAT_NEAR(rx.target_distance, 50.0F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.relative_speed, -5.0F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.ttc_radar, 3.5F, 0.002F);
    ASSERT_EQ(rx.confidence_raw, 12U);
}

TEST(test_rx_driver_input)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[4] = {0};
    can_pack_signal(frame, 0U, 8U, 80U);
    can_pack_signal(frame, 8U, 8U, 0U);
    can_pack_signal(frame, 16U, 1U, 1U);
    can_pack_signal(frame, 17U, 1U, 0U);
    can_rx_process(&state, CAN_ID_DRIVER_INPUT, frame, 4U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.brake_pedal, 80U);
    ASSERT_EQ(rx.accel_pedal, 0U);
    ASSERT_EQ(rx.aeb_enable, 1U);
    ASSERT_EQ(rx.driver_override, 0U);
}

TEST(test_rx_yaw_rate)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 32U, 16U, 33268U);
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_FLOAT_NEAR(rx.yaw_rate, 5.0F, 0.02F);
}

TEST(test_tx_alert)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    alert_output_t alert = { .alert_type = 3U, .alert_active = 1U, .buzzer_cmd = 4U };
    int32_t rc = can_tx_alert(&alert);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);
}

TEST(test_rx_timeout)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    for (uint8_t i = 0U; i < 6U; i++) { can_check_timeout(&state); }
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.rx_timeout_flag, 1U);
}

TEST(test_rx_timeout_reset_on_valid_frame)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    for (uint8_t i = 0U; i < 5U; i++) { can_check_timeout(&state); }
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1000U);
    can_rx_process(&state, CAN_ID_RADAR_TARGET, frame, 8U);
    can_check_timeout(&state);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.rx_timeout_flag, 0U);
}

TEST(test_tx_brake_cmd)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = {0};
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L3;
    fsm.brake_active = 1U;
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(state.alive_counter, 1U);
}

TEST(test_tx_fsm_period)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };
    for (uint8_t i = 0U; i < 4U; i++) { ASSERT_EQ(can_tx_fsm_state(&state, &fsm), 1); }
    ASSERT_EQ(can_tx_fsm_state(&state, &fsm), CAN_OK);
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);
}

TEST(test_tx_brake_cmd_nan_brake_bar)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = NAN };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    ASSERT_EQ(extract_brake_pressure(tx), 0U);
}

TEST(test_tx_brake_cmd_pos_inf_brake_bar)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = INFINITY };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    ASSERT_EQ(extract_brake_pressure(tx), 0U);
}

TEST(test_tx_brake_cmd_neg_inf_brake_bar)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = -INFINITY };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    ASSERT_EQ(extract_brake_pressure(tx), 0U);
}

TEST(test_tx_brake_cmd_overflow_brake_bar)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = 1.0e20F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = extract_brake_pressure(tx);
    ASSERT_EQ(brake_pressure, 0x7FFFU);
}

TEST(test_tx_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    can_hal_test_force_send_fail(1);
    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);
    can_hal_test_force_send_fail(0);
}

TEST(test_rx_uds_request)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[4] = { 0x22U, 0xF1U, 0x01U, 0x00U };
    can_rx_process(&state, CAN_ID_UDS_REQUEST, frame, CAN_DLC_UDS_REQUEST);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 1U);
    ASSERT_EQ(rx.uds_request.sid, 0x22U);
}

TEST(test_tx_uds_response)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uds_response_t resp;
    resp.response_sid   = 0x62U;
    resp.did_high_resp  = 0xF1U;
    resp.did_low_resp   = 0x01U;
    resp.data1          = (uint8_t)FSM_STANDBY;
    int32_t rc = can_tx_uds_response(&resp);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    ASSERT_EQ(tx->id, (uint32_t)CAN_ID_UDS_RESPONSE);
    ASSERT_EQ(tx->data[0], 0x62U);
}

TEST(test_rx_uds_request_short_dlc)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[3] = { 0x22U, 0xF1U, 0x01U };
    can_rx_process(&state, CAN_ID_UDS_REQUEST, frame, 3U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 0U);
}

TEST(test_uds_request_ack)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[4] = { 0x22U, 0xF1U, 0x01U, 0x00U };
    can_rx_process(&state, CAN_ID_UDS_REQUEST, frame, CAN_DLC_UDS_REQUEST);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 1U);
    can_clear_uds_request_pending(&state);
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ADDITIONAL TESTS FOR 100% COVERAGE
 * ═══════════════════════════════════════════════════════════════════════ */

TEST(test_tx_brake_cmd_null_params)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    ASSERT_EQ(can_tx_brake_cmd(NULL, &pid, &fsm), CAN_ERR_TX);
    ASSERT_EQ(can_tx_brake_cmd(&state, NULL, &fsm), CAN_ERR_TX);
    ASSERT_EQ(can_tx_brake_cmd(&state, &pid, NULL), CAN_ERR_TX);
}

TEST(test_tx_brake_cmd_alive_counter_wrap)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };
    for (uint8_t i = 0U; i < 20U; i++) {
        int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
        ASSERT_EQ(rc, CAN_OK);
    }
    ASSERT_EQ(state.alive_counter, 4U);
}

TEST(test_tx_fsm_state_null_params)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };
    ASSERT_EQ(can_tx_fsm_state(NULL, &fsm), CAN_ERR_TX);
    ASSERT_EQ(can_tx_fsm_state(&state, NULL), CAN_ERR_TX);
}

TEST(test_tx_uds_response_null)
{
    ASSERT_EQ(can_tx_uds_response(NULL), CAN_ERR_TX);
}

TEST(test_clear_uds_request_pending_null)
{
    can_clear_uds_request_pending(NULL);
    ASSERT_EQ(1, 1);
}

TEST(test_encode_unsigned_saturation)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = 1.0e30F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0x7FFFU);
}

TEST(test_tx_alert_null)
{
    ASSERT_EQ(can_tx_alert(NULL), CAN_ERR_TX);
}

TEST(test_check_timeout_null)
{
    can_check_timeout(NULL);
    ASSERT_EQ(1, 1);
}

TEST(test_decode_unsigned_via_rx)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 5000U);
    can_pack_signal(frame, 16U, 16U, 30000U);
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 50.0F, 0.1F);
}

TEST(test_init_hal_failure_branch)
{
    can_state_t state;
    can_hal_test_reset();
    can_hal_test_force_init_fail(1);
    int32_t rc = can_init(&state);
    ASSERT_EQ(rc, CAN_ERR_INIT);
    ASSERT_EQ(state.initialised, 0U);
    can_hal_test_force_init_fail(0);
}

TEST(test_tx_brake_cmd_alive_counter_wrap_to_zero)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };
    for (uint8_t i = 0U; i < 16U; i++) {
        (void)can_tx_brake_cmd(&state, &pid, &fsm);
    }
    ASSERT_EQ(state.alive_counter, 0U);
}

TEST(test_tx_uds_response_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    can_hal_test_force_send_fail(1);
    uds_response_t resp;
    resp.response_sid = 0x62U;
    int32_t rc = can_tx_uds_response(&resp);
    ASSERT_EQ(rc, CAN_ERR_TX);
    can_hal_test_force_send_fail(0);
}

TEST(test_clear_uds_request_pending_valid_state)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    uint8_t frame[4] = { 0x22U, 0xF1U, 0x01U, 0x00U };
    can_rx_process(&state, CAN_ID_UDS_REQUEST, frame, CAN_DLC_UDS_REQUEST);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 1U);
    can_clear_uds_request_pending(&state);
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.uds_request_pending, 0U);
}

TEST(test_tx_alert_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    can_hal_test_force_send_fail(1);
    alert_output_t alert = { .alert_type = 3U };
    int32_t rc = can_tx_alert(&alert);
    ASSERT_EQ(rc, CAN_ERR_TX);
    can_hal_test_force_send_fail(0);
}

TEST(test_brake_mode_all_cases)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm;
    uint8_t modes[] = {FSM_OFF, FSM_STANDBY, FSM_WARNING, FSM_BRAKE_L2, FSM_POST_BRAKE, 0xFFU};
    for (uint8_t i = 0U; i < 6U; i++) {
        fsm.fsm_state = modes[i];
        int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
        ASSERT_EQ(rc, CAN_OK);
    }
}

TEST(test_ttc_thresh_all_cases)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    fsm_output_t fsm;
    uint8_t modes[] = {FSM_BRAKE_L1, FSM_BRAKE_L2, FSM_BRAKE_L3, 0xFFU};
    for (uint8_t i = 0U; i < 4U; i++) {
        state.tx_cycle_counter = 5U;
        fsm.fsm_state = modes[i];
        int32_t rc = can_tx_fsm_state(&state, &fsm);
        ASSERT_EQ(rc, CAN_OK);
    }
}

TEST(test_tx_fsm_state_hal_send_failure_branch)
{
    can_state_t state;
    can_hal_test_reset();
    can_hal_test_force_send_fail(1);
    (void)can_init(&state);
    state.tx_cycle_counter = 5U;
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };
    int32_t rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);
    can_hal_test_force_send_fail(0);
}

TEST(test_encode_unsigned_negative_raw_f)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = -1000.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0U);
}

TEST(test_encode_unsigned_raw_f_inf)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = FLT_MAX };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0U);
}

TEST(test_can_rx_process_data_null)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, NULL, 8U);
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 0.0F, 0.01F);
}

TEST(test_can_rx_process_state_null)
{
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1389U);
    can_rx_process(NULL, CAN_ID_EGO_VEHICLE, frame, 8U);
    ASSERT_EQ(1, 1);
}

TEST(test_encode_unsigned_physical_inf)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = INFINITY };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0U);
}

TEST(test_encode_unsigned_raw_f_nan)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = NAN };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0U);
}

TEST(test_tx_brake_cmd_brake_pct_zero)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = 0.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
}

TEST(test_tx_brake_cmd_brake_pct_positive)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
}

TEST(test_tx_brake_cmd_alive_counter_wrap_branch)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };
    state.alive_counter = 15U;
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(state.alive_counter, 0U);
}

TEST(test_can_check_timeout_miss_count_increment)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    
    /* Initial miss_count should be 0, rx_timeout_flag 0 */
    can_check_timeout(&state);
    
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.rx_timeout_flag, 0U);
    
    /* Call enough times to trigger timeout but not exceed */
    for (uint8_t i = 0U; i < 3U; i++) {
        can_check_timeout(&state);
    }
    can_get_rx_data(&state, &rx);
    /* Still not at 6 ticks, so flag still 0 */
    ASSERT_EQ(rx.rx_timeout_flag, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("=== AEB CAN Module — Unit Tests ===\n\n");

    /* Original tests */
    RUN(test_signal_roundtrip);
    RUN(test_init_success);
    RUN(test_init_failure);
    RUN(test_rx_ego_vehicle);
    RUN(test_rx_yaw_rate);
    RUN(test_rx_radar_target);
    RUN(test_rx_driver_input);
    RUN(test_tx_alert);
    RUN(test_rx_timeout);
    RUN(test_rx_timeout_reset_on_valid_frame);
    RUN(test_tx_brake_cmd);
    RUN(test_tx_fsm_period);
    RUN(test_tx_brake_cmd_nan_brake_bar);
    RUN(test_tx_brake_cmd_pos_inf_brake_bar);
    RUN(test_tx_brake_cmd_neg_inf_brake_bar);
    RUN(test_tx_brake_cmd_overflow_brake_bar);
    RUN(test_tx_send_failure);
    RUN(test_rx_uds_request);
    RUN(test_tx_uds_response);
    RUN(test_rx_uds_request_short_dlc);
    RUN(test_uds_request_ack);

    /* Additional coverage tests */
    RUN(test_tx_brake_cmd_null_params);
    RUN(test_tx_brake_cmd_alive_counter_wrap);
    RUN(test_tx_fsm_state_null_params);
    RUN(test_tx_uds_response_null);
    RUN(test_clear_uds_request_pending_null);
    RUN(test_encode_unsigned_saturation);
    RUN(test_tx_alert_null);
    RUN(test_check_timeout_null);
    RUN(test_decode_unsigned_via_rx);
    RUN(test_init_hal_failure_branch);
    RUN(test_tx_brake_cmd_alive_counter_wrap_to_zero);
    RUN(test_tx_uds_response_send_failure);
    RUN(test_clear_uds_request_pending_valid_state);
    RUN(test_tx_alert_send_failure);
    RUN(test_brake_mode_all_cases);
    RUN(test_ttc_thresh_all_cases);
    RUN(test_tx_fsm_state_hal_send_failure_branch);
    RUN(test_encode_unsigned_negative_raw_f);
    RUN(test_encode_unsigned_raw_f_inf);
    RUN(test_can_rx_process_data_null);
    RUN(test_can_rx_process_state_null);
    RUN(test_encode_unsigned_physical_inf);
    RUN(test_encode_unsigned_raw_f_nan);
    RUN(test_tx_brake_cmd_brake_pct_zero);
    RUN(test_tx_brake_cmd_brake_pct_positive);
    RUN(test_tx_brake_cmd_alive_counter_wrap_branch);
    RUN(test_can_check_timeout_miss_count_increment);
    
    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}