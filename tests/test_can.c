/**
 * @file  test_can.c
 * @brief Unit tests for aeb_can module.
 *
 * Uses a minimal assert framework for host compilation.
 * Can be adapted to Zephyr ztest by replacing ASSERT macros.
 */

#include "aeb_can.h"
#include "can_hal.h"
#include "can_hal_test.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

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

/* Test helpers from the CAN HAL stub are declared in can_hal_test.h */

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: Signal pack/unpack round-trip  (FR-CAN-003)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_signal_roundtrip)
{
    uint8_t buf[8] = {0};

    /* Pack 0xABCD into bits 4..19 (16 bits) */
    can_pack_signal(buf, 4U, 16U, 0xABCDU);
    uint32_t val = can_unpack_signal(buf, 4U, 16U);
    ASSERT_EQ(val, 0xABCDU);

    /* Pack single bit */
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 0U, 1U, 1U);
    ASSERT_EQ(can_unpack_signal(buf, 0U, 1U), 1U);
    ASSERT_EQ(buf[0] & 0x01U, 0x01U);

    /* Pack 4-bit value at bit 24 */
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 24U, 4U, 0x0FU);
    ASSERT_EQ(can_unpack_signal(buf, 24U, 4U), 0x0FU);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_init success and failure  (FR-CAN-004)
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — EgoVehicle (0x100)  (FR-CAN-002, FR-CAN-003)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_rx_ego_vehicle)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* Construct a frame: VehicleSpeed=13.89 m/s (50 km/h)
     * raw = 13.89 / 0.01 = 1389 = 0x056D */
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1389U);   /* VehicleSpeed */

    /* LongAccel = -2.0 m/s^2 → raw = (-2.0 - (-32)) / 0.001 = 30000 */
    can_pack_signal(frame, 16U, 16U, 30000U);

    /* SteeringAngle = 10.0 deg → raw = (10.0 - (-3276.8)) / 0.1 = 32868 */
    can_pack_signal(frame, 48U, 16U, 32868U);

    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);

    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 13.89F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.long_accel, -2.0F, 0.01F);
    ASSERT_FLOAT_NEAR(rx.steering_angle, 10.0F, 0.2F);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — RadarTarget (0x120)  (FR-CAN-002, FR-CAN-003)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_rx_radar_target)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* TargetDistance = 50.0 m → raw = 50.0 / 0.01 = 5000 */
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 5000U);

    /* RelativeSpeed = -5.0 m/s → raw = (-5.0 - (-327.68)) / 0.01 = 32268 */
    can_pack_signal(frame, 16U, 16U, 32268U);

    /* TTC = 3.5 s → raw = 3.5 / 0.001 = 3500 */
    can_pack_signal(frame, 32U, 16U, 3500U);

    /* Confidence = 12 */
    can_pack_signal(frame, 48U, 8U, 12U);

    can_rx_process(&state, CAN_ID_RADAR_TARGET, frame, 8U);

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);

    ASSERT_FLOAT_NEAR(rx.target_distance, 50.0F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.relative_speed, -5.0F, 0.02F);
    ASSERT_FLOAT_NEAR(rx.ttc_radar, 3.5F, 0.002F);
    ASSERT_EQ(rx.confidence_raw, 12U);
    ASSERT_EQ(rx.rx_timeout_flag, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — DriverInput (0x101)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_rx_driver_input)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* BrakePedal=80%, AccelPedal=0%, AEB_Enable=1, DriverOverride=0 */
    uint8_t frame[4] = {0};
    can_pack_signal(frame, 0U, 8U, 80U);   /* BrakePedal */
    can_pack_signal(frame, 8U, 8U, 0U);    /* AccelPedal */
    can_pack_signal(frame, 16U, 1U, 1U);   /* AEB_Enable */
    can_pack_signal(frame, 17U, 1U, 0U);   /* DriverOverride */

    can_rx_process(&state, CAN_ID_DRIVER_INPUT, frame, 4U);

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);

    ASSERT_EQ(rx.brake_pedal, 80U);
    ASSERT_EQ(rx.accel_pedal, 0U);
    ASSERT_EQ(rx.aeb_enable, 1U);
    ASSERT_EQ(rx.driver_override, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — YawRate from EgoVehicle
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_rx_yaw_rate)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* YawRate = 5.0 deg/s -> raw = (5.0 - (-327.68)) / 0.01 = 33268 */
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 32U, 16U, 33268U);

    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);

    ASSERT_FLOAT_NEAR(rx.yaw_rate, 5.0F, 0.02F);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: TX Alert (0x300)
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX timeout detection  (FR-CAN-002 acceptance: 60 ms)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_rx_timeout)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* Simulate 6 ticks (60 ms) with no RX */
    uint8_t i = 0U;
    for (i = 0U; i < 6U; i++)
    {
        can_check_timeout(&state);
    }

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.rx_timeout_flag, 1U);
}

TEST(test_rx_timeout_reset_on_valid_frame)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* 5 ticks without data */
    uint8_t i = 0U;
    for (i = 0U; i < 5U; i++)
    {
        can_check_timeout(&state);
    }

    /* Valid radar frame arrives — should reset counter */
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1000U); /* 10m */
    can_rx_process(&state, CAN_ID_RADAR_TARGET, frame, 8U);

    /* 1 more tick — should NOT be timed out */
    can_check_timeout(&state);

    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    ASSERT_EQ(rx.rx_timeout_flag, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: TX BrakeCmd encoding  (FR-CAN-003)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_brake_cmd)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = {0};
    fsm.fsm_state   = (uint8_t)FSM_BRAKE_L3;
    fsm.brake_active = 1U;

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);

    /* Alive counter should have incremented */
    ASSERT_EQ(state.alive_counter, 1U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: TX FSM State at 50 ms period  (FR-CAN-001)
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_fsm_period)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    fsm_output_t fsm = {0};
    fsm.fsm_state = (uint8_t)FSM_WARNING;
    fsm.alert_level = 1U;

    /* First 4 ticks: should NOT transmit (return 1 = not due) */
    int32_t rc = 0;
    uint8_t i = 0U;
    for (i = 0U; i < 4U; i++)
    {
        rc = can_tx_fsm_state(&state, &fsm);
        ASSERT_EQ(rc, 1);
    }

    /* 5th tick: SHOULD transmit */
    rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TESTS: encode_unsigned robustness — exercised through can_tx_brake_cmd
 * ═══════════════════════════════════════════════════════════════════════ */

/** Helper — extract the 15-bit BrakePressure field from a captured frame. */
static uint32_t extract_brake_pressure(const tx_record_t *tx)
{
    return can_unpack_signal(tx->data, 1U, 15U);
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
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);

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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: TX failure handling
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    can_hal_test_force_send_fail(1);

    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm = {0};
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L1;

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);

    can_hal_test_force_send_fail(0);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — UDS Request (0x7DF)  (FR-UDS-005)
 * ═══════════════════════════════════════════════════════════════════════ */
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
    ASSERT_EQ(rx.uds_request.did_high, 0xF1U);
    ASSERT_EQ(rx.uds_request.did_low, 0x01U);
    ASSERT_EQ(rx.uds_request.value, 0x00U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: TX UDS Response (0x7E8)  (FR-UDS-005)
 * ═══════════════════════════════════════════════════════════════════════ */
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
    resp.data2          = 0x00U;
    resp.data3          = 0x00U;
    resp.data4          = 0x00U;
    resp.data5          = 0x00U;

    int32_t rc = can_tx_uds_response(&resp);
    ASSERT_EQ(rc, CAN_OK);
    ASSERT_EQ(can_hal_test_get_tx_count(), 1U);

    const tx_record_t *tx = can_hal_test_get_tx(0U);
    ASSERT_EQ(tx->id, (uint32_t)CAN_ID_UDS_RESPONSE);
    ASSERT_EQ(tx->dlc, CAN_DLC_UDS_RESPONSE);
    ASSERT_EQ(tx->data[0], 0x62U);
    ASSERT_EQ(tx->data[1], 0xF1U);
    ASSERT_EQ(tx->data[2], 0x01U);
    ASSERT_EQ(tx->data[3], (uint8_t)FSM_STANDBY);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: RX decode — UDS Request with short DLC is rejected
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: UDS request ack clears the pending flag
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* TEST: can_tx_brake_cmd with NULL parameters (line 332) */
TEST(test_tx_brake_cmd_null_params)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    int32_t rc = can_tx_brake_cmd(NULL, &pid, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);

    rc = can_tx_brake_cmd(&state, NULL, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);

    rc = can_tx_brake_cmd(&state, &pid, NULL);
    ASSERT_EQ(rc, CAN_ERR_TX);
}

/* TEST: can_tx_brake_cmd alive_counter wrap (line 372) */
TEST(test_tx_brake_cmd_alive_counter_wrap)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };

    for (uint8_t i = 0U; i < 20U; i++)
    {
        int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
        ASSERT_EQ(rc, CAN_OK);
    }

    ASSERT_EQ(state.alive_counter, 4U);
}

/* TEST: can_tx_fsm_state with NULL parameters (line 396) */
TEST(test_tx_fsm_state_null_params)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };

    int32_t rc = can_tx_fsm_state(NULL, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);

    rc = can_tx_fsm_state(&state, NULL);
    ASSERT_EQ(rc, CAN_ERR_TX);
}

/* TEST: can_tx_uds_response with NULL parameter (line 500) */
TEST(test_tx_uds_response_null)
{
    int32_t rc = can_tx_uds_response(NULL);
    ASSERT_EQ(rc, CAN_ERR_TX);
}

/* TEST: can_clear_uds_request_pending with NULL (line 535) */
TEST(test_clear_uds_request_pending_null)
{
    can_clear_uds_request_pending(NULL);
    ASSERT_EQ(1, 1);
}

/* TEST: encode_unsigned saturation at UINT32_MAX (line 95) */
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

/* TEST: can_tx_alert with NULL parameter (line 453, 455) */
TEST(test_tx_alert_null)
{
    int32_t rc = can_tx_alert(NULL);
    ASSERT_EQ(rc, CAN_ERR_TX);
}

/* TEST: can_check_timeout with NULL state (lines 300, 317) */
TEST(test_check_timeout_null)
{
    can_check_timeout(NULL);
    ASSERT_EQ(1, 1);
}
/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: Trigger decode_unsigned via can_rx_process
 *  decode_unsigned is called indirectly when receiving messages
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_decode_unsigned_via_rx)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* EgoVehicle frame - decodes vehicle_speed, etc. */
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 5000U);
    can_pack_signal(frame, 16U, 16U, 30000U);
    
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);
    
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    
    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 50.0F, 0.1F);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_init HAL failure branch
 *  Force HAL initialization failure
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_tx_brake_cmd alive_counter wrap - verify zero
 *  Verify that when wrap occurs, the value returns to 0
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_brake_cmd_alive_counter_wrap_to_zero)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };

    for (uint8_t i = 0U; i < 16U; i++)
    {
        (void)can_tx_brake_cmd(&state, &pid, &fsm);
    }
    
    ASSERT_EQ(state.alive_counter, 0U);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_tx_uds_response HAL send failure
 *  Force UDS response send failure
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_uds_response_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    
    can_hal_test_force_send_fail(1);
    
    uds_response_t resp;
    resp.response_sid   = 0x62U;
    resp.did_high_resp  = 0xF1U;
    resp.did_low_resp   = 0x01U;
    resp.data1          = 0x00U;
    resp.data2          = 0x00U;
    resp.data3          = 0x00U;
    resp.data4          = 0x00U;
    resp.data5          = 0x00U;

    int32_t rc = can_tx_uds_response(&resp);
    ASSERT_EQ(rc, CAN_ERR_TX);
    
    can_hal_test_force_send_fail(0);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_clear_uds_request_pending with valid state
 *  Verify that the flag is cleared correctly
 * ═══════════════════════════════════════════════════════════════════════ */
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST: can_tx_alert HAL send failure
 *  Force alert send failure
 * ═══════════════════════════════════════════════════════════════════════ */
TEST(test_tx_alert_send_failure)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);
    
    can_hal_test_force_send_fail(1);
    
    alert_output_t alert = { .alert_type = 3U, .alert_active = 1U, .buzzer_cmd = 4U };
    int32_t rc = can_tx_alert(&alert);
    ASSERT_EQ(rc, CAN_ERR_TX);
    
    can_hal_test_force_send_fail(0);
}

/* ================================================================
 * TEST: All brake_mode switch cases in can_tx_brake_cmd
 * ================================================================ */
TEST(test_brake_mode_all_cases)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 50.0F, .brake_bar = 5.0F };
    fsm_output_t fsm;

    /* Test FSM_OFF */
    fsm.fsm_state = (uint8_t)FSM_OFF;
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    /* Test FSM_STANDBY */
    fsm.fsm_state = (uint8_t)FSM_STANDBY;
    rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    /* Test FSM_WARNING */
    fsm.fsm_state = (uint8_t)FSM_WARNING;
    rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    /* Test FSM_BRAKE_L2 */
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L2;
    rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    /* Test FSM_POST_BRAKE */
    fsm.fsm_state = (uint8_t)FSM_POST_BRAKE;
    rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    /* Test invalid (default case) */
    fsm.fsm_state = 0xFFU;
    rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
}

/* ================================================================
 * TEST: All ttc_thresh switch cases in can_tx_fsm_state
 * ================================================================ */
TEST(test_ttc_thresh_all_cases)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    fsm_output_t fsm;

    /* Force send by setting tx_cycle_counter high */
    state.tx_cycle_counter = 5U;

    /* Test FSM_BRAKE_L1 */
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L1;
    int32_t rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    state.tx_cycle_counter = 5U;
    /* Test FSM_BRAKE_L2 */
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L2;
    rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    state.tx_cycle_counter = 5U;
    /* Test FSM_BRAKE_L3 */
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L3;
    rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    state.tx_cycle_counter = 5U;
    /* Test invalid (default case) */
    fsm.fsm_state = 0xFFU;
    rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_OK);
}

/* ================================================================
 * TEST: can_tx_fsm_state HAL send failure branch 
 * ================================================================ */
TEST(test_tx_fsm_state_hal_send_failure_branch)
{
    can_state_t state;
    can_hal_test_reset();
    can_hal_test_force_send_fail(1);
    (void)can_init(&state);

    /* Force send by setting tx_cycle_counter high */
    state.tx_cycle_counter = 5U;

    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };
    int32_t rc = can_tx_fsm_state(&state, &fsm);
    ASSERT_EQ(rc, CAN_ERR_TX);

    can_hal_test_force_send_fail(0);
}

/* ================================================================
 * TEST: encode_unsigned with raw_f < 0 (negative) 
 * ================================================================ */
TEST(test_encode_unsigned_negative_raw_f)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* Brake_bar negative enough to make raw_f < 0 */
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = -1000.0F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    /* Negative raw_f should result in 0 (no saturation, just clamp) */
    ASSERT_EQ(brake_pressure, 0U);
}

/* ================================================================
 * TEST: encode_unsigned with physical = NaN 
 * isfinite(physical) returns false
 * ================================================================ */
TEST(test_encode_unsigned_physical_nan)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* brake_bar = NaN already tested in test_tx_brake_cmd_nan_brake_bar */
    /* This test is already covered by test_tx_brake_cmd_nan_brake_bar */
    /* No need to add duplicate */
}

/* ================================================================
 * TEST: encode_unsigned with raw_f = Inf 
 * ================================================================ */
TEST(test_encode_unsigned_raw_f_inf)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* Use a value that will cause raw_f to overflow to Inf */
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = 1.0e308F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    /* Inf raw_f should result in 0 */
    ASSERT_EQ(brake_pressure, 0U);
}

/* ================================================================
 * TEST: can_tx_brake_cmd with brake_pct = 0 (branch coverage)
 * ================================================================ */
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
/* ================================================================
 * TEST: can_rx_process with state != NULL and data == NULL
 * (line 202 - data != NULL condition)
 * ================================================================ */
TEST(test_can_rx_process_data_null)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    uint8_t frame[8] = {0};
    
    /* data == NULL should be handled safely */
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, NULL, 8U);
    
    /* No crash means test passes */
    can_rx_data_t rx;
    can_get_rx_data(&state, &rx);
    /* Values should remain unchanged */
    ASSERT_FLOAT_NEAR(rx.vehicle_speed, 0.0F, 0.01F);
}

/* ================================================================
 * TEST: can_rx_process with state == NULL and data != NULL
 * (line 202 - state != NULL condition)
 * ================================================================ */
TEST(test_can_rx_process_state_null)
{
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0U, 16U, 1389U);
    
    /* state == NULL should not crash */
    can_rx_process(NULL, CAN_ID_EGO_VEHICLE, frame, 8U);
    
    /* No crash means test passes */
    ASSERT_EQ(1, 1);
}
/* ================================================================
 * TEST: encode_unsigned with physical = Inf 
 * isfinite(physical) returns false
 * ================================================================ */
TEST(test_encode_unsigned_physical_inf)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* brake_bar = Infinity */
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = INFINITY };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    /* When physical is not finite, encode_unsigned returns 0 */
    ASSERT_EQ(brake_pressure, 0U);
}

/* ================================================================
 * TEST: encode_unsigned with raw_f = NaN 
 * isfinite(raw_f) returns false
 * ================================================================ */
TEST(test_encode_unsigned_raw_f_nan)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* Use value that causes raw_f to become NaN */
    /* brake_bar = NaN already covered by test_tx_brake_cmd_nan_brake_bar */
    /* This is just for completeness */
    pid_output_t pid = { .brake_pct = 0.0F, .brake_bar = NAN };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);

    const tx_record_t *tx = can_hal_test_get_tx(0U);
    uint32_t brake_pressure = can_unpack_signal(tx->data, 1U, 15U);
    ASSERT_EQ(brake_pressure, 0U);
}

/* ================================================================
 * TEST: can_tx_brake_cmd with brake_pct > 0.0F 
 * line 340 - ternary operator true branch
 * ================================================================ */
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

/* ================================================================
 * TEST: can_tx_brake_cmd with alive_counter > ALIVE_COUNTER_MAX 
 * trigger the wrap to zero branch
 * ================================================================ */
TEST(test_tx_brake_cmd_alive_counter_wrap_branch)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };

    /* Set alive_counter to max (15) */
    state.alive_counter = 15U;
    
    int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(rc, CAN_OK);
    
    /* After incrementing from 15, should wrap to 0 */
    ASSERT_EQ(state.alive_counter, 0U);
}
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

/* ═══════════════════════════════════════════════════════════════════════
 *  TESTES DE RASTREABILIDADE
 * ═══════════════════════════════════════════════════════════════════════ */

/** @req FR-CAN-003: Verificação de Empacotamento de Sinais */
TEST(test_signal_roundtrip) {
    uint8_t buf[8] = {0};
    can_pack_signal(buf, 4U, 16U, 0xABCDU);
    ASSERT_EQ(can_unpack_signal(buf, 4U, 16U), 0xABCDU);
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 0U, 1U, 1U);
    ASSERT_EQ(can_unpack_signal(buf, 0U, 1U), 1U);
}

/** @req FR-CAN-004: Inicialização do Driver e Filtros RX */
TEST(test_init_scenarios) {
    can_state_t state;
    can_hal_test_reset();
    ASSERT_EQ(can_init(&state), CAN_OK);
    ASSERT_EQ(state.initialised, 1U);

    can_hal_test_force_init_fail(1);
    ASSERT_EQ(can_init(&state), CAN_ERR_INIT);
    can_hal_test_force_init_fail(0);
}

/** @req FR-CAN-002: Processamento de Mensagens EgoVehicle e Radar */
TEST(test_rx_decode_logic) {
    can_state_t state;
    can_init(&state);
    uint8_t frame[8] = {0};

    /* EgoVehicle (0x100) */
    can_pack_signal(frame, 0U, 16U, 1389U); 
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, frame, 8U);
    ASSERT_FLOAT_NEAR(state.last_rx.vehicle_speed, 13.89F, 0.02F);

    /* RadarTarget (0x120) - Reset miss_count */
    state.rx_miss_count = 5U;
    can_rx_process(&state, CAN_ID_RADAR_TARGET, frame, 8U);
    ASSERT_EQ(state.rx_miss_count, 0U);
}

/** @req FR-CAN-003: Transmissão AEB_BrakeCmd com CRC e Alive Counter */
TEST(test_tx_brake_cmd_complete) {
    can_state_t state;
    can_init(&state);
    pid_output_t pid = { .brake_pct = 75.0F, .brake_bar = 7.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L3 };

    ASSERT_EQ(can_tx_brake_cmd(&state, &pid, &fsm), CAN_OK);
    ASSERT_EQ(state.alive_counter, 1U);

    /* Teste de saturação e robustez (MC/DC em encode_unsigned) */
    pid.brake_bar = 1e20F; /* Finite > ceiling */
    ASSERT_EQ(can_tx_brake_cmd(&state, &pid, &fsm), CAN_OK);
    pid.brake_bar = -10.0F; /* Negative raw_f */
    ASSERT_EQ(can_tx_brake_cmd(&state, &pid, &fsm), CAN_OK);
    pid.brake_bar = NAN;
    ASSERT_EQ(can_tx_brake_cmd(&state, &pid, &fsm), CAN_OK);
}

/** @req FR-CAN-001: Periodicidade de AEB_FSMState (50ms) */
TEST(test_tx_fsm_periodicity) {
    can_state_t state;
    can_init(&state);
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_WARNING };
    for (int i = 0; i < 4; i++) ASSERT_EQ(can_tx_fsm_state(&state, &fsm), 1);
    ASSERT_EQ(can_tx_fsm_state(&state, &fsm), CAN_OK);
}

/** @req FR-UDS-005: Resposta UDS e Gerenciamento de Flag */
TEST(test_uds_logic) {
    can_state_t state;
    can_init(&state);
    uint8_t frame[4] = { 0x22, 0xF1, 0x01, 0x00 };
    
    can_rx_process(&state, CAN_ID_UDS_REQUEST, frame, 4);
    ASSERT_EQ(state.last_rx.uds_request_pending, 1U);
    
    can_clear_uds_request_pending(&state);
    ASSERT_EQ(state.last_rx.uds_request_pending, 0U);

    uds_response_t resp = { .response_sid = 0x62 };
    ASSERT_EQ(can_tx_uds_response(&resp), CAN_OK);
}

/** Cobertura de Segurança: Null Guards e Timeouts */
TEST(test_safety_guards) {
    can_state_t state;
    can_init(&state);

    /* MC/DC: if ((state != NULL) && (data != NULL)) */
    can_rx_process(NULL, 0x100, NULL, 0);
    can_rx_process(&state, 0x100, NULL, 0);
    can_check_timeout(NULL);
    can_clear_uds_request_pending(NULL);
    can_get_rx_data(NULL, NULL);

    /* Timeout logic (FR-CAN-002) */
    for (int i = 0; i < 7; i++) can_check_timeout(&state);
    ASSERT_EQ(state.last_rx.rx_timeout_flag, 1U);
}

/** Cobertura de Ramificação: Switches e Falhas de HAL */
TEST(test_branches_and_failures) {
    can_state_t state;
    can_init(&state);
    pid_output_t pid = { .brake_pct = 0.0F };
    fsm_output_t fsm;

    /* Todos os cases de BrakeMode e TTCThreshold */
    uint8_t modes[] = {FSM_OFF, FSM_STANDBY, FSM_WARNING, FSM_BRAKE_L1, FSM_BRAKE_L2, FSM_BRAKE_L3, FSM_POST_BRAKE, 0xFF};
    for(int i=0; i<8; i++) {
        fsm.fsm_state = modes[i];
        (void)can_tx_brake_cmd(&state, &pid, &fsm);
        state.tx_cycle_counter = 5;
        (void)can_tx_fsm_state(&state, &fsm);
    }

    /* Wrap do Alive Counter */
    state.alive_counter = ALIVE_COUNTER_MAX;
    (void)can_tx_brake_cmd(&state, &pid, &fsm);
    ASSERT_EQ(state.alive_counter, 0U);

    /* Falhas de HAL */
    can_hal_test_force_send_fail(1);
    ASSERT_EQ(can_tx_alert(&fsm), CAN_ERR_TX); /* Usando fsm como dummy alert */
    ASSERT_EQ(can_tx_uds_response(NULL), CAN_ERR_TX);
    can_hal_test_force_send_fail(0);
}
/* ═══════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("=== AEB CAN Module — Unit Tests ===\n\n");

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
    RUN(test_tx_brake_cmd_nan_brake_bar);
    RUN(test_tx_brake_cmd_pos_inf_brake_bar);
    RUN(test_tx_brake_cmd_neg_inf_brake_bar);
    RUN(test_tx_brake_cmd_overflow_brake_bar);
    RUN(test_tx_fsm_period);
    RUN(test_tx_send_failure);
    RUN(test_rx_uds_request);
    RUN(test_rx_uds_request_short_dlc);
    RUN(test_tx_uds_response);
    RUN(test_uds_request_ack);
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
    RUN(test_tx_brake_cmd_brake_pct_zero);
    RUN(test_can_rx_process_data_null);
    RUN(test_can_rx_process_state_null);
    RUN(test_encode_unsigned_physical_inf);
    RUN(test_encode_unsigned_raw_f_nan);
    RUN(test_tx_brake_cmd_brake_pct_positive);
    RUN(test_tx_brake_cmd_alive_counter_wrap_branch);
    RUN(test_signal_roundtrip);
    RUN(test_init_scenarios);
    RUN(test_rx_decode_logic);
    RUN(test_tx_brake_cmd_complete);
    RUN(test_tx_fsm_periodicity);
    RUN(test_uds_logic);
    RUN(test_safety_guards);
    RUN(test_branches_and_failures);

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}
