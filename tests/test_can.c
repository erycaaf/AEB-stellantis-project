/**
 * @file  test_can.c
 * @brief Unit tests for aeb_can module.
 *
 * Uses a minimal assert framework for host compilation.
 * Can be adapted to Zephyr ztest by replacing ASSERT macros.
 */

#include "aeb_can.h"
#include "can_hal.h"
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

/* ── Extern test helpers from can_hal.c stub ────────────────────────── */
extern uint32_t       can_hal_test_get_tx_count(void);
extern void           can_hal_test_reset(void);
extern void           can_hal_test_force_init_fail(int32_t fail);
extern void           can_hal_test_force_send_fail(int32_t fail);

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
    RUN(test_tx_fsm_period);
    RUN(test_tx_send_failure);

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}
