/**
 * @file  test_can_fault.c
 * @brief Systematic fault injection for the AEB CAN module.
 *
 * Independent V&V author: @mphumapo2014  (cross-validation).
 * Original module author: @renatosfagundes  (aeb_can.{c,h}, test_can.c)
 *
 * Methodology:  ISO 26262-6:2018 Table 11 item 1e ("highly recommended"
 *               for ASIL-D) — fault injection / error guessing.
 *
 * Structure mirrors the UDS V&V fault-injection template (see tests/test_uds_fault.c):
 *   Category A — invalid values (NULL pointers, bad DLC, unknown ID, HAL fault)
 *   Category B — numeric extremes (NaN/Inf/negative in physical-to-raw,
 *                raw values exceeding the signal bit-width)
 *   Category C — state corruption (SEU: alive_counter, rx_miss_count,
 *                tx_cycle_counter, initialised flag set to garbage values)
 *   Category D — persistence (timeout recovery cycles, rapid force-fail
 *                toggles, TX buffer burst)
 *
 * The fail-safe predicate applied to categories A/B is:
 *   "Under abnormal input, the function must return an error code
 *    (CAN_ERR_TX / CAN_ERR_INIT) OR produce a frame whose packed raw
 *    value is bounded within the declared signal width. Any response
 *    that (i) dereferences a NULL pointer, (ii) writes bits outside
 *    the target signal, or (iii) transmits a frame with
 *    implementation-defined data is NOT fail-safe."
 *
 * @req NFR-SAF-ROB  Robustness of CAN module under invalid / corrupted
 *                   input.  Traces to ISO 26262-6 Table 11 item 1e.
 *
 * Build:   make fault-can
 * Artefacts: reports/vv_can/fault_injection/run.log
 *
 * @version 1.0
 * @date 2026-04
 */

#include "aeb_can.h"
#include "can_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ── Minimal assertion framework (pattern matches test_can.c) ─────────── */
static int32_t faults_run     = 0;
static int32_t faults_passed  = 0;
static int32_t faults_failed  = 0;

#define FAULT_ASSERT(cond, label) do {                                    \
    faults_run++;                                                         \
    if (cond) {                                                           \
        faults_passed++;                                                  \
        printf("  PASS  %s\n", label);                                    \
    } else {                                                              \
        faults_failed++;                                                  \
        printf("  FAIL  %s  (%s:%d)\n", label, __FILE__, __LINE__);       \
    }                                                                     \
} while (0)

#define CATEGORY(letter, title) \
    printf("\n--- Category %s: %s ---\n", letter, title)

/* ── HAL test helpers (stub exports) ──────────────────────────────────── */
extern uint32_t can_hal_test_get_tx_count(void);
extern void     can_hal_test_reset(void);
extern void     can_hal_test_force_init_fail(int32_t fail);
extern void     can_hal_test_force_send_fail(int32_t fail);

/* Opaque tx_record_t from stub (we only inspect via pointer). */
typedef struct { uint32_t id; uint8_t data[8]; uint8_t dlc; } tx_record_t;
extern const tx_record_t *can_hal_test_get_tx(uint32_t index);

/* ═════════════════════════════════════════════════════════════════════════
 *  CATEGORY A — invalid values
 *
 *  Rationale: any public function must reject malformed calls without
 *  dereferencing garbage.  Most of these probe the NULL guards; the
 *  DLC-too-short cases probe the length checks in can_rx_process().
 * ═════════════════════════════════════════════════════════════════════════ */

static void cat_A_null_pointers(void)
{
    can_state_t    state;
    pid_output_t   pid = { .brake_pct = 10.0F, .brake_bar = 1.0F };
    fsm_output_t   fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };

    can_hal_test_reset();
    (void)can_init(&state);

    /* A1: can_tx_brake_cmd with NULL state */
    FAULT_ASSERT(can_tx_brake_cmd(NULL, &pid, &fsm) == CAN_ERR_TX,
                 "A1  brake_cmd(NULL,pid,fsm) -> CAN_ERR_TX");

    /* A2: can_tx_brake_cmd with NULL pid */
    FAULT_ASSERT(can_tx_brake_cmd(&state, NULL, &fsm) == CAN_ERR_TX,
                 "A2  brake_cmd(state,NULL,fsm) -> CAN_ERR_TX");

    /* A3: can_tx_brake_cmd with NULL fsm */
    FAULT_ASSERT(can_tx_brake_cmd(&state, &pid, NULL) == CAN_ERR_TX,
                 "A3  brake_cmd(state,pid,NULL) -> CAN_ERR_TX");

    /* A4: can_tx_fsm_state with NULL state */
    FAULT_ASSERT(can_tx_fsm_state(NULL, &fsm) == CAN_ERR_TX,
                 "A4  fsm_state(NULL,fsm) -> CAN_ERR_TX");

    /* A5: can_tx_fsm_state with NULL fsm */
    FAULT_ASSERT(can_tx_fsm_state(&state, NULL) == CAN_ERR_TX,
                 "A5  fsm_state(state,NULL) -> CAN_ERR_TX");

    /* A6: can_tx_alert with NULL alert */
    FAULT_ASSERT(can_tx_alert(NULL) == CAN_ERR_TX,
                 "A6  alert(NULL) -> CAN_ERR_TX");

    /* A7: can_rx_process with NULL state must not crash */
    uint8_t frame[8] = {0};
    can_rx_process(NULL, CAN_ID_EGO_VEHICLE, frame, 8U);
    FAULT_ASSERT(1, "A7  rx_process(NULL,id,data,dlc) -> no crash");

    /* A8: can_rx_process with NULL data must not crash */
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, NULL, 8U);
    FAULT_ASSERT(1, "A8  rx_process(state,id,NULL,dlc) -> no crash");

    /* A9: can_check_timeout with NULL must not crash */
    can_check_timeout(NULL);
    FAULT_ASSERT(1, "A9  check_timeout(NULL) -> no crash");

    /* A10: can_get_rx_data with NULL state must not crash */
    can_rx_data_t dst = {0};
    can_get_rx_data(NULL, &dst);
    FAULT_ASSERT(1, "A10 get_rx_data(NULL,dst) -> no crash");

    /* A11: can_get_rx_data with NULL dst must not crash */
    can_get_rx_data(&state, NULL);
    FAULT_ASSERT(1, "A11 get_rx_data(state,NULL) -> no crash");
}

static void cat_A_invalid_frames(void)
{
    can_state_t state;
    can_hal_test_reset();
    (void)can_init(&state);

    /* A12: DLC shorter than required for EgoVehicle (8) — must be silently
     *      ignored (no decode, no rx_miss_count reset). */
    uint8_t short_frame[4] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU };
    can_rx_process(&state, CAN_ID_EGO_VEHICLE, short_frame, 4U);
    FAULT_ASSERT(state.last_rx.vehicle_speed == 0.0F,
                 "A12 rx(EgoVehicle, dlc=4<8) -> no decode");

    /* A13: DLC = 0 on a valid ID — silent */
    can_rx_process(&state, CAN_ID_RADAR_TARGET, short_frame, 0U);
    FAULT_ASSERT(state.last_rx.target_distance == 0.0F,
                 "A13 rx(RadarTarget, dlc=0) -> no decode");

    /* A14: unknown CAN ID — must be silently ignored. */
    uint8_t frame8[8] = { 0xAAU, 0xBBU, 0xCCU, 0xDDU, 0xEEU, 0xFFU, 0x11U, 0x22U };
    uint32_t tx_before = can_hal_test_get_tx_count();
    can_rx_process(&state, 0x7FFU, frame8, 8U);
    FAULT_ASSERT(can_hal_test_get_tx_count() == tx_before,
                 "A14 rx(unknown ID) -> no TX side-effect");

    /* A15: HAL init fault surfaces as CAN_ERR_INIT, initialised stays 0. */
    can_hal_test_reset();
    can_hal_test_force_init_fail(1);
    can_state_t state2;
    (void)memset(&state2, 0xA5, sizeof(state2));  /* garbage before init */
    int32_t rc = can_init(&state2);
    can_hal_test_force_init_fail(0);
    FAULT_ASSERT((rc == CAN_ERR_INIT) && (state2.initialised == 0U),
                 "A15 init with forced HAL failure -> CAN_ERR_INIT, initialised=0");
}

/* ═════════════════════════════════════════════════════════════════════════
 *  CATEGORY B — numeric extremes
 *
 *  The DBC encode path is:  raw = (physical - offset) / factor
 *  When `physical` is NaN, ±Inf, or extremely large, the float-to-uint
 *  cast in encode_unsigned() has *undefined behaviour* per C11 §6.3.1.4.
 *  Additionally, can_pack_signal() takes a 32-bit raw and shifts bit-by-bit
 *  into the frame: if the caller passes raw_value with bits above `length`,
 *  those should be discarded (current impl masks via shift, but the test
 *  verifies it doesn't corrupt adjacent signals).
 * ═════════════════════════════════════════════════════════════════════════ */

static void cat_B_encode_extremes(void)
{
    can_state_t  state;
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    pid_output_t pid;

    /* B1: brake_bar = NaN */
    can_hal_test_reset();
    (void)can_init(&state);
    pid.brake_pct = 10.0F;
    pid.brake_bar = NAN;
    int32_t rc1 = can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(rc1 == CAN_OK,
                 "B1  brake_bar=NaN -> returns, does not abort");

    /* B2: brake_bar = +Inf */
    can_hal_test_reset();
    (void)can_init(&state);
    pid.brake_bar = INFINITY;
    int32_t rc2 = can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(rc2 == CAN_OK,
                 "B2  brake_bar=+Inf -> returns, does not abort");

    /* B3: brake_bar = -Inf (negative branch of encode_unsigned clamps to 0) */
    can_hal_test_reset();
    (void)can_init(&state);
    pid.brake_bar = -INFINITY;
    int32_t rc3 = can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(rc3 == CAN_OK,
                 "B3  brake_bar=-Inf -> returns, does not abort");
    /* The packed 15-bit BrakePressure signal must be 0 after -Inf clamp. */
    const tx_record_t *rec = can_hal_test_get_tx(0);
    if (rec != NULL) {
        uint32_t raw_pressure = can_unpack_signal(rec->data, 1U, 15U);
        FAULT_ASSERT(raw_pressure == 0U,
                     "B3b packed BrakePressure after -Inf == 0 (clamped)");
    } else {
        FAULT_ASSERT(0, "B3b TX record missing");
    }

    /* B4: brake_bar large positive — raw overflows the 15-bit field.
     *     Spec: 15 bits can hold max 32767.  physical=100000 bar with
     *     factor 0.1 yields raw=1_000_000, which overflows uint15.
     *     Fail-safe: packed signal must stay within its 15-bit window
     *     (bits 16..31 must not be disturbed). */
    can_hal_test_reset();
    (void)can_init(&state);
    pid.brake_bar = 100000.0F;
    int32_t rc4 = can_tx_brake_cmd(&state, &pid, &fsm);
    rec = can_hal_test_get_tx(0);
    int32_t bits16_to_31_intact = 0;
    if (rec != NULL) {
        /* BrakeMode lives at bits 16..18.  Check that encode overflow
         * did NOT flip those bits away from the FSM_BRAKE_L1 value (=2). */
        uint32_t brake_mode = can_unpack_signal(rec->data, 16U, 3U);
        bits16_to_31_intact = (brake_mode == 2U) ? 1 : 0;
    }
    FAULT_ASSERT((rc4 == CAN_OK) && (bits16_to_31_intact == 1),
                 "B4  brake_bar=1e5 -> adjacent signals intact");

    /* B5: brake_bar = FLT_MAX — same overflow class, extreme magnitude. */
    can_hal_test_reset();
    (void)can_init(&state);
    pid.brake_bar = FLT_MAX;
    int32_t rc5 = can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(rc5 == CAN_OK,
                 "B5  brake_bar=FLT_MAX -> does not abort");

    /* B6: pack_signal with raw_value far above length — must not corrupt
     *     bits outside the target range.  We pack 0xFFFFFFFF into bits
     *     0..3 of an all-zero buffer and verify bits 4..63 stayed zero. */
    uint8_t buf[8];
    (void)memset(buf, 0, sizeof(buf));
    can_pack_signal(buf, 0U, 4U, 0xFFFFFFFFU);
    int32_t outside_zero = 1;
    if ((buf[0] & 0xF0U) != 0U) { outside_zero = 0; }
    for (uint8_t k = 1U; k < 8U; k++) {
        if (buf[k] != 0U) { outside_zero = 0; break; }
    }
    FAULT_ASSERT((buf[0] & 0x0FU) == 0x0FU && outside_zero == 1,
                 "B6  pack(0xFFFFFFFF, len=4) -> masked, no bleed");
}

/* ═════════════════════════════════════════════════════════════════════════
 *  CATEGORY C — state corruption (SEU simulation)
 *
 *  A single-event upset (cosmic ray / alpha particle) can flip bits in
 *  SRAM.  These tests write arbitrary values into module-private state
 *  fields and verify that the next public-API call behaves safely — it
 *  must not crash, hang, or emit undefined frames.  True resilience would
 *  require CRC-protected state (future safety mechanism); this test
 *  characterises today's behaviour.
 * ═════════════════════════════════════════════════════════════════════════ */

static void cat_C_state_corruption(void)
{
    can_state_t state;

    /* C1: alive_counter corrupted to 0xAA (> ALIVE_COUNTER_MAX of 15).
     *     Expected: next tx_brake_cmd resets it to 0 via the wrap-around
     *     branch `if (alive_counter > ALIVE_COUNTER_MAX)`. */
    can_hal_test_reset();
    (void)can_init(&state);
    state.alive_counter = 0xAAU;
    pid_output_t pid = { .brake_pct = 1.0F, .brake_bar = 0.1F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_STANDBY };
    (void)can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(state.alive_counter <= ALIVE_COUNTER_MAX,
                 "C1  alive_counter=0xAA -> bounded after next TX");

    /* C2: rx_miss_count corrupted to 0xFF — saturation clamp must hold. */
    can_hal_test_reset();
    (void)can_init(&state);
    state.rx_miss_count = 0xFFU;
    can_check_timeout(&state);
    FAULT_ASSERT(state.rx_miss_count == 0xFFU,
                 "C2  rx_miss_count=0xFF -> saturates at 0xFF (no wrap)");
    FAULT_ASSERT(state.last_rx.rx_timeout_flag == 1U,
                 "C2b rx_miss_count=0xFF -> timeout_flag=1");

    /* C3: tx_cycle_counter corrupted to UINT32_MAX.  Next call increments
     *     → wraps to 0 → below threshold → FSMState NOT sent.  This is
     *     not perfectly fail-safe (a SEU can delay a 50 ms transmission
     *     by up to 50 ms), but it must not crash or saturate. */
    can_hal_test_reset();
    (void)can_init(&state);
    state.tx_cycle_counter = 0xFFFFFFFFU;
    uint32_t tx_before = can_hal_test_get_tx_count();
    int32_t  rc = can_tx_fsm_state(&state, &fsm);
    uint32_t tx_after  = can_hal_test_get_tx_count();
    FAULT_ASSERT((rc == 1) && (tx_after == tx_before),
                 "C3  tx_cycle_counter=UINT32_MAX -> wraps safely, no TX");

    /* C4: initialised flag corrupted to 0x55.  No public API reads this
     *     field today, so the corruption is benign in current code.
     *     Document that and record as informational. */
    can_hal_test_reset();
    (void)can_init(&state);
    state.initialised = 0x55U;
    (void)can_tx_brake_cmd(&state, &pid, &fsm);
    FAULT_ASSERT(1,
                 "C4  initialised=0x55 -> no observable effect (informational)");
}

/* ═════════════════════════════════════════════════════════════════════════
 *  CATEGORY D — persistence
 *
 *  Sustained or repeated abnormal conditions and recovery scenarios.
 * ═════════════════════════════════════════════════════════════════════════ */

static void cat_D_persistence(void)
{
    can_state_t state;

    /* D1: Timeout → valid RX → timeout cycle.  After recovery the miss
     *     count and flag must both return to a clean state. */
    can_hal_test_reset();
    (void)can_init(&state);
    for (uint8_t i = 0U; i < 10U; i++) { can_check_timeout(&state); }
    FAULT_ASSERT(state.last_rx.rx_timeout_flag == 1U,
                 "D1a 10 ticks no RX -> timeout_flag=1");

    uint8_t radar_frame[8] = {0};
    can_pack_signal(radar_frame, 0U, 16U, 2500U);  /* 25.0 m */
    can_rx_process(&state, CAN_ID_RADAR_TARGET, radar_frame, 8U);
    FAULT_ASSERT((state.rx_miss_count == 0U) &&
                 (state.last_rx.rx_timeout_flag == 0U),
                 "D1b valid radar frame -> miss_count=0, flag cleared");

    /* D2: 100 rapid force_send_fail toggles — no inconsistency between
     *     the toggle state and the function's return code.
     *
     *     Stub artefact:  can_hal's TX capture buffer holds 32 frames.
     *     When it fills, can_hal_send returns -1 regardless of the
     *     force_send_fail toggle — that is a limitation of the test
     *     harness, NOT of aeb_can.c.  To probe only the CAN module we
     *     drain the TX buffer every 16 iterations via can_hal_test_reset,
     *     which is the test-harness equivalent of a real CAN controller
     *     consuming its transmit queue.  We keep the module state intact
     *     by NOT re-initialising; only the HAL buffers are cleared. */
    can_hal_test_reset();
    (void)can_init(&state);
    pid_output_t pid = { .brake_pct = 5.0F, .brake_bar = 0.5F };
    fsm_output_t fsm = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
    int32_t inconsistent = 0;
    for (uint32_t i = 0U; i < 100U; i++) {
        if ((i > 0U) && ((i % 16U) == 0U)) {
            can_hal_test_reset();            /* drain HAL TX buffer */
        }
        can_hal_test_force_send_fail((int32_t)(i & 1U));
        int32_t rc = can_tx_brake_cmd(&state, &pid, &fsm);
        int32_t expect_ok = ((i & 1U) == 0U) ? 1 : 0;
        int32_t got_ok    = (rc == CAN_OK) ? 1 : 0;
        if (expect_ok != got_ok) { inconsistent = 1; }
    }
    can_hal_test_force_send_fail(0);
    FAULT_ASSERT(inconsistent == 0,
                 "D2  100 send_fail toggles -> rc matches toggle state");

    /* D3: TX burst that fills the stub's 32-slot buffer.
     *     After the 33rd call the HAL returns -1 (buffer full) and
     *     can_tx_alert must surface that as CAN_ERR_TX. */
    can_hal_test_reset();
    (void)can_init(&state);
    alert_output_t alr = { .alert_type = 2U, .alert_active = 1U, .buzzer_cmd = 1U };
    int32_t first_fail_index = -1;
    for (int32_t i = 0; i < 40; i++) {
        int32_t rc = can_tx_alert(&alr);
        if ((rc == CAN_ERR_TX) && (first_fail_index < 0)) {
            first_fail_index = i;
        }
    }
    FAULT_ASSERT((first_fail_index >= 32) && (first_fail_index <= 35),
                 "D3  TX burst > buffer -> CAN_ERR_TX surfaces (expected ~32)");
}

/* ═════════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("=== AEB CAN Module — Fault Injection (ISO 26262-6 Tab.11) ===\n");

    CATEGORY("A", "Invalid values (NULL pointers, bad DLC, unknown ID, HAL fault)");
    cat_A_null_pointers();
    cat_A_invalid_frames();

    CATEGORY("B", "Numeric extremes (NaN, Inf, encode overflow, pack overflow)");
    cat_B_encode_extremes();

    CATEGORY("C", "State corruption (SEU on alive_counter, rx_miss, cycle_counter)");
    cat_C_state_corruption();

    CATEGORY("D", "Persistence (timeout recovery, 100 toggles, TX burst)");
    cat_D_persistence();

    printf("\n=== Fault injection summary: %d asserts, %d passed, %d failed ===\n",
           faults_run, faults_passed, faults_failed);

    /* Exit 0 iff all fail-safe predicates held.  A non-zero exit is a
     * signal to the CI that the module may have a robustness bug. */
    return (faults_failed > 0) ? 1 : 0;
}
