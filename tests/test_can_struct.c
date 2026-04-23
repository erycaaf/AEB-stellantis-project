/**
 * @file  test_can_struct.c
 * @brief Structural (coverage-driven) complementary tests for aeb_can.c.
 *
 * Independent V&V author: Lourenço  (cross-validation of Renato's module).
 *
 * Purpose:
 *   The nominal suite test_can.c (Renato) already passes 100% of its
 *   requirements-based cases, but does not exercise every branch of
 *   three defensive switch statements:
 *
 *     - can_tx_brake_cmd:  FSM_state -> brake_mode mapping
 *           (FSM_OFF, FSM_STANDBY, FSM_WARNING, FSM_BRAKE_L2,
 *            FSM_POST_BRAKE, default fall-through)
 *
 *     - can_tx_fsm_state:  FSM_state -> TTC threshold mapping
 *           (FSM_BRAKE_L1, FSM_BRAKE_L2, FSM_BRAKE_L3, default)
 *
 *   These are legitimate defensive branches that must execute to claim
 *   100% statement / branch coverage for ASIL-D.  This file adds the
 *   minimum set of tests to traverse each branch exactly once.
 *
 *   This pattern (complementary structural tests after requirements-
 *   based tests fall short of 100%) is identical to the one documented
 *   in the PID/Alert V&V report referenced in the Delivery Strategy.
 *
 * @req NFR-COD-006  Statement coverage >= 80% (target for this project: 100%)
 * @req NFR-COD-007  Branch    coverage >= 60% (target for this project: 100%)
 *
 * Build:   part of make mcdc-can (merged with nominal + fault .gcda)
 *
 * @version 1.0
 * @date 2026-04
 */

#include "aeb_can.h"
#include "can_hal.h"
#include <stdio.h>
#include <string.h>

static int32_t struct_run    = 0;
static int32_t struct_passed = 0;

#define SASSERT(cond, label) do {                              \
    struct_run++;                                              \
    if (cond) { struct_passed++; printf("  PASS  %s\n", label); } \
    else      { printf("  FAIL  %s\n", label); }               \
} while (0)

extern void can_hal_test_reset(void);
extern void can_hal_test_force_send_fail(int32_t fail);

/* Shared tx_record_t shape (matches stubs/can_hal.c). */
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
} tx_record_t;
extern const tx_record_t *can_hal_test_get_tx(uint32_t index);

/* Helper: run one brake_cmd with the given FSM state and return the
 * packed BrakeMode signal (bits 16..18) from the emitted frame. */
static uint32_t emit_brake_mode_for(uint8_t fsm_state)
{
    can_state_t  state;
    pid_output_t pid = { .brake_pct = 1.0F, .brake_bar = 0.1F };
    fsm_output_t fsm = { .fsm_state = fsm_state };

    can_hal_test_reset();
    (void)can_init(&state);
    (void)can_tx_brake_cmd(&state, &pid, &fsm);
    const tx_record_t *rec = can_hal_test_get_tx(0);
    if (rec == NULL) { return 0xFFFFU; }
    return can_unpack_signal(rec->data, 16U, 3U);
}

/* Helper: same idea, for the FSMState frame.  The TTCThreshold field
 * sits at bits 24..31, factor 0.1; we return raw = ttc * 10 (rounded). */
static uint32_t emit_ttc_raw_for(uint8_t fsm_state)
{
    can_state_t  state;
    fsm_output_t fsm = { .fsm_state = fsm_state };

    can_hal_test_reset();
    (void)can_init(&state);
    /* FSMState only emits every 5 ticks — drive 5 ticks. */
    int32_t rc = 0;
    for (uint8_t i = 0U; i < 5U; i++) {
        rc = can_tx_fsm_state(&state, &fsm);
    }
    if (rc != CAN_OK) { return 0xFFFFU; }
    const tx_record_t *rec = can_hal_test_get_tx(0);
    if (rec == NULL) { return 0xFFFFU; }
    return can_unpack_signal(rec->data, 24U, 8U);
}

int main(void)
{
    printf("=== Structural complementary tests for aeb_can.c ===\n");

    printf("\n--- BrakeMode switch (can_tx_brake_cmd) ---\n");
    /* Expected brake_mode values per the source-of-truth table in aeb_can.c. */
    SASSERT(emit_brake_mode_for((uint8_t)FSM_OFF)        == 0U, "brake_mode(FSM_OFF)=0");
    SASSERT(emit_brake_mode_for((uint8_t)FSM_STANDBY)    == 0U, "brake_mode(FSM_STANDBY)=0");
    SASSERT(emit_brake_mode_for((uint8_t)FSM_WARNING)    == 1U, "brake_mode(FSM_WARNING)=1");
    SASSERT(emit_brake_mode_for((uint8_t)FSM_BRAKE_L2)   == 3U, "brake_mode(FSM_BRAKE_L2)=3");
    SASSERT(emit_brake_mode_for((uint8_t)FSM_POST_BRAKE) == 5U, "brake_mode(FSM_POST_BRAKE)=5");
    SASSERT(emit_brake_mode_for(0xFFU)                   == 0U, "brake_mode(invalid=0xFF)=0 (default)");

    printf("\n--- TTC threshold switch (can_tx_fsm_state) ---\n");
    /* TTC thresholds from aeb_config.h: L1=3.0, L2=2.2, L3=1.8; default=0.
     * Raw = round(ttc / 0.1) so raw(3.0)=30, raw(2.2)=22, raw(1.8)=18. */
    SASSERT(emit_ttc_raw_for((uint8_t)FSM_BRAKE_L1) == 30U, "ttc_raw(FSM_BRAKE_L1)=30");
    SASSERT(emit_ttc_raw_for((uint8_t)FSM_BRAKE_L2) == 22U, "ttc_raw(FSM_BRAKE_L2)=22");
    SASSERT(emit_ttc_raw_for((uint8_t)FSM_BRAKE_L3) == 18U, "ttc_raw(FSM_BRAKE_L3)=18");
    SASSERT(emit_ttc_raw_for((uint8_t)FSM_OFF)      ==  0U, "ttc_raw(FSM_OFF)=0 (default)");

    printf("\n--- HAL failure branch in can_tx_fsm_state ---\n");
    /* The nominal suite covers the HAL-fail branch of can_tx_brake_cmd
     * (test_tx_send_failure) but not of can_tx_fsm_state.  Drive 5 ticks
     * with force_send_fail=1 and expect CAN_ERR_TX on the 5th call. */
    {
        can_state_t  state2;
        fsm_output_t fsm2 = { .fsm_state = (uint8_t)FSM_BRAKE_L1 };
        can_hal_test_reset();
        (void)can_init(&state2);
        can_hal_test_force_send_fail(1);
        int32_t rc2 = 0;
        for (uint8_t i = 0U; i < 5U; i++) {
            rc2 = can_tx_fsm_state(&state2, &fsm2);
        }
        can_hal_test_force_send_fail(0);
        SASSERT(rc2 == CAN_ERR_TX, "can_tx_fsm_state: HAL fail -> CAN_ERR_TX");
    }

    printf("\n--- DLC-too-short branch for DriverInput (0x101) ---\n");
    /* can_rx_process has a DLC guard for each ID.  The nominal suite only
     * tested the EgoVehicle / RadarTarget guards; this drives the
     * DriverInput (0x101) guard with dlc < CAN_DLC_DRIVER_INPUT (4). */
    {
        can_state_t  state3;
        can_hal_test_reset();
        (void)can_init(&state3);
        uint8_t short_di[2] = { 0x55U, 0x55U };
        can_rx_process(&state3, CAN_ID_DRIVER_INPUT, short_di, 2U);
        SASSERT(state3.last_rx.brake_pedal == 0U,
                "rx(DriverInput, dlc=2<4) -> no decode");
    }

    printf("\n--- brake_pct == 0 branch in can_tx_brake_cmd ---\n");
    /* The ternary `(brake_pct > 0.0F) ? 1U : 0U` has only been tested
     * with brake_pct > 0.  This drives brake_pct == 0 (and the resulting
     * BrakeRequest=0 path). */
    {
        can_state_t  state4;
        pid_output_t pid_zero = { .brake_pct = 0.0F, .brake_bar = 0.0F };
        fsm_output_t fsm_std  = { .fsm_state = (uint8_t)FSM_STANDBY };
        can_hal_test_reset();
        (void)can_init(&state4);
        int32_t rc4 = can_tx_brake_cmd(&state4, &pid_zero, &fsm_std);
        const tx_record_t *rec4 = can_hal_test_get_tx(0);
        uint32_t brake_req = (rec4 != NULL)
            ? can_unpack_signal(rec4->data, 0U, 1U) : 0xFFU;
        SASSERT((rc4 == CAN_OK) && (brake_req == 0U),
                "tx_brake_cmd(brake_pct=0) -> BrakeRequest=0");
    }

    printf("\n=== Structural summary: %d tests, %d passed ===\n",
           struct_run, struct_passed);
    return (struct_passed == struct_run) ? 0 : 1;
}
