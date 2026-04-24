/**
 * @file    aeb_core.c
 * @brief   AEB system core orchestrator — 10 ms execution cycle.
 * @module  Integration — module pipeline
 * @version 1.0
 * @date    2026-04-10
 *
 * @details Implements the main AEB execution loop that wires all
 *          modules in the correct data-flow order:
 *
 *          CAN RX check -> Perception -> TTC -> Override -> FSM
 *                       -> PID + Alert -> UDS monitor -> UDS request
 *                       -> CAN TX
 *
 *          This file contains no control logic of its own — it only
 *          calls module APIs in sequence and routes structs between them.
 *
 * @requirements FR-COD-001, FR-COD-003, NFR-PERF-001, NFR-POR-003
 * @standard MISRA C:2012 compliant.
 */

#include "aeb_core.h"
#include "aeb_perception.h"
#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_pid.h"
#include "aeb_alert.h"
#include "aeb_config.h"

#include <string.h>

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

int32_t aeb_core_init(aeb_core_state_t *state)
{
    int32_t rc = 0;

    /* Zero all shared state */
    (void)memset(state, 0, sizeof(aeb_core_state_t));

    /* Initialise each module */
    perception_init();
    pid_init();
    fsm_init(&state->fsm);

    rc = can_init(&state->can);
    if (rc != CAN_OK)
    {
        /* CAN init failure — caller should handle */
    }
    else
    {
        uds_init(&state->uds);

        /* Default: AEB enabled at power-on */
        state->driver.aeb_enabled = 1U;
    }

    return rc;
}

void aeb_core_step(aeb_core_state_t *state,
                   const raw_sensor_input_t *raw)
{
    can_rx_data_t can_rx = {0};

    /* ── 1. CAN RX timeout check ──────────────────────────────────────── */
    can_check_timeout(&state->can);

    /* ── 2. Populate driver inputs from CAN RX data ───────────────────── */
    can_get_rx_data(&state->can, &can_rx);
    state->driver.brake_pedal    = can_rx.brake_pedal;
    state->driver.accel_pedal    = can_rx.accel_pedal;
    state->driver.steering_angle = can_rx.steering_angle;
    state->driver.aeb_enabled    = can_rx.aeb_enable;

    /* ── 3. Perception: sensor fusion + fault detection ───────────────── */
    perception_step(raw, &state->perception);

    /* Inject CAN timeout into perception fault flag */
    if (can_rx.rx_timeout_flag != 0U)
    {
        state->perception.fault_flag |= AEB_FAULT_CAN_TO;
    }

    /* ── 4. TTC and braking distance calculation ──────────────────────── */
    ttc_process(&state->perception, &state->ttc);

    /* ── 5. Driver override detection ─────────────────────────────────── */
    state->override_active = override_detect(
        state->driver.steering_angle,
        state->driver.brake_pedal);

    /* Feed override into driver struct for FSM consumption */
    /* Note: FSM uses driver_input_t which already has brake_pedal
     * and steering_angle. The override_active is passed via
     * brake_pedal being non-zero OR steering > 5 deg, which the
     * FSM evaluates independently. No extra field needed. */

    /* ── 6. FSM state evaluation ──────────────────────────────────────── */
    fsm_step(AEB_DT,
             &state->perception,
             &state->driver,
             &state->ttc,
             &state->fsm);

    /* ── 7. PID brake controller ──────────────────────────────────────── */
    pid_brake_step(state->fsm.decel_target,
                   0.0F,  /* a_ego: not available from CAN in this build */
                   state->fsm.fsm_state,
                   &state->pid);

    /* ── 8. Alert mapping ─────────────────────────────────────────────── */
    alert_map(state->fsm.fsm_state, &state->alert);

    /* ── 9. UDS fault monitoring ──────────────────────────────────────── */
    uds_monitor_faults(&state->uds,
                       state->perception.fault_flag,
                       0U,   /* crc_error: checked by CAN layer */
                       0U);  /* actuator_fault: not modelled */

    /* ── 10. UDS request service — FR-UDS-005 (same-cycle response) ───── */
    if (can_rx.uds_request_pending != 0U)
    {
        uds_response_t uds_resp = {0};

        uds_process_request(&state->uds,
                            &can_rx.uds_request,
                            &uds_resp,
                            &state->fsm,
                            &state->pid,
                            &state->ttc);
        (void)can_tx_uds_response(&uds_resp);
        can_ack_uds_request(&state->can);
    }

    /* ── 11. CAN TX ───────────────────────────────────────────────────── */
    (void)can_tx_brake_cmd(&state->can, &state->pid, &state->fsm);
    (void)can_tx_fsm_state(&state->can, &state->fsm);
    (void)can_tx_alert(&state->alert);
}
