/**
 * @file    aeb_core.h
 * @brief   AEB system core orchestrator — public API.
 * @module  Integration — 10 ms execution cycle
 * @version 1.0
 * @date    2026-04-10
 *
 * @details Wires all AEB modules into a single 10 ms execution cycle:
 *          Perception -> TTC -> FSM -> PID + Alert + Override -> CAN TX.
 *          Also runs UDS fault monitoring each cycle.
 *
 * @requirements FR-COD-001, NFR-PERF-001, NFR-POR-003
 */

#ifndef AEB_CORE_H
#define AEB_CORE_H

#include "aeb_types.h"
#include "aeb_can.h"
#include "aeb_uds.h"

/**
 * @brief Persistent state for the AEB core orchestrator.
 *
 * Holds all inter-module data structs and module-internal state
 * that persists across execution cycles.
 */
typedef struct
{
    /* Module outputs (inter-module data flow) */
    perception_output_t perception;
    ttc_output_t        ttc;
    fsm_output_t        fsm;
    pid_output_t        pid;
    alert_output_t      alert;

    /* Module-internal persistent state */
    can_state_t         can;
    uds_state_t         uds;

    /* Driver inputs (populated from CAN RX) */
    driver_input_t      driver;

    /* Override detection result */
    uint8_t             override_active;
} aeb_core_state_t;

/**
 * @brief Initialise the AEB system.
 *
 * Calls all module init functions and zeroes shared state.
 * Must be called once at power-on before the first cycle.
 *
 * @param[out] state  Pointer to core state (must not be NULL).
 * @return 0 on success, non-zero on CAN init failure.
 */
int32_t aeb_core_init(aeb_core_state_t *state);

/**
 * @brief Execute one 10 ms AEB cycle.
 *
 * Pipeline order:
 *   1. CAN RX timeout check
 *   2. Perception (sensor fusion + fault detection)
 *   3. TTC and braking distance calculation
 *   4. Override detection
 *   5. FSM state evaluation
 *   6. PID brake controller
 *   7. Alert mapping
 *   8. UDS fault monitoring
 *   9. CAN TX (brake command, FSM state, alerts)
 *
 * @param[in,out] state  Pointer to core state.
 * @param[in]     raw    Raw sensor inputs for this cycle.
 */
void aeb_core_step(aeb_core_state_t *state,
                   const raw_sensor_input_t *raw);

#endif /* AEB_CORE_H */
