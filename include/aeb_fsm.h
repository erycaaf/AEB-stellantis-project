/**
 * @file aeb_fsm.h
 * @brief 7-state Autonomous Emergency Braking Finite State Machine.
 * @author Lourenço Jamba Mphili
 * @requirements FR-FSM-001 to FR-FSM-006, FR-DEC-004 to FR-DEC-011
 */

#ifndef AEB_FSM_H
#define AEB_FSM_H

#include <stdint.h>
#include "aeb_types.h"
#include "aeb_config.h"

/**
 * @brief Initialize FSM internal state.
 * @param fsm_out Pointer to FSM output structure
 * @requirement FR-FSM-001 (Initial state = STANDBY)
 */
void fsm_init(fsm_output_t * const fsm_out);

/**
 * @brief Execute one step of the AEB FSM.
 * @param delta_t_s Time elapsed since last call [s] (typically AEB_DT = 0.01s)
 * @param perception Input from perception module
 * @param driver Input from driver inputs
 * @param ttc_in Precomputed TTC results
 * @param fsm_out Current/next FSM state (updated in-place)
 * @requirement FR-FSM-002, FR-FSM-003, FR-FSM-004, FR-FSM-005, FR-FSM-006
 */
void fsm_step(float32_t delta_t_s,
              const perception_output_t * const perception,
              const driver_input_t * const driver,
              const ttc_output_t * const ttc_in,
              fsm_output_t * const fsm_out);

#endif /* AEB_FSM_H */