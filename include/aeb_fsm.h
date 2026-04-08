/**
 * @file    aeb_fsm.h
 * @brief   Finite State Machine module — public API.
 * @owner   Task B (Lourenço Jamba Mphili)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Implements the 7-state FSM controlling AEB intervention levels
 *          with hysteresis, distance floors, and driver override logic.
 *
 * @requirements FR-DEC-004 to FR-DEC-011, FR-FSM-001 to FR-FSM-006
 * @simulink     chart_58 (FSM_Logic)
 */

#ifndef AEB_FSM_H
#define AEB_FSM_H

#include "aeb_types.h"

/**
 * @brief Initialise FSM internal state.
 *
 * Sets initial state to FSM_OFF, clears all timers and flags.
 * Must be called once at system startup.
 */
void fsm_init(void);

/**
 * @brief Execute one FSM step.
 *
 * Evaluates state transitions based on TTC output, perception data,
 * and driver inputs. Applies priority logic:
 *   (i)   Fault → OFF
 *   (ii)  AEB disabled → OFF
 *   (iii) Speed out of range → STANDBY (with exceptions)
 *   (iv)  Driver override → STANDBY
 *   (v)   Threat evaluation via TTC + distance floors
 *   (vi)  Escalation immediate; de-escalation with 200 ms debounce
 *   (vii) POST_BRAKE: hold 2.0 s or accelerator override
 *
 * @param[in]  ttc_in     Pointer to TTC calculation output.
 * @param[in]  percep_in  Pointer to perception output.
 * @param[in]  driver_in  Pointer to driver input signals.
 * @param[out] output     Pointer to FSM output struct.
 *
 * @requirement FR-FSM-001 to FR-FSM-006, FR-DEC-004 to FR-DEC-011
 */
void fsm_step(const ttc_output_t *ttc_in,
              const perception_output_t *percep_in,
              const driver_input_t *driver_in,
              fsm_output_t *output);

#endif /* AEB_FSM_H */
