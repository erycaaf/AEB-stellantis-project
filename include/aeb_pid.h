/**
 * @file    aeb_pid.h
 * @brief   PI brake controller module — public API.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Implements a PI controller with anti-windup and jerk limiting
 *          to generate smooth brake commands based on FSM deceleration targets.
 *
 * @requirements FR-BRK-001 to FR-BRK-007
 * @simulink     chart_96 (PID_Logic)
 */

#ifndef AEB_PID_H
#define AEB_PID_H

#include "aeb_types.h"

/**
 * @brief Initialise PID controller internal state.
 *
 * Resets integrator accumulator, previous output, and internal flags.
 * Must be called once at system startup.
 */
void pid_init(void);

/**
 * @brief Execute one PI brake controller step.
 *
 * Computes brake command using PI control law:
 *   error = decel_target - a_ego
 *   integral += Ki * error * dt  (clamped to [0, PID_INTEG_MAX])
 *   output = Kp * error + integral
 *   output clamped to [0, 100] %
 *   jerk limited: |delta_output| <= MAX_JERK * (100/DECEL_L3) * dt
 *
 * Controller resets when decel_target <= 0 or fsm_state < FSM_BRAKE_L1.
 *
 * @param[in]  decel_target  Target deceleration from FSM [m/s^2].
 * @param[in]  a_ego         Current ego vehicle deceleration [m/s^2].
 * @param[in]  fsm_state     Current FSM state (fsm_state_e).
 * @param[out] output        Pointer to PID output struct.
 *
 * @requirement FR-BRK-001 to FR-BRK-007
 */
void pid_brake_step(float32_t decel_target, float32_t a_ego,
                    uint8_t fsm_state, pid_output_t *output);

#endif /* AEB_PID_H */
