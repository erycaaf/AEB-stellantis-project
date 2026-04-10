/**
 * @file    aeb_pid.h
 * @brief   PI brake controller module — public API.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-09
 *
 * @details Implements a PI controller with anti-windup and jerk limiting
 *          to generate smooth brake commands based on FSM deceleration targets.
 *          Derived from Simulink chart_96 (PID_Logic).
 *
 * @requirements FR-BRK-001 to FR-BRK-007
 * @simulink     chart_96 (PID_Logic) — system_150.xml
 */

#ifndef AEB_PID_H
#define AEB_PID_H

#include "aeb_types.h"

/**
 * @brief Initialise PID controller internal state.
 *
 * Resets integrator accumulator, previous brake output, and all flags.
 * Must be called once at system startup before the first cycle.
 *
 * @requirement FR-BRK-002 (controller initialisation)
 */
void pid_init(void);

/**
 * @brief Execute one PI brake controller step.
 *
 * Computes brake command [0-100%] using PI control law with:
 *   - Proportional + integral action (Kp=10.0, Ki=0.05)
 *   - Anti-windup clamping on integral term [0, 50%]
 *   - Jerk limiting: |delta_output| <= MAX_JERK * (100/DECEL_L3) * dt
 *   - Output clamped to [0, 100%]
 *   - Conversion to brake pressure: bar = pct / 10
 *   - Controller reset when not in braking states
 *
 * @param[in]  decel_target  Target deceleration from FSM [m/s^2].
 * @param[in]  a_ego         Current ego vehicle deceleration [m/s^2].
 * @param[in]  fsm_state     Current FSM state (fsm_state_e value).
 * @param[out] output        Pointer to PID output struct.
 *
 * @requirement FR-BRK-001, FR-BRK-002, FR-BRK-003, FR-BRK-004,
 *              FR-BRK-005, FR-BRK-006, FR-BRK-007
 */
void pid_brake_step(float32_t decel_target, float32_t a_ego,
                    uint8_t fsm_state, pid_output_t *output);

#endif /* AEB_PID_H */
