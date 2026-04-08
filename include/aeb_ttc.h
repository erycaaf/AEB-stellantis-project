/**
 * @file    aeb_ttc.h
 * @brief   TTC and braking distance calculation module — public API.
 * @owner   Task B (Lourenço Jamba Mphili)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Computes Time-To-Collision and minimum braking distance
 *          using the dual criterion defined in the SRS.
 *
 * @requirements FR-DEC-001, FR-DEC-002, FR-DEC-003
 * @simulink     chart_106 (TTC_Logic)
 */

#ifndef AEB_TTC_H
#define AEB_TTC_H

#include "aeb_types.h"

/**
 * @brief Compute TTC, braking distance, and closing condition.
 *
 * TTC = d / v_rel  when v_rel > V_REL_MIN (0.5 m/s), else TTC = TTC_MAX.
 * d_brake = v_ego^2 / (2 * DECEL_L3).
 * TTC clamped to [0, TTC_MAX].
 * is_closing set when v_rel > 0.
 *
 * @param[in]  distance  Relative distance to target [m].
 * @param[in]  v_ego     Ego vehicle speed [m/s].
 * @param[in]  v_target  Target vehicle speed [m/s].
 * @param[out] output    Pointer to TTC output struct.
 *
 * @requirement FR-DEC-001, FR-DEC-002
 */
void ttc_calc(float32_t distance, float32_t v_ego,
              float32_t v_target, ttc_output_t *output);

#endif /* AEB_TTC_H */
