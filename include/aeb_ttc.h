/**
 * @file aeb_ttc.h
 * @brief TTC (Time-to-Collision) and braking distance calculation.
 * @author Lourenço Jamba Mphili
 * @requirements FR-DEC-001, FR-DEC-002, FR-DEC-003
 */

#ifndef AEB_TTC_H
#define AEB_TTC_H

#include <stdint.h>
#include "aeb_types.h"
#include "aeb_config.h"

/**
 * @brief Calculates Time-to-Collision.
 * @param distance Distance to target [m]
 * @param v_rel Relative velocity (ego - target) [m/s]
 * @return TTC clamped to [0, TTC_MAX] seconds
 * @requirement FR-DEC-001
 */
float32_t ttc_calc(float32_t distance, float32_t v_rel);

/**
 * @brief Calculates minimum braking distance.
 * @param v_ego Ego vehicle speed [m/s]
 * @return d_brake = v² / (2 * DECEL_L3) [m]
 * @requirement FR-DEC-002
 */
float32_t d_brake_calc(float32_t v_ego);

/**
 * @brief Processes perception data and fills TTC output.
 * @param perception Input from perception module (contains distance, v_ego, v_rel)
 * @param ttc_out Output structure (defined in aeb_types.h)
 * @requirement FR-DEC-003
 */
void ttc_process(const perception_output_t * const perception,
                 ttc_output_t * const ttc_out);

#endif /* AEB_TTC_H */