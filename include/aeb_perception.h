/**
 * @file    aeb_perception.h
 * @brief   Perception and sensor fusion module — public API.
 * @owner   Task A (Eryca Francyele de Moura e Silva)
 * @version 1.0
 * @date    2026-04-08
 *
 * @details Provides sensor fault detection (radar, lidar) and Kalman-based
 *          fusion of distance and relative velocity measurements.
 *
 * @requirements FR-PER-001 to FR-PER-008
 * @simulink     chart_26 (LidarFault), chart_33 (RadarFault),
 *               chart_41 (KalmanFusion)
 */

#ifndef AEB_PERCEPTION_H
#define AEB_PERCEPTION_H

#include "aeb_types.h"

/* ========================================================================= */
/*  Public function prototypes                                               */
/* ========================================================================= */

/**
 * @brief Initialise perception module internal state.
 *
 * Resets Kalman filter state, fault counters, and previous-cycle values.
 * Must be called once at system startup before the first cycle.
 */
void perception_init(void);

/**
 * @brief Detect radar sensor fault via plausibility checks.
 *
 * Validates range [0.5, 200] m, relative velocity |vr| <= 50 m/s,
 * and rate-of-change limits. Latches fault after SENSOR_FAULT_CYCLES
 * consecutive invalid readings.
 *
 * @param[in] d_radar   Radar distance measurement [m].
 * @param[in] vr_radar  Radar relative velocity measurement [m/s].
 * @return    1 if fault detected (latched), 0 otherwise.
 *
 * @requirement FR-PER-006, FR-PER-007
 */
uint8_t radar_fault_detect(float32_t d_radar, float32_t vr_radar);

/**
 * @brief Detect lidar sensor fault via plausibility checks.
 *
 * Validates range [1, 100] m and rate-of-change limits.
 * Latches fault after SENSOR_FAULT_CYCLES consecutive invalid readings.
 *
 * @param[in] d_lidar  Lidar distance measurement [m].
 * @return    1 if fault detected (latched), 0 otherwise.
 *
 * @requirement FR-PER-006, FR-PER-007
 */
uint8_t lidar_fault_detect(float32_t d_lidar);

/**
 * @brief Kalman fusion of radar and lidar measurements.
 *
 * Discrete 1D Kalman filter with state x = [distance, v_rel]^T.
 * Sequential update: LiDAR first (H=[1,0], R=0.0025), then
 * Radar (H=I2, R=diag(0.09, 0.01)).
 *
 * Confidence based on sensor availability:
 * both=1.0, radar-only=0.7, lidar-only=0.5, scaled by trace(P).
 *
 * @param[in]  d_radar       Radar distance [m].
 * @param[in]  vr_radar      Radar relative velocity [m/s].
 * @param[in]  d_lidar       Lidar distance [m].
 * @param[in]  v_ego         Ego vehicle speed [m/s].
 * @param[in]  radar_valid   1 if radar data is valid, 0 otherwise.
 * @param[in]  lidar_valid   1 if lidar data is valid, 0 otherwise.
 * @param[out] output        Pointer to perception output struct.
 *
 * @requirement FR-PER-001, FR-PER-002, FR-PER-003
 */
void kalman_fusion(float32_t d_radar, float32_t vr_radar,
                   float32_t d_lidar, float32_t v_ego,
                   uint8_t radar_valid, uint8_t lidar_valid,
                   perception_output_t *output);

/**
 * @brief Execute one full perception cycle.
 *
 * Top-level function called every 10 ms. Runs fault detection
 * on both sensors, then Kalman fusion, producing a complete
 * perception_output_t.
 *
 * @param[in]  d_radar   Raw radar distance [m].
 * @param[in]  vr_radar  Raw radar relative velocity [m/s].
 * @param[in]  d_lidar   Raw lidar distance [m].
 * @param[in]  v_ego     Ego vehicle speed [m/s].
 * @param[out] output    Pointer to perception output struct.
 *
 * @requirement FR-PER-001 to FR-PER-008
 */
void perception_step(float32_t d_radar, float32_t vr_radar,
                     float32_t d_lidar, float32_t v_ego,
                     perception_output_t *output);

#endif /* AEB_PERCEPTION_H */
