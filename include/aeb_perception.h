/**
 * @file  aeb_perception.h
 * @brief Public API for the AEB Perception module.
 */

#ifndef AEB_PERCEPTION_H
#define AEB_PERCEPTION_H

#include "aeb_types.h"

/**
 * @brief One frame of raw sensor data (from CAN Bus or test harness).
 *
 * Fields:
 *   radar_d     Radar-measured distance to obstacle [m].
 *   radar_vr    Radar-measured relative velocity (positive = closing) [m/s].
 *   lidar_d     LiDAR-measured distance to obstacle [m].
 *   v_ego       Ego-vehicle speed from wheel/IMU sensors [m/s].
 *   can_timeout Non-zero if the CAN frame was not received within deadline.
 *   fi          Fault-injection flag for V&V (0 = normal, 1 = force fault).
 */
typedef struct {
    float32_t radar_d;
    float32_t radar_vr;
    float32_t lidar_d;
    float32_t v_ego;
    uint8_t   can_timeout;
    uint8_t   fi;
} raw_sensor_input_t;

/** Reset all internal state. Call once before first perception_step(). */
void perception_init(void);

/** Execute one perception cycle (10 ms). */
void perception_step(const raw_sensor_input_t *in, perception_output_t *out);

#endif /* AEB_PERCEPTION_H */
