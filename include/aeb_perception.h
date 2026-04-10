/**
 * @file  aeb_perception.h
 * @brief Public API for the AEB Perception module.
 */

#ifndef AEB_PERCEPTION_H
#define AEB_PERCEPTION_H

#include "aeb_types.h"


/** Reset all internal state. Call once before first perception_step(). */
void perception_init(void);

/** Execute one perception cycle (10 ms). */
void perception_step(const raw_sensor_input_t *in, perception_output_t *out);

#endif /* AEB_PERCEPTION_H */
