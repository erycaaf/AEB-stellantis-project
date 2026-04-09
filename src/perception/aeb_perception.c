/**
 * @file  aeb_perception.c
 * @brief Perception and Sensor Fusion — STUB (Task A placeholder).
 *
 * This file will be replaced by Eryca's implementation.
 * Provides empty function bodies so the project compiles and CI passes.
 */

#include "aeb_types.h"

void perception_init(void)
{
    /* STUB — Task A (Eryca) */
}

void perception_step(const float32_t d_radar,
                     const float32_t vr_radar,
                     const float32_t d_lidar,
                     const float32_t ve_ego,
                     perception_output_t *out)
{
    (void)d_radar;
    (void)vr_radar;
    (void)d_lidar;
    (void)ve_ego;

    if (out != (void *)0)
    {
        out->distance   = 0.0F;
        out->v_ego      = 0.0F;
        out->v_rel      = 0.0F;
        out->confidence = 0.0F;
        out->fault_flag = 0U;
    }
}
