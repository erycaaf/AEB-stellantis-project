/**
 * @file    aeb_perception.c
 * @brief   Perception module — STUB implementation (Sprint 0).
 * @owner   Task A (Eryca Francyele de Moura e Silva)
 *
 * @details Stub functions return safe default values (zero/no-fault).
 *          Replace with full implementation during Sprint 1.
 *
 * @todo    Implement radar_fault_detect, lidar_fault_detect,
 *          kalman_fusion, perception_step.
 */

#include "aeb_perception.h"
#include "aeb_config.h"

/* ========================================================================= */
/*  Stub implementations                                                     */
/* ========================================================================= */

void perception_init(void)
{
    /* STUB: no internal state to initialise yet. */
}

uint8_t radar_fault_detect(float32_t d_radar, float32_t vr_radar)
{
    (void)d_radar;
    (void)vr_radar;
    /* STUB: no fault detected. */
    return 0U;
}

uint8_t lidar_fault_detect(float32_t d_lidar)
{
    (void)d_lidar;
    /* STUB: no fault detected. */
    return 0U;
}

void kalman_fusion(float32_t d_radar, float32_t vr_radar,
                   float32_t d_lidar, float32_t v_ego,
                   uint8_t radar_valid, uint8_t lidar_valid,
                   perception_output_t *output)
{
    (void)d_radar;
    (void)vr_radar;
    (void)d_lidar;
    (void)radar_valid;
    (void)lidar_valid;

    /* STUB: pass-through with safe defaults. */
    output->distance   = 300.0f;
    output->v_ego      = v_ego;
    output->v_rel      = 0.0f;
    output->confidence = 1.0f;
    output->fault_flag = 0U;
}

void perception_step(float32_t d_radar, float32_t vr_radar,
                     float32_t d_lidar, float32_t v_ego,
                     perception_output_t *output)
{
    /* STUB: delegate to kalman_fusion with all sensors valid. */
    kalman_fusion(d_radar, vr_radar, d_lidar, v_ego, 1U, 1U, output);
}
