/**
 * @file  aeb_types.h
 * @brief Shared interface structs for the AEB system.
 *
 * This file is the integration contract between all modules.
 * Based on AEB_Tasks v1.0 Section 3.1.
 */

#ifndef AEB_TYPES_H
#define AEB_TYPES_H

#include <stdint.h>

/** @brief 32-bit IEEE-754 floating-point type (MISRA fixed-width). */
typedef float float32_t;

/** @brief Perception output (Task A -> Task B). */
typedef struct
{
    float32_t distance;       /**< [0, 300] m              */
    float32_t v_ego;          /**< m/s                     */
    float32_t v_rel;          /**< m/s (>0 = closing)      */
    float32_t confidence;     /**< [0.0, 1.0]              */
    uint8_t   fault_flag;     /**< 0=OK, 1=fault           */
} perception_output_t;

/** @brief TTC calculation output (Task B internal). */
typedef struct
{
    float32_t ttc;            /**< [0, 10] s               */
    float32_t d_brake;        /**< m                       */
    uint8_t   is_closing;     /**< boolean                 */
} ttc_output_t;

/** @brief FSM output (Task B -> Tasks C, D, E). */
typedef struct
{
    uint8_t   fsm_state;      /**< 0=OFF .. 6=POST_BRAKE   */
    float32_t decel_target;   /**< m/s^2                   */
    uint8_t   brake_active;   /**< boolean                 */
    uint8_t   alert_level;    /**< 0=none .. 3=critical    */
    float32_t warn_timer;     /**< s                       */
    float32_t state_timer;    /**< s                       */
} fsm_output_t;

/** @brief PID brake output (PID brake -> CAN TX). */
typedef struct
{
    float32_t brake_pct;      /**< [0, 100] %              */
    float32_t brake_bar;      /**< [0, 10] bar             */
} pid_output_t;

/** @brief Alert output (Task C -> GPIO). */
typedef struct
{
    uint8_t   alert_type;     /**< 0=NONE,1=VIS,2=AUD,3=BOTH */
    uint8_t   alert_active;   /**< boolean                 */
    uint8_t   buzzer_cmd;     /**< 0..4 (beep pattern)     */
} alert_output_t;

/** @brief Driver inputs (CAN RX -> Decision, Execution). */
typedef struct
{
    uint8_t   brake_pedal;    /**< boolean                 */
    uint8_t   accel_pedal;    /**< boolean                 */
    float32_t steering_angle; /**< degrees                 */
    uint8_t   aeb_enabled;    /**< boolean                 */
} driver_input_t;

/** @brief FSM state enumeration. */
typedef enum
{
    FSM_OFF        = 0,
    FSM_STANDBY    = 1,
    FSM_WARNING    = 2,
    FSM_BRAKE_L1   = 3,
    FSM_BRAKE_L2   = 4,
    FSM_BRAKE_L3   = 5,
    FSM_POST_BRAKE = 6
} fsm_state_e;

#endif /* AEB_TYPES_H */
