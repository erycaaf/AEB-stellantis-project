/**
 * @file    aeb_types.h
 * @brief   Shared data structures and enumerations for the AEB system.
 * @version 1.0
 * @date    2026-04-08
 *
 * @details This file defines the typed interface contract between all AEB
 *          modules. Every module communicates exclusively through these
 *          structs, passed by pointer. No global mutable state is shared
 *          outside of Zephyr kernel objects (mutexes, timers).
 *
 * @note    All types use fixed-width definitions per NFR-COD-005.
 *          This file shall be reviewed and agreed upon by the entire team
 *          before individual module development begins (Sprint 0).
 *
 * @requirements
 *  - NFR-COD-005: Exclusive use of fixed-width types.
 *  - NFR-POR-003: Modular architecture with well-defined interfaces.
 *  - FR-COD-001:  Structural correspondence between Simulink model
 *                 and C implementation.
 *
 * @standard MISRA C:2012 compliant.
 */

#ifndef AEB_TYPES_H
#define AEB_TYPES_H

/* ========================================================================= */
/*  Standard includes                                                        */
/* ========================================================================= */
#include <stdint.h>

/* ========================================================================= */
/*  Fixed-width float alias (NFR-COD-005)                                    */
/* ========================================================================= */

/** @brief 32-bit floating-point type alias for cross-platform portability. */
typedef float float32_t;

/* ========================================================================= */
/*  Enumerations                                                             */
/* ========================================================================= */

/**
 * @brief FSM state enumeration.
 *
 * Defines the seven operational states of the AEB Finite State Machine
 * as specified in FR-FSM-001 and Table 10 of the SRS v2.0.
 *
 * State transitions follow the rules defined in FR-FSM-002 to FR-FSM-006.
 *
 * @requirement FR-FSM-001
 */
typedef enum {
    FSM_OFF        = 0,  /**< System disabled (AEB switch off or fault). */
    FSM_STANDBY    = 1,  /**< System active, no threat detected.        */
    FSM_WARNING    = 2,  /**< Visual + audible alert, TTC <= 4.0 s.     */
    FSM_BRAKE_L1   = 3,  /**< Light pre-braking, 2 m/s^2.              */
    FSM_BRAKE_L2   = 4,  /**< Partial braking, 4 m/s^2.                */
    FSM_BRAKE_L3   = 5,  /**< Full emergency braking, 6 m/s^2.         */
    FSM_POST_BRAKE = 6   /**< Brake hold after full stop (2 s).         */
} fsm_state_e;

/**
 * @brief Alert type enumeration.
 *
 * Defines the alert output modes used by the alert_map() function.
 *
 * @requirement FR-ALR-001, FR-ALR-002
 */
typedef enum {
    ALERT_NONE    = 0,  /**< No alert active.                           */
    ALERT_VISUAL  = 1,  /**< Visual alert only (dashboard indicator).   */
    ALERT_AUDIBLE = 2,  /**< Audible alert only (buzzer/beep).          */
    ALERT_BOTH    = 3   /**< Visual + audible alert.                    */
} alert_type_e;

/**
 * @brief Buzzer command enumeration.
 *
 * Defines the buzzer patterns mapped from FSM states
 * as specified in AEB_Tasks Section 4.3 (Alert_Map table).
 */
typedef enum {
    BUZZER_SILENT     = 0,  /**< No sound — OFF / STANDBY / POST_BRAKE. */
    BUZZER_SINGLE     = 1,  /**< Single beep — WARNING state.           */
    BUZZER_DOUBLE     = 2,  /**< Double beep — BRAKE_L1 state.          */
    BUZZER_CONTINUOUS = 3,  /**< Continuous beep — BRAKE_L3 state.      */
    BUZZER_FAST_PULSE = 4   /**< Fast pulse — BRAKE_L2 state.           */
} buzzer_cmd_e;

/* ========================================================================= */
/*  Data structures — Inter-module interface contracts                       */
/* ========================================================================= */

/**
 * @brief Perception module output (Task A -> Task B).
 *
 * Contains validated and fused sensor data produced by the perception
 * module after Kalman fusion and fault detection.
 *
 * @requirement FR-PER-001 to FR-PER-008
 */
typedef struct {
    float32_t distance;    /**< Fused relative distance [0, 300] m.         */
    float32_t v_ego;       /**< Ego vehicle speed [m/s].                    */
    float32_t v_rel;       /**< Relative velocity [m/s] (>0 = closing).     */
    float32_t confidence;  /**< Fusion confidence [0.0, 1.0].               */
    uint8_t   fault_flag;  /**< Sensor fault indicator: 0=OK, 1=fault.      */
} perception_output_t;

/**
 * @brief TTC calculation output (Task B internal).
 *
 * Contains the Time-To-Collision, minimum braking distance, and
 * approach condition flag.
 *
 * @requirement FR-DEC-001, FR-DEC-002, FR-DEC-003
 */
typedef struct {
    float32_t ttc;         /**< Time-To-Collision [0, 10] s.                */
    float32_t d_brake;     /**< Minimum braking distance [m].               */
    uint8_t   is_closing;  /**< Approach condition: 1 if v_rel > 0.         */
} ttc_output_t;

/**
 * @brief FSM output (Task B -> Tasks C, D, E).
 *
 * Contains the current state machine output including target
 * deceleration, brake activation flag, alert level, and internal timers.
 *
 * @requirement FR-FSM-001 to FR-FSM-006, FR-DEC-004 to FR-DEC-011
 */
typedef struct {
    uint8_t   fsm_state;     /**< Current FSM state (fsm_state_e).          */
    float32_t decel_target;  /**< Target deceleration [m/s^2].              */
    uint8_t   brake_active;  /**< Brake actuation flag: 0=off, 1=on.        */
    uint8_t   alert_level;   /**< Alert severity: 0=none .. 3=critical.     */
    float32_t warn_timer;    /**< Time elapsed in WARNING state [s].         */
    float32_t state_timer;   /**< Time elapsed in current state [s].         */
} fsm_output_t;

/**
 * @brief PID brake controller output (Task C -> Task D TX).
 *
 * Contains the computed brake command as percentage and pressure.
 *
 * @requirement FR-BRK-001 to FR-BRK-007
 */
typedef struct {
    float32_t brake_pct;   /**< Brake command [0, 100] %.                   */
    float32_t brake_bar;   /**< Brake pressure [0, 10] bar.                 */
} pid_output_t;

/**
 * @brief Alert output (Task C -> GPIO).
 *
 * Contains the alert signals to be output via GPIO to the HMI.
 *
 * @requirement FR-ALR-001 to FR-ALR-004
 */
typedef struct {
    uint8_t   alert_type;    /**< Alert type (alert_type_e).                */
    uint8_t   alert_active;  /**< Alert active flag: 0=off, 1=on.           */
    uint8_t   buzzer_cmd;    /**< Buzzer pattern (buzzer_cmd_e).            */
} alert_output_t;

/**
 * @brief Driver input signals (Task D RX -> Tasks B, C).
 *
 * Contains driver interaction signals decoded from CAN bus.
 *
 * @requirement FR-PER-004, FR-PER-005, FR-PER-008, FR-DEC-006, FR-DEC-007
 */
typedef struct {
    uint8_t   brake_pedal;     /**< Brake pedal pressed: 0=no, 1=yes.       */
    uint8_t   accel_pedal;     /**< Accelerator pedal pressed: 0=no, 1=yes. */
    float32_t steering_angle;  /**< Steering wheel angle [degrees].         */
    uint8_t   aeb_enabled;     /**< AEB function enabled: 0=no, 1=yes.      */
} driver_input_t;

#endif /* AEB_TYPES_H */
