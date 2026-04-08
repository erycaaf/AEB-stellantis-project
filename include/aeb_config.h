/**
 * @file    aeb_config.h
 * @brief   Calibration parameters and system constants for the AEB system.
 * @version 1.0
 * @date    2026-04-08
 *
 * @details This file centralises all tuneable parameters used across the AEB
 *          modules. Separating calibration from source code facilitates
 *          maintenance and future system tuning without modifying functional
 *          logic (NFR-POR-002).
 *
 *          Parameter values are derived from:
 *          - SRS v2.0 (March 2026): Tables 4-18
 *          - AEB_Tasks v1.0 (April 2026): Sections 3.2, 4.1-4.5
 *          - Simulink model AEB_Integration.slx: validated parameters
 *
 * @requirements
 *  - NFR-POR-002: Calibration parameters separated from code.
 *  - NFR-COD-005: Fixed-width types used throughout.
 *  - FR-COD-001:  Structural correspondence with Simulink model.
 *
 * @standard MISRA C:2012 compliant.
 */

#ifndef AEB_CONFIG_H
#define AEB_CONFIG_H

/* ========================================================================= */
/*  Timing — NFR-PERF-001, NFR-PERF-002                                     */
/* ========================================================================= */

/** @brief AEB algorithm execution cycle period [ms].
 *  @requirement NFR-PERF-001: 10 ms (100 Hz) execution cycle. */
#define AEB_CYCLE_TIME_MS       10U

/** @brief Execution cycle period in seconds [s].
 *  @note Used in integration and filter calculations. */
#define AEB_DT                  0.01f

/* ========================================================================= */
/*  TTC Thresholds — FR-DEC-004, Table 10 (SRS)                             */
/* ========================================================================= */

/** @brief TTC threshold for WARNING state entry [s].
 *  @requirement FR-DEC-004: TTC <= 4.0 s triggers warning. */
#define TTC_WARNING             4.0f

/** @brief TTC threshold for BRAKE_L1 state entry [s].
 *  @requirement FR-DEC-004: TTC <= 3.0 s triggers light pre-braking. */
#define TTC_BRAKE_L1            3.0f

/** @brief TTC threshold for BRAKE_L2 state entry [s].
 *  @requirement FR-DEC-004: TTC <= 2.2 s triggers partial braking. */
#define TTC_BRAKE_L2            2.2f

/** @brief TTC threshold for BRAKE_L3 state entry [s].
 *  @requirement FR-DEC-004: TTC <= 1.8 s triggers full emergency braking. */
#define TTC_BRAKE_L3            1.8f

/** @brief TTC saturation upper limit [s].
 *  @note TTC is clamped to [0, TTC_MAX] to prevent unbounded values. */
#define TTC_MAX                 10.0f

/** @brief Minimum relative velocity for valid TTC calculation [m/s].
 *  @requirement FR-DEC-001: TTC valid only when ego is approaching target. */
#define V_REL_MIN               0.5f

/* ========================================================================= */
/*  Distance Floors — Table 10 (SRS), FR-DEC-003                            */
/* ========================================================================= */

/** @brief Distance floor for BRAKE_L1: do not de-escalate below L1
 *         when distance <= 20 m [m]. */
#define D_FLOOR_L1              20.0f

/** @brief Distance floor for BRAKE_L2: do not de-escalate below L2
 *         when distance <= 10 m [m]. */
#define D_FLOOR_L2              10.0f

/** @brief Distance floor for BRAKE_L3: do not de-escalate below L3
 *         when distance <= 5 m [m]. */
#define D_FLOOR_L3              5.0f

/* ========================================================================= */
/*  Speed Range — FR-DEC-008                                                 */
/* ========================================================================= */

/** @brief Minimum ego speed for AEB activation [m/s].
 *  @note 2.78 m/s = 10 km/h.
 *  @requirement FR-DEC-008: System active only when v_ego >= 10 km/h. */
#define V_EGO_MIN               2.78f

/** @brief Maximum ego speed for AEB activation [m/s].
 *  @note 16.67 m/s = 60 km/h.
 *  @requirement FR-DEC-008: System active only when v_ego <= 60 km/h. */
#define V_EGO_MAX               16.67f

/* ========================================================================= */
/*  Deceleration Levels — Table 10 (SRS), FR-BRK-003                        */
/* ========================================================================= */

/** @brief Target deceleration for BRAKE_L1 [m/s^2].
 *  @requirement Table 10: Light pre-braking at 2 m/s^2. */
#define DECEL_L1                2.0f

/** @brief Target deceleration for BRAKE_L2 [m/s^2].
 *  @requirement Table 10: Partial braking at 4 m/s^2. */
#define DECEL_L2                4.0f

/** @brief Target deceleration for BRAKE_L3 [m/s^2].
 *  @requirement Table 10: Full emergency braking at 6 m/s^2.
 *  @note Also used as amax in d_brake calculation (FR-DEC-002). */
#define DECEL_L3                6.0f

/* ========================================================================= */
/*  Hysteresis and Timing — FR-DEC-010, FR-ALR-003, FR-BRK-005              */
/* ========================================================================= */

/** @brief Minimum de-escalation debounce time [s].
 *  @requirement FR-DEC-010: Hysteresis >= 200 ms to prevent chattering. */
#define HYSTERESIS_TIME         0.2f

/** @brief Minimum time in WARNING state before braking [s].
 *  @requirement FR-ALR-003: Alert precedes braking by >= 800 ms. */
#define WARNING_MIN_TIME        0.8f

/** @brief Post-brake hold time after full stop [s].
 *  @requirement FR-BRK-005: Maintain braking for >= 2.0 s after stop. */
#define POST_BRAKE_HOLD         2.0f

/** @brief Velocity threshold for complete vehicle stop [m/s].
 *  @requirement FR-BRK-005: v_ego < 0.01 m/s considered full stop. */
#define V_STOP_THRESHOLD        0.01f

/* ========================================================================= */
/*  PI Controller — FR-BRK-002, FR-BRK-004, FR-BRK-007                     */
/* ========================================================================= */

/** @brief Proportional gain for PI brake controller.
 *  @requirement FR-BRK-002: PI controller with anti-windup. */
#define PID_KP                  10.0f

/** @brief Integral gain for PI brake controller.
 *  @requirement FR-BRK-002: PI controller with anti-windup. */
#define PID_KI                  0.05f

/** @brief Maximum integrator accumulation [%].
 *  @note Anti-windup clamp for integral term. */
#define PID_INTEG_MAX           50.0f

/** @brief Maximum brake command output [%].
 *  @requirement FR-BRK-007: Output clamped to [0, 100] %. */
#define BRAKE_OUT_MAX           100.0f

/** @brief Minimum brake command output [%].
 *  @requirement FR-BRK-007: Output clamped to [0, 100] %. */
#define BRAKE_OUT_MIN           0.0f

/** @brief Maximum longitudinal jerk [m/s^3].
 *  @requirement FR-BRK-004: |jerk| <= 10 m/s^3 in all scenarios. */
#define MAX_JERK                10.0f

/* ========================================================================= */
/*  Sensor Fault Detection — FR-PER-006, FR-PER-007                         */
/* ========================================================================= */

/** @brief Consecutive invalid cycles to trigger sensor fault latch.
 *  @requirement FR-PER-007: Fault detected within 3 cycles (30 ms). */
#define SENSOR_FAULT_CYCLES     3U

/** @brief Maximum allowed rate-of-change for distance per cycle [m/cycle].
 *  @requirement FR-PER-006: Rate-of-change validation. */
#define DIST_ROC_LIMIT          10.0f

/** @brief Maximum allowed rate-of-change for velocity per cycle [m/s/cycle].
 *  @requirement FR-PER-006: Rate-of-change validation. */
#define VEL_ROC_LIMIT           2.0f

/** @brief Steering angle threshold for driver override [degrees].
 *  @requirement FR-DEC-007: Override if steering angle > 5 degrees. */
#define STEERING_OVERRIDE_DEG   5.0f

/* ========================================================================= */
/*  CAN Bus — FR-CAN-001 to FR-CAN-004                                      */
/* ========================================================================= */

/** @brief CAN bus baud rate [bit/s].
 *  @requirement FR-CAN-004: Network operates at 500 kbit/s. */
#define CAN_BAUD_RATE           500000U

/** @brief CAN TX period for ego vehicle dynamics [ms].
 *  @requirement FR-CAN-001: Ego dynamics transmitted every 50 ms. */
#define CAN_TX_PERIOD_MS        50U

/** @brief CAN RX timeout threshold [ms].
 *  @note 3 consecutive missed frames at 20 ms period = 60 ms.
 *  @requirement FR-CAN-002: Radar target data expected every 20 ms. */
#define CAN_RX_TIMEOUT_MS       60U

/* ========================================================================= */
/*  UDS Diagnostics — FR-UDS-001 to FR-UDS-005                              */
/* ========================================================================= */

/** @brief UDS DID for TTC value (scaled: TTC x 100).
 *  @requirement FR-UDS-001: ReadDataByIdentifier for TTC. */
#define UDS_DID_TTC             0xF100U

/** @brief UDS DID for FSM state value.
 *  @requirement FR-UDS-001: ReadDataByIdentifier for FSM state. */
#define UDS_DID_FSM_STATE       0xF101U

/** @brief UDS DID for brake pressure value (scaled: pressure x 10).
 *  @requirement FR-UDS-001: ReadDataByIdentifier for brake pressure. */
#define UDS_DID_BRAKE_PRESS     0xF102U

/** @brief UDS Routine ID for AEB enable/disable control.
 *  @requirement FR-UDS-004: RoutineControl with routine 0x0301. */
#define UDS_ROUTINE_AEB         0x0301U

#endif /* AEB_CONFIG_H */
