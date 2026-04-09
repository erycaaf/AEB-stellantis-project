/**
 * @file  aeb_config.h
 * @brief Calibration parameters for the AEB system (NFR-POR-002).
 *
 * All tuneable parameters centralised here, enabling independent
 * calibration without modifying source code.
 *
 * Based on AEB_Tasks v1.0 Section 3.2.
 *
 * @req NFR-POR-002 Calibration separated from code.
 * @req FR-COD-001 Structural correspondence with Simulink model.
 *
 * @version 2.0 — Integration
 * @date 2026-04
 */

#ifndef AEB_CONFIG_H
#define AEB_CONFIG_H

/* ===== Timing (NFR-PERF-001) ===== */
#define AEB_CYCLE_TIME_MS     10U        /**< 100 Hz                  */
#define AEB_DT                0.01F      /**< 10 ms in seconds        */

/* ===== TTC thresholds (FR-DEC-004) ===== */
#define TTC_WARNING           4.0F       /**< s                       */
#define TTC_BRAKE_L1          3.0F       /**< s                       */
#define TTC_BRAKE_L2          2.2F       /**< s                       */
#define TTC_BRAKE_L3          1.8F       /**< s                       */
#define TTC_MAX               10.0F      /**< s — saturation          */
#define V_REL_MIN             0.5F       /**< m/s                     */

/* ===== Distance floors (Table 10) ===== */
#define D_FLOOR_L1            20.0F      /**< m                       */
#define D_FLOOR_L2            10.0F      /**< m                       */
#define D_FLOOR_L3            5.0F       /**< m                       */

/* ===== Speed range (FR-DEC-008) ===== */
#define V_EGO_MIN             2.78F      /**< m/s = 10 km/h           */
#define V_EGO_MAX             16.67F     /**< m/s = 60 km/h           */

/* ===== Deceleration levels ===== */
#define DECEL_L1              2.0F       /**< m/s^2                   */
#define DECEL_L2              4.0F       /**< m/s^2                   */
#define DECEL_L3              6.0F       /**< m/s^2 (= amax)          */

/* ===== Hysteresis & timing ===== */
#define HYSTERESIS_TIME       0.2F       /**< s — de-escalation       */
#define WARNING_MIN_TIME      0.8F       /**< s — FR-ALR-003          */
#define POST_BRAKE_HOLD       2.0F       /**< s — FR-BRK-005          */
#define V_STOP_THRESHOLD      0.01F      /**< m/s                     */

/* ===== PI controller (FR-BRK-002) ===== */
#define PID_KP                10.0F
#define PID_KI                0.05F
#define PID_INTEG_MAX         50.0F      /**< % — anti-windup         */
#define BRAKE_OUT_MAX         100.0F     /**< %                       */
#define BRAKE_OUT_MIN         0.0F       /**< %                       */
#define MAX_JERK              10.0F      /**< m/s^3                   */

/* ===== Sensor fault (FR-PER-007) ===== */
#define SENSOR_FAULT_CYCLES   3U         /**< consecutive bad cycles  */
#define DIST_ROC_LIMIT        10.0F      /**< m/cycle                 */
#define VEL_ROC_LIMIT         2.0F       /**< m/s per cycle           */
#define STEERING_OVERRIDE_DEG 5.0F       /**< degrees                 */

/* ===== CAN (FR-CAN-001..004) ===== */
#define CAN_BAUD_RATE         500000U    /**< 500 kbit/s              */
#define CAN_TX_PERIOD_MS      50U        /**< ego dynamics TX period  */
#define CAN_RX_TIMEOUT_MS     60U        /**< 3 x 20 ms              */

/* ===== UDS diagnostics (FR-UDS-001..005) ===== */
#define UDS_DID_TTC           0xF100U    /**< DID — TTC value         */
#define UDS_DID_FSM_STATE     0xF101U    /**< DID — FSM state         */
#define UDS_DID_BRAKE_PRESS   0xF102U    /**< DID — Brake pressure    */
#define UDS_ROUTINE_AEB       0x0301U    /**< RID — AEB enable/disable*/

#endif /* AEB_CONFIG_H */
