/**
 * @file  aeb_config.h
 * @brief Calibration parameters for the AEB system (NFR-POR-002).
 *
 * CAN-relevant subset of calibration parameters.
 * Full file maintained by Task E (Rian).
 */

#ifndef AEB_CONFIG_H
#define AEB_CONFIG_H

/* Timing */
#define AEB_CYCLE_TIME_MS     10U        /**< 100 Hz                  */
#define AEB_DT                0.01F      /**< 10 ms in seconds        */

/* TTC thresholds (FR-DEC-004) */
#define TTC_WARNING           4.0F       /**< s                       */
#define TTC_BRAKE_L1          3.0F       /**< s                       */
#define TTC_BRAKE_L2          2.2F       /**< s                       */
#define TTC_BRAKE_L3          1.8F       /**< s                       */
#define TTC_MAX               10.0F      /**< s — saturation          */
#define V_REL_MIN             0.5F       /**< m/s                     */

/* Speed range (FR-DEC-008) */
#define V_EGO_MIN             2.78F      /**< m/s = 10 km/h           */
#define V_EGO_MAX             16.67F     /**< m/s = 60 km/h           */

/* Deceleration levels */
#define DECEL_L1              2.0F       /**< m/s^2                   */
#define DECEL_L2              4.0F       /**< m/s^2                   */
#define DECEL_L3              6.0F       /**< m/s^2 (= amax)          */

/* CAN (FR-CAN) */
#define CAN_BAUD_RATE         500000U    /**< 500 kbit/s              */
#define CAN_TX_PERIOD_MS      50U        /**< ego dynamics TX period  */
#define CAN_RX_TIMEOUT_MS     60U        /**< 3 × 20 ms              */

/* UDS DIDs */
#define UDS_DID_TTC           0xF100U
#define UDS_DID_FSM_STATE     0xF101U
#define UDS_DID_BRAKE_PRESS   0xF102U
#define UDS_ROUTINE_AEB       0x0301U

#endif /* AEB_CONFIG_H */
