/**
 * @file    aeb_alert.h
 * @brief   Alert mapping and driver override detection — public API.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-09
 *
 * @details Maps FSM state to visual/audible alert outputs and detects
 *          driver override conditions. Derived from Simulink chart_79
 *          (Alert_Map) and chart_88 (Override_Det).
 *
 * @requirements FR-ALR-001 to FR-ALR-004, FR-DEC-006, FR-DEC-007
 * @simulink     chart_79 (Alert_Map) — system_137.xml
 *               chart_88 (Override_Det) — system_144.xml
 */

#ifndef AEB_ALERT_H
#define AEB_ALERT_H

#include "aeb_types.h"

/* ── Alert enumerations (Task C internal, used by alert_map) ──────────── */

/** @brief Alert type: visual, audible, or both. */
typedef enum
{
    ALERT_NONE    = 0,   /**< No alert active.                          */
    ALERT_VISUAL  = 1,   /**< Visual alert only (dashboard indicator).  */
    ALERT_AUDIBLE = 2,   /**< Audible alert only (buzzer/beep).         */
    ALERT_BOTH    = 3    /**< Visual + audible alert.                   */
} alert_type_e;

/** @brief Buzzer command pattern mapped from FSM state. */
typedef enum
{
    BUZZER_SILENT     = 0,   /**< No sound — OFF / STANDBY / POST_BRAKE. */
    BUZZER_SINGLE     = 1,   /**< Single beep — WARNING state.           */
    BUZZER_DOUBLE     = 2,   /**< Double beep — BRAKE_L1 state.          */
    BUZZER_CONTINUOUS = 3,   /**< Continuous beep — BRAKE_L3 state.      */
    BUZZER_FAST_PULSE = 4    /**< Fast pulse — BRAKE_L2 state.           */
} buzzer_cmd_e;

/**
 * @brief Map FSM state to alert output signals.
 *
 * Mapping (from Simulink Alert_Map):
 *   OFF / STANDBY    -> ALERT_NONE,  BUZZER_SILENT
 *   WARNING          -> ALERT_BOTH,  BUZZER_SINGLE
 *   BRAKE_L1         -> ALERT_BOTH,  BUZZER_DOUBLE
 *   BRAKE_L2         -> ALERT_BOTH,  BUZZER_FAST_PULSE
 *   BRAKE_L3         -> ALERT_BOTH,  BUZZER_CONTINUOUS
 *   POST_BRAKE       -> ALERT_NONE,  BUZZER_SILENT
 *
 * @param[in]  fsm_state  Current FSM state (fsm_state_e value).
 * @param[out] output     Pointer to alert output struct.
 *
 * @requirement FR-ALR-001, FR-ALR-002, FR-ALR-004
 */
void alert_map(uint8_t fsm_state, alert_output_t *output);

/**
 * @brief Detect driver override condition.
 *
 * Override is active when:
 *   - brake_pedal != 0, OR
 *   - |steering_angle| > STEERING_OVERRIDE_DEG (5 degrees)
 *
 * @note In POST_BRAKE state, only the accelerator pedal triggers override
 *       (FR-FSM-006). This function handles the general case; the FSM
 *       module applies the POST_BRAKE exception externally.
 *
 * @param[in] steering_angle  Steering wheel angle [degrees].
 * @param[in] brake_pedal     Brake pedal status: 0=released, 1=pressed.
 * @return    1 if override detected, 0 otherwise.
 *
 * @requirement FR-DEC-006, FR-DEC-007
 */
uint8_t override_detect(float32_t steering_angle, uint8_t brake_pedal);

#endif /* AEB_ALERT_H */
