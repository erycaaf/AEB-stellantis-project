/**
 * @file    aeb_alert.c
 * @brief   Alert mapping and driver override detection — full implementation.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-09
 *
 * @details Implements two functions:
 *
 *          alert_map():  Maps FSM state to alert type and buzzer pattern.
 *          Derived from Simulink chart_79 (Alert_Map) — system_137.xml.
 *          Data flow: FSM.fsm_state -> Alert_Logic -> alert_type,
 *          alert_active, buzzer_cmd.
 *
 *          override_detect():  Detects driver override via brake pedal
 *          or steering input. Derived from Simulink chart_88
 *          (Override_Det) — system_144.xml.
 *          Data flow: steering_angle, brake_pedal -> Driver_Override
 *          -> override_active (fed to FSM input 9).
 *
 * @requirements FR-ALR-001 to FR-ALR-004, FR-DEC-006, FR-DEC-007
 *
 * @req FR-ALR-001  Visual alert upon WARNING entry, within 1 cycle.
 * @req FR-ALR-002  Audible alert upon WARNING entry, within 1 cycle.
 * @req FR-ALR-003  Alert precedes braking by >= 800 ms (enforced by FSM).
 * @req FR-ALR-004  Alert ceases when returning to STANDBY or driver override.
 * @req FR-DEC-006  Override on brake pedal press.
 * @req FR-DEC-007  Override on steering angle > 5 degrees.
 *
 * @standard MISRA C:2012 compliant.
 */

#include "aeb_alert.h"
#include "aeb_config.h"

/* ========================================================================= */
/*  Private helper — absolute value for float32                              */
/* ========================================================================= */

/**
 * @brief Compute absolute value of a float32.
 * @param[in] val  Input value.
 * @return Absolute value.
 */
static float32_t abs_f32(float32_t val)
{
    float32_t result = val;

    if (result < 0.0F)
    {
        result = -result;
    }

    return result;
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void alert_map(uint8_t fsm_state, alert_output_t *output)
{
    /*
     * Alert mapping table — derived from Simulink Alert_Map (chart_79).
     *
     * The mapping is a pure combinational function: no internal state,
     * no memory between cycles. The FSM guarantees that WARNING is held
     * for >= 800 ms before any braking state (FR-ALR-003), so this
     * function does not need to enforce timing.
     *
     * alert_type:  0=NONE, 1=VISUAL, 2=AUDIBLE, 3=BOTH
     * buzzer_cmd:  0=SILENT, 1=SINGLE, 2=DOUBLE, 3=CONTINUOUS, 4=FAST_PULSE
     * alert_active: 1 when any alert is on, 0 otherwise
     *
     * State          | alert_type | buzzer_cmd    | alert_active
     * ---------------|------------|---------------|-------------
     * OFF (0)        | NONE (0)   | SILENT (0)    | 0
     * STANDBY (1)    | NONE (0)   | SILENT (0)    | 0
     * WARNING (2)    | BOTH (3)   | SINGLE (1)    | 1
     * BRAKE_L1 (3)   | BOTH (3)   | DOUBLE (2)    | 1
     * BRAKE_L2 (4)   | BOTH (3)   | FAST_PULSE (4)| 1
     * BRAKE_L3 (5)   | BOTH (3)   | CONTINUOUS (3)| 1
     * POST_BRAKE (6) | NONE (0)   | SILENT (0)    | 0
     */

    switch (fsm_state)
    {
        case (uint8_t)FSM_WARNING:
            output->alert_type   = (uint8_t)ALERT_BOTH;
            output->alert_active = 1U;
            output->buzzer_cmd   = (uint8_t)BUZZER_SINGLE;
            break;

        case (uint8_t)FSM_BRAKE_L1:
            output->alert_type   = (uint8_t)ALERT_BOTH;
            output->alert_active = 1U;
            output->buzzer_cmd   = (uint8_t)BUZZER_DOUBLE;
            break;

        case (uint8_t)FSM_BRAKE_L2:
            output->alert_type   = (uint8_t)ALERT_BOTH;
            output->alert_active = 1U;
            output->buzzer_cmd   = (uint8_t)BUZZER_FAST_PULSE;
            break;

        case (uint8_t)FSM_BRAKE_L3:
            output->alert_type   = (uint8_t)ALERT_BOTH;
            output->alert_active = 1U;
            output->buzzer_cmd   = (uint8_t)BUZZER_CONTINUOUS;
            break;

        case (uint8_t)FSM_OFF:        /* fall-through */
        case (uint8_t)FSM_STANDBY:    /* fall-through */
        case (uint8_t)FSM_POST_BRAKE: /* fall-through */
        default:
            output->alert_type   = (uint8_t)ALERT_NONE;
            output->alert_active = 0U;
            output->buzzer_cmd   = (uint8_t)BUZZER_SILENT;
            break;
    }
}

uint8_t override_detect(float32_t steering_angle, uint8_t brake_pedal)
{
    /*
     * Driver override detection — derived from Simulink Override_Det (chart_88).
     *
     * The function is pure combinational: returns 1 if the driver is
     * actively intervening via brake pedal or evasive steering.
     *
     * Conditions (OR logic):
     *   - brake_pedal != 0                          (FR-DEC-006)
     *   - |steering_angle| > STEERING_OVERRIDE_DEG  (FR-DEC-007, 5 degrees)
     *
     * Note: In POST_BRAKE state, only the accelerator pedal triggers
     * override (FR-FSM-006). That exception is handled by the FSM module
     * (Task B), which ignores brake_pedal and steering_angle override
     * signals when in POST_BRAKE. This function always reports the
     * general-case override condition.
     */

    uint8_t result = 0U;

    if (brake_pedal != 0U)
    {
        result = 1U;
    }
    else if (abs_f32(steering_angle) > STEERING_OVERRIDE_DEG)
    {
        result = 1U;
    }
    else
    {
        result = 0U;
    }

    return result;
}
