/**
 * @file    aeb_pid.c
 * @brief   PI brake controller — full implementation.
 * @owner   Task C (Jéssica Roberta de Souza Santos)
 * @version 1.0
 * @date    2026-04-09
 *
 * @details Implements a PI controller with anti-windup clamping and jerk
 *          limiting, generating a brake command in [0, 100]% and a brake
 *          pressure in [0, 10] bar. The algorithm is derived from the
 *          Simulink chart_96 (PID_Logic) in AEB_Integration.slx.
 *
 *          Data flow in Simulink (system_107.xml):
 *            FSM.decel_target -> PID_Brake.in:1
 *            a_ego (inport 4) -> PID_Brake.in:2
 *            FSM.fsm_state   -> PID_Brake.in:3
 *            PID_Brake.out:1 -> brake_pct
 *            PID_Brake.out:2 -> brake_bar
 *
 * @requirements FR-BRK-001 to FR-BRK-007
 *
 * @req FR-BRK-001  Progressive deceleration, no jumps > 2 m/s^2 between cycles.
 * @req FR-BRK-002  PI controller with anti-windup, error < 0.5 m/s^2 in 500 ms.
 * @req FR-BRK-003  Capable of generating >= 5.0 m/s^2 braking demand.
 * @req FR-BRK-004  Jerk limited to |jerk| <= 10 m/s^3.
 * @req FR-BRK-005  Maintain braking >= 2 s after full stop (POST_BRAKE).
 * @req FR-BRK-006  Release brake within 1 cycle after accelerator in POST_BRAKE.
 * @req FR-BRK-007  Output clamped to [0, 100]%.
 *
 * @standard MISRA C:2012 compliant.
 */

#include "aeb_pid.h"
#include "aeb_config.h"

/* ========================================================================= */
/*  Module-internal persistent state                                         */
/* ========================================================================= */

/** @brief Integral accumulator for the PI controller [%]. */
static float32_t s_integral = 0.0F;

/** @brief Previous cycle brake output for jerk limiting [%]. */
static float32_t s_prev_brake_pct = 0.0F;

/* ========================================================================= */
/*  Private helper — clamp a value to [lo, hi]                               */
/* ========================================================================= */

/**
 * @brief Clamp a float32 value to a specified range.
 * @param[in] val  Value to clamp.
 * @param[in] lo   Lower bound.
 * @param[in] hi   Upper bound.
 * @return Clamped value.
 */
static float32_t clamp_f32(float32_t val, float32_t lo, float32_t hi)
{
    float32_t result = val;

    if (result < lo)
    {
        result = lo;
    }
    else if (result > hi)
    {
        result = hi;
    }
    else
    {
        /* value already within range — no action */
    }

    return result;
}

/* ========================================================================= */
/*  Public API                                                               */
/* ========================================================================= */

void pid_init(void)
{
    s_integral      = 0.0F;
    s_prev_brake_pct = 0.0F;
}

void pid_brake_step(float32_t decel_target, float32_t a_ego,
                    uint8_t fsm_state, pid_output_t *output)
{
    float32_t error       = 0.0F;
    float32_t p_term      = 0.0F;
    float32_t raw_output  = 0.0F;
    float32_t max_delta   = 0.0F;
    float32_t delta       = 0.0F;
    float32_t brake_pct   = 0.0F;

    /* ------------------------------------------------------------------
     * Guard: controller is only active in braking states (L1, L2, L3)
     * and POST_BRAKE. In all other states, reset and output zero.
     *
     * FSM_BRAKE_L1 = 3, FSM_BRAKE_L2 = 4, FSM_BRAKE_L3 = 5,
     * FSM_POST_BRAKE = 6
     *
     * Rationale: decel_target is zero outside braking states, but we
     * also reset the integrator to avoid wind-up carry-over.
     * FR-BRK-006 is handled by the FSM: when the driver presses the
     * accelerator in POST_BRAKE, the FSM transitions to STANDBY and
     * decel_target becomes 0, so the controller resets here.
     *
     * MISRA C:2012 Rule 15.5: single exit point — no early return.
     * ------------------------------------------------------------------ */
    if ((fsm_state < (uint8_t)FSM_BRAKE_L1) || (decel_target <= 0.0F))
    {
        /* Reset controller state */
        s_integral       = 0.0F;
        s_prev_brake_pct = 0.0F;
    }
    else
    {
        /* --------------------------------------------------------------
         * PI control law
         *
         * error = decel_target - a_ego
         *   (positive error means we need more braking)
         *
         * integral += Ki * error * dt
         *   (anti-windup: integral clamped to [0, PID_INTEG_MAX])
         *
         * output = Kp * error + integral
         * -------------------------------------------------------------- */

        /* Compute error: decel_target is positive (e.g. 6.0 m/s^2),
         * a_ego is the measured deceleration (positive when braking). */
        error = decel_target - a_ego;

        /* Proportional term */
        p_term = PID_KP * error;

        /* Integral term with anti-windup (FR-BRK-002) */
        s_integral = s_integral + (PID_KI * error * AEB_DT);
        s_integral = clamp_f32(s_integral, 0.0F, PID_INTEG_MAX);

        /* Combined PI output */
        raw_output = p_term + s_integral;

        /* --------------------------------------------------------------
         * Jerk limiting (FR-BRK-004)
         *
         * Maximum allowed change per cycle:
         *   max_delta = MAX_JERK * (100 / DECEL_L3) * AEB_DT
         *
         * This converts the jerk limit from m/s^3 to %/cycle:
         *   10 m/s^3 * (100% / 6 m/s^2) * 0.01 s = 1.667 %/cycle
         *
         * This ensures no abrupt deceleration jumps > 2 m/s^2 between
         * cycles (FR-BRK-001), since 1.667% maps to approximately
         * 1.0 m/s^2 change per cycle.
         * -------------------------------------------------------------- */
        max_delta = MAX_JERK * (BRAKE_OUT_MAX / DECEL_L3) * AEB_DT;

        /* Clamp output to valid range first */
        brake_pct = clamp_f32(raw_output, BRAKE_OUT_MIN, BRAKE_OUT_MAX);

        /* Apply jerk limit relative to previous output */
        delta = brake_pct - s_prev_brake_pct;
        delta = clamp_f32(delta, -max_delta, max_delta);
        brake_pct = s_prev_brake_pct + delta;

        /* Final clamp after jerk limiting (FR-BRK-007) */
        brake_pct = clamp_f32(brake_pct, BRAKE_OUT_MIN, BRAKE_OUT_MAX);

        /* Store for next cycle */
        s_prev_brake_pct = brake_pct;
    }

    /* ------------------------------------------------------------------
     * Output assignment — single exit point (MISRA C:2012 Rule 15.5)
     *
     * brake_pct: [0, 100] %  (FR-BRK-007)
     * brake_bar: [0, 10] bar (linear mapping: bar = pct / 10)
     *
     * When guard resets the controller, brake_pct remains 0.0F
     * (initialised at declaration), producing zero output.
     * ------------------------------------------------------------------ */
    output->brake_pct = brake_pct;
    output->brake_bar = brake_pct / 10.0F;
}
