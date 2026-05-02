/**
 * @file aeb_fsm.c
 * @brief Implementation of 7-state AEB Finite State Machine.
 * @author Lourenço Jamba Mphili
 * @requirements FR-FSM-001 to FR-FSM-006, FR-DEC-004 to FR-DEC-011
 * @misra Fully compliant with MISRA C:2012
 */

#include "aeb_fsm.h"
#include "aeb_config.h"
#include "aeb_types.h"
#include <math.h>    /* isfinite() — Bug #3 guard in fsm_step entry */
#include <stddef.h>

/* Steering override threshold (degrees) - FR-FSM-006 */
#ifndef STEERING_OVERRIDE_DEG
#define STEERING_OVERRIDE_DEG   5.0f
#endif

/* FSM internal persistent memory (private to this module) */
static struct {
    float32_t warn_timer_s;       /**< Time spent in WARNING state */
    float32_t debounce_timer_s;   /**< Debounce timer for de-escalation */
    float32_t post_brake_timer_s; /**< Hold timer for POST_BRAKE state */
} m_fsm_mem = {0.0f, 0.0f, 0.0f};

/**
 * @brief Converts FSM state to deceleration target.
 * @requirement FR-BRK-001, SRS Table 10
 */
static float32_t state_to_decel_target(fsm_state_e state)
{
    float32_t decel = 0.0f;

    switch (state) {
        case FSM_BRAKE_L1:
            decel = DECEL_L1;  /* 2.0f m/s² */
            break;
        case FSM_BRAKE_L2:
            decel = DECEL_L2;  /* 4.0f m/s² */
            break;
        case FSM_BRAKE_L3:
            decel = DECEL_L3;  /* 6.0f m/s² */
            break;
        default:
            decel = 0.0f;
            break;
    }

    return decel;
}

/**
 * @brief Converts FSM state to alert level (0..2)
 */
static uint8_t state_to_alert_level(fsm_state_e state)
{
    uint8_t alert = 0U;

    if ((state == FSM_WARNING) || (state == FSM_BRAKE_L1))
    {
        alert = 1U;  /* Warning alert */
    }
    else if ((state == FSM_BRAKE_L2) || (state == FSM_BRAKE_L3))
    {
        alert = 2U;  /* Emergency alert */
    }
    else
    {
        alert = 0U;  /* No alert */
    }

    return alert;
}

/**
 * @brief Evaluates desired state based on TTC, distance, and braking distance.
 * @return Desired FSM state
 * @requirement FR-DEC-004, FR-DEC-005
 */
static fsm_state_e evaluate_desired_state(const perception_output_t * const perception,
                                           const ttc_output_t * const ttc_in)
{
    fsm_state_e desired = FSM_STANDBY;

    /* FR-DEC-004: No threat if not closing */
    if (ttc_in->is_closing == 0U)
    {
        desired = FSM_STANDBY;
    }
    /* FR-DEC-005: Braking floor condition (highest priority) */
    else if (ttc_in->d_brake >= perception->distance)
    {
        desired = FSM_BRAKE_L3;
    }
    /* Distance floors - per SRS Table 10 */
    else if (perception->distance <= 5.0f)
    {
        desired = FSM_BRAKE_L3;
    }
    else if (perception->distance <= 10.0f)
    {
        desired = FSM_BRAKE_L2;
    }
    else if (perception->distance <= 20.0f)
    {
        desired = FSM_BRAKE_L1;
    }
    /* TTC-based evaluation - per SRS Table 10 */
    else if (ttc_in->ttc <= TTC_BRAKE_L3)
    {
        desired = FSM_BRAKE_L3;
    }
    else if (ttc_in->ttc <= TTC_BRAKE_L2)
    {
        desired = FSM_BRAKE_L2;
    }
    else if (ttc_in->ttc <= TTC_BRAKE_L1)
    {
        desired = FSM_BRAKE_L1;
    }
    else if (ttc_in->ttc <= TTC_WARNING)
    {
        desired = FSM_WARNING;
    }
    else
    {
        desired = FSM_STANDBY;
    }

    return desired;
}

void fsm_init(fsm_output_t * const fsm_out)
{
    if (fsm_out != NULL)
    {
        /* FR-FSM-001: Initial state is STANDBY */
        fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
        fsm_out->decel_target = 0.0f;
        fsm_out->brake_active = 0U;
        fsm_out->alert_level = 0U;
        fsm_out->warn_timer = 0.0f;
        fsm_out->state_timer = 0.0f;

        /* Reset internal memory */
        m_fsm_mem.warn_timer_s = 0.0f;
        m_fsm_mem.debounce_timer_s = 0.0f;
        m_fsm_mem.post_brake_timer_s = 0.0f;
    }
}

void fsm_step(float32_t delta_t_s,
              const perception_output_t * const perception,
              const driver_input_t * const driver,
              const ttc_output_t * const ttc_in,
              fsm_output_t * const fsm_out)
{
    fsm_state_e current_state;
    fsm_state_e desired_state;
    fsm_state_e new_state;
    uint8_t driver_override = 0U;
    uint8_t speed_out_of_range = 0U;
    uint8_t aeb_enabled_norm;

    /* Defensive checks.
     * Bug #3: reject non-finite delta_t_s. IEEE 754 comparisons with NaN
     * are always false, so (delta_t_s <= 0.0f) alone lets NaN through.
     * A NaN delta_t would poison the warn_timer / debounce_timer
     * accumulators on the first `+= delta_t_s`, silently breaking
     * debounce, escalation, and POST_BRAKE timeout logic. */
    if ((!isfinite(delta_t_s)) || (delta_t_s <= 0.0f) ||
        (perception == NULL) || (driver == NULL) ||
        (ttc_in == NULL) || (fsm_out == NULL))
    {
        return;
    }

    /* Bug #2 SEU hardening: normalise driver->aeb_enabled to a strict
     * Boolean at entry. A bit-flip that turns 1U into (say) 0xAA would
     * otherwise make (aeb_enabled == 0U) false and leave the FSM running
     * on corrupted state. Strict match: only the canonical 1U bit pattern
     * counts as enabled. Any other value (0U, SEU-corrupted bit patterns,
     * garbage from uninitialised memory) falls safe to disabled → FSM_OFF
     * via the Priority-1 check below. */
    aeb_enabled_norm = (driver->aeb_enabled == 1U) ? 1U : 0U;

    current_state = (fsm_state_e)fsm_out->fsm_state;

    /* Bug #4 SEU hardening: normalise out-of-enum fsm_state to the
     * fail-safe FSM_STANDBY. A bit-flip in RAM could corrupt the
     * persisted fsm_state byte (e.g. 1 -> 42), which would then be
     * inherited by new_state at the top of the transition logic and,
     * on iterations where desired_state resolves to STANDBY, would be
     * written back unchanged into fsm_out->fsm_state — and broadcast
     * via CAN/UDS to actuators and displays. STANDBY is the conservative
     * choice: zero braking demand, no alert. */
    if ((uint8_t)current_state > (uint8_t)FSM_POST_BRAKE)
    {
        current_state = FSM_STANDBY;
        fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
    }

    desired_state = evaluate_desired_state(perception, ttc_in);

    /* ===== PRIORITY 1: Fault and Safety ===== */
    if ((perception->fault_flag != 0U) || (aeb_enabled_norm == 0U))
    {
        fsm_out->fsm_state = (uint8_t)FSM_OFF;
        fsm_out->brake_active = 0U;
        fsm_out->alert_level = 0U;
        fsm_out->decel_target = 0.0f;
        fsm_out->warn_timer = 0.0f;
        fsm_out->state_timer = 0.0f;
        m_fsm_mem.warn_timer_s = 0.0f;
        m_fsm_mem.debounce_timer_s = 0.0f;
        m_fsm_mem.post_brake_timer_s = 0.0f;
        return;
    }

    /* ===== PRIORITY 2: Driver Override ===== */
    driver_override = ((driver->brake_pedal != 0U) ||
                       (driver->steering_angle > STEERING_OVERRIDE_DEG) ||
                       (driver->steering_angle < -STEERING_OVERRIDE_DEG)) ? 1U : 0U;

    if ((driver_override != 0U) && (current_state != FSM_POST_BRAKE))
    {
        fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
        fsm_out->brake_active = 0U;
        fsm_out->alert_level = 0U;
        fsm_out->decel_target = 0.0f;
        fsm_out->warn_timer = 0.0f;
        fsm_out->state_timer = 0.0f;
        m_fsm_mem.warn_timer_s = 0.0f;
        m_fsm_mem.debounce_timer_s = 0.0f;
        return;
    }

    /* ===== PRIORITY 3: Speed Range Validation ===== */
    speed_out_of_range = ((perception->v_ego < V_EGO_MIN) ||
                          (perception->v_ego > V_EGO_MAX)) ? 1U : 0U;

    if (speed_out_of_range != 0U)
    {
        if ((perception->v_ego < 0.01f) && (current_state >= FSM_BRAKE_L1))
        {
            /* Vehicle has come to a full stop while braking — hand off to
             * POST_BRAKE for the brake-hold timer. */
            fsm_out->fsm_state = (uint8_t)FSM_POST_BRAKE;
            m_fsm_mem.post_brake_timer_s = 0.0f;
            fsm_out->brake_active = 0U;
            fsm_out->alert_level = 0U;
            fsm_out->decel_target = 0.0f;
            fsm_out->state_timer += delta_t_s;
            return;
        }
        if (current_state < FSM_BRAKE_L1)
        {
            /* Not braking and out-of-range: drop cleanly to STANDBY. */
            fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
            fsm_out->brake_active = 0U;
            fsm_out->alert_level = 0U;
            fsm_out->decel_target = 0.0f;
            fsm_out->state_timer += delta_t_s;
            return;
        }
        /* Actively braking with v_ego in [0.01, V_EGO_MIN): fall through
         * to Priority 4. The distance-floor logic in evaluate_desired_state
         * resolves to BRAKE_L1/L2/L3 while the ego is still close to the
         * target, so the brake stays applied until v_ego < 0.01 hits the
         * branch above and hands off to POST_BRAKE.
         *
         * Closes #95: previously every speed_out_of_range case forced
         * STANDBY and zeroed the brake — the ego coasted the last few
         * metres of every CCRs run with no braking command, missing the
         * Euro NCAP residual-speed criterion. The SIL stack worked
         * around this with a build-time string patch
         * (sil/zephyr_aeb/patch_fsm.py); with this change the patch can
         * be removed. */
    }

    /* ===== PRIORITY 4: State Transition Logic ===== */
    new_state = current_state;

    switch (current_state)
    {
        case FSM_POST_BRAKE:
            m_fsm_mem.post_brake_timer_s += delta_t_s;
            if ((m_fsm_mem.post_brake_timer_s >= 2.0f) ||
                (driver->accel_pedal != 0U))
            {
                new_state = FSM_STANDBY;
                m_fsm_mem.post_brake_timer_s = 0.0f;
            }
            break;

        case FSM_WARNING:
            m_fsm_mem.warn_timer_s += delta_t_s;
            if (desired_state == FSM_STANDBY)
            {
                m_fsm_mem.debounce_timer_s += delta_t_s;
                if (m_fsm_mem.debounce_timer_s >= 0.2f)
                {
                    new_state = desired_state;
                    m_fsm_mem.warn_timer_s = 0.0f;
                    m_fsm_mem.debounce_timer_s = 0.0f;
                }
            }
            else if ((desired_state == FSM_BRAKE_L1) ||
                     (desired_state == FSM_BRAKE_L2) ||
                     (desired_state == FSM_BRAKE_L3))
            {
                if (m_fsm_mem.warn_timer_s >= 0.8f)
                {
                    new_state = desired_state;
                    m_fsm_mem.debounce_timer_s = 0.0f;
                }
            }
            else
            {
                /* MISRA C:2012 Rule 15.7: terminal else clause.
                 * Reached when desired_state is WARNING (same as current)
                 * — no state change, no timer reset. */
            }
            break;

        case FSM_BRAKE_L1:
        case FSM_BRAKE_L2:
        case FSM_BRAKE_L3:
            /* D3: the former `if (perception->v_ego < 0.01f)` branch
             * that transitioned to FSM_POST_BRAKE was unreachable — it
             * is dominated by the Priority-3 speed guard above, which
             * fires on v_ego < V_EGO_MIN = 2.78 m/s before control
             * ever reaches this switch. Removed per V&V report Sec. 7.5
             * desvio D3 (MISRA C:2012 Rule 2.1). Low-speed exit into
             * POST_BRAKE is now handled exclusively by the speed guard. */
            if (desired_state > current_state)
            {
                /* Escalation: immediate */
                new_state = desired_state;
                m_fsm_mem.debounce_timer_s = 0.0f;
            }
            else if (desired_state < current_state)
            {
                /* De-escalation: requires 200ms debounce */
                m_fsm_mem.debounce_timer_s += delta_t_s;
                if (m_fsm_mem.debounce_timer_s >= 0.2f)
                {
                    new_state = desired_state;
                    m_fsm_mem.debounce_timer_s = 0.0f;
                }
            }
            else
            {
                m_fsm_mem.debounce_timer_s = 0.0f;
            }
            break;

        case FSM_STANDBY:
        default:
            m_fsm_mem.warn_timer_s = 0.0f;
            m_fsm_mem.debounce_timer_s = 0.0f;
            /* FR-DEC-011: STANDBY must go to WARNING first */
            if (desired_state == FSM_WARNING)
            {
                new_state = FSM_WARNING;
            }
            else if ((desired_state == FSM_BRAKE_L1) ||
                     (desired_state == FSM_BRAKE_L2) ||
                     (desired_state == FSM_BRAKE_L3))
            {
                /* Force pass through WARNING (FR-DEC-011) */
                new_state = FSM_WARNING;
            }
            else
            {
                /* MISRA C:2012 Rule 15.7: terminal else clause.
                 * Reached when desired_state is STANDBY or POST_BRAKE
                 * — no transition, stay in STANDBY. */
            }
            break;
    }

    /* Update timers */
    if (new_state != current_state)
    {
        fsm_out->state_timer = 0.0f;
    }
    else
    {
        fsm_out->state_timer += delta_t_s;
    }

    /* Write outputs */
    fsm_out->fsm_state = (uint8_t)new_state;
    fsm_out->warn_timer = m_fsm_mem.warn_timer_s;
    fsm_out->brake_active = ((new_state >= FSM_BRAKE_L1) && (new_state <= FSM_BRAKE_L3)) ? 1U : 0U;
    fsm_out->alert_level = state_to_alert_level(new_state);
    fsm_out->decel_target = state_to_decel_target(new_state);
}