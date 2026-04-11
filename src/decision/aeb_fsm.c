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

    /* Defensive checks */
    if ((delta_t_s <= 0.0f) || (perception == NULL) ||
        (driver == NULL) || (ttc_in == NULL) || (fsm_out == NULL))
    {
        return;
    }

    current_state = (fsm_state_e)fsm_out->fsm_state;
    desired_state = evaluate_desired_state(perception, ttc_in);

    /* ===== PRIORITY 1: Fault and Safety ===== */
    if ((perception->fault_flag != 0U) || (driver->aeb_enabled == 0U))
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
    driver_override = (driver->brake_pedal != 0U) ||
                      (driver->steering_angle > STEERING_OVERRIDE_DEG) ||
                      (driver->steering_angle < -STEERING_OVERRIDE_DEG);

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
    speed_out_of_range = (perception->v_ego < V_EGO_MIN) ||
                         (perception->v_ego > V_EGO_MAX);

    if (speed_out_of_range != 0U)
    {
        if ((perception->v_ego < 0.01f) && (current_state >= FSM_BRAKE_L1))
        {
            fsm_out->fsm_state = (uint8_t)FSM_POST_BRAKE;
            m_fsm_mem.post_brake_timer_s = 0.0f;
        }
        else
        {
            fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
        }
        fsm_out->brake_active = 0U;
        fsm_out->alert_level = 0U;
        fsm_out->decel_target = 0.0f;
        fsm_out->state_timer += delta_t_s;
        return;
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
            break;

        case FSM_BRAKE_L1:
        case FSM_BRAKE_L2:
        case FSM_BRAKE_L3:
            if (perception->v_ego < 0.01f)
            {
                new_state = FSM_POST_BRAKE;
                m_fsm_mem.post_brake_timer_s = 0.0f;
            }
            else if (desired_state > current_state)
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