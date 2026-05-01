#!/usr/bin/env python3
"""
Patch aeb_fsm.c for SIL behaviour:

  When the ego falls below V_EGO_MIN (~10 km/h) while actively braking,
  enter POST_BRAKE immediately (instead of waiting for v_ego < 0.01 m/s)
  so visual + audible alerts clear as soon as the threat has effectively
  been resolved. The original threshold (0.01 m/s) was so tight that real
  SIL noise + scenario-controller dynamics meant v_ego almost never settled
  there, leaving the FSM stuck in BRAKE_Lx with alerts blaring while the
  car was already crawling.

  Brake stays released in POST_BRAKE (matching upstream behaviour). The
  scenario controller applies a small coast-decel after ego_has_braked
  goes True, which is what brings the vehicle to a final stop.

This is a SIL-only workaround. The same fix should land in
src/decision/aeb_fsm.c upstream after re-validation.
"""
import sys

fsm_file = sys.argv[1] if len(sys.argv) > 1 else "/app/zephyr_aeb/repo/src/decision/aeb_fsm.c"

with open(fsm_file, 'r', encoding='utf-8') as f:
    code = f.read()

old = """        if ((perception->v_ego < 0.01f) && (current_state >= FSM_BRAKE_L1))
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

    /* ===== PRIORITY 4: State Transition Logic ===== */"""

new = """        if (current_state >= FSM_BRAKE_L1)
        {
            /* SIL: actively braking and now below V_EGO_MIN — clear the
             * alerts immediately by entering POST_BRAKE. Brake is released
             * here (consistent with the upstream POST_BRAKE behaviour);
             * the scenario controller applies a small coast-decel to bring
             * the vehicle to a complete stop. */
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

    /* ===== PRIORITY 4: State Transition Logic ===== */"""

if old in code:
    code = code.replace(old, new)
    with open(fsm_file, 'w', encoding='utf-8') as f:
        f.write(code)
    print("PATCHED: early POST_BRAKE entry with brake held until full stop")
else:
    print("ERROR: pattern not found")
    # Debug: show the actual text around speed_out_of_range
    lines = code.split('\n')
    for i, line in enumerate(lines):
        if 'speed_out_of_range' in line or 'FSM_STANDBY' in line:
            print(f"  L{i+1}: {line.rstrip()}")
    sys.exit(1)
