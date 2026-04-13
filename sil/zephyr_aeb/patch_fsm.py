#!/usr/bin/env python3
"""
Patch aeb_fsm.c: fix speed range check to continue braking below V_EGO_MIN.
"""
import sys

fsm_file = sys.argv[1] if len(sys.argv) > 1 else "/app/zephyr_aeb/repo/src/decision/aeb_fsm.c"

with open(fsm_file, 'r', encoding='utf-8') as f:
    code = f.read()

old = """        else
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

new = """        else if (current_state < FSM_BRAKE_L1)
        {
            /* Not braking: go to STANDBY */
            fsm_out->fsm_state = (uint8_t)FSM_STANDBY;
            fsm_out->brake_active = 0U;
            fsm_out->alert_level = 0U;
            fsm_out->decel_target = 0.0f;
            fsm_out->state_timer += delta_t_s;
            return;
        }
        /* else: actively braking below V_EGO_MIN — fall through
         * to Priority 4 where distance floors keep brakes applied
         * until vehicle fully stops (v < 0.01 → POST_BRAKE) */
    }

    /* ===== PRIORITY 4: State Transition Logic ===== */"""

if old in code:
    code = code.replace(old, new)
    with open(fsm_file, 'w', encoding='utf-8') as f:
        f.write(code)
    print("PATCHED: speed range check now allows continued braking")
else:
    print("ERROR: pattern not found")
    # Debug: show the actual text around speed_out_of_range
    lines = code.split('\n')
    for i, line in enumerate(lines):
        if 'speed_out_of_range' in line or 'FSM_STANDBY' in line:
            print(f"  L{i+1}: {line.rstrip()}")
    sys.exit(1)
