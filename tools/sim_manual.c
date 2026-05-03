/**
 * @file  sim_manual.c
 * @brief AEB manual simulator -- interactive terminal input.
 *
 * Allows entering sensor and driver values manually and observing
 * system behavior (FSM, braking, alerts, TTC) cycle by cycle.
 * Also supports UDS diagnostic requests and CAN bus inspection.
 *
 * Build:  make sim_manual
 * Run:    ./sim_manual
 */

#include <stdio.h>
#include <string.h>
#include "aeb_core.h"
#include "aeb_can.h"
#include "aeb_uds.h"
#include "aeb_config.h"
#include "can_hal.h"
#include "can_hal_test.h"

/* TX capture buffer capacity -- must match TX_BUF_SIZE in stubs/can_hal.c */
#define CAN_TX_BUF_CAP 32U

/* =========================================================================
 *  Input helpers
 * ========================================================================= */

static int32_t read_float(const char *prompt, float32_t *out)
{
    printf("%s", prompt);
    if (scanf("%f", out) != 1)
    {
        printf("  Invalid input -- value unchanged.\n");
        return 0;
    }
    return 1;
}

static int32_t read_int(const char *prompt, int32_t *out)
{
    printf("%s", prompt);
    if (scanf("%d", out) != 1)
    {
        printf("  Invalid input -- value unchanged.\n");
        return 0;
    }
    return 1;
}

/* =========================================================================
 *  State description helpers
 * ========================================================================= */

static const char *fsm_name(uint8_t s)
{
    switch (s)
    {
        case 0:  return "OFF         - system disabled (fault or speed out of range)";
        case 1:  return "STANDBY     - monitoring, no threat detected";
        case 2:  return "WARNING     - obstacle detected, alert issued!";
        case 3:  return "BRAKE L1    - light braking  (2.0 m/s2)";
        case 4:  return "BRAKE L2    - moderate braking (4.0 m/s2)";
        case 5:  return "BRAKE L3    - EMERGENCY full braking (6.0 m/s2)!";
        case 6:  return "POST_BRAKE  - post-braking hold (2 s)";
        default: return "UNKNOWN";
    }
}

static const char *fsm_short(uint8_t s)
{
    switch (s)
    {
        case 0:  return "OFF";
        case 1:  return "STANDBY";
        case 2:  return "WARNING";
        case 3:  return "BRAKE_L1";
        case 4:  return "BRAKE_L2";
        case 5:  return "BRAKE_L3";
        case 6:  return "POST_BRK";
        default: return "???";
    }
}

static const char *alert_name(uint8_t t)
{
    switch (t)
    {
        case 0:  return "None";
        case 1:  return "VISUAL (light)";
        case 2:  return "AUDIBLE (buzzer)";
        case 3:  return "VISUAL + AUDIBLE";
        default: return "Unknown";
    }
}

static const char *can_id_name(uint32_t id)
{
    switch (id)
    {
        case 0x080U: return "AEB_BrakeCmd";
        case 0x100U: return "AEB_EgoVehicle";
        case 0x101U: return "AEB_DriverInput";
        case 0x120U: return "AEB_RadarTarget";
        case 0x200U: return "AEB_FSMState";
        case 0x300U: return "AEB_Alert";
        default:     return "Unknown";
    }
}

/* =========================================================================
 *  CAN frame injection
 * ========================================================================= */

static void inject_driver(aeb_core_state_t *st,
                           uint8_t brake, uint8_t accel, uint8_t aeb_en)
{
    uint8_t frame[8] = {0};
    can_pack_signal(frame, 0,  1, (uint32_t)brake);
    can_pack_signal(frame, 8,  1, (uint32_t)accel);
    can_pack_signal(frame, 16, 1, (uint32_t)aeb_en);
    can_rx_process(&st->can, CAN_ID_DRIVER_INPUT, frame, 4U);
}

static void inject_radar(aeb_core_state_t *st,
                          float32_t dist, float32_t v_rel)
{
    uint8_t  frame[8] = {0};
    uint32_t raw_dist = (uint32_t)(dist  / 0.01F);
    /* DBC offset/factor for RelativeSpeed signal -- see aeb_can.c */
    uint32_t raw_vrel = (uint32_t)((v_rel + 327.68F) / 0.01F);
    can_pack_signal(frame, 0,  16, raw_dist);
    can_pack_signal(frame, 16, 16, raw_vrel);
    can_rx_process(&st->can, CAN_ID_RADAR_TARGET, frame, 8U);
}

/* =========================================================================
 *  Print system state
 * ========================================================================= */

static void print_state(const aeb_core_state_t *st, int32_t cycle)
{
    printf("\n+---------------------------------------------+\n");
    printf("| SYSTEM STATE  --  cycle %-4d               |\n", (int)cycle);
    printf("+---------------------------------------------+\n");

    printf("| PERCEPTION\n");
    printf("|   Obstacle distance : %.1f m\n",
           (double)st->perception.distance);
    printf("|   Ego speed         : %.1f m/s  (%.1f km/h)\n",
           (double)st->perception.v_ego,
           (double)(st->perception.v_ego * 3.6F));
    printf("|   Relative speed    : %.1f m/s  (>0 = closing)\n",
           (double)st->perception.v_rel);
    printf("|   Sensor confidence : %.0f %%\n",
           (double)(st->perception.confidence * 100.0F));

    if (st->perception.fault_flag != 0U)
    {
        printf("|   *** SENSOR FAULT DETECTED  (flag=0x%02X) ***\n",
               (unsigned)st->perception.fault_flag);
    }

    printf("+---------------------------------------------+\n");
    printf("| COLLISION CALCULATION\n");
    if (st->ttc.is_closing)
    {
        printf("|   TTC (time to collision) : %.2f s  <<< CLOSING!\n",
               (double)st->ttc.ttc);
    }
    else
    {
        printf("|   TTC (time to collision) : ---  (not closing)\n");
    }
    printf("|   Required braking distance : %.1f m\n",
           (double)st->ttc.d_brake);

    printf("+---------------------------------------------+\n");
    printf("| FSM STATE\n");
    printf("|   %s\n", fsm_name(st->fsm.fsm_state));
    printf("|   Decel target : %.1f m/s2\n",
           (double)st->fsm.decel_target);

    printf("+---------------------------------------------+\n");
    printf("| AUTONOMOUS BRAKING\n");
    if (st->fsm.brake_active)
    {
        printf("|   Brake ACTIVE : %.1f %%  /  %.2f bar\n",
               (double)st->pid.brake_pct,
               (double)st->pid.brake_bar);
    }
    else
    {
        printf("|   Brake inactive\n");
    }

    printf("+---------------------------------------------+\n");
    printf("| ALERT : %s  (active=%s)\n",
           alert_name(st->alert.alert_type),
           st->alert.alert_active ? "YES" : "no");

    if (st->override_active)
    {
        printf("| *** DRIVER OVERRIDE DETECTED -- AEB suspended ***\n");
    }

    printf("+---------------------------------------------+\n");
}

/* =========================================================================
 *  CAN bus inspection
 * ========================================================================= */

static void print_can_bus(const aeb_core_state_t *st)
{
    can_rx_data_t rx;
    uint32_t      tx_count;
    uint32_t      i;

    can_get_rx_data(&st->can, &rx);

    printf("\n+---------------------------------------------+\n");
    printf("| CAN BUS INSPECTION\n");
    printf("+---------------------------------------------+\n");

    printf("| LAST RECEIVED (RX)\n");
    printf("|   [0x100] AEB_EgoVehicle\n");
    printf("|     Vehicle speed   : %.2f m/s  (%.1f km/h)\n",
           (double)rx.vehicle_speed, (double)(rx.vehicle_speed * 3.6F));
    printf("|     Long accel      : %.3f m/s2\n",  (double)rx.long_accel);
    printf("|     Yaw rate        : %.2f deg/s\n", (double)rx.yaw_rate);
    printf("|     Steering angle  : %.1f deg\n",   (double)rx.steering_angle);
    printf("|   [0x101] AEB_DriverInput\n");
    printf("|     Brake pedal     : %s\n", rx.brake_pedal  ? "PRESSED" : "released");
    printf("|     Accel pedal     : %s\n", rx.accel_pedal  ? "PRESSED" : "released");
    printf("|     AEB enable      : %s\n", rx.aeb_enable   ? "YES"     : "NO");
    printf("|     Driver override : %s\n", rx.driver_override ? "YES"  : "NO");
    printf("|   [0x120] AEB_RadarTarget\n");
    printf("|     Target distance : %.2f m\n",  (double)rx.target_distance);
    printf("|     Relative speed  : %.2f m/s\n",(double)rx.relative_speed);
    printf("|     RX timeout flag : %s\n",
           rx.rx_timeout_flag ? "TIMEOUT!" : "OK");

    printf("+---------------------------------------------+\n");

    tx_count = can_hal_test_get_tx_count();
    printf("| TRANSMITTED (TX) -- %u frame(s) captured%s\n",
           (unsigned)tx_count,
           (tx_count >= CAN_TX_BUF_CAP) ? " (buffer full -- older frames dropped)" : "");

    for (i = 0U; i < tx_count && i < CAN_TX_BUF_CAP; i++)
    {
        const tx_record_t *rec = can_hal_test_get_tx(i);
        if (rec != NULL)
        {
            uint8_t b;
            printf("|   [0x%03X] %-16s DLC=%u  data:",
                   (unsigned)rec->id,
                   can_id_name(rec->id),
                   (unsigned)rec->dlc);
            for (b = 0U; b < rec->dlc; b++)
            {
                printf(" %02X", (unsigned)rec->data[b]);
            }
            printf("\n");
        }
    }

    if (tx_count == 0U)
    {
        printf("|   (no TX frames captured -- press 'r' to run cycles first)\n");
    }

    printf("+---------------------------------------------+\n");
}

/* =========================================================================
 *  UDS diagnostic menu
 * ========================================================================= */

static void print_uds_response(const uds_response_t *resp)
{
    printf("\n  UDS Response:\n");

    if (resp->response_sid == UDS_SID_READ_DID_RESP)         /* 0x62 */
    {
        uint16_t did = ((uint16_t)resp->did_high_resp << 8U)
                     | (uint16_t)resp->did_low_resp;

        printf("  SID 0x62 - ReadDataByIdentifier positive response\n");
        printf("  DID: 0x%04X\n", (unsigned)did);

        if (did == UDS_DID_TTC_VAL)
        {
            uint16_t raw = (uint16_t)resp->data1
                         | ((uint16_t)resp->data2 << 8U);
            printf("  TTC = %.2f s\n", (double)(raw / 100.0F));
        }
        else if (did == UDS_DID_FSM_STATE_VAL)
        {
            printf("  FSM state = %u (%s)\n",
                   (unsigned)resp->data1, fsm_name(resp->data1));
        }
        else if (did == UDS_DID_BRAKE_PRESS_VAL)
        {
            uint16_t raw = (uint16_t)resp->data1
                         | ((uint16_t)resp->data2 << 8U);
            printf("  Brake pressure = %.1f %%\n", (double)(raw / 10.0F));
        }
    }
    else if (resp->response_sid == UDS_SID_CLEAR_DTC_RESP)   /* 0x54 */
    {
        printf("  SID 0x54 - ClearDTC positive response: all DTCs cleared\n");
    }
    else if (resp->response_sid == UDS_SID_ROUTINE_CTRL_RESP) /* 0x71 */
    {
        printf("  SID 0x71 - RoutineControl positive response\n");
        printf("  AEB enabled = %s\n", resp->data1 ? "YES" : "NO");
    }
    else if (resp->response_sid == UDS_SID_NEGATIVE_RESP)    /* 0x7F */
    {
        printf("  SID 0x7F - NEGATIVE RESPONSE\n");
        printf("  Original SID: 0x%02X   NRC: 0x%02X\n",
               (unsigned)resp->did_high_resp,
               (unsigned)resp->did_low_resp);
    }
    else
    {
        printf("  No response (SID=0x00) — send a UDS request first\n");
    }
}

static void uds_menu(aeb_core_state_t *st)
{
    uds_request_t  req;
    uds_response_t resp;
    uds_output_t   out;
    int32_t        in_uds = 1;
    char           choice;

    while (in_uds)
    {
        uds_get_output(&st->uds, &out);

        printf("\n+---------------------------------------------+\n");
        printf("| UDS DIAGNOSTICS MENU\n");
        printf("+---------------------------------------------+\n");
        printf("| DTC status   : %u active DTC(s)  "
               " Fault lamp: %s\n",
               (unsigned)out.dtc_count,
               out.fault_lamp ? "ON" : "off");
        printf("| AEB enabled  : %s\n", out.aeb_enabled ? "YES" : "NO");
        printf("+---------------------------------------------+\n");
        printf("| READ DATA (SID 0x22)\n");
        printf("|  1. DID 0xF100 - TTC value\n");
        printf("|  2. DID 0xF101 - FSM state\n");
        printf("|  3. DID 0xF102 - Brake pressure\n");
        printf("| CLEAR / CONTROL\n");
        printf("|  4. SID 0x14  - Clear all DTCs\n");
        printf("|  5. SID 0x31  - Enable AEB  (RID 0x0301, value=1)\n");
        printf("|  6. SID 0x31  - Disable AEB (RID 0x0301, value=0)\n");
        printf("|  b. Back to main menu\n");
        printf("| Choice: ");

        if (scanf(" %c", &choice) != 1) { break; }

        (void)memset(&req,  0, sizeof(req));
        (void)memset(&resp, 0, sizeof(resp));

        switch (choice)
        {
            case '1':
                req.sid      = UDS_SID_READ_DID;
                req.did_high = 0xF1U;
                req.did_low  = 0x00U;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case '2':
                req.sid      = UDS_SID_READ_DID;
                req.did_high = 0xF1U;
                req.did_low  = 0x01U;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case '3':
                req.sid      = UDS_SID_READ_DID;
                req.did_high = 0xF1U;
                req.did_low  = 0x02U;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case '4':
                req.sid      = UDS_SID_CLEAR_DTC;
                req.did_high = 0xFFU;
                req.did_low  = 0xFFU;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case '5':
                req.sid      = UDS_SID_ROUTINE_CTRL;
                req.did_high = 0x03U;
                req.did_low  = 0x01U;
                req.value    = 1U;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case '6':
                req.sid      = UDS_SID_ROUTINE_CTRL;
                req.did_high = 0x03U;
                req.did_low  = 0x01U;
                req.value    = 0U;
                uds_process_request(&st->uds, &req, &resp,
                                    &st->fsm, &st->pid, &st->ttc);
                print_uds_response(&resp);
                break;

            case 'b':
            case 'B':
                in_uds = 0;
                break;

            default:
                printf("  Invalid option.\n");
                break;
        }
    }
}

/* =========================================================================
 *  Generic linear ramp
 * ========================================================================= */

typedef enum { RAMP_V_EGO, RAMP_V_REL, RAMP_RADAR_D, RAMP_LIDAR_D } ramp_target_t;

static void run_ramp(aeb_core_state_t *st,
                     int32_t         *total_cycle,
                     float32_t       *p_radar_d,
                     float32_t       *p_lidar_d,
                     float32_t       *p_v_ego,
                     float32_t       *p_v_rel,
                     uint8_t          brake,
                     uint8_t          accel,
                     uint8_t          aeb_on,
                     uint8_t          fault_inject,
                     ramp_target_t    target)
{
    static const char *names[] = { "v_ego", "v_rel", "radar_d", "lidar_d" };
    const char        *var_name    = names[(int32_t)target];
    float32_t          v_start     = 0.0F;
    float32_t          v_end       = 0.0F;
    float32_t          duration_s  = 0.0F;
    int32_t            track_vrel  = 0;
    int32_t            n_cycles;
    int32_t            i;
    uint8_t            last_state;

    printf("\n=== %s ramp ===\n", var_name);
    printf("  %s start: ", var_name);
    if (scanf("%f", &v_start) != 1) { return; }
    printf("  %s end  : ", var_name);
    if (scanf("%f", &v_end) != 1) { return; }
    printf("  Duration [s]: ");
    if (scanf("%f", &duration_s) != 1) { return; }

    if (target == RAMP_V_EGO)
    {
        printf("  Track v_rel = v_ego (target stopped)? "
               "[1=yes / 0=keep v_rel=%.2f]: ",
               (double)(*p_v_rel));
        if (scanf("%d", &track_vrel) != 1) { track_vrel = 0; }
    }

    n_cycles = (int32_t)(duration_s / 0.01F + 0.5F);
    if (n_cycles < 1) { n_cycles = 1; }

    printf("\n  Ramp %s: %.3f -> %.3f  |  %d cycles  |  %d ms\n",
           var_name, (double)v_start, (double)v_end, (int)n_cycles, (int)(n_cycles * 10));

    can_hal_test_reset();

    printf("\n%-6s  %-8s  %-10s  %-10s  %-8s  %-9s  %-8s  %-6s\n",
           "Cycle", "Time(ms)", "v_ego m/s", "v_rel m/s",
           "dist(m)", "FSM", "TTC(s)", "Brake");
    printf("------  --------  ----------  ----------  "
           "--------  ---------  --------  ------\n");

    last_state = st->fsm.fsm_state;

    for (i = 0; i < n_cycles; i++)
    {
        float32_t          t;
        float32_t          cur_val;
        raw_sensor_input_t raw;
        uint8_t            now;

        t       = (n_cycles > 1)
                      ? ((float32_t)i / (float32_t)(n_cycles - 1))
                      : 0.0F;
        cur_val = v_start + t * (v_end - v_start);

        switch (target)
        {
            case RAMP_RADAR_D: *p_radar_d = cur_val; break;
            case RAMP_LIDAR_D: *p_lidar_d = cur_val; break;
            case RAMP_V_REL:   *p_v_rel   = cur_val; break;
            case RAMP_V_EGO:
                *p_v_ego = cur_val;
                if (track_vrel != 0) { *p_v_rel = cur_val; }
                break;
            default: break;
        }

        (*total_cycle)++;

        (void)memset(&raw, 0, sizeof(raw));
        raw.radar_d  = *p_radar_d;
        raw.lidar_d  = *p_lidar_d;
        raw.radar_vr = *p_v_rel;
        raw.v_ego    = *p_v_ego;
        raw.fi       = fault_inject;

        inject_radar(st, *p_radar_d, *p_v_rel);
        inject_driver(st, brake, accel, aeb_on);
        aeb_core_step(st, &raw);

        now = st->fsm.fsm_state;

        printf("%-6d  %-8d  %-10.3f  %-10.3f  %-8.1f  %-9s  %-8.2f  %-6s",
               (int)(*total_cycle),
               (int)((*total_cycle) * 10),
               (double)(*p_v_ego),
               (double)(*p_v_rel),
               (double)(*p_radar_d),
               fsm_short(now),
               (double)st->ttc.ttc,
               st->fsm.brake_active ? "YES" : "no");

        if (now != last_state)
        {
            printf("  <<< %s -> %s", fsm_short(last_state), fsm_short(now));
            last_state = now;
        }
        printf("\n");
    }

    printf("\n=== Ramp done.  %d cycles  (%d ms simulated) ===\n",
           (int)n_cycles, (int)(n_cycles * 10));
}

/* =========================================================================
 *  main -- interactive loop
 * ========================================================================= */

int main(void)
{
    aeb_core_state_t   state;
    raw_sensor_input_t raw;
    int32_t            running     = 1;
    int32_t            n_cycles    = 10;
    int32_t            total_cycle = 0;
    char               choice;
    float32_t          kmh;

    /* Default: ego at 40 km/h, stationary target at 50 m */
    float32_t radar_d      = 50.0F;
    float32_t lidar_d      = 50.0F;
    float32_t v_ego        = 11.11F;   /* 40 km/h */
    float32_t v_rel        = 11.11F;   /* target stopped => v_rel = v_ego */
    uint8_t   brake        = 0U;
    uint8_t   accel        = 0U;
    uint8_t   aeb_on       = 1U;
    uint8_t   fault_inject = 0U;

    printf("================================================\n");
    printf("  AEB MANUAL SIMULATOR -- Stellantis\n");
    printf("  Each cycle = 10 ms simulated.\n");
    printf("  Adjust inputs and run to observe behavior.\n");
    printf("================================================\n");

    (void)aeb_core_init(&state);
    inject_driver(&state, 0U, 0U, 1U);
    can_hal_test_reset();

    printf("System initialized. FSM starts in STANDBY.\n");

    while (running)
    {
        kmh = v_ego * 3.6F;

        printf("\n================================================\n");
        printf("  CURRENT INPUTS\n");
        printf("  1. Radar distance    : %.1f m\n",   (double)radar_d);
        printf("  2. LiDAR distance    : %.1f m\n",   (double)lidar_d);
        printf("  3. Ego speed         : %.1f m/s  (%.1f km/h)\n",
               (double)v_ego, (double)kmh);
        printf("  4. Relative speed    : %.1f m/s  (>0 = closing on target)\n",
               (double)v_rel);
        printf("  5. Brake pedal       : %s\n",
               brake ? "PRESSED" : "released");
        printf("  6. Accelerator       : %s\n",
               accel ? "PRESSED" : "released");
        printf("  7. AEB enabled       : %s\n",
               aeb_on ? "YES" : "NO");
        printf("  8. Fault injection   : %s\n",
               fault_inject ? "ACTIVE" : "off");
        printf("  9. Cycles to run     : %d  (= %d ms)\n",
               (int)n_cycles, (int)(n_cycles * 10));
        printf("  p. Scenario presets\n");
        printf("  r. RUN simulation\n");
        printf("  e. v_ego ramp\n");
        printf("  f. v_rel ramp\n");
        printf("  g. radar_d ramp\n");
        printf("  h. lidar_d ramp\n");
        printf("  c. CAN bus inspection\n");
        printf("  u. UDS diagnostics\n");
        printf("  z. Restart system (new init)\n");
        printf("  q. Quit\n");
        printf("  Choice: ");

        if (scanf(" %c", &choice) != 1) { break; }

        switch (choice)
        {
            case '1':
                (void)read_float("  Radar distance [m] (0.5 - 200): ", &radar_d);
                break;

            case '2':
                (void)read_float("  LiDAR distance [m] (1.0 - 100): ", &lidar_d);
                break;

            case '3':
            {
                float32_t input_kmh = 0.0F;
                if (read_float("  Ego speed [km/h] (10 - 60): ", &input_kmh))
                {
                    v_ego = input_kmh / 3.6F;
                }
                break;
            }

            case '4':
                printf("  Relative speed [m/s]\n");
                printf("  (use ego speed if target is stopped, 0 if same speed): ");
                (void)read_float("", &v_rel);
                break;

            case '5':
                brake = (brake == 0U) ? 1U : 0U;
                printf("  Brake pedal: %s\n", brake ? "PRESSED" : "released");
                break;

            case '6':
                accel = (accel == 0U) ? 1U : 0U;
                printf("  Accelerator: %s\n", accel ? "PRESSED" : "released");
                break;

            case '7':
                aeb_on = (aeb_on == 0U) ? 1U : 0U;
                printf("  AEB: %s\n", aeb_on ? "ENABLED" : "DISABLED");
                break;

            case '8':
                fault_inject = (fault_inject == 0U) ? 1U : 0U;
                printf("  Fault injection: %s\n",
                       fault_inject ? "ACTIVE" : "off");
                break;

            case '9':
            {
                int32_t input_cycles = n_cycles;
                if (read_int("  Number of cycles (1 - 500): ", &input_cycles))
                {
                    if (input_cycles < 1)   { input_cycles = 1;   }
                    if (input_cycles > 500) { input_cycles = 500; }
                    n_cycles = input_cycles;
                }
                break;
            }

            case 'p':
            case 'P':
            {
                char preset;
                printf("\n+---------------------------------------------+\n");
                printf("| SCENARIO PRESETS\n");
                printf("+---------------------------------------------+\n");
                printf("|  1. Emergency brake  -- d=8m  v_ego=120km/h\n");
                printf("|  2. Highway cruise   -- d=80m v_ego=80km/h\n");
                printf("|  3. Urban stop-and-go-- d=15m v_ego=30km/h\n");
                printf("|  4. Sensor fault     -- d=50m fault_inject=ON\n");
                printf("|  b. Back\n");
                printf("| Choice: ");
                if (scanf(" %c", &preset) != 1) { break; }
                switch (preset)
                {
                    case '1':
                        radar_d = 8.0F; lidar_d = 8.0F;
                        v_ego = 33.33F; v_rel = 33.33F;
                        brake = 0U; accel = 0U; aeb_on = 1U; fault_inject = 0U;
                        printf("  Preset: Emergency brake loaded.\n");
                        break;
                    case '2':
                        radar_d = 80.0F; lidar_d = 80.0F;
                        v_ego = 22.22F; v_rel = 5.0F;
                        brake = 0U; accel = 0U; aeb_on = 1U; fault_inject = 0U;
                        printf("  Preset: Highway cruise loaded.\n");
                        break;
                    case '3':
                        radar_d = 15.0F; lidar_d = 15.0F;
                        v_ego = 8.33F; v_rel = 8.33F;
                        brake = 0U; accel = 0U; aeb_on = 1U; fault_inject = 0U;
                        printf("  Preset: Urban stop-and-go loaded.\n");
                        break;
                    case '4':
                        radar_d = 50.0F; lidar_d = 50.0F;
                        v_ego = 11.11F; v_rel = 11.11F;
                        brake = 0U; accel = 0U; aeb_on = 1U; fault_inject = 1U;
                        printf("  Preset: Sensor fault loaded.\n");
                        break;
                    default:
                        break;
                }
                break;
            }

            case 'r':
            case 'R':
            {
                uint8_t last_state;
                int32_t i;

                printf("\n=== Running %d cycle(s) [%d ms] ===\n",
                       (int)n_cycles, (int)(n_cycles * 10));

                can_hal_test_reset();
                last_state = state.fsm.fsm_state;

                for (i = 0; i < n_cycles; i++)
                {
                    uint8_t now;

                    total_cycle++;

                    (void)memset(&raw, 0, sizeof(raw));
                    raw.radar_d  = radar_d;
                    raw.lidar_d  = lidar_d;
                    raw.radar_vr = v_rel;
                    raw.v_ego    = v_ego;
                    raw.fi       = fault_inject;

                    inject_radar(&state, radar_d, v_rel);
                    inject_driver(&state, brake, accel, aeb_on);

                    aeb_core_step(&state, &raw);

                    now = state.fsm.fsm_state;
                    if (now != last_state || i == n_cycles - 1)
                    {
                        print_state(&state, total_cycle);
                        if (now != last_state)
                        {
                            printf("  >>> STATE CHANGE: %d -> %d <<<\n",
                                   (int)last_state, (int)now);
                        }
                        last_state = now;
                    }
                }
                printf("=== Done. Total cycles: %d  (%d ms simulated) ===\n",
                       (int)total_cycle, (int)(total_cycle * 10));
                break;
            }

            case 'e':
            case 'E':
                run_ramp(&state, &total_cycle, &radar_d, &lidar_d, &v_ego, &v_rel,
                         brake, accel, aeb_on, fault_inject, RAMP_V_EGO);
                break;

            case 'f':
            case 'F':
                run_ramp(&state, &total_cycle, &radar_d, &lidar_d, &v_ego, &v_rel,
                         brake, accel, aeb_on, fault_inject, RAMP_V_REL);
                break;

            case 'g':
            case 'G':
                run_ramp(&state, &total_cycle, &radar_d, &lidar_d, &v_ego, &v_rel,
                         brake, accel, aeb_on, fault_inject, RAMP_RADAR_D);
                break;

            case 'h':
            case 'H':
                run_ramp(&state, &total_cycle, &radar_d, &lidar_d, &v_ego, &v_rel,
                         brake, accel, aeb_on, fault_inject, RAMP_LIDAR_D);
                break;

            case 'c':
            case 'C':
                print_can_bus(&state);
                break;

            case 'u':
            case 'U':
                uds_menu(&state);
                break;

            case 'z':
            case 'Z':
                (void)aeb_core_init(&state);
                inject_driver(&state, 0U, 0U, aeb_on);
                can_hal_test_reset();
                total_cycle = 0;
                printf("  System restarted.\n");
                break;

            case 'q':
            case 'Q':
                running = 0;
                break;

            default:
                printf("  Invalid option.\n");
                break;
        }
    }

    printf("\nSimulation ended. Total cycles: %d  (%d ms simulated).\n",
           (int)total_cycle, (int)(total_cycle * 10));
    return 0;
}
