/**
 * @file  aeb_perception.c
 * @brief AEB Perception module implementation.
 */

#include "aeb_perception.h"
#include "aeb_config.h"
#include <stddef.h>
#include <math.h>

#define FABSF(x) (((x) < 0.0f) ? -(x) : (x))

static float32_t clampf(float32_t v, float32_t lo, float32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* --- LiDAR fault detector (chart_26) --- */

typedef struct {
    float32_t prev_d;
    uint8_t   ctr;
    uint8_t   is_first;
} lidar_state_t;

static lidar_state_t s_lidar;

static void lidar_state_reset(lidar_state_t *s)
{
    s->prev_d   = 0.0f;
    s->ctr      = 0U;
    s->is_first = 1U;
}

static uint8_t lidar_fault_detect(float32_t d_l)
{
    uint8_t bad;
    uint8_t fault;

    bad = (!isfinite(d_l) ||
           (d_l < LIDAR_DIST_MIN) || (d_l > LIDAR_DIST_MAX)) ? 1U : 0U;

    if ((s_lidar.is_first == 0U) && (bad == 0U)) {
        if (FABSF(d_l - s_lidar.prev_d) > DIST_ROC_LIMIT) {
            bad = 1U;
        }
    }

    if (bad != 0U) {
        if (s_lidar.ctr < 255U) { s_lidar.ctr++; }
    } else {
        /* Update ROC baseline and arm is_first only on trustworthy frames —
         * prevents a bad first frame from contaminating prev_d. */
        s_lidar.ctr      = 0U;
        s_lidar.prev_d   = d_l;
        s_lidar.is_first = 0U;
    }

    fault = (s_lidar.ctr >= (uint8_t)SENSOR_FAULT_CYCLES) ? 1U : 0U;
    return fault;
}

/* --- Radar fault detector (chart_33) --- */

typedef struct {
    float32_t prev_d;
    float32_t prev_vr;
    uint8_t   ctr;
    uint8_t   is_first;
} radar_state_t;

static radar_state_t s_radar;

static void radar_state_reset(radar_state_t *s)
{
    s->prev_d   = 0.0f;
    s->prev_vr  = 0.0f;
    s->ctr      = 0U;
    s->is_first = 1U;
}

static uint8_t radar_fault_detect(float32_t d_r, float32_t vr_r)
{
    uint8_t bad;
    uint8_t fault;

    bad = (!isfinite(d_r) || !isfinite(vr_r) ||
           (d_r < RADAR_DIST_MIN) || (d_r > RADAR_DIST_MAX) ||
           (FABSF(vr_r) > MAX_REL_VEL)) ? 1U : 0U;

    if ((s_radar.is_first == 0U) && (bad == 0U)) {
        if ((FABSF(d_r  - s_radar.prev_d)  > DIST_ROC_LIMIT) ||
            (FABSF(vr_r - s_radar.prev_vr) > VEL_ROC_LIMIT)) {
            bad = 1U;
        }
    }

    if (bad != 0U) {
        if (s_radar.ctr < 255U) { s_radar.ctr++; }
    } else {
        /* Update ROC baseline and arm is_first only on trustworthy frames —
         * prevents a bad first frame from contaminating prev_d/prev_vr. */
        s_radar.ctr      = 0U;
        s_radar.prev_d   = d_r;
        s_radar.prev_vr  = vr_r;
        s_radar.is_first = 0U;
    }

    fault = (s_radar.ctr >= (uint8_t)SENSOR_FAULT_CYCLES) ? 1U : 0U;
    return fault;
}

/* --- Kalman fusion (chart_41) ---
 *
 * State: x = [distance, v_rel]'
 * F = | 1  -dt |    Q = diag(Q_d, Q_v)
 *     | 0   1  |
 *
 * Sequential update:
 *   1) LiDAR:  H = [1,0],  R = R_lidar
 *   2) Radar:  H = I_2x2,  R = diag(R_radar_d, R_radar_v)
 *
 * Confidence: both=1.0, radar-only=0.7, lidar-only=0.5, none=0.0
 *             scaled by max(0, 1 - tr(P)/2)
 */

typedef struct {
    float32_t x[2];
    float32_t P[2][2];
    uint8_t   initialized;
} kalman_state_t;

static kalman_state_t s_kalman;

static void kalman_state_reset(kalman_state_t *s)
{
    s->x[0] = 0.0f;
    s->x[1] = 0.0f;
    s->P[0][0] = KALMAN_P0_D;  s->P[0][1] = 0.0f;
    s->P[1][0] = 0.0f;         s->P[1][1] = KALMAN_P0_V;
    s->initialized = 0U;
}

static void kalman_fusion(
    float32_t d_l, float32_t d_r, float32_t vr_r,
    uint8_t l_fault, uint8_t r_fault, uint8_t fi,
    float32_t *out_d, float32_t *out_v, float32_t *out_conf)
{
    uint8_t l_ok = ((l_fault == 0U) && (fi == 0U) && isfinite(d_l)) ? 1U : 0U;
    uint8_t r_ok = ((r_fault == 0U) && (fi == 0U) &&
                    isfinite(d_r) && isfinite(vr_r)) ? 1U : 0U;

    /* Seed deferred until a finite radar frame; until then, l_ok/r_ok are
     * both 0 and *out_conf below is forced to 0.0f — "no information" signalled. */
    if ((s_kalman.initialized == 0U) && isfinite(d_r) && isfinite(vr_r)) {
        s_kalman.x[0] = d_r;
        s_kalman.x[1] = vr_r;
        s_kalman.initialized = 1U;
    }

    /* --- PREDICT: xp = F*x, Pp = F*P*F' + Q --- */
    float32_t dt = AEB_DT;
    float32_t xp0 = s_kalman.x[0] - (dt * s_kalman.x[1]);
    float32_t xp1 = s_kalman.x[1];

    float32_t A00 = s_kalman.P[0][0] - (dt * s_kalman.P[1][0]);
    float32_t A01 = s_kalman.P[0][1] - (dt * s_kalman.P[1][1]);
    float32_t A10 = s_kalman.P[1][0];
    float32_t A11 = s_kalman.P[1][1];

    float32_t Pp00 = (A00 - (dt * A01)) + KALMAN_Q_D;
    float32_t Pp01 = A01;
    float32_t Pp10 = A10 - (dt * A11);
    float32_t Pp11 = A11 + KALMAN_Q_V;

    /* --- UPDATE 1: LiDAR (H=[1,0], scalar) --- */
    float32_t x1_0 = xp0;
    float32_t x1_1 = xp1;
    float32_t P1_00 = Pp00;
    float32_t P1_01 = Pp01;
    float32_t P1_10 = Pp10;
    float32_t P1_11 = Pp11;

    if (l_ok != 0U) {
        float32_t S_l  = Pp00 + KALMAN_R_LIDAR;
        float32_t K0   = Pp00 / S_l;
        float32_t K1   = Pp10 / S_l;
        float32_t innov = d_l - xp0;

        x1_0 = xp0 + (K0 * innov);
        x1_1 = xp1 + (K1 * innov);

        float32_t imK0 = 1.0f - K0;
        P1_00 = imK0 * Pp00;
        P1_01 = imK0 * Pp01;
        P1_10 = (-K1 * Pp00) + Pp10;
        P1_11 = (-K1 * Pp01) + Pp11;
    }

    /* --- UPDATE 2: Radar (H=I_2x2, 2x2 inverse) --- */
    float32_t x2_0 = x1_0;
    float32_t x2_1 = x1_1;
    float32_t P2_00 = P1_00;
    float32_t P2_01 = P1_01;
    float32_t P2_10 = P1_10;
    float32_t P2_11 = P1_11;

    if (r_ok != 0U) {
        float32_t S00 = P1_00 + KALMAN_R_RADAR_D;
        float32_t S01 = P1_01;
        float32_t S10 = P1_10;
        float32_t S11 = P1_11 + KALMAN_R_RADAR_V;

        float32_t det = (S00 * S11) - (S01 * S10);

        if (FABSF(det) > 1e-9f) {
            float32_t inv_det = 1.0f / det;
            float32_t Si00 =  S11 * inv_det;
            float32_t Si01 = -S01 * inv_det;
            float32_t Si10 = -S10 * inv_det;
            float32_t Si11 =  S00 * inv_det;

            float32_t K00 = (P1_00 * Si00) + (P1_01 * Si10);
            float32_t K01 = (P1_00 * Si01) + (P1_01 * Si11);
            float32_t K10 = (P1_10 * Si00) + (P1_11 * Si10);
            float32_t K11 = (P1_10 * Si01) + (P1_11 * Si11);

            float32_t inn0 = d_r  - x1_0;
            float32_t inn1 = vr_r - x1_1;

            x2_0 = x1_0 + (K00 * inn0) + (K01 * inn1);
            x2_1 = x1_1 + (K10 * inn0) + (K11 * inn1);

            float32_t imK00 = 1.0f - K00;
            float32_t imK11 = 1.0f - K11;

            P2_00 = (imK00 * P1_00) + ((-K01) * P1_10);
            P2_01 = (imK00 * P1_01) + ((-K01) * P1_11);
            P2_10 = ((-K10) * P1_00) + (imK11 * P1_10);
            P2_11 = ((-K10) * P1_01) + (imK11 * P1_11);
        }
    }

    /* Persist state */
    s_kalman.x[0]    = x2_0;
    s_kalman.x[1]    = x2_1;
    s_kalman.P[0][0] = P2_00;
    s_kalman.P[0][1] = P2_01;
    s_kalman.P[1][0] = P2_10;
    s_kalman.P[1][1] = P2_11;

    /* Confidence */
    if ((l_ok == 0U) && (r_ok == 0U)) {
        *out_conf = 0.0f;
    } else {
        float32_t base;
        if ((l_ok != 0U) && (r_ok != 0U)) {
            base = 1.0f;
        } else if (r_ok != 0U) {
            base = 0.7f;
        } else {
            base = 0.5f;
        }
        float32_t trace_P = P2_00 + P2_11;
        float32_t quality = 1.0f - (trace_P / 2.0f);
        if (quality < 0.0f) { quality = 0.0f; }
        *out_conf = base * quality;
    }

    *out_d = clampf(x2_0, DIST_MIN_OUTPUT, DIST_MAX_OUTPUT);
    *out_v = clampf(x2_1, -MAX_REL_VEL,   MAX_REL_VEL);
}

/* --- Public API --- */

void perception_init(void)
{
    lidar_state_reset(&s_lidar);
    radar_state_reset(&s_radar);
    kalman_state_reset(&s_kalman);
}

void perception_step(const raw_sensor_input_t *in, perception_output_t *out)
{
    uint8_t   l_fault;
    uint8_t   r_fault;
    float32_t fused_d;
    float32_t fused_v;
    float32_t conf;

    out->fault_flag = 0U;

    if (isfinite(in->v_ego)) {
        out->v_ego = in->v_ego;
    } else {
        out->v_ego       = 0.0f;
        out->fault_flag |= AEB_FAULT_CAN_TO;
    }

    /* CAN timeout: hold last Kalman estimate, flag all faults */
    if (in->can_timeout != 0U) {
        out->fault_flag = AEB_FAULT_LIDAR | AEB_FAULT_RADAR | AEB_FAULT_CAN_TO;
        out->distance   = clampf(s_kalman.x[0], DIST_MIN_OUTPUT, DIST_MAX_OUTPUT);
        out->v_rel      = clampf(s_kalman.x[1], -MAX_REL_VEL, MAX_REL_VEL);
        out->confidence = 0.0f;
        return;
    }

    /* Sensor fault detection */
    l_fault = lidar_fault_detect(in->lidar_d);
    r_fault = radar_fault_detect(in->radar_d, in->radar_vr);

    if (l_fault != 0U) { out->fault_flag |= AEB_FAULT_LIDAR; }
    if (r_fault != 0U) { out->fault_flag |= AEB_FAULT_RADAR; }

    /* Kalman fusion (runs even if both faulty for dead-reckoning) */
    kalman_fusion(in->lidar_d, in->radar_d, in->radar_vr,
                  l_fault, r_fault, in->fi,
                  &fused_d, &fused_v, &conf);

    /* Fault injection propagation (NFR-SAF-005) */
    if (in->fi != 0U) {
        out->fault_flag |= AEB_FAULT_LIDAR | AEB_FAULT_RADAR;
    }

    out->distance   = fused_d;
    out->v_rel      = fused_v;
    out->confidence = conf;
}
