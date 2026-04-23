/**
 * @file test_uds_fault.c
 * @brief Systematic fault-injection tests for the UDS module (ASIL-D).
 *
 * Exercises the module with invalid, corrupted and extreme inputs to verify
 * that the implementation remains fail-safe. Independent verification of
 * aeb_uds.c authored by Rian Linhares; validated (cross) by Renato Fagundes.
 *
 * Categories exercised (ISO 26262-6 Tab. 11 item 1e — "++ for ASIL-D"):
 *   A  Invalid float values (NaN, +/-Inf) on live-data inputs.
 *   B  Numeric extremes (negative, overflow of scaled cast target).
 *   C  Corrupted server state (simulated single-event upsets).
 *   D  Persistence of state under abusive sequences.
 *   E  Timing / concurrency — OUT OF SCOPE for unit level,
 *      deferred to SIL (timing is bench-harness-independent here).
 *
 * Each assertion is non-fatal: failures are counted, not aborted, so the
 * full robustness picture is visible in a single run.
 *
 * @req FR-UDS-001..005, NFR-SAF-ROB
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "aeb_uds.h"
#include "aeb_types.h"
#include "aeb_config.h"

/* ===================================================================
 * Non-fatal test harness (counters only, no abort)
 * =================================================================== */

static int32_t asserts_run    = 0;
static int32_t asserts_passed = 0;
static int32_t asserts_failed = 0;
static const char *current_case = "";

#define FAULT_ASSERT(cond, tag)                                        \
    do {                                                               \
        asserts_run++;                                                 \
        if (cond) {                                                    \
            asserts_passed++;                                          \
        } else {                                                       \
            asserts_failed++;                                          \
            printf("    FAIL  %-8s  %s\n", current_case, tag);         \
        }                                                              \
    } while (0)

#define RUN_CASE(fn)                                                   \
    do {                                                               \
        current_case = #fn;                                            \
        printf("  [%s]\n", #fn);                                       \
        (fn)();                                                        \
    } while (0)

/* ===================================================================
 * Shared test fixtures
 * =================================================================== */

static uds_state_t    st;
static uds_request_t  req;
static uds_response_t resp;
static fsm_output_t   fsm;
static pid_output_t   pid;
static ttc_output_t   ttc;
static uds_output_t   out;

static void fault_reset(void)
{
    uds_init(&st);
    (void)memset(&req,  0, sizeof(req));
    (void)memset(&resp, 0, sizeof(resp));
    (void)memset(&fsm,  0, sizeof(fsm));
    (void)memset(&pid,  0, sizeof(pid));
    (void)memset(&ttc,  0, sizeof(ttc));
    (void)memset(&out,  0, sizeof(out));
}

/**
 * @brief Fail-safe predicate for ReadDID responses under abnormal inputs.
 *
 * Under ASIL-D, the response to abnormal live-data is expected to be one of:
 *   - Negative response (SID 0x7F) with an appropriate NRC, or
 *   - Positive response (SID 0x62) with a defined sentinel (e.g. zero)
 *     that the tester can interpret as "data currently unreliable".
 *
 * A response containing implementation-defined or undefined bytes (from a
 * float-to-uint cast of NaN/Inf/negatives) is NOT fail-safe.
 */
static int resp_is_failsafe_or_negative(const uds_response_t *r)
{
    if (r->response_sid == UDS_SID_NEGATIVE_RESP) {
        return 1;
    }
    if (r->response_sid == UDS_SID_READ_DID_RESP) {
        /* Positive response must contain a defined sentinel (zero). */
        return (r->data1 == 0U) && (r->data2 == 0U);
    }
    return 0;
}

/* ===================================================================
 * CATEGORY A — NaN / Infinity on live-data inputs
 * =================================================================== */

/** A1 — TTC = NaN propagates through scaled cast (FR-UDS-001 robustness). */
static void fault_a1_ttc_nan(void)
{
    fault_reset();
    ttc.ttc      = (float)NAN;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "A1: TTC=NaN fail-safe");
}

/** A2 — TTC = +Infinity. */
static void fault_a2_ttc_pos_inf(void)
{
    fault_reset();
    ttc.ttc      = (float)INFINITY;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "A2: TTC=+Inf fail-safe");
}

/** A3 — TTC = -Infinity. */
static void fault_a3_ttc_neg_inf(void)
{
    fault_reset();
    ttc.ttc      = -(float)INFINITY;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "A3: TTC=-Inf fail-safe");
}

/** A4 — Brake pressure = NaN. */
static void fault_a4_brake_nan(void)
{
    fault_reset();
    pid.brake_pct = (float)NAN;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "A4: Brake=NaN fail-safe");
}

/** A5 — Brake pressure = +Infinity. */
static void fault_a5_brake_pos_inf(void)
{
    fault_reset();
    pid.brake_pct = (float)INFINITY;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "A5: Brake=+Inf fail-safe");
}

/* ===================================================================
 * CATEGORY B — Numeric extremes (negative, overflow of cast target)
 * =================================================================== */

/** B1 — TTC negative (float-to-uint16_t cast is UB per C11 §6.3.1.4). */
static void fault_b1_ttc_negative(void)
{
    fault_reset();
    ttc.ttc      = -1.0f;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "B1: TTC<0 fail-safe");
}

/** B2 — TTC above uint16_t/100 range (overflow of scaled cast). */
static void fault_b2_ttc_overflow(void)
{
    fault_reset();
    ttc.ttc      = 700.0f;   /* 700 * 100 = 70000 > UINT16_MAX */
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "B2: TTC overflow fail-safe");
}

/** B3 — TTC = FLT_MAX (massive overflow). */
static void fault_b3_ttc_float_max(void)
{
    fault_reset();
    ttc.ttc      = FLT_MAX;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "B3: TTC=FLT_MAX fail-safe");
}

/** B4 — Brake pressure negative. */
static void fault_b4_brake_negative(void)
{
    fault_reset();
    pid.brake_pct = -25.0f;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "B4: Brake<0 fail-safe");
}

/** B5 — Brake pressure > 100% specification (overflow scaled). */
static void fault_b5_brake_overflow(void)
{
    fault_reset();
    pid.brake_pct = 7000.0f;   /* 7000 * 10 = 70000 > UINT16_MAX */
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp_is_failsafe_or_negative(&resp), "B5: Brake overflow fail-safe");
}

/* ===================================================================
 * CATEGORY C — Corrupted server state (simulated SEU / bit-flip)
 * =================================================================== */

/**
 * C1 — aeb_enabled corrupted to 0xAA (not 0 or 1).
 *
 * The spec treats aeb_enabled as a Boolean. Any value other than 0/1 is
 * out-of-domain. uds_get_output currently propagates the raw byte — this
 * test asserts it should normalise to a defined Boolean (0 or 1).
 */
static void fault_c1_aeb_enabled_corrupted(void)
{
    fault_reset();
    st.aeb_enabled = 0xAAU;
    uds_get_output(&st, &out);

    FAULT_ASSERT((out.aeb_enabled == 0U) || (out.aeb_enabled == 1U),
                 "C1: aeb_enabled normalised to {0,1}");
}

/**
 * C2 — All three DTC flags corrupted to 0xFF simultaneously.
 *
 * uds_get_output computes count = dtc_sensor + dtc_crc + dtc_actuator as
 * uint8_t. If flags are corrupted to 0xFF each, the sum is 0x02FD, which
 * wraps to 0xFD on assignment to uint8_t. fault_lamp = (count > 0) is then
 * 1 (benign here) but count is a garbage value for the caller.
 *
 * This test asserts count should be clamped to [0,3] and fault_lamp to {0,1}.
 */
static void fault_c2_dtc_flags_corrupted(void)
{
    fault_reset();
    st.dtc_sensor   = 0xFFU;
    st.dtc_crc      = 0xFFU;
    st.dtc_actuator = 0xFFU;
    uds_get_output(&st, &out);

    FAULT_ASSERT(out.dtc_count <= 3U,          "C2: dtc_count bounded [0..3]");
    FAULT_ASSERT((out.fault_lamp == 0U)
              || (out.fault_lamp == 1U),       "C2: fault_lamp is Boolean");
}

/**
 * C3 — One DTC flag corrupted to 0x80 (MSB set, not 0 or 1).
 *
 * Under the spec, each flag is Boolean. The current code only checks
 * `!= 0` when latching, so 0x80 is treated as "fault active" and contributes
 * 128 to count, giving count = 128 on output (clearly wrong).
 */
static void fault_c3_single_dtc_corrupted(void)
{
    fault_reset();
    st.dtc_sensor = 0x80U;
    uds_get_output(&st, &out);

    FAULT_ASSERT(out.dtc_count <= 3U,          "C3: dtc_count bounded [0..3]");
}

/* ===================================================================
 * CATEGORY D — Persistence under abusive sequences
 * =================================================================== */

/** D1 — Clear DTC while fault line is still active — should not re-latch in same call. */
static void fault_d1_clear_while_active(void)
{
    fault_reset();
    uds_monitor_faults(&st, 1U, 0U, 0U);

    req.sid = UDS_SID_CLEAR_DTC;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    uds_get_output(&st, &out);
    FAULT_ASSERT(out.dtc_count == 0U,          "D1: cleared in same cycle");
}

/** D2 — 100 rapid toggles of aeb_enabled — state must end coherent. */
static void fault_d2_rapid_toggle(void)
{
    int i;
    fault_reset();

    for (i = 0; i < 100; i++) {
        req.sid      = UDS_SID_ROUTINE_CTRL;
        req.did_high = 0x03U;
        req.did_low  = 0x01U;
        req.value    = (uint8_t)((i & 1) ? 1U : 0U);
        uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);
    }

    uds_get_output(&st, &out);
    FAULT_ASSERT((out.aeb_enabled == 0U) || (out.aeb_enabled == 1U),
                 "D2: aeb_enabled coherent after 100 toggles");
    /* Last i was 99 (odd) → value=1 → enabled */
    FAULT_ASSERT(out.aeb_enabled == 1U,        "D2: ends on value written by last request");
}

/** D3 — Unknown SID sequence should never leave response_sid at a valid positive code. */
static void fault_d3_unknown_sid_does_not_spoof(void)
{
    fault_reset();
    req.sid      = 0xAAU;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    FAULT_ASSERT(resp.response_sid == UDS_SID_NEGATIVE_RESP,
                 "D3: unknown SID -> 0x7F (not a spoofed positive)");
}

/* ===================================================================
 * MAIN — runner
 * =================================================================== */

int main(void)
{
    printf("\n======================================\n");
    printf("  AEB UDS — Fault Injection Suite\n");
    printf("  ISO 26262-6 Tab. 11 item 1e (++ ASIL-D)\n");
    printf("======================================\n");

    printf("\n-- Category A: NaN / Infinity --\n");
    RUN_CASE(fault_a1_ttc_nan);
    RUN_CASE(fault_a2_ttc_pos_inf);
    RUN_CASE(fault_a3_ttc_neg_inf);
    RUN_CASE(fault_a4_brake_nan);
    RUN_CASE(fault_a5_brake_pos_inf);

    printf("\n-- Category B: Numeric extremes --\n");
    RUN_CASE(fault_b1_ttc_negative);
    RUN_CASE(fault_b2_ttc_overflow);
    RUN_CASE(fault_b3_ttc_float_max);
    RUN_CASE(fault_b4_brake_negative);
    RUN_CASE(fault_b5_brake_overflow);

    printf("\n-- Category C: Corrupted state (SEU) --\n");
    RUN_CASE(fault_c1_aeb_enabled_corrupted);
    RUN_CASE(fault_c2_dtc_flags_corrupted);
    RUN_CASE(fault_c3_single_dtc_corrupted);

    printf("\n-- Category D: Persistence --\n");
    RUN_CASE(fault_d1_clear_while_active);
    RUN_CASE(fault_d2_rapid_toggle);
    RUN_CASE(fault_d3_unknown_sid_does_not_spoof);

    printf("\n======================================\n");
    printf("  Assertions: %d run  %d passed  %d failed\n",
           (int)asserts_run, (int)asserts_passed, (int)asserts_failed);
    printf("======================================\n\n");

    return (asserts_failed == 0) ? 0 : 1;
}
