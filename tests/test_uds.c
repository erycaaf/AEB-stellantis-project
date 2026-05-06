/**
 * @file test_uds.c
 * @brief Unit tests for the AEB UDS diagnostics module.
 *
 * Native host test runner (GCC + assert). No Zephyr dependency.
 * Run with: make test
 *
 * Test coverage:
 *   - Initialisation and power-on state.
 *   - ReadDataByIdentifier (SID 0x22) for all 3 DIDs.
 *   - ClearDiagnosticInformation (SID 0x14).
 *   - RoutineControl (SID 0x31, routine 0x0301).
 *   - DTC monitoring and fault lamp.
 *   - Negative responses for unsupported SID/DID.
 *   - Edge cases and boundary values.
 *
 * @req FR-UDS-001..005
 * @req NFR-COD-006 Statement coverage >= 80%.
 * @req NFR-COD-007 Branch coverage >= 60%.
 *
 * @version 1.0
 * @date 2026-04
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "aeb_uds.h"
#include "aeb_types.h"
#include "aeb_config.h"
#include <math.h>

/* ===================================================================
 * Test infrastructure
 * =================================================================== */

static int32_t tests_run    = 0;
static int32_t tests_passed = 0;

#define RUN_TEST(fn)                                            \
    do {                                                        \
        tests_run++;                                            \
        printf("  [%02d] %-50s ", tests_run, #fn);              \
        (fn)();                                                 \
        tests_passed++;                                         \
        printf("PASS\n");                                       \
    } while (0)

#define ASSERT_EQ(actual, expected, msg)                        \
    do {                                                        \
        if ((actual) != (expected)) {                           \
            printf("FAIL\n       %s\n"                          \
                   "       expected: %d  got: %d\n",            \
                   (msg), (int)(expected), (int)(actual));      \
            assert((actual) == (expected));                     \
        }                                                       \
    } while (0)

/* ===================================================================
 * Shared test variables
 * =================================================================== */

static uds_state_t    st;
static uds_request_t  req;
static uds_response_t resp;
static fsm_output_t   fsm;
static pid_output_t   pid;
static ttc_output_t   ttc;
static uds_output_t   out;

/** @brief Reset all test structures. */
static void reset(void)
{
    uds_init(&st);
    (void)memset(&req,  0, sizeof(req));
    (void)memset(&resp, 0, sizeof(resp));
    (void)memset(&fsm,  0, sizeof(fsm));
    (void)memset(&pid,  0, sizeof(pid));
    (void)memset(&ttc,  0, sizeof(ttc));
    (void)memset(&out,  0, sizeof(out));
}

/* ===================================================================
 * Tests: Initialisation
 * =================================================================== */

/** @req NFR-SAF-004 */
static void test_init_defaults(void)
{
    reset();
    uds_get_output(&st, &out);

    ASSERT_EQ(out.aeb_enabled, 1U, "AEB enabled at power-on");
    ASSERT_EQ(out.dtc_count,   0U, "No DTCs at power-on");
    ASSERT_EQ(out.fault_lamp,  0U, "Fault lamp OFF at power-on");
}

/* ===================================================================
 * Tests: ReadDataByIdentifier (SID 0x22)
 * =================================================================== */

/** @req FR-UDS-001 — TTC (DID 0xF100, scaled x100) */
static void test_read_did_ttc(void)
{
    uint16_t ttc_raw;

    reset();
    ttc.ttc       = 3.45f;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid,  UDS_SID_READ_DID_RESP, "SID 0x62");
    ASSERT_EQ(resp.did_high_resp, 0xF1U, "DID high echoed");
    ASSERT_EQ(resp.did_low_resp,  0x00U, "DID low echoed");

    ttc_raw = (uint16_t)resp.data1 | ((uint16_t)resp.data2 << 8U);
    ASSERT_EQ(ttc_raw, 345U, "TTC = 3.45 * 100 = 345");
}

/** @req FR-UDS-001 — FSM state (DID 0xF101, raw integer) */
static void test_read_did_fsm_state(void)
{
    reset();
    fsm.fsm_state = (uint8_t)FSM_BRAKE_L2;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x01U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_READ_DID_RESP, "SID 0x62");
    ASSERT_EQ(resp.data1, (uint8_t)FSM_BRAKE_L2, "FSM = BRAKE_L2 (4)");
}

/** @req FR-UDS-001 — Brake pressure (DID 0xF102, scaled x10) */
static void test_read_did_brake_pressure(void)
{
    uint16_t p_raw;

    reset();
    pid.brake_pct = 75.5f;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_READ_DID_RESP, "SID 0x62");

    p_raw = (uint16_t)resp.data1 | ((uint16_t)resp.data2 << 8U);
    ASSERT_EQ(p_raw, 755U, "Brake = 75.5 * 10 = 755");
}

/** @req FR-UDS-005 — Unsupported DID -> NRC 0x31 */
static void test_read_did_unsupported(void)
{
    reset();
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xFFU;
    req.did_low  = 0xFFU;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid,  UDS_SID_NEGATIVE_RESP, "SID 0x7F");
    ASSERT_EQ(resp.did_high_resp, UDS_SID_READ_DID, "Rejected SID echoed");
    ASSERT_EQ(resp.did_low_resp,  UDS_NRC_REQUEST_OOR, "NRC 0x31");
}

/* ===================================================================
 * Tests: ClearDiagnosticInformation (SID 0x14)
 * =================================================================== */

/** @req FR-UDS-003 */
static void test_clear_dtc(void)
{
    reset();

    /* Set all faults */
    uds_monitor_faults(&st, 1U, 1U, 1U);
    uds_get_output(&st, &out);
    ASSERT_EQ(out.dtc_count,  3U, "3 DTCs before clear");
    ASSERT_EQ(out.fault_lamp, 1U, "Lamp ON before clear");

    /* Clear */
    req.sid = UDS_SID_CLEAR_DTC;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_CLEAR_DTC_RESP, "SID 0x54");

    /* Readback */
    uds_get_output(&st, &out);
    ASSERT_EQ(out.dtc_count,  0U, "0 DTCs after clear");
    ASSERT_EQ(out.fault_lamp, 0U, "Lamp OFF after clear");
}

/* ===================================================================
 * Tests: RoutineControl (SID 0x31)
 * =================================================================== */

/** @req FR-UDS-004 — Disable AEB */
static void test_routine_disable_aeb(void)
{
    reset();
    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0x03U;
    req.did_low  = 0x01U;
    req.value    = 0U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_ROUTINE_CTRL_RESP, "SID 0x71");
    ASSERT_EQ(resp.data1, 0U, "data1 = 0 (disabled)");

    uds_get_output(&st, &out);
    ASSERT_EQ(out.aeb_enabled, 0U, "AEB disabled");
}

/** @req FR-UDS-004 — Enable AEB */
static void test_routine_enable_aeb(void)
{
    reset();
    st.aeb_enabled = 0U;

    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0x03U;
    req.did_low  = 0x01U;
    req.value    = 1U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_ROUTINE_CTRL_RESP, "SID 0x71");
    ASSERT_EQ(resp.data1, 1U, "data1 = 1 (enabled)");

    uds_get_output(&st, &out);
    ASSERT_EQ(out.aeb_enabled, 1U, "AEB enabled");
}

/** Invalid value for routine 0x0301 */
static void test_routine_invalid_value(void)
{
    reset();
    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0x03U;
    req.did_low  = 0x01U;
    req.value    = 5U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID 0x7F");
}

/** Unsupported routine ID */
static void test_routine_unsupported_rid(void)
{
    reset();
    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0xFFU;
    req.did_low  = 0xFFU;
    req.value    = 1U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID 0x7F");
}

/* ===================================================================
 * Tests: DTC Monitoring
 * =================================================================== */

/** @req FR-UDS-002 — Sensor fault (C1001) */
static void test_dtc_sensor_fault(void)
{
    reset();
    uds_monitor_faults(&st, 1U, 0U, 0U);
    uds_get_output(&st, &out);

    ASSERT_EQ(out.dtc_count,  1U, "DTC count = 1");
    ASSERT_EQ(out.fault_lamp, 1U, "Lamp ON");
}

/** @req FR-UDS-002 — CRC error (C1004) */
static void test_dtc_crc_error(void)
{
    reset();
    uds_monitor_faults(&st, 0U, 1U, 0U);
    uds_get_output(&st, &out);

    ASSERT_EQ(out.dtc_count, 1U, "DTC count = 1");
}

/** @req FR-UDS-002 — Actuator fault (C1006) */
static void test_dtc_actuator_fault(void)
{
    reset();
    uds_monitor_faults(&st, 0U, 0U, 1U);
    uds_get_output(&st, &out);

    ASSERT_EQ(out.dtc_count, 1U, "DTC count = 1");
}

/** @req FR-UDS-002 — Latch persists after fault clears (DD-UDS-01) */
static void test_dtc_latch_persists(void)
{
    reset();
    uds_monitor_faults(&st, 1U, 0U, 0U);   /* fault ON  */
    uds_monitor_faults(&st, 0U, 0U, 0U);   /* fault OFF */

    uds_get_output(&st, &out);
    ASSERT_EQ(out.dtc_count,  1U, "DTC persists");
    ASSERT_EQ(out.fault_lamp, 1U, "Lamp still ON");
}

/** All 3 faults simultaneously */
static void test_dtc_multiple_faults(void)
{
    reset();
    uds_monitor_faults(&st, 1U, 1U, 1U);
    uds_get_output(&st, &out);

    ASSERT_EQ(out.dtc_count, 3U, "DTC count = 3");
}

/* ===================================================================
 * Tests: Negative Responses
 * =================================================================== */

/** @req FR-UDS-005 — Unsupported SID -> NRC 0x11 */
static void test_unsupported_sid(void)
{
    reset();
    req.sid = 0xAAU;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid,  UDS_SID_NEGATIVE_RESP, "SID 0x7F");
    ASSERT_EQ(resp.did_high_resp, 0xAAU, "Rejected SID echoed");
    ASSERT_EQ(resp.did_low_resp,  UDS_NRC_SERVICE_NOT_SUPP, "NRC 0x11");
}

/** Empty request (sid=0) -> no response */
static void test_empty_request(void)
{
    reset();
    req.sid = 0U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, 0U, "No response for sid=0");
}

/* ===================================================================
 * Tests: Edge Cases
 * =================================================================== */

/** TTC = 0.0 boundary */
static void test_ttc_zero(void)
{
    uint16_t ttc_raw;

    reset();
    ttc.ttc      = 0.0f;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ttc_raw = (uint16_t)resp.data1 | ((uint16_t)resp.data2 << 8U);
    ASSERT_EQ(ttc_raw, 0U, "TTC=0 -> 0");
}

/** TTC = 10.0 (max) boundary */
static void test_ttc_max(void)
{
    uint16_t ttc_raw;

    reset();
    ttc.ttc      = 10.0f;
    req.sid      = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low  = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ttc_raw = (uint16_t)resp.data1 | ((uint16_t)resp.data2 << 8U);
    ASSERT_EQ(ttc_raw, 1000U, "TTC=10 -> 1000");
}

/** Brake pressure = 100% boundary */
static void test_brake_pressure_max(void)
{
    uint16_t p_raw;

    reset();
    pid.brake_pct = 100.0f;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    p_raw = (uint16_t)resp.data1 | ((uint16_t)resp.data2 << 8U);
    ASSERT_EQ(p_raw, 1000U, "Brake 100%% -> 1000");
}

/** FSM POST_BRAKE (state 6) readable */
static void test_fsm_state_post_brake(void)
{
    reset();
    fsm.fsm_state = (uint8_t)FSM_POST_BRAKE;
    req.sid       = UDS_SID_READ_DID;
    req.did_high  = 0xF1U;
    req.did_low   = 0x01U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.data1, (uint8_t)FSM_POST_BRAKE, "FSM = 6");
}

/** @req FR-UDS-004 — aeb_enabled persists across cycles (DD-UDS-02) */
static void test_enable_disable_cycle(void)
{
    reset();

    /* Disable */
    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0x03U;
    req.did_low  = 0x01U;
    req.value    = 0U;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    uds_get_output(&st, &out);
    ASSERT_EQ(out.aeb_enabled, 0U, "Disabled");

    /* Empty cycle — persists */
    req.sid = 0U;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    uds_get_output(&st, &out);
    ASSERT_EQ(out.aeb_enabled, 0U, "Still disabled");

    /* Re-enable */
    req.sid      = UDS_SID_ROUTINE_CTRL;
    req.did_high = 0x03U;
    req.did_low  = 0x01U;
    req.value    = 1U;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    uds_get_output(&st, &out);
    ASSERT_EQ(out.aeb_enabled, 1U, "Re-enabled");
}

/** Clear DTCs then re-fault — verify re-latch */
static void test_dtc_clear_then_refault(void)
{
    reset();

    uds_monitor_faults(&st, 1U, 0U, 0U);

    req.sid = UDS_SID_CLEAR_DTC;
    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    uds_get_output(&st, &out);
    ASSERT_EQ(out.dtc_count, 0U, "Cleared");

    uds_monitor_faults(&st, 0U, 1U, 0U);

    uds_get_output(&st, &out);
    ASSERT_EQ(out.dtc_count,  1U, "Re-latched");
    ASSERT_EQ(out.fault_lamp, 1U, "Lamp ON after re-fault");
}

/* ================================================================
 * TEST: Read DID TTC with Infinity value (covers line 87: !isfinite)
 * ================================================================ */
static void test_read_did_ttc_inf(void)
{
    reset();
    ttc.ttc = __builtin_inff();
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    /* Should return negative response (NRC 0x31 - Request Out Of Range) */
    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID TTC with NaN value (covers line 87: !isfinite)
 * ================================================================ */
static void test_read_did_ttc_nan(void)
{
    reset();
    ttc.ttc = __builtin_nanf("");
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID TTC with negative value (covers line 87: ttc < 0.0f)
 * ================================================================ */
static void test_read_did_ttc_negative(void)
{
    reset();
    ttc.ttc = -5.0f;
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID TTC with value above max (covers line 87: ttc > 655.35f)
 * ================================================================ */
static void test_read_did_ttc_overflow(void)
{
    reset();
    ttc.ttc = 1000.0f;
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x00U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID Brake Pressure with Infinity value (covers line 121: !isfinite)
 * ================================================================ */
static void test_read_did_brake_pressure_inf(void)
{
    reset();
    pid.brake_pct = __builtin_inff();
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID Brake Pressure with NaN value (covers line 121: !isfinite)
 * ================================================================ */
static void test_read_did_brake_pressure_nan(void)
{
    reset();
    pid.brake_pct = __builtin_nanf("");
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ================================================================
 * TEST: Read DID Brake Pressure with negative value (line 121: brake_pct < 0.0f)
 * ================================================================ */
static void test_read_did_brake_pressure_negative(void)
{
    reset();
    pid.brake_pct = -10.0f;
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}
/* ================================================================
 * TEST: Read DID Brake Pressure with value above max (covers line 121: brake_pct > 6553.5f)
 * condition 2 not covered (true)
 * ================================================================ */
static void test_read_did_brake_pressure_overflow(void)
{
    reset();
    pid.brake_pct = 10000.0f;  /* > 6553.5, should trigger overflow */
    req.sid = UDS_SID_READ_DID;
    req.did_high = 0xF1U;
    req.did_low = 0x02U;

    uds_process_request(&st, &req, &resp, &fsm, &pid, &ttc);

    /* Should return negative response (NRC 0x31 - Request Out Of Range) */
    ASSERT_EQ(resp.response_sid, UDS_SID_NEGATIVE_RESP, "SID should be 0x7F");
    ASSERT_EQ(resp.did_low_resp, UDS_NRC_REQUEST_OOR, "NRC should be 0x31");
}

/* ===================================================================
 * Main — test runner
 * =================================================================== */

int main(void)
{
    printf("\n======================================\n");
    printf("  AEB UDS Module — Unit Tests\n");
    printf("======================================\n\n");

    /* Initialisation */
    RUN_TEST(test_init_defaults);

    /* SID 0x22 — ReadDataByIdentifier */
    RUN_TEST(test_read_did_ttc);
    RUN_TEST(test_read_did_fsm_state);
    RUN_TEST(test_read_did_brake_pressure);
    RUN_TEST(test_read_did_unsupported);

    /* SID 0x14 — ClearDiagnosticInformation */
    RUN_TEST(test_clear_dtc);

    /* SID 0x31 — RoutineControl */
    RUN_TEST(test_routine_disable_aeb);
    RUN_TEST(test_routine_enable_aeb);
    RUN_TEST(test_routine_invalid_value);
    RUN_TEST(test_routine_unsupported_rid);

    /* DTC Monitoring */
    RUN_TEST(test_dtc_sensor_fault);
    RUN_TEST(test_dtc_crc_error);
    RUN_TEST(test_dtc_actuator_fault);
    RUN_TEST(test_dtc_latch_persists);
    RUN_TEST(test_dtc_multiple_faults);

    /* Negative Responses */
    RUN_TEST(test_unsupported_sid);
    RUN_TEST(test_empty_request);

    /* Edge Cases */
    RUN_TEST(test_ttc_zero);
    RUN_TEST(test_ttc_max);
    RUN_TEST(test_brake_pressure_max);
    RUN_TEST(test_fsm_state_post_brake);
    RUN_TEST(test_enable_disable_cycle);
    RUN_TEST(test_dtc_clear_then_refault);

    /* Edge cases for TTC DID (line 87) */
    RUN_TEST(test_read_did_ttc_inf);
    RUN_TEST(test_read_did_ttc_nan);
    RUN_TEST(test_read_did_ttc_negative);
    RUN_TEST(test_read_did_ttc_overflow);
    
    /* Edge cases for Brake Pressure DID (line 121) */
    RUN_TEST(test_read_did_brake_pressure_inf);
    RUN_TEST(test_read_did_brake_pressure_nan);
    RUN_TEST(test_read_did_brake_pressure_negative);
    RUN_TEST(test_read_did_brake_pressure_overflow);

    printf("\n======================================\n");
    printf("  Results: %d / %d passed\n",
           (int)tests_passed, (int)tests_run);
    printf("======================================\n\n");

    return (tests_passed == tests_run) ? 0 : 1;
}