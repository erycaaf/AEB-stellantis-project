/**
 * @file    aeb_uds.c
 * @brief   UDS diagnostics — STUB implementation (Sprint 0).
 * @owner   Task E (Rian Ithalo da Costa Linhares)
 *
 * @todo    Implement UDS request processing, DTC monitoring,
 *          and live data management.
 */

#include "aeb_uds.h"
#include "aeb_config.h"

void uds_init(void)
{
    /* STUB: no internal state to initialise yet. */
}

void uds_process_request(uint8_t sid, uint8_t did_high, uint8_t did_low,
                         uint8_t value, uint8_t *response, uint8_t *resp_len)
{
    (void)sid;
    (void)did_high;
    (void)did_low;
    (void)value;

    /* STUB: negative response (unsupported service). */
    response[0] = 0x7FU;
    response[1] = sid;
    response[2] = 0x11U;  /* serviceNotSupported */
    *resp_len   = 3U;
}

uint8_t uds_monitor_faults(uint8_t sensor_fault, uint8_t crc_error,
                           uint8_t actuator_fault)
{
    (void)sensor_fault;
    (void)crc_error;
    (void)actuator_fault;

    /* STUB: no faults. */
    return 0U;
}

uint8_t uds_get_aeb_enabled(void)
{
    /* STUB: AEB enabled by default. */
    return 1U;
}

void uds_update_live_data(const fsm_output_t *fsm_out,
                          const ttc_output_t *ttc_out,
                          const pid_output_t *pid_out)
{
    (void)fsm_out;
    (void)ttc_out;
    (void)pid_out;
    /* STUB: no update. */
}
