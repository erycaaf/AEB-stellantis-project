/**
 * @file  can_hal.c
 * @brief CAN HAL stub implementation for host-based testing.
 *
 * Records TX frames in a circular buffer so unit tests can inspect them.
 * On Zephyr, this file is NOT compiled — the real CAN driver is used.
 */

#include "can_hal.h"
#include "can_hal_test.h"
#include <string.h>

/* ── TX capture buffer for test inspection ──────────────────────────── */
#define TX_BUF_SIZE  (32U)

static tx_record_t tx_buffer[TX_BUF_SIZE];
static uint32_t    tx_count = 0U;

/* ── RX filter registry ─────────────────────────────────────────────── */
#define MAX_FILTERS  (8U)

static uint32_t rx_filters[MAX_FILTERS];
static uint32_t rx_filter_count = 0U;

/* ── Fault injection ────────────────────────────────────────────────── */
static int32_t  force_init_fail = 0;
static int32_t  force_send_fail = 0;

/* ═══════════════════════════════════════════════════════════════════════
 *  HAL API (stub)
 * ═══════════════════════════════════════════════════════════════════════ */

int32_t can_hal_init(uint32_t baud_rate)
{
    (void)baud_rate;

    if (force_init_fail != 0)
    {
        return -1;
    }

    tx_count = 0U;
    rx_filter_count = 0U;
    return 0;
}

int32_t can_hal_add_rx_filter(uint32_t msg_id)
{
    int32_t result = -1;

    if (rx_filter_count < MAX_FILTERS)
    {
        rx_filters[rx_filter_count] = msg_id;
        result = (int32_t)rx_filter_count;
        rx_filter_count++;
    }

    return result;
}

int32_t can_hal_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    int32_t result = -1;

    if (force_send_fail != 0)
    {
        result = -1;
    }
    else if ((data != NULL) && (dlc <= 8U) && (tx_count < TX_BUF_SIZE))
    {
        tx_buffer[tx_count].id  = id;
        tx_buffer[tx_count].dlc = dlc;
        (void)memcpy(tx_buffer[tx_count].data, data, (size_t)dlc);
        tx_count++;
        result = 0;
    }
    else
    {
        /* buffer full or invalid args */
    }

    return result;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  TEST HELPERS (called from test_can.c)
 * ═══════════════════════════════════════════════════════════════════════ */

/** @brief Get count of transmitted frames since last init/reset. */
uint32_t can_hal_test_get_tx_count(void)
{
    return tx_count;
}

/** @brief Get pointer to the Nth transmitted frame. */
const tx_record_t *can_hal_test_get_tx(uint32_t index)
{
    const tx_record_t *rec = NULL;

    if (index < tx_count)
    {
        rec = &tx_buffer[index];
    }

    return rec;
}

/** @brief Reset TX buffer. */
void can_hal_test_reset(void)
{
    tx_count = 0U;
    rx_filter_count = 0U;
    force_init_fail = 0;
    force_send_fail = 0;
}

/** @brief Force can_hal_init() to fail (for negative testing). */
void can_hal_test_force_init_fail(int32_t fail)
{
    force_init_fail = fail;
}

/** @brief Force can_hal_send() to fail (for negative testing). */
void can_hal_test_force_send_fail(int32_t fail)
{
    force_send_fail = fail;
}
