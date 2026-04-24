/**
 * @file  can_hal_test.h
 * @brief Test-only helpers exposed by the CAN HAL stub.
 *
 * Included only by unit tests.  Gives tests read access to the TX
 * capture buffer and fault-injection hooks used to drive error paths.
 * The real Zephyr CAN driver does not provide these — they are
 * specific to the host-side stub in can_hal.c.
 */

#ifndef CAN_HAL_TEST_H
#define CAN_HAL_TEST_H

#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
} tx_record_t;

void                 can_hal_test_reset(void);
uint32_t             can_hal_test_get_tx_count(void);
const tx_record_t   *can_hal_test_get_tx(uint32_t index);
void                 can_hal_test_force_init_fail(int32_t fail);
void                 can_hal_test_force_send_fail(int32_t fail);

#endif /* CAN_HAL_TEST_H */
