/**
 * @file  can_hal.h
 * @brief CAN HAL interface — shared between stub and Zephyr implementation.
 *
 * This is a local copy matching the repo's stubs/can_hal.h.
 * The implementation is in can_hal_zephyr.c (real Zephyr CAN driver).
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

#include <stdint.h>

#define CAN_BAUD_500K   500000U

int32_t can_hal_init(uint32_t baud_rate);
int32_t can_hal_add_rx_filter(uint32_t msg_id);
int32_t can_hal_send(uint32_t id, const uint8_t *data, uint8_t dlc);

#endif /* CAN_HAL_H */
