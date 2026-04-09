/**
 * @file  can_hal.h
 * @brief CAN Hardware Abstraction Layer — stub for host compilation.
 *
 * On Zephyr, this file is replaced by the real Zephyr CAN driver calls.
 * This stub allows compiling and testing aeb_can.c on a host PC
 * (gcc / cppcheck / MISRA checker) without the Zephyr SDK.
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

#include <stdint.h>

/** @brief CAN baud rate selector. */
#define CAN_BAUD_500K   500000U

/**
 * @brief Initialise CAN peripheral.
 * @param baud_rate  Baud rate in bit/s (e.g. 500000).
 * @return 0 on success, -1 on failure.
 */
int32_t can_hal_init(uint32_t baud_rate);

/**
 * @brief Register an RX acceptance filter for a CAN message ID.
 * @param msg_id  Standard 11-bit CAN ID to accept.
 * @return Filter handle (>=0) on success, -1 on failure.
 */
int32_t can_hal_add_rx_filter(uint32_t msg_id);

/**
 * @brief Send a CAN frame (non-blocking).
 * @param id    Standard 11-bit CAN ID.
 * @param data  Pointer to payload (up to 8 bytes).
 * @param dlc   Data Length Code (0..8).
 * @return 0 on success, -1 on failure.
 */
int32_t can_hal_send(uint32_t id, const uint8_t *data, uint8_t dlc);

#endif /* CAN_HAL_H */
