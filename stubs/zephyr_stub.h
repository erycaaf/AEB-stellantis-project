/**
 * @file zephyr_stub.h
 * @brief Minimal Zephyr RTOS stubs for host (PC) compilation and testing.
 *
 * Provides empty definitions for Zephyr kernel types and macros
 * used by aeb_main.c, allowing the UDS module to be compiled and
 * tested on a host PC with GCC (NFR-POR-001).
 *
 * This file is NOT used when compiling for the Zephyr target.
 *
 * @version 1.0
 * @date 2026-04
 */

#ifndef ZEPHYR_STUB_H
#define ZEPHYR_STUB_H

#include <stdint.h>
#include <stdio.h>

/* ===================================================================
 * printk stub
 * =================================================================== */

#define printk(fmt, ...)    printf(fmt, ##__VA_ARGS__)

#endif /* ZEPHYR_STUB_H */
