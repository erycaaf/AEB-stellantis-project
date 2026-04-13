# SPDX-License-Identifier: Apache-2.0
#
# AEB System — Zephyr Application
#
# Compiles the repo C code as a Zephyr app targeting native_sim
# with SocketCAN on vcan0.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aeb_zephyr C)

# ── Repo C source files (cloned from AEB-stellantis-project) ──────────────
set(AEB_REPO ${CMAKE_CURRENT_SOURCE_DIR}/repo)

target_include_directories(app PRIVATE
    ${AEB_REPO}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(app PRIVATE
    # Zephyr-specific
    src/main.c
    src/can_hal_zephyr.c

    # Application modules (from repo)
    ${AEB_REPO}/src/perception/aeb_perception.c
    ${AEB_REPO}/src/decision/aeb_ttc.c
    ${AEB_REPO}/src/decision/aeb_fsm.c
    ${AEB_REPO}/src/execution/aeb_pid.c
    ${AEB_REPO}/src/execution/aeb_alert.c
    ${AEB_REPO}/src/communication/aeb_can.c
    ${AEB_REPO}/src/communication/aeb_uds.c
    ${AEB_REPO}/src/integration/aeb_core.c
)
