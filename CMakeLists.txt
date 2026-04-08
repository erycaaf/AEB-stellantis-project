# ===========================================================================
# CMakeLists.txt — AEB System for Zephyr RTOS
# ===========================================================================
# Project: Autonomous Emergency Braking (AEB)
# Version: 1.0 (Sprint 0 — Project Skeleton)
# Standard: MISRA C:2012, ISO 26262 ASIL B
# ===========================================================================

cmake_minimum_required(VERSION 3.20.0)

# --- Zephyr project setup ---------------------------------------------------
# In a full Zephyr environment, find_package(Zephyr) sets up the toolchain,
# kernel, and devicetree. For PC-based compilation and unit testing without
# Zephyr SDK, we fall back to a standard C project.
# ---------------------------------------------------------------------------
find_package(Zephyr QUIET)

if(Zephyr_FOUND)
    # ---- Zephyr build (target hardware / QEMU) ----------------------------
    project(aeb_system LANGUAGES C)

    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_sources(app PRIVATE
        # Entry point
        src/main.c

        # Task A — Perception
        src/perception/aeb_perception.c

        # Task B — Decision Logic
        src/decision/aeb_ttc.c
        src/decision/aeb_fsm.c

        # Task C — Execution
        src/execution/aeb_pid.c
        src/execution/aeb_alert.c

        # Task D — CAN Bus
        src/communication/aeb_can.c

        # Task E — UDS Diagnostics
        src/communication/aeb_uds.c
    )

else()
    # ---- Native / PC build (GCC, unit testing) ----------------------------
    project(aeb_system LANGUAGES C)

    # Strict MISRA-aligned warnings
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)

    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wformat=2
        -Wundef
        -fno-common
    )

    # Include path
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    # ---- Main application library (all modules) ---------------------------
    add_library(aeb_modules STATIC
        src/perception/aeb_perception.c
        src/decision/aeb_ttc.c
        src/decision/aeb_fsm.c
        src/execution/aeb_pid.c
        src/execution/aeb_alert.c
        src/communication/aeb_can.c
        src/communication/aeb_uds.c
    )

    # ---- Main executable --------------------------------------------------
    add_executable(aeb_main src/main.c)
    target_link_libraries(aeb_main PRIVATE aeb_modules)

    # ---- Unit test executables (one per module) ---------------------------
    # Tests are added as they are implemented during Sprint 1.
    # Example:
    #   add_executable(test_pid tests/test_pid.c)
    #   target_link_libraries(test_pid PRIVATE aeb_modules)
    #
    #   add_executable(test_alert tests/test_alert.c)
    #   target_link_libraries(test_alert PRIVATE aeb_modules)

endif()
