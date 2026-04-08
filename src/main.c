/**
 * @file    main.c
 * @brief   AEB system entry point — STUB implementation (Sprint 0).
 * @owner   Task E (Rian Ithalo da Costa Linhares)
 *
 * @details In the final implementation, this file will:
 *          - Initialise all modules
 *          - Configure Zephyr threads and k_timer (10 ms)
 *          - Run the main execution cycle
 *          - Manage mutexes for shared data
 *
 * @todo    Implement Zephyr threading model, timer, and integration.
 */

#include "aeb_types.h"
#include "aeb_config.h"
#include "aeb_perception.h"
#include "aeb_ttc.h"
#include "aeb_fsm.h"
#include "aeb_pid.h"
#include "aeb_alert.h"
#include "aeb_can.h"
#include "aeb_uds.h"

/**
 * @brief Application entry point.
 *
 * Initialises all AEB modules. In the full implementation,
 * this will set up Zephyr threads and the 10 ms timer.
 *
 * @return 0 on success.
 */
int main(void)
{
    /* Initialise all modules */
    perception_init();
    fsm_init();
    pid_init();
    (void)can_init();
    uds_init();

    /*
     * STUB: In full implementation, this will:
     * 1. Create k_timer for 10 ms periodic execution
     * 2. Create thread_perception (priority 1)
     * 3. Create thread_controller (priority 2)
     * 4. Create thread_communication (priority 3)
     * 5. Enter Zephyr scheduler
     */

    return 0;
}
