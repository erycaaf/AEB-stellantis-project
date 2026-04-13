/**
 * @file  main.c
 * @brief AEB Zephyr main — 10 ms execution cycle on native_sim.
 *
 * On native_sim, uses usleep for the 10ms tick since the POSIX
 * RX pthread doesn't integrate with Zephyr's simulated clock.
 * On real hardware, replace usleep with k_timer + k_sem.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <unistd.h>

#include "aeb_core.h"
#include "aeb_types.h"

LOG_MODULE_REGISTER(aeb_main, LOG_LEVEL_INF);

extern void can_hal_drain_rx(void *can_state);

static aeb_core_state_t aeb_state;

int main(void)
{
    int32_t rc;

    LOG_INF("=== AEB ECU (Zephyr native_sim) ===");
    LOG_INF("Cycle: 10 ms | TCP CAN bus");

    rc = aeb_core_init(&aeb_state);
    if (rc != 0) {
        LOG_ERR("AEB core init failed: %d", rc);
        return rc;
    }

    LOG_INF("AEB system initialised — entering main loop");

    uint32_t cycle_count = 0U;

    while (1) {
        usleep(10000);  /* 10 ms — matches NFR-PERF-001 */

        can_hal_drain_rx(&aeb_state.can);

        can_rx_data_t can_rx;
        can_get_rx_data(&aeb_state.can, &can_rx);

        raw_sensor_input_t raw = {0};
        raw.radar_d     = can_rx.target_distance;
        raw.radar_vr    = can_rx.relative_speed;
        raw.lidar_d     = can_rx.target_distance;
        raw.v_ego       = can_rx.vehicle_speed;
        raw.can_timeout = can_rx.rx_timeout_flag;
        raw.fi          = 0U;

        aeb_core_step(&aeb_state, &raw);

        cycle_count++;
        if ((cycle_count % 50U) == 0U) {
            LOG_INF("c=%u st=%u ttc=%.1f brk=%.0f%% d=%.1f v=%.1f flt=%u to=%u",
                    cycle_count,
                    (unsigned)aeb_state.fsm.fsm_state,
                    (double)aeb_state.ttc.ttc,
                    (double)aeb_state.pid.brake_pct,
                    (double)aeb_state.perception.distance,
                    (double)can_rx.vehicle_speed,
                    (unsigned)aeb_state.perception.fault_flag,
                    (unsigned)can_rx.rx_timeout_flag);
        }
    }

    return 0;
}
