/**
 * @file main.c
 * @brief RTOS-Style Cyclic Executive for the Active Eco-Assist BMS.
 *
 * @note Demonstrates a deterministic cyclic executive simulating a 100 ms
 *       RTOS periodic task loop (e.g., FreeRTOS, AUTOSAR OS, or bare-metal).
 *       Zero dynamic allocation is used; all data structures are stack-allocated
 *       or statically mapped.
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#include <time.h>
#include <string.h>
#include "bms_types.h"
#include "bms_core.h"
#include "hal_sensors.h"
#include "hal_display.h"

/** @brief Total time-ticks in simulated automotive drive cycle mission. */
#define BMS_SIMULATION_TOTAL_TICKS      (100U)

/**
 * @brief Portable C99 RTOS time-tick delay generator.
 *
 * @param[in] delay_ms Milliseconds to pause.
 */
static void rtos_delay_ms(const uint32_t delay_ms)
{
    if (delay_ms == 0U) {
        return;
    }
    const clock_t start = clock();
    const clock_t wait_ticks = (clock_t)((delay_ms * (uint32_t)CLOCKS_PER_SEC) / 1000U);
    while ((clock() - start) < wait_ticks) {
        /* Deterministic RTOS periodic task scheduler loop */
    }
}

/**
 * @brief Main entry point executing the RTOS-style cyclic executive.
 *
 * @param[in] argc Argument count.
 * @param[in] argv Argument values: pass "--live" or "-l" for animated RTOS pacing.
 * @return 0 on nominal completion.
 */
int main(int argc, char *argv[])
{
    /* Determine pacing: live animated cockpit mode or instant batch mode */
    uint32_t tick_delay_ms = 0U;
    if (argc > 1) {
        if ((strcmp(argv[1], "--live") == 0) || (strcmp(argv[1], "-l") == 0)) {
            tick_delay_ms = 70U; /* 70 ms pacing per tick for animated live cockpit */
        }
    }

    /* ====================================================================== */
    /* 1. Stack Allocation (Zero Dynamic Memory Allocation)                   */
    /* ====================================================================== */
    bms_core_instance_t     bms_inst;
    bms_sensor_telemetry_t  telemetry_buffer;

    /* ====================================================================== */
    /* 2. Hardware and Core Initialization                                    */
    /* ====================================================================== */
    (void)hal_sensors_init();
    (void)hal_display_init();

    const bms_status_t init_status = bms_core_init(&bms_inst);
    if (init_status != BMS_STATUS_OK) {
        return (int)init_status;
    }

    /* ====================================================================== */
    /* 3. RTOS Cyclic Task Loop (Simulating 100 ms periodic task dispatch)    */
    /* ====================================================================== */
    for (uint32_t tick = 0U; tick < BMS_SIMULATION_TOTAL_TICKS; ++tick) {
        /* Step A: Hardware Abstraction Layer - Sensor Ingestion */
        const bms_status_t sensor_status = hal_sensors_read(tick, &telemetry_buffer);
        if (sensor_status != BMS_STATUS_OK) {
            break;
        }

        /* Step B: Pure BMS Mathematical & FSM Core Processing */
        const bms_status_t core_status = bms_core_step(&bms_inst, &telemetry_buffer);
        if (core_status != BMS_STATUS_OK) {
            break;
        }

        /* Step C: Hardware Abstraction Layer - Telemetry & Display Output */
        (void)hal_display_render_tick(&bms_inst, &telemetry_buffer);

        /* Step D: RTOS periodic task period sleep */
        if (tick_delay_ms > 0U) {
            rtos_delay_ms(tick_delay_ms);
        }
    }

    /* ====================================================================== */
    /* 4. Verification and Mission Summary Report                             */
    /* ====================================================================== */
    hal_display_render_summary(&bms_inst, BMS_SIMULATION_TOTAL_TICKS);

    return 0;
}
