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

#include "bms_types.h"
#include "bms_core.h"
#include "hal_sensors.h"
#include "hal_display.h"

/** @brief Total time-ticks in simulated automotive drive cycle mission. */
#define BMS_SIMULATION_TOTAL_TICKS      (100U)

/**
 * @brief Main entry point executing the RTOS-style cyclic executive.
 *
 * @return 0 on nominal completion.
 */
int main(void)
{
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
    }

    /* ====================================================================== */
    /* 4. Verification and Mission Summary Report                             */
    /* ====================================================================== */
    hal_display_render_summary(&bms_inst, BMS_SIMULATION_TOTAL_TICKS);

    return 0;
}
