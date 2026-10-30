#include <time.h>
#include <string.h>
#include "bms_types.h"
#include "bms_core.h"
#include "hal_sensors.h"
#include "hal_display.h"

#define BMS_SIMULATION_TOTAL_TICKS      (100U)

static void rtos_delay_ms(const uint32_t delay_ms)
{
    if (delay_ms == 0U) {
        return;
    }
    const clock_t start = clock();
    const clock_t wait_ticks = (clock_t)((delay_ms * (uint32_t)CLOCKS_PER_SEC) / 1000U);
    while ((clock() - start) < wait_ticks) {
    }
}

int main(int argc, char *argv[])
{
    uint32_t tick_delay_ms = 0U;
    bool json_mode = false;
    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "--live") == 0) || (strcmp(argv[i], "-l") == 0)) {
            tick_delay_ms = 70U;
        } else if (strcmp(argv[i], "--json") == 0) {
            json_mode = true;
        }
    }

    if (json_mode) {
        hal_display_set_json_mode(true);
    }

    bms_core_instance_t     bms_inst;
    bms_sensor_telemetry_t  telemetry_buffer;

    (void)hal_sensors_init();
    (void)hal_display_init();

    const bms_status_t init_status = bms_core_init(&bms_inst);
    if (init_status != BMS_STATUS_OK) {
        return (int)init_status;
    }

    for (uint32_t tick = 0U; tick < BMS_SIMULATION_TOTAL_TICKS; ++tick) {
        const bms_status_t sensor_status = hal_sensors_read(tick, &telemetry_buffer);
        if (sensor_status != BMS_STATUS_OK) {
            break;
        }

        const bms_status_t core_status = bms_core_step(&bms_inst, &telemetry_buffer);
        if (core_status != BMS_STATUS_OK) {
            break;
        }

        (void)hal_display_render_tick(&bms_inst, &telemetry_buffer);

        if (tick_delay_ms > 0U) {
            rtos_delay_ms(tick_delay_ms);
        }
    }

    hal_display_render_summary(&bms_inst, BMS_SIMULATION_TOTAL_TICKS);

    return 0;
}
