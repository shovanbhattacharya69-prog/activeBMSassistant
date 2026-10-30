#include "hal_sensors.h"

static bool s_noise_injection_enabled = true;

bms_status_t hal_sensors_init(void)
{
    s_noise_injection_enabled = true;
    return BMS_STATUS_OK;
}

void hal_sensors_set_noise_injection(const bool enable_noise)
{
    s_noise_injection_enabled = enable_noise;
}

bms_status_t hal_sensors_read(const uint32_t tick,
                             bms_sensor_telemetry_t * const p_telemetry)
{
    if (p_telemetry == (void *)0) {
        return BMS_STATUS_ERR_NULL_PTR;
    }

    uint16_t base_cell_mv = 3820U;
    int16_t  base_temp_ddegc = 280;
    int32_t  current_ma = 25000;
    int16_t  road_grade = 0;
    uint16_t speed_dkph = 500U;
    uint8_t  accel_pct  = 25U;
    uint8_t  brake_pct  = 0U;

    if (tick < 5U) {
        base_cell_mv    = 3850U;
        base_temp_ddegc = 260;
        current_ma      = 0;
        road_grade      = 0;
        speed_dkph      = 0U;
        accel_pct       = 0U;
        brake_pct       = 0U;
    }
    else if (tick < 21U) {
        base_cell_mv    = (uint16_t)(3840U - (tick * 2U));
        base_temp_ddegc = (int16_t)(260 + (int16_t)(tick * 2));
        current_ma      = 32000;
        road_grade      = 10;
        speed_dkph      = 520U;
        accel_pct       = 30U;
        brake_pct       = 0U;
    }
    else if (tick < 36U) {
        base_cell_mv    = (uint16_t)(3800U - ((tick - 20U) * 8U));
        base_temp_ddegc = (int16_t)(300 + (int16_t)((tick - 20U) * 11));
        current_ma      = 95000;
        road_grade      = 85;
        speed_dkph      = 580U;
        accel_pct       = 75U;
        brake_pct       = 0U;
    }
    else if (tick < 38U) {
        base_cell_mv    = 3680U;
        base_temp_ddegc = 460;
        current_ma      = 85000;
        road_grade      = 40;
        speed_dkph      = 600U;
        accel_pct       = 60U;
        brake_pct       = 0U;
    }
    else if (tick < 43U) {
        base_cell_mv    = 3670U;
        base_temp_ddegc = 465;
        current_ma      = 40000;
        road_grade      = 0;
        speed_dkph      = 620U;
        accel_pct       = 35U;
        brake_pct       = 0U;
    }
    else if (tick < 63U) {
        base_cell_mv    = (uint16_t)(3670U + ((tick - 43U) * 5U));
        base_temp_ddegc = (int16_t)(465 - (int16_t)((tick - 43U) * 2));
        current_ma      = -65000;
        road_grade      = -85;
        speed_dkph      = 680U;
        accel_pct       = 0U;
        brake_pct       = 20U;
    }
    else if (tick < 76U) {
        base_cell_mv    = (uint16_t)(3760U - ((tick - 63U) * 7U));
        base_temp_ddegc = (int16_t)(460 + (int16_t)((tick - 63U) * 9));
        current_ma      = 110000;
        road_grade      = 0;
        speed_dkph      = 1100U;
        accel_pct       = 90U;
        brake_pct       = 0U;
    }
    else if (tick < 88U) {
        base_cell_mv    = 3650U;
        base_temp_ddegc = (int16_t)(550 - (int16_t)((tick - 76U) * 11));
        current_ma      = 22000;
        road_grade      = 0;
        speed_dkph      = 450U;
        accel_pct       = 25U;
        brake_pct       = 0U;
    }
    else if (tick < 93U) {
        base_cell_mv    = 3620U;
        base_temp_ddegc = (int16_t)(655 + (int16_t)((tick - 88U) * 12));
        current_ma      = 75000;
        road_grade      = 0;
        speed_dkph      = 700U;
        accel_pct       = 50U;
        brake_pct       = 0U;
    }
    else {
        base_cell_mv    = 3600U;
        base_temp_ddegc = 620;
        current_ma      = 0;
        road_grade      = 0;
        speed_dkph      = 0U;
        accel_pct       = 0U;
        brake_pct       = 100U;
    }

    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        const uint16_t offset_v = (uint16_t)((i % 4U) * 4U);
        p_telemetry->cell_voltages_mv[i] = (uint16_t)(base_cell_mv + offset_v);

        const int16_t temp_offset = (int16_t)((i == 7U || i == 8U) ? 15 : 0);
        p_telemetry->cell_temperatures_ddegc[i] = (int16_t)(base_temp_ddegc + temp_offset);
    }

    if (s_noise_injection_enabled && (tick == 36U || tick == 37U)) {
        p_telemetry->cell_temperatures_ddegc[3] = 720;
    }

    p_telemetry->pack_current_ma         = current_ma;
    p_telemetry->road_gradient_tenth_pct = road_grade;
    p_telemetry->vehicle_speed_tenth_kph = speed_dkph;
    p_telemetry->accelerator_pedal_pct   = accel_pct;
    p_telemetry->brake_pedal_pct         = brake_pct;
    p_telemetry->sensor_comm_ok          = true;

    return BMS_STATUS_OK;
}
