/**
 * @file hal_sensors.c
 * @brief Mock automotive sensor acquisition driver with road gradient
 *        simulation and intentional EMI transient noise injection.
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#include "hal_sensors.h"

/* ========================================================================== */
/* Private Variables                                                          */
/* ========================================================================== */

static bool s_noise_injection_enabled = true;

/* ========================================================================== */
/* Public Interface Implementation                                            */
/* ========================================================================== */

/**
 * @brief Initializes the sensor hardware abstraction layer.
 */
bms_status_t hal_sensors_init(void)
{
    s_noise_injection_enabled = true;
    return BMS_STATUS_OK;
}

/**
 * @brief Injects a manual transient sensor anomaly for test bench harness.
 */
void hal_sensors_set_noise_injection(const bool enable_noise)
{
    s_noise_injection_enabled = enable_noise;
}

/**
 * @brief Reads all pack telemetry for the given simulation time-tick.
 *
 * Drive Profile Across 100 Ticks (100 ms per tick):
 * - Ticks 0..4:   System Boot & Self-Test Verification.
 * - Ticks 5..20:  City Cruising (nominal 50 km/h, 30A discharge, 28 degC).
 * - Ticks 21..35: Steep Mountain Ascent (+8.5% incline, 95A discharge, temp rising).
 * - Ticks 36..37: Transient Sensor Noise Spike (Cell #3 thermistor hits 72.0 degC
 *                 for 2 consecutive ticks, testing noise rejection).
 * - Ticks 38..42: Noise disappears (temp returns to 46.0 degC, proving no false trip).
 * - Ticks 43..62: Alpine Mountain Descent (-8.5% downhill gradient, coasting/regen,
 *                 -65A regen charging, triggering SYS_REGEN_ADVISORY).
 * - Ticks 63..75: Aggressive Sprint / Heavy Discharge (110A, max temp climbs > 56 degC
 *                 for 3+ ticks, triggering debounced SYS_LIMP_MODE).
 * - Ticks 76..85: Limp Mode Power Cap (30% power, current reduced, temp cools < 48 degC,
 *                 debounces recovery for 3 ticks and restores SYS_NORMAL).
 * - Ticks 86..92: Thermal Runaway Simulation (Coolant pump failure: cell #7 reaches
 *                 68 degC for 3 consecutive ticks 89, 90, 91 -> triggers SYS_CRITICAL_FAULT).
 * - Ticks 93..99: Post-Trip Safe Lock (Contactors open, current zero, HV isolation).
 */
bms_status_t hal_sensors_read(const uint32_t tick,
                             bms_sensor_telemetry_t * const p_telemetry)
{
    if (p_telemetry == (void *)0) {
        return BMS_STATUS_ERR_NULL_PTR;
    }

    /* Baseline nominal cell values */
    uint16_t base_cell_mv = 3820U;
    int16_t  base_temp_ddegc = 280; /* 28.0 degC */
    int32_t  current_ma = 25000;    /* 25.0 A discharge */
    int16_t  road_grade = 0;        /* 0.0% grade */
    uint16_t speed_dkph = 500U;     /* 50.0 km/h */
    uint8_t  accel_pct  = 25U;
    uint8_t  brake_pct  = 0U;

    /* ---------------------------------------------------------------------- */
    /* Phased Profile Generation                                              */
    /* ---------------------------------------------------------------------- */
    if (tick < 5U) {
        /* Phase 1: Boot & Stabilization */
        base_cell_mv    = 3850U;
        base_temp_ddegc = 260;
        current_ma      = 0;
        road_grade      = 0;
        speed_dkph      = 0U;
        accel_pct       = 0U;
        brake_pct       = 0U;
    }
    else if (tick < 21U) {
        /* Phase 2: Moderate City Driving */
        base_cell_mv    = (uint16_t)(3840U - (tick * 2U));
        base_temp_ddegc = (int16_t)(260 + (int16_t)(tick * 2));
        current_ma      = 32000;
        road_grade      = 10;   /* +1.0% grade */
        speed_dkph      = 520U;
        accel_pct       = 30U;
        brake_pct       = 0U;
    }
    else if (tick < 36U) {
        /* Phase 3: Steep Mountain Ascent (+8.5% grade) */
        base_cell_mv    = (uint16_t)(3800U - ((tick - 20U) * 8U));
        base_temp_ddegc = (int16_t)(300 + (int16_t)((tick - 20U) * 11)); /* warms to ~46.5 C */
        current_ma      = 95000; /* Heavy 95A draw */
        road_grade      = 85;    /* +8.5% incline */
        speed_dkph      = 580U;
        accel_pct       = 75U;
        brake_pct       = 0U;
    }
    else if (tick < 38U) {
        /* Phase 4: Transient Sensor EMI Glitch Injection (2 Ticks Only!)
         * Injected on Cell #3 thermistor in cell temperature loop below.
         */
        base_cell_mv    = 3680U;
        base_temp_ddegc = 460;
        current_ma      = 85000;
        road_grade      = 40;
        speed_dkph      = 600U;
        accel_pct       = 60U;
        brake_pct       = 0U;
    }
    else if (tick < 43U) {
        /* Phase 5: Cresting Mountain Pass (Glitch is gone!) */
        base_cell_mv    = 3670U;
        base_temp_ddegc = 465;
        current_ma      = 40000;
        road_grade      = 0;
        speed_dkph      = 620U;
        accel_pct       = 35U;
        brake_pct       = 0U;
    }
    else if (tick < 63U) {
        /* Phase 6: Long Alpine Mountain Descent (-8.5% downhill gradient)
         * Driver lifts accelerator. High regen current (-65A) charges pack!
         */
        base_cell_mv    = (uint16_t)(3670U + ((tick - 43U) * 5U)); /* Recharging! */
        base_temp_ddegc = (int16_t)(465 - (int16_t)((tick - 43U) * 2));
        current_ma      = -65000; /* Regenerative braking charge */
        road_grade      = -85;    /* -8.5% steep descent */
        speed_dkph      = 680U;   /* 68.0 km/h */
        accel_pct       = 0U;     /* Lifted off throttle */
        brake_pct       = 20U;    /* Gentle brake pedal touch */
    }
    else if (tick < 76U) {
        /* Phase 7: Aggressive Sprint / Thermal Stress (> 55.0 degC Warning) */
        base_cell_mv    = (uint16_t)(3760U - ((tick - 63U) * 7U));
        /* Temperature steadily climbs past 55.0 degC */
        base_temp_ddegc = (int16_t)(460 + (int16_t)((tick - 63U) * 9)); /* Reaches 57.7 C */
        current_ma      = 110000; /* 110 A heavy discharge */
        road_grade      = 0;
        speed_dkph      = 1100U;  /* 110.0 km/h */
        accel_pct       = 90U;
        brake_pct       = 0U;
    }
    else if (tick < 88U) {
        /* Phase 8: Limp Mode Power Throttling & Cool-Down Recovery */
        base_cell_mv    = 3650U;
        /* Temperature drops as vehicle power is capped */
        base_temp_ddegc = (int16_t)(550 - (int16_t)((tick - 76U) * 11)); /* Cools < 48 degC */
        current_ma      = 22000;  /* Throttled limp current */
        road_grade      = 0;
        speed_dkph      = 450U;
        accel_pct       = 25U;
        brake_pct       = 0U;
    }
    else if (tick < 93U) {
        /* Phase 9: Coolant Failure / Sustained Thermal Runaway
         * Temperature breaches 65.0 degC critical threshold for 3 consecutive ticks!
         */
        base_cell_mv    = 3620U;
        /* Ticks 88, 89, 90: over 65.0 degC */
        base_temp_ddegc = (int16_t)(655 + (int16_t)((tick - 88U) * 12)); /* 65.5C, 66.7C, 67.9C... */
        current_ma      = 75000;
        road_grade      = 0;
        speed_dkph      = 700U;
        accel_pct       = 50U;
        brake_pct       = 0U;
    }
    else {
        /* Phase 10: Post-Fault Locked Safe State */
        base_cell_mv    = 3600U;
        base_temp_ddegc = 620;
        current_ma      = 0;      /* Contactors opened by BMS core */
        road_grade      = 0;
        speed_dkph      = 0U;
        accel_pct       = 0U;
        brake_pct       = 100U;
    }

    /* ---------------------------------------------------------------------- */
    /* Cell Array Population with Small Physical Variance                     */
    /* ---------------------------------------------------------------------- */
    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        /* Small cell voltage imbalance (up to 15 mV) */
        const uint16_t offset_v = (uint16_t)((i % 4U) * 4U);
        p_telemetry->cell_voltages_mv[i] = (uint16_t)(base_cell_mv + offset_v);

        /* Cell temperature distribution (middle cells run slightly hotter) */
        const int16_t temp_offset = (int16_t)((i == 7U || i == 8U) ? 15 : 0);
        p_telemetry->cell_temperatures_ddegc[i] = (int16_t)(base_temp_ddegc + temp_offset);
    }

    /* ---------------------------------------------------------------------- */
    /* Transient EMI Noise Spike Injection (Cell #3 on Ticks 36 & 37)        */
    /* ---------------------------------------------------------------------- */
    if (s_noise_injection_enabled && (tick == 36U || tick == 37U)) {
        /* Inject 72.0 degC spike (exceeds 65.0 degC trip point) for 2 ticks ONLY */
        p_telemetry->cell_temperatures_ddegc[3] = 720;
    }

    /* Vehicle Telemetry */
    p_telemetry->pack_current_ma         = current_ma;
    p_telemetry->road_gradient_tenth_pct = road_grade;
    p_telemetry->vehicle_speed_tenth_kph = speed_dkph;
    p_telemetry->accelerator_pedal_pct   = accel_pct;
    p_telemetry->brake_pedal_pct         = brake_pct;
    p_telemetry->sensor_comm_ok          = true;

    return BMS_STATUS_OK;
}
