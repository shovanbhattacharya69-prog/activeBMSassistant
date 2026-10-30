/**
 * @file hal_sensors.h
 * @brief Hardware Abstraction Layer (HAL) for BMS Sensor Acquisition.
 *
 * @note In production hardware, this module reads analog front-end (AFE)
 *       ASICs (e.g., ADI ADBMS6818, TI BQ79616) over isolated SPI or CAN.
 *       In this SIL (Software-in-the-Loop) environment, it synthesizes
 *       deterministic automotive drive cycles simulating mountain ascents,
 *       regenerative descents, and transient sensor noise spikes.
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#ifndef HAL_SENSORS_H
#define HAL_SENSORS_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the sensor hardware abstraction layer.
 *
 * @return BMS_STATUS_OK on success.
 */
bms_status_t hal_sensors_init(void);

/**
 * @brief Reads all pack telemetry for the given simulation time-tick.
 *
 * Simulates:
 * - 16 individual cell voltages (in mV) with realistic cell-to-cell delta.
 * - 16 thermistor channels (in 0.1 degC) with thermal gradient modeling.
 * - Hall-effect shunt current sensor (in mA, bidirectional).
 * - IMU vehicle pitch / road gradient (in 0.1%).
 * - Vehicle speed and driver pedal demands.
 * - Intentional transient EMI noise glitches to evaluate debouncing.
 *
 * @param[in]  tick         Discrete simulation tick counter.
 * @param[out] p_telemetry  Target buffer for acquired sensor telemetry.
 * @return BMS_STATUS_OK on success, or error code on invalid parameters.
 */
bms_status_t hal_sensors_read(const uint32_t tick,
                             bms_sensor_telemetry_t * const p_telemetry);

/**
 * @brief Injects a manual transient sensor anomaly for test bench harness.
 *
 * @param[in] enable_noise When true, injects non-debounced 1-tick anomalies.
 */
void hal_sensors_set_noise_injection(const bool enable_noise);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SENSORS_H */
