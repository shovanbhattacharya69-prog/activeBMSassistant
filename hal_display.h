/**
 * @file hal_display.h
 * @brief Hardware Abstraction Layer (HAL) for Cockpit Display & HMI Output.
 *
 * @note Decouples core BMS calculation from presentation. In physical vehicle
 *       architecture, this layer serializes CAN frames (e.g., CAN-FD, J1939)
 *       or feeds Qt/Kanzi digital instrument clusters. In this simulator,
 *       it formats an automotive-grade ANSI terminal dashboard.
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the display abstraction layer.
 *
 * @return BMS_STATUS_OK on success.
 */
bms_status_t hal_display_init(void);

/**
 * @brief Renders a single time-tick dashboard frame to the output console.
 *
 * Intercepts FSM state codes, computed metrics, and debounce registers to
 * render an automotive digital instrument cluster view.
 *
 * @param[in] p_inst      Pointer to the active BMS core instance.
 * @param[in] p_telemetry Pointer to the raw sensor acquisition telemetry.
 * @return BMS_STATUS_OK on success, or error code on invalid parameters.
 */
bms_status_t hal_display_render_tick(const bms_core_instance_t * const p_inst,
                                     const bms_sensor_telemetry_t * const p_telemetry);

/**
 * @brief Outputs the final drive-cycle validation report and debouncing telemetry.
 *
 * @param[in] p_inst Pointer to final BMS core instance.
 * @param[in] total_ticks Total execution ticks completed.
 */
void hal_display_render_summary(const bms_core_instance_t * const p_inst,
                                const uint32_t total_ticks);

#ifdef __cplusplus
}
#endif

#endif /* HAL_DISPLAY_H */
