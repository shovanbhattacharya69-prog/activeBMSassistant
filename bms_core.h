/**
 * @file bms_core.h
 * @brief Public interface for the pure mathematical BMS core logic and FSM.
 *
 * @note This module is 100% decoupled from hardware I/O and display mechanisms.
 *       It contains zero stdio calls, zero heap allocation, and operates purely
 *       on caller-provided data structures with strict MISRA C compliance.
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#ifndef BMS_CORE_H
#define BMS_CORE_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the BMS core runtime instance and internal registers.
 *
 * @param[out] p_inst Pointer to pre-allocated BMS core instance.
 * @return BMS_STATUS_OK on success, or error code on invalid parameters.
 */
bms_status_t bms_core_init(bms_core_instance_t * const p_inst);

/**
 * @brief Executes one discrete time-tick cycle of the BMS core logic.
 *
 * Performs:
 * 1. Cell array traversal and aggregate metrics calculation (min/max/delta).
 * 2. State-of-Charge (SOC) estimation update.
 * 3. Sensor fault debouncing and noise filtering.
 * 4. Active Eco-Assist telemetry analysis.
 * 5. Finite State Machine (FSM) state evaluation and contactor management.
 *
 * @param[in,out] p_inst      Pointer to active BMS core instance.
 * @param[in]     p_telemetry Pointer to raw sensor telemetry acquisition.
 * @return BMS_STATUS_OK on success, or error code on failure.
 */
bms_status_t bms_core_step(bms_core_instance_t * const p_inst,
                          const bms_sensor_telemetry_t * const p_telemetry);

/**
 * @brief Retrieves the current FSM operational state code.
 *
 * @param[in] p_inst Pointer to BMS core instance.
 * @return Active bms_state_t enumeration value.
 */
bms_state_t bms_core_get_state(const bms_core_instance_t * const p_inst);

/**
 * @brief Retrieves a read-only pointer to the computed pack metrics.
 *
 * @param[in] p_inst Pointer to BMS core instance.
 * @return Read-only pointer to bms_pack_metrics_t, or NULL if invalid.
 */
const bms_pack_metrics_t * bms_core_get_metrics(const bms_core_instance_t * const p_inst);

/**
 * @brief Retrieves active critical fault bitmask.
 *
 * @param[in] p_inst Pointer to BMS core instance.
 * @return 32-bit active fault flags.
 */
uint32_t bms_core_get_faults(const bms_core_instance_t * const p_inst);

/**
 * @brief Retrieves active warning bitmask.
 *
 * @param[in] p_inst Pointer to BMS core instance.
 * @return 32-bit active warning flags.
 */
uint32_t bms_core_get_warnings(const bms_core_instance_t * const p_inst);

/**
 * @brief Retrieves current high-voltage contactor actuation status.
 *
 * @param[in] p_inst Pointer to BMS core instance.
 * @return true if contactor is closed (energized), false if open (safe).
 */
bool bms_core_is_contactor_closed(const bms_core_instance_t * const p_inst);

#ifdef __cplusplus
}
#endif

#endif /* BMS_CORE_H */
