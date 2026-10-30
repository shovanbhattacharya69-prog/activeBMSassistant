#ifndef BMS_CORE_H
#define BMS_CORE_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bms_status_t bms_core_init(bms_core_instance_t * const p_inst);

bms_status_t bms_core_step(bms_core_instance_t * const p_inst,
                          const bms_sensor_telemetry_t * const p_telemetry);

bms_state_t bms_core_get_state(const bms_core_instance_t * const p_inst);

const bms_pack_metrics_t * bms_core_get_metrics(const bms_core_instance_t * const p_inst);

uint32_t bms_core_get_faults(const bms_core_instance_t * const p_inst);

uint32_t bms_core_get_warnings(const bms_core_instance_t * const p_inst);

bool bms_core_is_contactor_closed(const bms_core_instance_t * const p_inst);

#ifdef __cplusplus
}
#endif

#endif
