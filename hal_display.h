#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bms_status_t hal_display_init(void);

void hal_display_set_json_mode(bool json_mode);

bms_status_t hal_display_render_tick(const bms_core_instance_t *p_inst,
                                     const bms_sensor_telemetry_t *p_telemetry);

void hal_display_render_summary(const bms_core_instance_t *p_inst,
                                uint32_t total_ticks);

#ifdef __cplusplus
}
#endif

#endif
