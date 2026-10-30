#ifndef HAL_SENSORS_H
#define HAL_SENSORS_H

#include "bms_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bms_status_t hal_sensors_init(void);

bms_status_t hal_sensors_read(uint32_t tick,
                             bms_sensor_telemetry_t *p_telemetry);

void hal_sensors_set_noise_injection(bool enable_noise);

#ifdef __cplusplus
}
#endif

#endif
