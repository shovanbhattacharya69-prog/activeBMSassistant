#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMS_CELL_COUNT                       (16U)
#define BMS_DEBOUNCE_FAULT_LIMIT             (3U)
#define BMS_DEBOUNCE_RECOVERY_LIMIT          (3U)

#define BMS_TEMP_CRITICAL_HIGH_DDEGC         ((int16_t)650)
#define BMS_TEMP_WARN_HIGH_DDEGC             ((int16_t)550)
#define BMS_TEMP_RECOVERY_HIGH_DDEGC         ((int16_t)480)
#define BMS_TEMP_CRITICAL_LOW_DDEGC          ((int16_t)-200)

#define BMS_CELL_OVERVOLT_CRITICAL_MV        ((uint16_t)4250)
#define BMS_CELL_OVERVOLT_WARN_MV            ((uint16_t)4200)
#define BMS_CELL_UNDERVOLT_CRITICAL_MV       ((uint16_t)2700)
#define BMS_CELL_UNDERVOLT_WARN_MV           ((uint16_t)3000)
#define BMS_CELL_DELTA_WARN_MV               ((uint16_t)150)

#define BMS_CURRENT_OVERCURRENT_DISCHARGE_MA ((int32_t)120000)
#define BMS_CURRENT_OVERCURRENT_CHARGE_MA    ((int32_t)-80000)

#define BMS_ECO_DESCENT_GRADIENT_THRESHOLD_D ((int16_t)-40)
#define BMS_ECO_MIN_SPEED_D_KPH              ((uint16_t)200)
#define BMS_SOC_LIMP_THRESHOLD_DPCT          ((uint16_t)100)
#define BMS_NOMINAL_CELL_CAPACITY_MAH        ((uint32_t)50000)

typedef enum {
    SYS_INIT = 0,
    SYS_NORMAL,
    SYS_REGEN_ADVISORY,
    SYS_LIMP_MODE,
    SYS_CRITICAL_FAULT
} bms_state_t;

typedef enum {
    BMS_STATUS_OK = 0,
    BMS_STATUS_ERR_NULL_PTR,
    BMS_STATUS_ERR_PARAM,
    BMS_STATUS_ERR_NOT_INIT
} bms_status_t;

typedef enum {
    REGEN_ADVISORY_NONE = 0,
    REGEN_ADVISORY_DOWNHILL_GRADE,
    REGEN_ADVISORY_COASTING_OPPORTUNITY,
    REGEN_ADVISORY_DECEL_BUFFER
} bms_regen_advisory_t;

typedef enum {
    BMS_FAULT_NONE              = 0x00000000U,
    BMS_FAULT_OVERTEMP_CRIT     = 0x00000001U,
    BMS_FAULT_UNDERTEMP_CRIT    = 0x00000002U,
    BMS_FAULT_CELL_UNDERVOLT    = 0x00000004U,
    BMS_FAULT_CELL_OVERVOLT     = 0x00000008U,
    BMS_FAULT_OVERCURRENT_DISC  = 0x00000010U,
    BMS_FAULT_OVERCURRENT_CHG   = 0x00000020U,
    BMS_FAULT_SENSOR_COMM_LOSS  = 0x00000040U,
    BMS_FAULT_CELL_IMBALANCE    = 0x00000080U
} bms_fault_mask_t;

typedef enum {
    BMS_WARN_NONE               = 0x00000000U,
    BMS_WARN_TEMP_HIGH          = 0x00000001U,
    BMS_WARN_SOC_LOW            = 0x00000002U,
    BMS_WARN_CELL_DELTA_HIGH    = 0x00000004U,
    BMS_WARN_POWER_DERATED      = 0x00000008U
} bms_warn_mask_t;

typedef struct {
    uint16_t cell_voltages_mv[BMS_CELL_COUNT];
    int16_t  cell_temperatures_ddegc[BMS_CELL_COUNT];
    int32_t  pack_current_ma;
    int16_t  road_gradient_tenth_pct;
    uint16_t vehicle_speed_tenth_kph;
    uint8_t  accelerator_pedal_pct;
    uint8_t  brake_pedal_pct;
    bool     sensor_comm_ok;
} bms_sensor_telemetry_t;

typedef struct {
    uint8_t overtemp_fault_ticks;
    uint8_t undervolt_fault_ticks;
    uint8_t overvolt_fault_ticks;
    uint8_t overcurrent_fault_ticks;
    uint8_t overtemp_warn_ticks;
    uint8_t limp_recovery_ticks;
} bms_debounce_counters_t;

typedef struct {
    uint32_t pack_voltage_mv;
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;
    uint16_t cell_delta_mv;
    uint8_t  min_cell_index;
    uint8_t  max_cell_index;
    int16_t  min_temp_ddegc;
    int16_t  max_temp_ddegc;
    int16_t  avg_temp_ddegc;
    uint16_t soc_deci_pct;
    uint8_t  throttle_limit_pct;
    uint8_t  regen_torque_bias_pct;
} bms_pack_metrics_t;

typedef struct {
    bms_state_t              current_state;
    bms_state_t              previous_state;
    uint32_t                 system_tick;
    uint32_t                 fault_flags;
    uint32_t                 warning_flags;
    bms_regen_advisory_t     regen_advisory;
    bms_debounce_counters_t  debounce;
    bms_pack_metrics_t       metrics;
    bool                     contactor_closed;
    bool                     is_initialized;
} bms_core_instance_t;

#ifdef __cplusplus
}
#endif

#endif
