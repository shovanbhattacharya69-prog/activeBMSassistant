/**
 * @file bms_types.h
 * @brief Global types, enumerations, constants, and data structures for the
 *        Active Eco-Assist Battery Management System (BMS).
 *
 * @note Adheres strictly to C99 and MISRA C-inspired coding guidelines:
 *       - Uses fixed-width integers from <stdint.h> exclusively.
 *       - Zero dynamic memory allocation (no malloc/free).
 *       - Explicit struct padding and alignment awareness.
 *       - Deterministic integer units (mV, mA, deci-degC, deci-percent).
 *
 * @author Staff Embedded Automotive Software Engineer
 * @date 2026-10-30
 * @copyright Automotive Tier-1 Supplier Specification
 */

#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Hardware and Pack Geometry Constants                                       */
/* ========================================================================== */

/** @brief Number of series battery cells monitored in the module/pack. */
#define BMS_CELL_COUNT                  (16U)

/** @brief Debounce threshold: consecutive fault ticks required to trip. */
#define BMS_DEBOUNCE_FAULT_LIMIT        (3U)

/** @brief Debounce threshold: consecutive healthy ticks required to clear. */
#define BMS_DEBOUNCE_RECOVERY_LIMIT     (3U)

/* ========================================================================== */
/* Safety and Operational Thresholds (Integer Engineering Units)               */
/* ========================================================================== */

/** @brief Critical over-temperature trip point: 65.0 degC (650 deci-degC). */
#define BMS_TEMP_CRITICAL_HIGH_DDEGC    ((int16_t)650)

/** @brief Thermal warning trip point: 55.0 degC (550 deci-degC). */
#define BMS_TEMP_WARN_HIGH_DDEGC        ((int16_t)550)

/** @brief Thermal recovery threshold: 48.0 degC (480 deci-degC). */
#define BMS_TEMP_RECOVERY_HIGH_DDEGC    ((int16_t)480)

/** @brief Critical under-temperature trip point: -20.0 degC (-200 deci-degC). */
#define BMS_TEMP_CRITICAL_LOW_DDEGC     ((int16_t)-200)

/** @brief Cell critical over-voltage trip point: 4250 mV (4.25 V). */
#define BMS_CELL_OVERVOLT_CRITICAL_MV   ((uint16_t)4250)

/** @brief Cell warning over-voltage trip point: 4200 mV (4.20 V). */
#define BMS_CELL_OVERVOLT_WARN_MV       ((uint16_t)4200)

/** @brief Cell critical under-voltage trip point: 2700 mV (2.70 V). */
#define BMS_CELL_UNDERVOLT_CRITICAL_MV  ((uint16_t)2700)

/** @brief Cell warning under-voltage trip point: 3000 mV (3.00 V). */
#define BMS_CELL_UNDERVOLT_WARN_MV      ((uint16_t)3000)

/** @brief Max permissible cell voltage delta before warning: 150 mV. */
#define BMS_CELL_DELTA_WARN_MV          ((uint16_t)150)

/** @brief Maximum pack continuous discharge current: 120,000 mA (120.0 A). */
#define BMS_CURRENT_OVERCURRENT_DISCHARGE_MA ((int32_t)120000)

/** @brief Maximum pack continuous charge/regen current: -80,000 mA (-80.0 A). */
#define BMS_CURRENT_OVERCURRENT_CHARGE_MA    ((int32_t)-80000)

/** @brief Road gradient threshold for downhill regen advisory: -4.0% (-40 d%). */
#define BMS_ECO_DESCENT_GRADIENT_THRESHOLD_D ((int16_t)-40)

/** @brief Minimum vehicle speed for downhill regen advisory: 20.0 km/h (200 dkm/h). */
#define BMS_ECO_MIN_SPEED_D_KPH         ((uint16_t)200)

/** @brief Low SOC threshold entering limp mode: 10.0% (100 deci-%). */
#define BMS_SOC_LIMP_THRESHOLD_DPCT     ((uint16_t)100)

/** @brief Nominal nominal pack cell capacity in milliampere-hours. */
#define BMS_NOMINAL_CELL_CAPACITY_MAH   ((uint32_t)50000)

/* ========================================================================== */
/* Enumeration Definitions                                                    */
/* ========================================================================== */

/**
 * @brief Finite State Machine (FSM) System States.
 */
typedef enum {
    SYS_INIT = 0,             /**< Boot, self-test, parameter verification. */
    SYS_NORMAL,               /**< Nominal traction and charging enabled. */
    SYS_REGEN_ADVISORY,       /**< Active Eco-Assist: optimized energy harvest. */
    SYS_LIMP_MODE,            /**< Throttled power envelope (30% max output). */
    SYS_CRITICAL_FAULT        /**< Contactors open, HV interlock tripped. */
} bms_state_t;

/**
 * @brief Return status codes for core BMS API functions.
 */
typedef enum {
    BMS_STATUS_OK = 0,        /**< Operation successful. */
    BMS_STATUS_ERR_NULL_PTR,  /**< Invalid null pointer passed as argument. */
    BMS_STATUS_ERR_PARAM,     /**< Parameter out of valid operational range. */
    BMS_STATUS_ERR_NOT_INIT   /**< Core module invoked prior to initialization. */
} bms_status_t;

/**
 * @brief Active Eco-Assist Regen Advisory Modes.
 */
typedef enum {
    REGEN_ADVISORY_NONE = 0,            /**< No eco-assist action recommended. */
    REGEN_ADVISORY_DOWNHILL_GRADE,      /**< Steep descent detected: maximize regen. */
    REGEN_ADVISORY_COASTING_OPPORTUNITY,/**< Vehicle coasting: recommend one-pedal regen. */
    REGEN_ADVISORY_DECEL_BUFFER         /**< Kinetic energy buffer approaching saturation. */
} bms_regen_advisory_t;

/**
 * @brief Bitmask definitions for system critical faults.
 */
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

/**
 * @brief Bitmask definitions for system warnings (sub-critical).
 */
typedef enum {
    BMS_WARN_NONE               = 0x00000000U,
    BMS_WARN_TEMP_HIGH          = 0x00000001U,
    BMS_WARN_SOC_LOW            = 0x00000002U,
    BMS_WARN_CELL_DELTA_HIGH    = 0x00000004U,
    BMS_WARN_POWER_DERATED      = 0x00000008U
} bms_warn_mask_t;

/* ========================================================================== */
/* Data Structure Definitions                                                 */
/* ========================================================================== */

/**
 * @brief Raw sensor acquisition telemetry structure supplied by HAL.
 *
 * All physical values use fixed-width integer scaling:
 * - Voltage: 1 mV per LSB (3700 = 3.700 V)
 * - Current: 1 mA per LSB, positive = discharge, negative = regen charge
 * - Temperature: 0.1 degC per LSB (250 = 25.0 degC)
 * - Gradient: 0.1% slope (-85 = -8.5% downhill)
 * - Speed: 0.1 km/h (650 = 65.0 km/h)
 * - Pedals: 0 - 100%
 */
typedef struct {
    uint16_t cell_voltages_mv[BMS_CELL_COUNT];       /**< Individual cell voltages in mV. */
    int16_t  cell_temperatures_ddegc[BMS_CELL_COUNT]; /**< Thermistor temperatures in 0.1 C. */
    int32_t  pack_current_ma;                        /**< Pack current: +discharge, -regen. */
    int16_t  road_gradient_tenth_pct;                /**< Pitch angle / gradient in 0.1%. */
    uint16_t vehicle_speed_tenth_kph;                /**< Speed in 0.1 km/h. */
    uint8_t  accelerator_pedal_pct;                  /**< Accelerator travel 0-100%. */
    uint8_t  brake_pedal_pct;                        /**< Brake travel 0-100%. */
    bool     sensor_comm_ok;                         /**< Sensor ASIC communication status. */
} bms_sensor_telemetry_t;

/**
 * @brief Fault debouncing counters for noise rejection.
 *
 * Automotive sensors exhibit EMI noise spikes. A fault condition must
 * persist for consecutive ticks before triggering state transition.
 */
typedef struct {
    uint8_t overtemp_fault_ticks;     /**< Counter for critical overtemp. */
    uint8_t undervolt_fault_ticks;    /**< Counter for cell undervoltage. */
    uint8_t overvolt_fault_ticks;     /**< Counter for cell overvoltage. */
    uint8_t overcurrent_fault_ticks;  /**< Counter for pack overcurrent. */
    uint8_t overtemp_warn_ticks;      /**< Counter for elevated temp warning. */
    uint8_t limp_recovery_ticks;      /**< Consecutive healthy ticks to clear limp. */
} bms_debounce_counters_t;

/**
 * @brief Computed BMS pack-level metrics and state estimation.
 */
typedef struct {
    uint32_t pack_voltage_mv;         /**< Sum of all cell voltages in mV. */
    uint16_t min_cell_mv;             /**< Minimum cell voltage in mV. */
    uint16_t max_cell_mv;             /**< Maximum cell voltage in mV. */
    uint16_t cell_delta_mv;           /**< Difference between max and min cell. */
    uint8_t  min_cell_index;          /**< Zero-based index of lowest voltage cell. */
    uint8_t  max_cell_index;          /**< Zero-based index of highest voltage cell. */
    int16_t  min_temp_ddegc;          /**< Minimum measured temperature in 0.1 C. */
    int16_t  max_temp_ddegc;          /**< Maximum measured temperature in 0.1 C. */
    int16_t  avg_temp_ddegc;          /**< Average temperature across module in 0.1 C. */
    uint16_t soc_deci_pct;            /**< State of Charge (0 - 1000 = 0.0 - 100.0%). */
    uint8_t  throttle_limit_pct;      /**< Active power cap (100% normal, 30% limp). */
    uint8_t  regen_torque_bias_pct;   /**< Eco-assist regen harvest bias (0 - 100%). */
} bms_pack_metrics_t;

/**
 * @brief Master runtime instance for the BMS core logic module.
 *
 * Maintained with zero dynamic memory allocation. Contained on stack or
 * statically allocated in the system supervisor task.
 */
typedef struct {
    bms_state_t              current_state;      /**< Active FSM state. */
    bms_state_t              previous_state;     /**< Previous FSM state. */
    uint32_t                 system_tick;        /**< Monotonic system tick count. */
    uint32_t                 fault_flags;        /**< Active critical faults (bitmask). */
    uint32_t                 warning_flags;      /**< Active warnings (bitmask). */
    bms_regen_advisory_t     regen_advisory;     /**< Eco-assist advisory status. */
    bms_debounce_counters_t  debounce;           /**< Filter counter registers. */
    bms_pack_metrics_t       metrics;            /**< Aggregated module metrics. */
    bool                     contactor_closed;   /**< High-voltage contactor state. */
    bool                     is_initialized;     /**< Module initialization flag. */
} bms_core_instance_t;

#ifdef __cplusplus
}
#endif

#endif /* BMS_TYPES_H */
