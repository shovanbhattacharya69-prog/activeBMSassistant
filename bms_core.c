#include "bms_core.h"

static void bms_calculate_cell_metrics(bms_pack_metrics_t * const p_metrics,
                                       const bms_sensor_telemetry_t * const p_telemetry);

static void bms_update_soc(bms_pack_metrics_t * const p_metrics,
                           const int32_t current_ma);

static void bms_debounce_faults(bms_core_instance_t * const p_inst,
                                const bms_sensor_telemetry_t * const p_telemetry);

static void bms_evaluate_eco_advisory(bms_core_instance_t * const p_inst,
                                      const bms_sensor_telemetry_t * const p_telemetry);

static void bms_fsm_transition(bms_core_instance_t * const p_inst,
                               const bms_state_t new_state);

static void bms_fsm_process(bms_core_instance_t * const p_inst,
                            const bms_sensor_telemetry_t * const p_telemetry);

bms_status_t bms_core_init(bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return BMS_STATUS_ERR_NULL_PTR;
    }

    p_inst->current_state     = SYS_INIT;
    p_inst->previous_state    = SYS_INIT;
    p_inst->system_tick       = 0U;
    p_inst->fault_flags       = BMS_FAULT_NONE;
    p_inst->warning_flags     = BMS_WARN_NONE;
    p_inst->regen_advisory    = REGEN_ADVISORY_NONE;
    p_inst->contactor_closed  = false;
    p_inst->is_initialized    = true;

    p_inst->debounce.overtemp_fault_ticks    = 0U;
    p_inst->debounce.undervolt_fault_ticks   = 0U;
    p_inst->debounce.overvolt_fault_ticks    = 0U;
    p_inst->debounce.overcurrent_fault_ticks = 0U;
    p_inst->debounce.overtemp_warn_ticks     = 0U;
    p_inst->debounce.limp_recovery_ticks     = 0U;

    p_inst->metrics.pack_voltage_mv          = 0U;
    p_inst->metrics.min_cell_mv              = 0xFFFFU;
    p_inst->metrics.max_cell_mv              = 0U;
    p_inst->metrics.cell_delta_mv            = 0U;
    p_inst->metrics.min_cell_index           = 0U;
    p_inst->metrics.max_cell_index           = 0U;
    p_inst->metrics.min_temp_ddegc           = 0;
    p_inst->metrics.max_temp_ddegc           = 0;
    p_inst->metrics.avg_temp_ddegc           = 0;
    p_inst->metrics.soc_deci_pct             = 850U;
    p_inst->metrics.throttle_limit_pct       = 0U;
    p_inst->metrics.regen_torque_bias_pct    = 0U;

    return BMS_STATUS_OK;
}

bms_status_t bms_core_step(bms_core_instance_t * const p_inst,
                          const bms_sensor_telemetry_t * const p_telemetry)
{
    if ((p_inst == (void *)0) || (p_telemetry == (void *)0)) {
        return BMS_STATUS_ERR_NULL_PTR;
    }

    if (!p_inst->is_initialized) {
        return BMS_STATUS_ERR_NOT_INIT;
    }

    p_inst->system_tick++;
    const bms_state_t state_at_tick_start = p_inst->current_state;

    bms_calculate_cell_metrics(&p_inst->metrics, p_telemetry);
    bms_update_soc(&p_inst->metrics, p_telemetry->pack_current_ma);
    bms_debounce_faults(p_inst, p_telemetry);
    bms_evaluate_eco_advisory(p_inst, p_telemetry);
    bms_fsm_process(p_inst, p_telemetry);

    p_inst->previous_state = state_at_tick_start;

    return BMS_STATUS_OK;
}

bms_state_t bms_core_get_state(const bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return SYS_CRITICAL_FAULT;
    }
    return p_inst->current_state;
}

const bms_pack_metrics_t * bms_core_get_metrics(const bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return (const bms_pack_metrics_t *)0;
    }
    return &p_inst->metrics;
}

uint32_t bms_core_get_faults(const bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return 0xFFFFFFFFU;
    }
    return p_inst->fault_flags;
}

uint32_t bms_core_get_warnings(const bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return 0xFFFFFFFFU;
    }
    return p_inst->warning_flags;
}

bool bms_core_is_contactor_closed(const bms_core_instance_t * const p_inst)
{
    if (p_inst == (void *)0) {
        return false;
    }
    return p_inst->contactor_closed;
}

static void bms_calculate_cell_metrics(bms_pack_metrics_t * const p_metrics,
                                       const bms_sensor_telemetry_t * const p_telemetry)
{
    uint32_t pack_volt_accum = 0U;
    int32_t  temp_accum      = 0;
    uint16_t min_v           = 0xFFFFU;
    uint16_t max_v           = 0U;
    uint8_t  min_v_idx       = 0U;
    uint8_t  max_v_idx       = 0U;

    int16_t  min_t           = 30000;
    int16_t  max_t           = -30000;

    const uint16_t *p_volt_curr = p_telemetry->cell_voltages_mv;
    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        const uint16_t v = *p_volt_curr;
        pack_volt_accum += (uint32_t)v;

        if (v < min_v) {
            min_v = v;
            min_v_idx = i;
        }
        if (v > max_v) {
            max_v = v;
            max_v_idx = i;
        }
        p_volt_curr++;
    }

    const int16_t *p_temp_curr = p_telemetry->cell_temperatures_ddegc;
    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        const int16_t t = *p_temp_curr;
        temp_accum += (int32_t)t;

        if (t < min_t) {
            min_t = t;
        }
        if (t > max_t) {
            max_t = t;
        }
        p_temp_curr++;
    }

    p_metrics->pack_voltage_mv = pack_volt_accum;
    p_metrics->min_cell_mv     = min_v;
    p_metrics->max_cell_mv     = max_v;
    p_metrics->min_cell_index  = min_v_idx;
    p_metrics->max_cell_index  = max_v_idx;
    p_metrics->cell_delta_mv   = (max_v >= min_v) ? (uint16_t)(max_v - min_v) : 0U;

    p_metrics->min_temp_ddegc  = min_t;
    p_metrics->max_temp_ddegc  = max_t;
    p_metrics->avg_temp_ddegc  = (int16_t)(temp_accum / (int32_t)BMS_CELL_COUNT);
}

static void bms_update_soc(bms_pack_metrics_t * const p_metrics,
                           const int32_t current_ma)
{
    const int32_t delta_deci_pct = -(current_ma) / 25000;
    int32_t new_soc = (int32_t)p_metrics->soc_deci_pct + delta_deci_pct;

    if (new_soc > 1000) {
        new_soc = 1000;
    } else if (new_soc < 0) {
        new_soc = 0;
    }

    p_metrics->soc_deci_pct = (uint16_t)new_soc;
}

static void bms_debounce_faults(bms_core_instance_t * const p_inst,
                                const bms_sensor_telemetry_t * const p_telemetry)
{
    bms_debounce_counters_t * const p_deb = &p_inst->debounce;
    const bms_pack_metrics_t * const p_met = &p_inst->metrics;

    if (p_met->max_temp_ddegc >= BMS_TEMP_CRITICAL_HIGH_DDEGC) {
        if (p_deb->overtemp_fault_ticks < 255U) {
            p_deb->overtemp_fault_ticks++;
        }
        if (p_deb->overtemp_fault_ticks >= BMS_DEBOUNCE_FAULT_LIMIT) {
            p_inst->fault_flags |= (uint32_t)BMS_FAULT_OVERTEMP_CRIT;
        }
    } else {
        p_deb->overtemp_fault_ticks = 0U;
    }

    if (p_met->min_cell_mv <= BMS_CELL_UNDERVOLT_CRITICAL_MV) {
        if (p_deb->undervolt_fault_ticks < 255U) {
            p_deb->undervolt_fault_ticks++;
        }
        if (p_deb->undervolt_fault_ticks >= BMS_DEBOUNCE_FAULT_LIMIT) {
            p_inst->fault_flags |= (uint32_t)BMS_FAULT_CELL_UNDERVOLT;
        }
    } else {
        p_deb->undervolt_fault_ticks = 0U;
    }

    if (p_met->max_cell_mv >= BMS_CELL_OVERVOLT_CRITICAL_MV) {
        if (p_deb->overvolt_fault_ticks < 255U) {
            p_deb->overvolt_fault_ticks++;
        }
        if (p_deb->overvolt_fault_ticks >= BMS_DEBOUNCE_FAULT_LIMIT) {
            p_inst->fault_flags |= (uint32_t)BMS_FAULT_CELL_OVERVOLT;
        }
    } else {
        p_deb->overvolt_fault_ticks = 0U;
    }

    if ((p_telemetry->pack_current_ma >= BMS_CURRENT_OVERCURRENT_DISCHARGE_MA) ||
        (p_telemetry->pack_current_ma <= BMS_CURRENT_OVERCURRENT_CHARGE_MA)) {
        if (p_deb->overcurrent_fault_ticks < 255U) {
            p_deb->overcurrent_fault_ticks++;
        }
        if (p_deb->overcurrent_fault_ticks >= BMS_DEBOUNCE_FAULT_LIMIT) {
            if (p_telemetry->pack_current_ma > 0) {
                p_inst->fault_flags |= (uint32_t)BMS_FAULT_OVERCURRENT_DISC;
            } else {
                p_inst->fault_flags |= (uint32_t)BMS_FAULT_OVERCURRENT_CHG;
            }
        }
    } else {
        p_deb->overcurrent_fault_ticks = 0U;
    }

    if (!p_telemetry->sensor_comm_ok) {
        p_inst->fault_flags |= (uint32_t)BMS_FAULT_SENSOR_COMM_LOSS;
    }

    if (p_met->max_temp_ddegc >= BMS_TEMP_WARN_HIGH_DDEGC) {
        if (p_deb->overtemp_warn_ticks < 255U) {
            p_deb->overtemp_warn_ticks++;
        }
        if (p_deb->overtemp_warn_ticks >= BMS_DEBOUNCE_FAULT_LIMIT) {
            p_inst->warning_flags |= (uint32_t)BMS_WARN_TEMP_HIGH;
        }
    } else {
        p_deb->overtemp_warn_ticks = 0U;
    }

    if (p_met->soc_deci_pct <= BMS_SOC_LIMP_THRESHOLD_DPCT) {
        p_inst->warning_flags |= (uint32_t)BMS_WARN_SOC_LOW;
    } else {
        p_inst->warning_flags &= ~((uint32_t)BMS_WARN_SOC_LOW);
    }

    if (p_met->cell_delta_mv >= BMS_CELL_DELTA_WARN_MV) {
        p_inst->warning_flags |= (uint32_t)BMS_WARN_CELL_DELTA_HIGH;
    } else {
        p_inst->warning_flags &= ~((uint32_t)BMS_WARN_CELL_DELTA_HIGH);
    }
}

static void bms_evaluate_eco_advisory(bms_core_instance_t * const p_inst,
                                      const bms_sensor_telemetry_t * const p_telemetry)
{
    const bool is_downhill = (p_telemetry->road_gradient_tenth_pct <= BMS_ECO_DESCENT_GRADIENT_THRESHOLD_D);
    const bool is_speed_adequate = (p_telemetry->vehicle_speed_tenth_kph >= BMS_ECO_MIN_SPEED_D_KPH);
    const bool accel_idle = (p_telemetry->accelerator_pedal_pct < 5U);
    const bool brake_idle = (p_telemetry->brake_pedal_pct < 5U);

    if (is_downhill && is_speed_adequate && accel_idle) {
        p_inst->regen_advisory = REGEN_ADVISORY_DOWNHILL_GRADE;

        int16_t abs_grade = (int16_t)(-p_telemetry->road_gradient_tenth_pct);
        if (abs_grade > 120) {
            abs_grade = 120;
        }
        p_inst->metrics.regen_torque_bias_pct = (uint8_t)(50U + (uint8_t)((abs_grade * 35) / 100));
    }
    else if (accel_idle && brake_idle && (p_telemetry->vehicle_speed_tenth_kph >= 300U)) {
        p_inst->regen_advisory = REGEN_ADVISORY_COASTING_OPPORTUNITY;
        p_inst->metrics.regen_torque_bias_pct = 40U;
    }
    else {
        p_inst->regen_advisory = REGEN_ADVISORY_NONE;
        p_inst->metrics.regen_torque_bias_pct = 15U;
    }
}

static void bms_fsm_transition(bms_core_instance_t * const p_inst,
                               const bms_state_t new_state)
{
    p_inst->current_state = new_state;

    switch (new_state) {
        case SYS_NORMAL:
            p_inst->contactor_closed            = true;
            p_inst->metrics.throttle_limit_pct  = 100U;
            break;

        case SYS_REGEN_ADVISORY:
            p_inst->contactor_closed            = true;
            p_inst->metrics.throttle_limit_pct  = 80U;
            break;

        case SYS_LIMP_MODE:
            p_inst->contactor_closed            = true;
            p_inst->metrics.throttle_limit_pct  = 30U;
            p_inst->warning_flags              |= (uint32_t)BMS_WARN_POWER_DERATED;
            break;

        case SYS_CRITICAL_FAULT:
            p_inst->contactor_closed            = false;
            p_inst->metrics.throttle_limit_pct  = 0U;
            p_inst->metrics.regen_torque_bias_pct = 0U;
            break;

        case SYS_INIT:
        default:
            p_inst->contactor_closed            = false;
            p_inst->metrics.throttle_limit_pct  = 0U;
            break;
    }
}

static void bms_fsm_process(bms_core_instance_t * const p_inst,
                            const bms_sensor_telemetry_t * const p_telemetry)
{
    if ((p_inst->fault_flags != BMS_FAULT_NONE) &&
        (p_inst->current_state != SYS_CRITICAL_FAULT)) {
        bms_fsm_transition(p_inst, SYS_CRITICAL_FAULT);
    }

    switch (p_inst->current_state) {
        case SYS_INIT:
        {
            const bool comm_ok = p_telemetry->sensor_comm_ok;
            const bool volt_in_range = (p_inst->metrics.min_cell_mv >= BMS_CELL_UNDERVOLT_WARN_MV) &&
                                       (p_inst->metrics.max_cell_mv <= BMS_CELL_OVERVOLT_WARN_MV);
            const bool temp_in_range = (p_inst->metrics.max_temp_ddegc < BMS_TEMP_WARN_HIGH_DDEGC) &&
                                       (p_inst->metrics.min_temp_ddegc > BMS_TEMP_CRITICAL_LOW_DDEGC);

            if (comm_ok && volt_in_range && temp_in_range) {
                bms_fsm_transition(p_inst, SYS_NORMAL);
            } else {
                bms_fsm_transition(p_inst, SYS_CRITICAL_FAULT);
            }
            break;
        }

        case SYS_NORMAL:
        {
            if ((p_inst->warning_flags & (uint32_t)BMS_WARN_TEMP_HIGH) != 0U) {
                bms_fsm_transition(p_inst, SYS_LIMP_MODE);
            }
            else if ((p_inst->warning_flags & (uint32_t)BMS_WARN_SOC_LOW) != 0U) {
                bms_fsm_transition(p_inst, SYS_LIMP_MODE);
            }
            else if (p_inst->regen_advisory == REGEN_ADVISORY_DOWNHILL_GRADE) {
                bms_fsm_transition(p_inst, SYS_REGEN_ADVISORY);
            }
            break;
        }

        case SYS_REGEN_ADVISORY:
        {
            if ((p_inst->warning_flags & (uint32_t)BMS_WARN_TEMP_HIGH) != 0U) {
                bms_fsm_transition(p_inst, SYS_LIMP_MODE);
            }
            else if ((p_telemetry->accelerator_pedal_pct > 15U) ||
                     (p_inst->regen_advisory == REGEN_ADVISORY_NONE)) {
                bms_fsm_transition(p_inst, SYS_NORMAL);
            }
            break;
        }

        case SYS_LIMP_MODE:
        {
            const bool temp_recovered = (p_inst->metrics.max_temp_ddegc <= BMS_TEMP_RECOVERY_HIGH_DDEGC);
            const bool soc_ok = (p_inst->metrics.soc_deci_pct > BMS_SOC_LIMP_THRESHOLD_DPCT);

            if (temp_recovered && soc_ok) {
                p_inst->debounce.limp_recovery_ticks++;
                if (p_inst->debounce.limp_recovery_ticks >= BMS_DEBOUNCE_RECOVERY_LIMIT) {
                    p_inst->warning_flags &= ~((uint32_t)BMS_WARN_TEMP_HIGH);
                    p_inst->warning_flags &= ~((uint32_t)BMS_WARN_POWER_DERATED);
                    p_inst->debounce.limp_recovery_ticks = 0U;
                    bms_fsm_transition(p_inst, SYS_NORMAL);
                }
            } else {
                p_inst->debounce.limp_recovery_ticks = 0U;
            }
            break;
        }

        case SYS_CRITICAL_FAULT:
        {
            p_inst->contactor_closed            = false;
            p_inst->metrics.throttle_limit_pct  = 0U;
            p_inst->metrics.regen_torque_bias_pct = 0U;
            break;
        }

        default:
        {
            bms_fsm_transition(p_inst, SYS_CRITICAL_FAULT);
            break;
        }
    }
}
