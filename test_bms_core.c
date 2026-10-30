#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "bms_types.h"
#include "bms_core.h"

static uint32_t s_tests_run    = 0U;
static uint32_t s_tests_passed = 0U;

#define TEST_ASSERT(cond, msg) do { \
    s_tests_run++; \
    if (cond) { \
        s_tests_passed++; \
        printf("  [PASS] %s\n", msg); \
    } else { \
        printf("  [FAIL] %s (Line %d)\n", msg, __LINE__); \
    } \
} while(0)

static void test_null_pointer_defensive(void)
{
    printf("\n=== TEST CASE: NULL Pointer Defensive Boundary Checks ===\n");
    bms_core_instance_t inst;
    bms_sensor_telemetry_t telem;

    TEST_ASSERT(bms_core_init((void *)0) == BMS_STATUS_ERR_NULL_PTR,
                "bms_core_init rejects NULL instance");

    TEST_ASSERT(bms_core_step((void *)0, &telem) == BMS_STATUS_ERR_NULL_PTR,
                "bms_core_step rejects NULL instance");

    TEST_ASSERT(bms_core_step(&inst, (void *)0) == BMS_STATUS_ERR_NULL_PTR,
                "bms_core_step rejects NULL telemetry");

    TEST_ASSERT(bms_core_get_state((void *)0) == SYS_CRITICAL_FAULT,
                "bms_core_get_state fails safe to SYS_CRITICAL_FAULT on NULL");

    TEST_ASSERT(bms_core_is_contactor_closed((void *)0) == false,
                "bms_core_is_contactor_closed fails safe (false) on NULL");
}

static void test_pointer_array_metrics(void)
{
    printf("\n=== TEST CASE: Pointer-Based Cell Array Traversal Metrics ===\n");
    bms_core_instance_t inst;
    bms_sensor_telemetry_t telem;

    (void)bms_core_init(&inst);

    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        telem.cell_voltages_mv[i] = (uint16_t)(3600U + (uint16_t)(i * 10U));
        telem.cell_temperatures_ddegc[i] = 250;
    }
    telem.pack_current_ma         = 10000;
    telem.road_gradient_tenth_pct = 0;
    telem.vehicle_speed_tenth_kph = 500U;
    telem.accelerator_pedal_pct   = 20U;
    telem.brake_pedal_pct         = 0U;
    telem.sensor_comm_ok          = true;

    (void)bms_core_step(&inst, &telem);

    const bms_pack_metrics_t *p_met = bms_core_get_metrics(&inst);
    TEST_ASSERT(p_met != (void *)0, "Metrics pointer valid");
    TEST_ASSERT(p_met->min_cell_mv == 3600U, "Min cell voltage matches cell #0 (3600 mV)");
    TEST_ASSERT(p_met->max_cell_mv == (3600U + (15U * 10U)), "Max cell voltage matches cell #15 (3750 mV)");
    TEST_ASSERT(p_met->cell_delta_mv == 150U, "Cell delta correctly computed as 150 mV");
    TEST_ASSERT(p_met->min_temp_ddegc == 250, "Min temp is 25.0 degC");
    TEST_ASSERT(p_met->max_temp_ddegc == 250, "Max temp is 25.0 degC");
    TEST_ASSERT(p_met->avg_temp_ddegc == 250, "Avg temp is 25.0 degC");
}

static void test_sensor_debounce_noise_rejection(void)
{
    printf("\n=== TEST CASE: Transient EMI Noise Rejection (2-Tick Spike) ===\n");
    bms_core_instance_t inst;
    bms_sensor_telemetry_t telem;

    (void)bms_core_init(&inst);

    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        telem.cell_voltages_mv[i] = 3800U;
        telem.cell_temperatures_ddegc[i] = 300;
    }
    telem.pack_current_ma         = 20000;
    telem.road_gradient_tenth_pct = 0;
    telem.vehicle_speed_tenth_kph = 500U;
    telem.accelerator_pedal_pct   = 20U;
    telem.brake_pedal_pct         = 0U;
    telem.sensor_comm_ok          = true;

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Init step advances to SYS_NORMAL");

    telem.cell_temperatures_ddegc[3] = 720;
    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Noise Tick 1: System remains in SYS_NORMAL");
    TEST_ASSERT(inst.debounce.overtemp_fault_ticks == 1U, "Debounce count incremented to 1/3");

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Noise Tick 2: System remains in SYS_NORMAL");
    TEST_ASSERT(inst.debounce.overtemp_fault_ticks == 2U, "Debounce count incremented to 2/3");

    telem.cell_temperatures_ddegc[3] = 300;
    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Glitch ended: System STILL in SYS_NORMAL (NO TRIP)");
    TEST_ASSERT(inst.debounce.overtemp_fault_ticks == 0U, "Debounce counter successfully reset to 0/3");
    TEST_ASSERT(bms_core_is_contactor_closed(&inst) == true, "Contactors remain safely CLOSED");
}

static void test_sensor_debounce_trip_and_interlock(void)
{
    printf("\n=== TEST CASE: 3-Tick Debounced Fault Trip & Contactor Interlock ===\n");
    bms_core_instance_t inst;
    bms_sensor_telemetry_t telem;

    (void)bms_core_init(&inst);

    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        telem.cell_voltages_mv[i] = 3800U;
        telem.cell_temperatures_ddegc[i] = 300;
    }
    telem.pack_current_ma         = 20000;
    telem.road_gradient_tenth_pct = 0;
    telem.vehicle_speed_tenth_kph = 500U;
    telem.accelerator_pedal_pct   = 20U;
    telem.brake_pedal_pct         = 0U;
    telem.sensor_comm_ok          = true;

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "System entered SYS_NORMAL");

    telem.cell_temperatures_ddegc[5] = 680;

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Fault Tick 1/3: No trip yet");

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Fault Tick 2/3: No trip yet");

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_CRITICAL_FAULT,
                "Fault Tick 3/3: FSM successfully transitioned to SYS_CRITICAL_FAULT");
    TEST_ASSERT(bms_core_is_contactor_closed(&inst) == false,
                "High-Voltage Contactor FORCED OPEN (Safety Interlock)");
    TEST_ASSERT((bms_core_get_faults(&inst) & (uint32_t)BMS_FAULT_OVERTEMP_CRIT) != 0U,
                "Critical over-temperature fault flag latched");
    TEST_ASSERT(inst.metrics.throttle_limit_pct == 0U,
                "Traction power output throttled to 0%");
}

static void test_eco_assist_regen_advisory(void)
{
    printf("\n=== TEST CASE: Active Eco-Assist Downhill Incline Regen Biasing ===\n");
    bms_core_instance_t inst;
    bms_sensor_telemetry_t telem;

    (void)bms_core_init(&inst);

    for (uint8_t i = 0U; i < BMS_CELL_COUNT; ++i) {
        telem.cell_voltages_mv[i] = 3700U;
        telem.cell_temperatures_ddegc[i] = 300;
    }
    telem.pack_current_ma         = 20000;
    telem.road_gradient_tenth_pct = 0;
    telem.vehicle_speed_tenth_kph = 500U;
    telem.accelerator_pedal_pct   = 20U;
    telem.brake_pedal_pct         = 0U;
    telem.sensor_comm_ok          = true;

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_NORMAL, "Nominal baseline established");

    telem.road_gradient_tenth_pct = -85;
    telem.vehicle_speed_tenth_kph = 650U;
    telem.accelerator_pedal_pct   = 0U;
    telem.brake_pedal_pct         = 10U;

    (void)bms_core_step(&inst, &telem);
    TEST_ASSERT(bms_core_get_state(&inst) == SYS_REGEN_ADVISORY,
                "Eco-Assist downhill condition triggered SYS_REGEN_ADVISORY");
    TEST_ASSERT(inst.metrics.regen_torque_bias_pct >= 75U,
                "Dynamic regen torque bias elevated (> 75%) to harvest kinetic energy");
    TEST_ASSERT(inst.regen_advisory == REGEN_ADVISORY_DOWNHILL_GRADE,
                "Advisory mode correctly identified as REGEN_ADVISORY_DOWNHILL_GRADE");
}

int main(void)
{
    printf("======================================================================\n");
    printf("   ACTIVE ECO-ASSIST BMS CORE - AUTOMOTIVE UNIT TEST QUALIFICATION    \n");
    printf("   MISRA C & ISO 26262 Part 6 Software Verification Test Suite        \n");
    printf("======================================================================\n");

    test_null_pointer_defensive();
    test_pointer_array_metrics();
    test_sensor_debounce_noise_rejection();
    test_sensor_debounce_trip_and_interlock();
    test_eco_assist_regen_advisory();

    printf("\n======================================================================\n");
    printf("   TEST SUMMARY: %u / %u Passed (%.1f%% Success Rate)\n",
           (unsigned int)s_tests_passed,
           (unsigned int)s_tests_run,
           ((float)s_tests_passed / (float)s_tests_run) * 100.0f);
    printf("======================================================================\n");

    return (s_tests_passed == s_tests_run) ? 0 : 1;
}
