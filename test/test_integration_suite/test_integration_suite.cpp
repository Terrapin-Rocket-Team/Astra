#include <unity.h>
#include "NativeTestHelper.h"

#include "test_astra/test_astra.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_astra
    test_astra_setUp();
    RUN_TEST(test_astra_test_constructor_with_config);
    RUN_TEST(test_astra_test_constructor_null_config);
    RUN_TEST(test_astra_test_init_minimal_config);
    RUN_TEST(test_astra_test_init_with_all_sensors);
    RUN_TEST(test_astra_test_init_with_failing_sensor);
    RUN_TEST(test_astra_test_init_sets_baro_origin);
    RUN_TEST(test_astra_test_init_creates_message_router);
    RUN_TEST(test_astra_test_update_without_init_auto_initializes);
    RUN_TEST(test_astra_test_update_with_simulation_time);
    RUN_TEST(test_astra_test_update_without_time_uses_millis);
    RUN_TEST(test_astra_test_update_sets_did_flags);
    RUN_TEST(test_astra_test_update_clears_did_flags_each_cycle);
    RUN_TEST(test_astra_test_update_sensors_every_cycle);
    RUN_TEST(test_astra_test_update_orientation_when_imu_available);
    RUN_TEST(test_astra_test_update_skips_unhealthy_imu);
    RUN_TEST(test_astra_test_predict_runs_after_orientation_update);
    RUN_TEST(test_astra_test_update_measurements_with_gps_fix);
    RUN_TEST(test_astra_test_update_measurements_with_baro);
    RUN_TEST(test_astra_test_update_measurements_with_gps_and_baro);
    RUN_TEST(test_astra_test_update_skips_gps_without_fix);
    RUN_TEST(test_astra_test_update_skips_unhealthy_gps);
    RUN_TEST(test_astra_test_update_skips_unhealthy_baro);
    RUN_TEST(test_astra_test_logging_at_specified_interval);
    RUN_TEST(test_astra_test_logging_rate_conversion);
    RUN_TEST(test_astra_test_hitl_mode_enabled);
    RUN_TEST(test_astra_test_hitl_mode_requires_simulation_time);
    RUN_TEST(test_astra_test_complete_update_cycle);
    RUN_TEST(test_astra_test_multiple_update_cycles);
    RUN_TEST(test_astra_test_flight_simulation);
    RUN_TEST(test_astra_test_sensor_failure_during_flight);
    RUN_TEST(test_astra_test_update_without_state);
    RUN_TEST(test_astra_test_rapid_updates);
    RUN_TEST(test_astra_test_large_time_jump);
    RUN_TEST(test_astra_test_backwards_time);
    test_astra_tearDown();

    UNITY_END();
    return 0;
}
