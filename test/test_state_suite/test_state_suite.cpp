#include <unity.h>
#include "NativeTestHelper.h"

#include "test_state/test_state.inc"
#include "test_default_state/test_default_state.inc"
#include "test_astra_config/test_astra_config.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_state
    test_state_setUp();
    RUN_TEST(test_state_test_constructor_requires_filters);
    RUN_TEST(test_state_test_begin_fails_without_kalman_filter);
    RUN_TEST(test_state_test_begin_fails_without_orientation_filter);
    RUN_TEST(test_state_test_begin_succeeds_with_both_filters);
    RUN_TEST(test_state_test_set_gps_origin);
    RUN_TEST(test_state_test_gps_origin_auto_set_on_first_measurement);
    RUN_TEST(test_state_test_set_baro_origin);
    RUN_TEST(test_state_test_baro_relative_to_origin);
    RUN_TEST(test_state_test_update_orientation_low_g);
    RUN_TEST(test_state_test_update_orientation_high_g);
    RUN_TEST(test_state_test_update_orientation_null_filter);
    RUN_TEST(test_state_test_predict_calls_kalman_filter);
    RUN_TEST(test_state_test_predict_updates_state_from_filter);
    RUN_TEST(test_state_test_predict_null_filter);
    RUN_TEST(test_state_test_update_measurements_gps_only);
    RUN_TEST(test_state_test_update_measurements_baro_only);
    RUN_TEST(test_state_test_update_measurements_gps_and_baro);
    RUN_TEST(test_state_test_update_measurements_no_sensors);
    RUN_TEST(test_state_test_update_measurements_updates_state_from_filter);
    RUN_TEST(test_state_test_update_measurements_null_filter);
    RUN_TEST(test_state_test_getters_return_correct_values);
    RUN_TEST(test_state_test_get_orientation);
    RUN_TEST(test_state_test_get_acceleration);
    RUN_TEST(test_state_test_get_orientation_filter);
    RUN_TEST(test_state_test_deprecated_update_returns_error);
    RUN_TEST(test_state_test_deprecated_predict_state_does_nothing);
    RUN_TEST(test_state_test_full_update_cycle);
    RUN_TEST(test_state_test_multiple_orientation_updates);
    test_state_tearDown();

    // Tests from test_default_state
    test_default_state_setUp();
    RUN_TEST(test_default_state_test_constructor_default_parameters);
    RUN_TEST(test_default_state_test_constructor_custom_parameters);
    RUN_TEST(test_default_state_test_constructor_creates_owned_filters);
    RUN_TEST(test_default_state_test_begin_initializes_filters);
    RUN_TEST(test_default_state_test_begin_returns_success);
    RUN_TEST(test_default_state_test_get_default_kalman_filter);
    RUN_TEST(test_default_state_test_get_orientation_filter);
    RUN_TEST(test_default_state_test_update_orientation_integrates_gyro);
    RUN_TEST(test_default_state_test_update_orientation_transforms_acceleration);
    RUN_TEST(test_default_state_test_predict_updates_position_velocity);
    RUN_TEST(test_default_state_test_predict_with_zero_dt);
    RUN_TEST(test_default_state_test_predict_accumulates_over_time);
    RUN_TEST(test_default_state_test_update_measurements_gps_only);
    RUN_TEST(test_default_state_test_update_measurements_auto_sets_gps_origin);
    RUN_TEST(test_default_state_test_update_measurements_relative_to_origin);
    RUN_TEST(test_default_state_test_update_measurements_baro_only);
    RUN_TEST(test_default_state_test_update_measurements_baro_relative_to_origin);
    RUN_TEST(test_default_state_test_update_measurements_gps_and_baro);
    RUN_TEST(test_default_state_test_update_measurements_fuses_horizontal_and_vertical);
    RUN_TEST(test_default_state_test_get_position);
    RUN_TEST(test_default_state_test_get_velocity);
    RUN_TEST(test_default_state_test_get_acceleration);
    RUN_TEST(test_default_state_test_get_orientation);
    RUN_TEST(test_default_state_test_set_gps_origin_once);
    RUN_TEST(test_default_state_test_set_baro_origin_once);
    RUN_TEST(test_default_state_test_deprecated_update_returns_error);
    RUN_TEST(test_default_state_test_full_flight_simulation);
    RUN_TEST(test_default_state_test_continuous_updates);
    RUN_TEST(test_default_state_test_filter_parameter_tuning);
    RUN_TEST(test_default_state_test_asynchronous_sensor_updates);
    test_default_state_tearDown();

    // Tests from test_astra_config
    test_astra_config_setUp();
    RUN_TEST(test_astra_config_test_constructor_default_values);
    RUN_TEST(test_astra_config_test_with_state);
    RUN_TEST(test_astra_config_test_with_state_chaining);
    RUN_TEST(test_astra_config_test_with_accel);
    RUN_TEST(test_astra_config_test_with_gyro);
    RUN_TEST(test_astra_config_test_with_mag);
    RUN_TEST(test_astra_config_test_with_baro);
    RUN_TEST(test_astra_config_test_with_gps);
    RUN_TEST(test_astra_config_test_with_all_sensors_chaining);
    RUN_TEST(test_astra_config_test_with_misc_sensor_single);
    RUN_TEST(test_astra_config_test_with_misc_sensor_multiple);
    RUN_TEST(test_astra_config_test_with_misc_sensor_null_rejected);
    RUN_TEST(test_astra_config_test_with_misc_sensor_max_capacity);
    RUN_TEST(test_astra_config_test_with_6dof_imu);
    RUN_TEST(test_astra_config_test_with_6dof_imu_null_rejected);
    RUN_TEST(test_astra_config_test_with_9dof_imu);
    RUN_TEST(test_astra_config_test_with_9dof_imu_null_rejected);
    RUN_TEST(test_astra_config_test_with_logging_rate);
    RUN_TEST(test_astra_config_test_with_logging_rate_sets_interval);
    RUN_TEST(test_astra_config_test_with_logging_interval);
    RUN_TEST(test_astra_config_test_with_logging_interval_sets_rate);
    RUN_TEST(test_astra_config_test_logging_rate_and_interval_mutually_exclusive);
    RUN_TEST(test_astra_config_test_with_buzzer_pin);
    RUN_TEST(test_astra_config_test_with_bb_pin_single);
    RUN_TEST(test_astra_config_test_with_bb_pin_multiple);
    RUN_TEST(test_astra_config_test_with_bb_pin_duplicate_rejected);
    RUN_TEST(test_astra_config_test_with_bb_async);
    RUN_TEST(test_astra_config_test_with_bb_async_disabled);
    RUN_TEST(test_astra_config_test_with_status_led);
    RUN_TEST(test_astra_config_test_with_status_buzzer);
    RUN_TEST(test_astra_config_test_with_gps_fix_led);
    RUN_TEST(test_astra_config_test_status_indicators_auto_add_to_bb);
    RUN_TEST(test_astra_config_test_with_data_logs_single);
    RUN_TEST(test_astra_config_test_with_data_logs_multiple);
    RUN_TEST(test_astra_config_test_with_data_logs_max_capacity);
    RUN_TEST(test_astra_config_test_with_hitl_enabled);
    RUN_TEST(test_astra_config_test_with_hitl_disabled);
    RUN_TEST(test_astra_config_test_complete_configuration_chain);
    RUN_TEST(test_astra_config_test_imu_based_configuration);
    RUN_TEST(test_astra_config_test_minimal_configuration);
    RUN_TEST(test_astra_config_test_reconfiguration_override);
    RUN_TEST(test_astra_config_test_mixed_logging_configuration);
    RUN_TEST(test_astra_config_test_multiple_state_assignments);
    RUN_TEST(test_astra_config_test_bb_pin_limit);
    test_astra_config_tearDown();

    UNITY_END();
    return 0;
}
