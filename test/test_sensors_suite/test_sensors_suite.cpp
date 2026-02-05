#include <unity.h>
#include "NativeTestHelper.h"

#include "test_accel/test_accel.inc"
#include "test_gyro/test_gyro.inc"
#include "test_mag/test_mag.inc"
#include "test_baro/test_baro.inc"
#include "test_gps/test_gps.inc"
#include "test_imu/test_imu.inc"
#include "test_dual_range_accel/test_dual_range_accel.inc"
#include "test_rotatable_sensor/test_rotatable_sensor.inc"
#include "test_voltage_sensor/test_voltage_sensor.inc"
#include "test_sensor/test_sensor.inc"
#include "test_sensor_manager/test_sensor_manager.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_accel
    test_accel_setUp();
    RUN_TEST(test_accel_test_accel_begin);
    RUN_TEST(test_accel_test_accel_set);
    RUN_TEST(test_accel_test_accel_get_accel);
    RUN_TEST(test_accel_test_accel_orientation_identity);
    RUN_TEST(test_accel_test_accel_orientation_flip_yz);
    RUN_TEST(test_accel_test_accel_orientation_rotate_90_z);
    RUN_TEST(test_accel_test_accel_zero);
    RUN_TEST(test_accel_test_accel_negative);
    RUN_TEST(test_accel_test_accel_large_values);
    RUN_TEST(test_accel_test_accel_multiple_updates);
    RUN_TEST(test_accel_test_accel_change_orientation);
    RUN_TEST(test_accel_test_accel_magnitude);
    RUN_TEST(test_accel_test_accel_orientation_preserves_magnitude);
    RUN_TEST(test_accel_test_accel_get_mounting_orientation);
    test_accel_tearDown();

    // Tests from test_gyro
    test_gyro_setUp();
    RUN_TEST(test_gyro_test_gyro_begin);
    RUN_TEST(test_gyro_test_gyro_set);
    RUN_TEST(test_gyro_test_gyro_get_ang_vel);
    RUN_TEST(test_gyro_test_gyro_orientation_identity);
    RUN_TEST(test_gyro_test_gyro_orientation_flip_yz);
    RUN_TEST(test_gyro_test_gyro_orientation_rotate_90_z);
    RUN_TEST(test_gyro_test_gyro_orientation_rotate_90_x);
    RUN_TEST(test_gyro_test_gyro_zero);
    RUN_TEST(test_gyro_test_gyro_negative);
    RUN_TEST(test_gyro_test_gyro_large_values);
    RUN_TEST(test_gyro_test_gyro_small_values);
    RUN_TEST(test_gyro_test_gyro_multiple_updates);
    RUN_TEST(test_gyro_test_gyro_change_orientation);
    RUN_TEST(test_gyro_test_gyro_magnitude);
    RUN_TEST(test_gyro_test_gyro_orientation_preserves_magnitude);
    RUN_TEST(test_gyro_test_gyro_get_mounting_orientation);
    RUN_TEST(test_gyro_test_gyro_typical_rotation_rates);
    test_gyro_tearDown();

    // Tests from test_mag
    test_mag_setUp();
    RUN_TEST(test_mag_test_mag_begin);
    RUN_TEST(test_mag_test_mag_set);
    RUN_TEST(test_mag_test_mag_get_mag);
    RUN_TEST(test_mag_test_mag_orientation_identity);
    RUN_TEST(test_mag_test_mag_orientation_flip_yz);
    RUN_TEST(test_mag_test_mag_orientation_rotate_90_z);
    RUN_TEST(test_mag_test_mag_orientation_rotate_90_x);
    RUN_TEST(test_mag_test_mag_zero);
    RUN_TEST(test_mag_test_mag_negative);
    RUN_TEST(test_mag_test_mag_earth_field_typical);
    RUN_TEST(test_mag_test_mag_small_values);
    RUN_TEST(test_mag_test_mag_large_values);
    RUN_TEST(test_mag_test_mag_multiple_updates);
    RUN_TEST(test_mag_test_mag_change_orientation);
    RUN_TEST(test_mag_test_mag_magnitude);
    RUN_TEST(test_mag_test_mag_orientation_preserves_magnitude);
    RUN_TEST(test_mag_test_mag_get_mounting_orientation);
    RUN_TEST(test_mag_test_mag_horizontal_component);
    RUN_TEST(test_mag_test_mag_isotropic_field);
    test_mag_tearDown();

    // Tests from test_baro
    test_baro_setUp();
    RUN_TEST(test_baro_test_function_name);
    RUN_TEST(test_baro_test_baro_begin);
    RUN_TEST(test_baro_test_baro_set);
    RUN_TEST(test_baro_test_baro_alt);
    RUN_TEST(test_baro_test_baro_conversions);
    test_baro_tearDown();

    // Tests from test_gps
    test_gps_setUp();
    RUN_TEST(test_gps_test_function_name);
    RUN_TEST(test_gps_test_gps_distance_formula);
    RUN_TEST(test_gps_test_gps_begin);
    RUN_TEST(test_gps_test_gps_set);
    RUN_TEST(test_gps_test_gps_first_fix);
    test_gps_tearDown();

    // Tests from test_imu
    test_imu_setUp();
    RUN_TEST(test_imu_test_imu_begin);
    RUN_TEST(test_imu_test_imu_set);
    RUN_TEST(test_imu_test_imu_get_sensors);
    RUN_TEST(test_imu_test_imu_mounting_orientation);
    RUN_TEST(test_imu_test_imu_component_orientation);
    RUN_TEST(test_imu_test_imu_multiple_updates);
    RUN_TEST(test_imu_test_imu_zero_values);
    RUN_TEST(test_imu_test_imu_large_values);
    RUN_TEST(test_imu_test_imu_negative_values);
    test_imu_tearDown();

    // Tests from test_dual_range_accel
    test_dual_range_accel_setUp();
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_begin);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_low_g_range);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_high_g_range);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_transition_range);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_at_min_high_g);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_at_max_low_g);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_multi_axis);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_mounting_orientation_warning);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_update_failure);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_different_sensor_readings);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_zero);
    RUN_TEST(test_dual_range_accel_test_dual_range_accel_negative_values);
    test_dual_range_accel_tearDown();

    // Tests from test_rotatable_sensor
    test_rotatable_sensor_setUp();
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_identity);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_flip_yz);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_flip_xz);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_flip_xy);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_90_x);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_neg90_x);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_90_y);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_neg90_y);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_90_z);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_rotate_neg90_z);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_change_orientation);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_zero_vector);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_negative_values);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_preserves_magnitude);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_gyro);
    RUN_TEST(test_rotatable_sensor_test_rotatable_sensor_orientation_after_data_change);
    test_rotatable_sensor_tearDown();

    // Tests from test_voltage_sensor
    test_voltage_sensor_setUp();
    RUN_TEST(test_voltage_sensor_test_simple_constructor);
    RUN_TEST(test_voltage_sensor_test_voltage_divider_constructor);
    RUN_TEST(test_voltage_sensor_test_constructor_adds_column);
    RUN_TEST(test_voltage_sensor_test_begin_calls_init);
    RUN_TEST(test_voltage_sensor_test_read_basic);
    RUN_TEST(test_voltage_sensor_test_read_zero_value);
    RUN_TEST(test_voltage_sensor_test_read_max_value);
    RUN_TEST(test_voltage_sensor_test_read_with_voltage_divider);
    RUN_TEST(test_voltage_sensor_test_read_without_voltage_divider);
    RUN_TEST(test_voltage_sensor_test_getVoltage);
    RUN_TEST(test_voltage_sensor_test_getRawValue);
    RUN_TEST(test_voltage_sensor_test_update_calls_read);
    RUN_TEST(test_voltage_sensor_test_voltage_divider_ratio_2);
    RUN_TEST(test_voltage_sensor_test_voltage_divider_ratio_4);
    RUN_TEST(test_voltage_sensor_test_voltage_divider_with_zero_resistors);
    RUN_TEST(test_voltage_sensor_test_multiple_reads_update_value);
    RUN_TEST(test_voltage_sensor_test_sensor_inherits_from_sensor_base);
    RUN_TEST(test_voltage_sensor_test_default_update_rate);
    RUN_TEST(test_voltage_sensor_test_voltage_sensor_is_healthy_after_begin);
    RUN_TEST(test_voltage_sensor_test_custom_reference_voltage);
    RUN_TEST(test_voltage_sensor_test_different_pins);
    test_voltage_sensor_tearDown();

    // Tests from test_sensor
    test_sensor_setUp();
    RUN_TEST(test_sensor_test_constructor);
    test_sensor_tearDown();

    // Tests from test_sensor_manager
    test_sensor_manager_setUp();
    RUN_TEST(test_sensor_manager_test_sensor_manager_empty_not_ok);
    RUN_TEST(test_sensor_manager_test_sensor_manager_set_get_primary_accel);
    RUN_TEST(test_sensor_manager_test_sensor_manager_set_get_primary_gyro);
    RUN_TEST(test_sensor_manager_test_sensor_manager_set_get_primary_mag);
    RUN_TEST(test_sensor_manager_test_sensor_manager_set_get_primary_baro);
    RUN_TEST(test_sensor_manager_test_sensor_manager_set_get_primary_gps);
    RUN_TEST(test_sensor_manager_test_sensor_manager_is_ok_with_accel_only);
    RUN_TEST(test_sensor_manager_test_sensor_manager_is_ok_with_all_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_not_ok_accel_not_initialized);
    RUN_TEST(test_sensor_manager_test_sensor_manager_begin_all_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_begin_partial_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_add_misc_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_add_null_misc_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_add_too_many_misc_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_update_all_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_accel_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_accel);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_gyro_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_gyro);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_mag_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_mag);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_pressure_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_pressure);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_temp_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_temp);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_gps_pos_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_gps_pos);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_heading_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_heading);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_time_of_day_no_sensor);
    RUN_TEST(test_sensor_manager_test_sensor_manager_get_time_of_day);
    RUN_TEST(test_sensor_manager_test_sensor_manager_full_integration);
    RUN_TEST(test_sensor_manager_test_sensor_manager_update_with_misc_sensors);
    RUN_TEST(test_sensor_manager_test_sensor_manager_update_handles_nulls);
    test_sensor_manager_tearDown();

    UNITY_END();
    return 0;
}
