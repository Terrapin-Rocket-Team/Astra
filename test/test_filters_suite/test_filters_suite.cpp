#include <unity.h>
#include "NativeTestHelper.h"

#include "test_mahony/test_mahony.inc"
#include "test_kalman_filter/test_kalman_filter.inc"
#include "test_mounting_transform/test_mounting_transform.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_mahony
    test_mahony_setUp();
    RUN_TEST(test_mahony_test_always_ready);
    RUN_TEST(test_mahony_test_initial_quaternion);
    RUN_TEST(test_mahony_test_constructor_with_gains);
    RUN_TEST(test_mahony_test_reset);
    RUN_TEST(test_mahony_test_set_quaternion);
    RUN_TEST(test_mahony_test_static_orientation_upright);
    RUN_TEST(test_mahony_test_static_orientation_tilted);
    RUN_TEST(test_mahony_test_gyro_only_no_drift);
    RUN_TEST(test_mahony_test_gyro_rotation_z_axis_90deg);
    RUN_TEST(test_mahony_test_gyro_rotation_x_axis_45deg);
    RUN_TEST(test_mahony_test_continuous_rotation);
    RUN_TEST(test_mahony_test_full_rotation_360deg);
    RUN_TEST(test_mahony_test_accel_correction_from_tilt);
    RUN_TEST(test_mahony_test_accel_correction_ignores_linear_accel);
    RUN_TEST(test_mahony_test_earth_frame_acceleration_static);
    RUN_TEST(test_mahony_test_earth_frame_acceleration_upward);
    RUN_TEST(test_mahony_test_earth_frame_acceleration_lateral);
    RUN_TEST(test_mahony_test_mag_calibration_status);
    RUN_TEST(test_mahony_test_mag_calibration_sufficient_samples);
    RUN_TEST(test_mahony_test_mag_calibration_persists_after_reset);
    RUN_TEST(test_mahony_test_9dof_update);
    RUN_TEST(test_mahony_test_zero_dt);
    RUN_TEST(test_mahony_test_large_dt);
    RUN_TEST(test_mahony_test_zero_acceleration);
    RUN_TEST(test_mahony_test_high_gyro_rate);
    RUN_TEST(test_mahony_test_quaternion_normalization_maintained);
    test_mahony_tearDown();

    // Tests from test_kalman_filter
    test_kalman_filter_setUp();
    RUN_TEST(test_kalman_filter_test_default_kalman_constructor);
    RUN_TEST(test_kalman_filter_test_default_kalman_constructor_with_params);
    RUN_TEST(test_kalman_filter_test_default_kalman_initialize);
    RUN_TEST(test_kalman_filter_test_default_kalman_state_transition_matrix);
    RUN_TEST(test_kalman_filter_test_default_kalman_control_matrix);
    RUN_TEST(test_kalman_filter_test_default_kalman_measurement_matrix);
    RUN_TEST(test_kalman_filter_test_default_kalman_measurement_noise_matrix);
    RUN_TEST(test_kalman_filter_test_default_kalman_process_noise_matrix);
    RUN_TEST(test_kalman_filter_test_default_kalman_predict_zero_control);
    RUN_TEST(test_kalman_filter_test_default_kalman_predict_with_acceleration);
    RUN_TEST(test_kalman_filter_test_default_kalman_update_gps_only);
    RUN_TEST(test_kalman_filter_test_default_kalman_update_baro_only);
    RUN_TEST(test_kalman_filter_test_default_kalman_update_gps_baro_combined);
    RUN_TEST(test_kalman_filter_test_default_kalman_predict_update_cycle);
    RUN_TEST(test_kalman_filter_test_default_kalman_free_fall_scenario);
    RUN_TEST(test_kalman_filter_test_kalman_zero_dt);
    RUN_TEST(test_kalman_filter_test_kalman_large_dt);
    RUN_TEST(test_kalman_filter_test_kalman_high_acceleration);
    RUN_TEST(test_kalman_filter_test_kalman_repeated_predictions);
    RUN_TEST(test_kalman_filter_test_kalman_repeated_updates);
    RUN_TEST(test_kalman_filter_test_kalman_alternating_predict_update);
    RUN_TEST(test_kalman_filter_test_kalman_measurement_noise_effect);
    RUN_TEST(test_kalman_filter_test_kalman_matrix_dimensions_consistency);
    RUN_TEST(test_kalman_filter_test_kalman_state_vector_dimension);
    test_kalman_filter_tearDown();

    // Tests from test_mounting_transform
    test_mounting_transform_setUp();
    RUN_TEST(test_mounting_transform_test_default_constructor);
    RUN_TEST(test_mounting_transform_test_constructor_with_preset);
    RUN_TEST(test_mounting_transform_test_constructor_with_axis_signs);
    RUN_TEST(test_mounting_transform_test_constructor_with_custom_matrix);
    RUN_TEST(test_mounting_transform_test_identity_transform);
    RUN_TEST(test_mounting_transform_test_flip_yz_transform);
    RUN_TEST(test_mounting_transform_test_flip_xz_transform);
    RUN_TEST(test_mounting_transform_test_flip_xy_transform);
    RUN_TEST(test_mounting_transform_test_rotate_90_x_transform);
    RUN_TEST(test_mounting_transform_test_rotate_neg90_x_transform);
    RUN_TEST(test_mounting_transform_test_rotate_90_y_transform);
    RUN_TEST(test_mounting_transform_test_rotate_neg90_y_transform);
    RUN_TEST(test_mounting_transform_test_rotate_90_z_transform);
    RUN_TEST(test_mounting_transform_test_rotate_neg90_z_transform);
    RUN_TEST(test_mounting_transform_test_setOrientation);
    RUN_TEST(test_mounting_transform_test_setOrientation_to_identity);
    RUN_TEST(test_mounting_transform_test_setMatrix);
    RUN_TEST(test_mounting_transform_test_isIdentity_for_identity_orientation);
    RUN_TEST(test_mounting_transform_test_isIdentity_for_custom_identity_matrix);
    RUN_TEST(test_mounting_transform_test_isIdentity_for_non_identity);
    RUN_TEST(test_mounting_transform_test_getMatrix);
    RUN_TEST(test_mounting_transform_test_axis_signs_all_positive);
    RUN_TEST(test_mounting_transform_test_axis_signs_all_negative);
    RUN_TEST(test_mounting_transform_test_axis_signs_mixed);
    RUN_TEST(test_mounting_transform_test_custom_rotation_90_degrees);
    RUN_TEST(test_mounting_transform_test_custom_arbitrary_rotation);
    RUN_TEST(test_mounting_transform_test_transform_zero_vector);
    RUN_TEST(test_mounting_transform_test_transform_unit_vectors);
    RUN_TEST(test_mounting_transform_test_multiple_transforms);
    RUN_TEST(test_mounting_transform_test_transform_preserves_magnitude);
    RUN_TEST(test_mounting_transform_test_double_rotation_equals_flip);
    test_mounting_transform_tearDown();

    UNITY_END();
    return 0;
}
