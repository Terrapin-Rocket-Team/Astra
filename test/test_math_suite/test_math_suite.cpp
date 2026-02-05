#include <unity.h>
#include "NativeTestHelper.h"

#include "test_matrix/test_matrix.inc"
#include "test_vector/test_vector.inc"
#include "test_quaternion/test_quaternion.inc"

void setUp(void) {
    // Global suite setup
}

void tearDown(void) {
    // Global suite teardown
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Tests from test_matrix
    test_matrix_setUp();
    RUN_TEST(test_matrix_test_default_constructor);
    RUN_TEST(test_matrix_test_allocating_constructor);
    RUN_TEST(test_matrix_test_array_constructor);
    RUN_TEST(test_matrix_test_copy_constructor);
    RUN_TEST(test_matrix_test_assignment_operator);
    RUN_TEST(test_matrix_test_move_constructor);
    RUN_TEST(test_matrix_test_move_assignment);
    RUN_TEST(test_matrix_test_self_assignment);
    RUN_TEST(test_matrix_test_get_method);
    RUN_TEST(test_matrix_test_parenthesis_operator_mutable);
    RUN_TEST(test_matrix_test_parenthesis_operator_const);
    RUN_TEST(test_matrix_test_multiply_operator_matrix);
    RUN_TEST(test_matrix_test_multiply_operator_scalar);
    RUN_TEST(test_matrix_test_add_operator_matrix);
    RUN_TEST(test_matrix_test_subtract_operator_matrix);
    RUN_TEST(test_matrix_test_multiply_method);
    RUN_TEST(test_matrix_test_multiply_scalar_method);
    RUN_TEST(test_matrix_test_add_method);
    RUN_TEST(test_matrix_test_subtract_method);
    RUN_TEST(test_matrix_test_transpose);
    RUN_TEST(test_matrix_test_transpose_method);
    RUN_TEST(test_matrix_test_inverse_small);
    RUN_TEST(test_matrix_test_inverse_large);
    RUN_TEST(test_matrix_test_inverse_method);
    RUN_TEST(test_matrix_test_trace);
    RUN_TEST(test_matrix_test_identity_matrix);
    RUN_TEST(test_matrix_test_matrix_chain_operations);
    RUN_TEST(test_matrix_test_matrix_dimension_mismatch_multiply);
    RUN_TEST(test_matrix_test_matrix_dimension_mismatch_add);
    RUN_TEST(test_matrix_test_matrix_dimension_mismatch_subtract);
    RUN_TEST(test_matrix_test_inverse_non_square);
    RUN_TEST(test_matrix_test_trace_non_square);
    RUN_TEST(test_matrix_test_empty_matrix_operations);
    test_matrix_tearDown();

    // Tests from test_vector
    test_vector_setUp();
    RUN_TEST(test_vector_test_default_constructor);
    RUN_TEST(test_vector_test_single_parameter_constructor);
    RUN_TEST(test_vector_test_two_parameter_constructor);
    RUN_TEST(test_vector_test_three_parameter_constructor);
    RUN_TEST(test_vector_test_four_parameter_constructor);
    RUN_TEST(test_vector_test_copy_constructor);
    RUN_TEST(test_vector_test_bracket_operator_read);
    RUN_TEST(test_vector_test_bracket_operator_write);
    RUN_TEST(test_vector_test_parenthesis_operator_read);
    RUN_TEST(test_vector_test_parenthesis_operator_write);
    RUN_TEST(test_vector_test_xyz_accessors_read);
    RUN_TEST(test_vector_test_xyz_accessors_write);
    RUN_TEST(test_vector_test_n_method);
    RUN_TEST(test_vector_test_pointer_accessors);
    RUN_TEST(test_vector_test_pointer_accessors_write);
    RUN_TEST(test_vector_test_magnitude);
    RUN_TEST(test_vector_test_magnitude_zero_vector);
    RUN_TEST(test_vector_test_normalize);
    RUN_TEST(test_vector_test_normalize_zero_vector);
    RUN_TEST(test_vector_test_dot_product);
    RUN_TEST(test_vector_test_dot_product_orthogonal);
    RUN_TEST(test_vector_test_cross_product);
    RUN_TEST(test_vector_test_cross_product_general);
    RUN_TEST(test_vector_test_scale);
    RUN_TEST(test_vector_test_invert);
    RUN_TEST(test_vector_test_assignment_operator);
    RUN_TEST(test_vector_test_plus_equals_operator);
    RUN_TEST(test_vector_test_addition_operator);
    RUN_TEST(test_vector_test_subtraction_operator);
    RUN_TEST(test_vector_test_scalar_multiplication_operator);
    RUN_TEST(test_vector_test_scalar_division_operator);
    RUN_TEST(test_vector_test_to_degrees);
    RUN_TEST(test_vector_test_to_radians);
    RUN_TEST(test_vector_test_vector_2d);
    RUN_TEST(test_vector_test_vector_4d);
    RUN_TEST(test_vector_test_vector_5d);
    RUN_TEST(test_vector_test_very_large_magnitude);
    RUN_TEST(test_vector_test_very_small_magnitude);
    RUN_TEST(test_vector_test_negative_values);
    RUN_TEST(test_vector_test_chain_operations);
    test_vector_tearDown();

    // Tests from test_quaternion
    test_quaternion_setUp();
    RUN_TEST(test_quaternion_test_default_constructor);
    RUN_TEST(test_quaternion_test_wxyz_constructor);
    RUN_TEST(test_quaternion_test_w_vector_constructor);
    RUN_TEST(test_quaternion_test_mutable_accessors);
    RUN_TEST(test_quaternion_test_const_accessors);
    RUN_TEST(test_quaternion_test_magnitude);
    RUN_TEST(test_quaternion_test_normalize);
    RUN_TEST(test_quaternion_test_conjugate);
    RUN_TEST(test_quaternion_test_from_axis_angle);
    RUN_TEST(test_quaternion_test_to_axis_angle);
    RUN_TEST(test_quaternion_test_roundtrip_axis_angle_conversion);
    RUN_TEST(test_quaternion_test_quat_to_matrix);
    RUN_TEST(test_quaternion_test_matrix_to_quat);
    RUN_TEST(test_quaternion_test_from_matrix_invalid_dimensions);
    RUN_TEST(test_quaternion_test_roundtrip_matrix_conversion);
    RUN_TEST(test_quaternion_test_to_euler321);
    RUN_TEST(test_quaternion_test_rotate_vector_3d);
    RUN_TEST(test_quaternion_test_rotate_vector_2d);
    RUN_TEST(test_quaternion_test_to_angular_velocity);
    RUN_TEST(test_quaternion_test_quaternion_multiplication);
    RUN_TEST(test_quaternion_test_quaternion_addition);
    RUN_TEST(test_quaternion_test_quaternion_subtraction);
    RUN_TEST(test_quaternion_test_quaternion_scalar_multiplication);
    RUN_TEST(test_quaternion_test_quaternion_scalar_division);
    RUN_TEST(test_quaternion_test_quaternion_scale);
    RUN_TEST(test_quaternion_test_interpolation_lerp);
    RUN_TEST(test_quaternion_test_interpolation_slerp);
    test_quaternion_tearDown();

    UNITY_END();
    return 0;
}
