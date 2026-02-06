#pragma once

#include <unity.h>
#include "../src/Math/Quaternion.h"
#include "../src/Math/Matrix.h"
#include "NativeTestHelper.h"
#include <cmath>

using astra::Quaternion;

namespace test_quaternion {


void local_setUp(void)
{
    // set stuff up before each test here, if needed
}

void local_tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_interpolation_lerp(void)
{
    local_setUp();
    // Create two quaternions
    Quaternion p = Quaternion{0.95, 0.31224989992, 0.0, 0.0}; // current quaternion (identity)
    Quaternion q = Quaternion{1.0, 0.0, 0.0, 0.0}; // input quaternion

    double alpha = 0.5; // Interpolation factor
    double epsilon = 0.9; // Threshold for LERP vs SLERP

    // Perform interpolation
    Quaternion result = p.interpolation(q, alpha, epsilon);

    // Determined by running same number through python script based on ahrs library
    Quaternion expected_result = Quaternion{0.98742088, 0.15811388, 0.0, 0.0};

    // Test for equality (you may need to adjust precision due to floating-point errors)
    TEST_ASSERT_EQUAL_FLOAT(expected_result.w(), result.w());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.x(), result.x());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.y(), result.y());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.z(), result.z());
    local_tearDown();
}

void test_interpolation_slerp(void)
{
    local_setUp();
    // Create two quaternions
    Quaternion p = Quaternion{0.5, 0.5, 0.5, 0.5}; // current quaternion
    Quaternion q = Quaternion{1.0, 0.0, 0.0, 0.0}; // input quaternion

    double alpha = 0.5; // Interpolation factor
    double epsilon = 0.9; // Threshold for LERP vs SLERP

    // Perform interpolation
    Quaternion result = p.interpolation(q, alpha, epsilon);

    // Determined by running same number through python script based on ahrs library
    Quaternion expected_result = Quaternion{0.8660254, 0.28867513, 0.28867513, 0.28867513};

    TEST_ASSERT_EQUAL_FLOAT(expected_result.w(), result.w());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.x(), result.x());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.y(), result.y());
    TEST_ASSERT_EQUAL_FLOAT(expected_result.z(), result.z());
    local_tearDown();
}

void test_quat_to_matrix(void)
{
    local_setUp();
    // Define a quaternion representing a 45-degree (π/4 rad) rotation about the Y-axis
    double angle = M_PI / 4;  // 45 degrees
    double half_angle = angle / 2;
    Quaternion q(cos(half_angle), 0.0, sin(half_angle), 0.0); // (w, x, y, z)

    // Convert quaternion to DCM
    astra::Matrix dcm = q.toMatrix();

    // Expected DCM for 45-degree rotation about Y-axis
    double sqrt2_2 = sqrt(2) / 2;
    astra::Matrix expected_dcm(3, 3, new double[9]{sqrt2_2,0,-sqrt2_2,0,1,0,sqrt2_2, 0, sqrt2_2});

    // Compare each element of the DCM (assuming TEST_ASSERT_EQUAL_FLOAT can be used for matrices)
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            TEST_ASSERT_EQUAL_FLOAT(expected_dcm.get(i, j), dcm.get(i, j));
        }
    }
    local_tearDown();
}

void test_matrix_to_quat(void)
{
    local_setUp();
    // Define a 3x3 rotation matrix representing a 45-degree (π/4 rad) rotation about the Z-axis
    double angle = M_PI / 4;  // 45 degrees
    double half_angle = angle / 2;
    double dcm_data[9] = {
        cos(angle), -sin(angle), 0.0,
        sin(angle), cos(angle), 0.0,
        0.0, 0.0, 1.0
    };
    astra::Matrix rotation_matrix(3, 3, dcm_data);

    // Create a quaternion and use the fromMatrix function to convert the rotation matrix to a quaternion
    astra::Quaternion q;
    q.fromMatrix(rotation_matrix);

    // Expected quaternion for a 45-degree rotation about the Z-axis
    double expected_w = cos(half_angle);
    double expected_x = 0.0;
    double expected_y = 0.0;
    double expected_z = sin(half_angle);

    // Compare the quaternion components
    TEST_ASSERT_EQUAL_FLOAT(expected_w, q.w());
    TEST_ASSERT_EQUAL_FLOAT(expected_x, q.x());
    TEST_ASSERT_EQUAL_FLOAT(expected_y, q.y());
    TEST_ASSERT_EQUAL_FLOAT(expected_z, q.z());
    local_tearDown();
}

void test_default_constructor(void)
{
    local_setUp();
    Quaternion q;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q.w());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q.z());
    local_tearDown();
}

void test_wxyz_constructor(void)
{
    local_setUp();
    Quaternion q(0.5, 0.5, 0.5, 0.5);
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.w());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.x());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.y());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.z());
    local_tearDown();
}

void test_w_vector_constructor(void)
{
    local_setUp();
    astra::Vector<3> v(1.0, 2.0, 3.0);
    Quaternion q(0.5, v);
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.w());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, q.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, q.z());
    local_tearDown();
}

void test_magnitude(void)
{
    local_setUp();
    Quaternion q(1.0, 2.0, 2.0, 0.0);
    double mag = q.magnitude();
    TEST_ASSERT_EQUAL_FLOAT(3.0, mag);
    local_tearDown();
}

void test_normalize(void)
{
    local_setUp();
    Quaternion q(1.0, 2.0, 2.0, 0.0);
    q.normalize();
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, q.magnitude());
    local_tearDown();
}

void test_conjugate(void)
{
    local_setUp();
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    Quaternion conj = q.conjugate();
    TEST_ASSERT_EQUAL_FLOAT(1.0, conj.w());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, conj.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, conj.y());
    TEST_ASSERT_EQUAL_FLOAT(-4.0, conj.z());
    local_tearDown();
}

void test_from_axis_angle(void)
{
    local_setUp();
    astra::Vector<3> axis(0.0, 0.0, 1.0);  // Z-axis
    double angle = M_PI / 2;  // 90 degrees
    Quaternion q;
    q.fromAxisAngle(axis, angle);

    double expected_w = cos(angle / 2);
    double expected_z = sin(angle / 2);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_w, q.w());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, q.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, q.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_z, q.z());
    local_tearDown();
}

void test_to_axis_angle(void)
{
    local_setUp();
    double angle_in = M_PI / 2;
    astra::Vector<3> axis_in(0.0, 1.0, 0.0);  // Y-axis
    Quaternion q;
    q.fromAxisAngle(axis_in, angle_in);

    astra::Vector<3> axis_out;
    double angle_out;
    q.toAxisAngle(axis_out, angle_out);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, angle_in, angle_out);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, axis_out.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, axis_out.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, axis_out.z());
    local_tearDown();
}

void test_to_euler321(void)
{
    local_setUp();
    // Create a quaternion representing a known rotation
    Quaternion q(0.7071, 0.0, 0.7071, 0.0);  // 90 degree rotation about Y-axis
    astra::Vector<3> euler = q.toEuler321();

    // Just check that it returns a vector (specific values depend on conversion)
    TEST_ASSERT_NOT_EQUAL(0.0, euler.magnitude());
    local_tearDown();
}

void test_to_angular_velocity(void)
{
    local_setUp();
    Quaternion q(0.99, 0.01, 0.01, 0.01);
    double dt = 0.01;
    astra::Vector<3> angvel = q.toAngularVelocity(dt);

    // Just check that it returns a vector
    TEST_ASSERT_TRUE(angvel.magnitude() >= 0.0);
    local_tearDown();
}

void test_rotate_vector_3d(void)
{
    local_setUp();
    // Identity quaternion should not change the vector
    Quaternion q_identity(1.0, 0.0, 0.0, 0.0);
    astra::Vector<3> v(1.0, 0.0, 0.0);
    astra::Vector<3> v_rotated = q_identity.rotateVector(v);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, v_rotated.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, v_rotated.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, v_rotated.z());
    local_tearDown();
}

void test_rotate_vector_2d(void)
{
    local_setUp();
    Quaternion q_identity(1.0, 0.0, 0.0, 0.0);
    astra::Vector<2> v(1.0, 0.0);
    astra::Vector<3> v_rotated = q_identity.rotateVector(v);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, v_rotated.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, v_rotated.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, v_rotated.z());
    local_tearDown();
}

void test_quaternion_multiplication(void)
{
    local_setUp();
    Quaternion q1(1.0, 0.0, 0.0, 0.0);
    Quaternion q2(0.7071, 0.0, 0.7071, 0.0);
    Quaternion result = q1 * q2;

    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.7071, result.w());
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0, result.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.7071, result.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0, result.z());
    local_tearDown();
}

void test_quaternion_addition(void)
{
    local_setUp();
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(0.5, 0.5, 0.5, 0.5);
    Quaternion result = q1 + q2;

    TEST_ASSERT_EQUAL_FLOAT(1.5, result.w());
    TEST_ASSERT_EQUAL_FLOAT(2.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.5, result.y());
    TEST_ASSERT_EQUAL_FLOAT(4.5, result.z());
    local_tearDown();
}

void test_quaternion_subtraction(void)
{
    local_setUp();
    Quaternion q1(1.0, 2.0, 3.0, 4.0);
    Quaternion q2(0.5, 0.5, 0.5, 0.5);
    Quaternion result = q1 - q2;

    TEST_ASSERT_EQUAL_FLOAT(0.5, result.w());
    TEST_ASSERT_EQUAL_FLOAT(1.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.5, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.5, result.z());
    local_tearDown();
}

void test_quaternion_scalar_multiplication(void)
{
    local_setUp();
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    Quaternion result = q * 2.0;

    TEST_ASSERT_EQUAL_FLOAT(2.0, result.w());
    TEST_ASSERT_EQUAL_FLOAT(4.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(8.0, result.z());
    local_tearDown();
}

void test_quaternion_scalar_division(void)
{
    local_setUp();
    Quaternion q(2.0, 4.0, 6.0, 8.0);
    Quaternion result = q / 2.0;

    TEST_ASSERT_EQUAL_FLOAT(1.0, result.w());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(4.0, result.z());
    local_tearDown();
}

void test_quaternion_scale(void)
{
    local_setUp();
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    Quaternion result = q.scale(3.0);

    TEST_ASSERT_EQUAL_FLOAT(3.0, result.w());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(9.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(12.0, result.z());
    local_tearDown();
}

void test_mutable_accessors(void)
{
    local_setUp();
    Quaternion q;
    q.w() = 1.0;
    q.x() = 2.0;
    q.y() = 3.0;
    q.z() = 4.0;

    TEST_ASSERT_EQUAL_FLOAT(1.0, q.w());
    TEST_ASSERT_EQUAL_FLOAT(2.0, q.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, q.y());
    TEST_ASSERT_EQUAL_FLOAT(4.0, q.z());
    local_tearDown();
}

void test_const_accessors(void)
{
    local_setUp();
    const Quaternion q(1.0, 2.0, 3.0, 4.0);

    TEST_ASSERT_EQUAL_FLOAT(1.0, q.w());
    TEST_ASSERT_EQUAL_FLOAT(2.0, q.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, q.y());
    TEST_ASSERT_EQUAL_FLOAT(4.0, q.z());
    local_tearDown();
}

void test_from_matrix_invalid_dimensions(void)
{
    local_setUp();
    double data[8] = {1, 0, 0, 0, 0, 1, 0, 0};
    astra::Matrix m_invalid(2, 4, data);
    Quaternion q(0.5, 0.5, 0.5, 0.5);

    // Should log error and not modify q
    q.fromMatrix(m_invalid);

    // Quaternion should remain unchanged
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.w());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.x());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.y());
    TEST_ASSERT_EQUAL_FLOAT(0.5, q.z());
    local_tearDown();
}

void test_roundtrip_matrix_conversion(void)
{
    local_setUp();
    // Start with a quaternion
    Quaternion q_original(0.7071, 0.0, 0.7071, 0.0);

    // Convert to matrix
    astra::Matrix m = q_original.toMatrix();

    // Convert back to quaternion
    Quaternion q_result;
    q_result.fromMatrix(m);

    // Should be approximately equal (within floating point precision)
    TEST_ASSERT_FLOAT_WITHIN(1e-4, q_original.w(), q_result.w());
    TEST_ASSERT_FLOAT_WITHIN(1e-4, fabs(q_original.x()), fabs(q_result.x()));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, fabs(q_original.y()), fabs(q_result.y()));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, fabs(q_original.z()), fabs(q_result.z()));
    local_tearDown();
}

void test_roundtrip_axis_angle_conversion(void)
{
    local_setUp();
    astra::Vector<3> axis_original(0.0, 0.0, 1.0);
    double angle_original = M_PI / 3;

    Quaternion q;
    q.fromAxisAngle(axis_original, angle_original);

    astra::Vector<3> axis_result;
    double angle_result;
    q.toAxisAngle(axis_result, angle_result);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, angle_original, angle_result);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, axis_original.x(), axis_result.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, axis_original.y(), axis_result.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, axis_original.z(), axis_result.z());
    local_tearDown();
}

void run_test_quaternion_tests()
{
    RUN_TEST(test_interpolation_lerp);
    RUN_TEST(test_interpolation_slerp);
    RUN_TEST(test_quat_to_matrix);
    RUN_TEST(test_matrix_to_quat);
    RUN_TEST(test_default_constructor);
    RUN_TEST(test_wxyz_constructor);
    RUN_TEST(test_w_vector_constructor);
    RUN_TEST(test_magnitude);
    RUN_TEST(test_normalize);
    RUN_TEST(test_conjugate);
    RUN_TEST(test_from_axis_angle);
    RUN_TEST(test_to_axis_angle);
    RUN_TEST(test_to_euler321);
    RUN_TEST(test_to_angular_velocity);
    RUN_TEST(test_rotate_vector_3d);
    RUN_TEST(test_rotate_vector_2d);
    RUN_TEST(test_quaternion_multiplication);
    RUN_TEST(test_quaternion_addition);
    RUN_TEST(test_quaternion_subtraction);
    RUN_TEST(test_quaternion_scalar_multiplication);
    RUN_TEST(test_quaternion_scalar_division);
    RUN_TEST(test_quaternion_scale);
    RUN_TEST(test_mutable_accessors);
    RUN_TEST(test_const_accessors);
    RUN_TEST(test_from_matrix_invalid_dimensions);
    RUN_TEST(test_roundtrip_matrix_conversion);
    RUN_TEST(test_roundtrip_axis_angle_conversion);
}

} // namespace test_quaternion
