#include <unity.h>
#include "../src/Math/Vector.h"
#include "NativeTestHelper.h"
#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using astra::Vector;

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    // set stuff up before each test here, if needed
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
}
// ---

// Constructor Tests

void test_default_constructor(void)
{
    Vector<3> v;
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.z());
}

void test_single_parameter_constructor(void)
{
    Vector<3> v(5.0);
    TEST_ASSERT_EQUAL_FLOAT(5.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.z());
}

void test_two_parameter_constructor(void)
{
    Vector<3> v(1.0, 2.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.z());
}

void test_three_parameter_constructor(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, v.z());
}

void test_four_parameter_constructor(void)
{
    Vector<4> v(1.0, 2.0, 3.0, 4.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0, v[2]);
    TEST_ASSERT_EQUAL_FLOAT(4.0, v[3]);
}

void test_copy_constructor(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2(v1);

    TEST_ASSERT_EQUAL_FLOAT(1.0, v2.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, v2.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, v2.z());
}

// Accessor Tests

void test_bracket_operator_read(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0, v[2]);
}

void test_bracket_operator_write(void)
{
    Vector<3> v;
    v[0] = 5.0;
    v[1] = 6.0;
    v[2] = 7.0;

    TEST_ASSERT_EQUAL_FLOAT(5.0, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(6.0, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(7.0, v[2]);
}

void test_parenthesis_operator_read(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v(0));
    TEST_ASSERT_EQUAL_FLOAT(2.0, v(1));
    TEST_ASSERT_EQUAL_FLOAT(3.0, v(2));
}

void test_parenthesis_operator_write(void)
{
    Vector<3> v;
    v(0) = 5.0;
    v(1) = 6.0;
    v(2) = 7.0;

    TEST_ASSERT_EQUAL_FLOAT(5.0, v(0));
    TEST_ASSERT_EQUAL_FLOAT(6.0, v(1));
    TEST_ASSERT_EQUAL_FLOAT(7.0, v(2));
}

void test_xyz_accessors_read(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, v.z());
}

void test_xyz_accessors_write(void)
{
    Vector<3> v;
    v.x() = 5.0;
    v.y() = 6.0;
    v.z() = 7.0;

    TEST_ASSERT_EQUAL_FLOAT(5.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(7.0, v.z());
}

void test_n_method(void)
{
    Vector<3> v3;
    Vector<4> v4;
    Vector<5> v5;

    TEST_ASSERT_EQUAL(3, v3.n());
    TEST_ASSERT_EQUAL(4, v4.n());
    TEST_ASSERT_EQUAL(5, v5.n());
}

// Math Operation Tests

void test_magnitude(void)
{
    Vector<3> v(3.0, 4.0, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(5.0, v.magnitude());
}

void test_magnitude_zero_vector(void)
{
    Vector<3> v(0.0, 0.0, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.magnitude());
}

void test_normalize(void)
{
    Vector<3> v(3.0, 4.0, 0.0);
    v.normalize();

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, v.magnitude());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.6, v.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.8, v.y());
}

void test_normalize_zero_vector(void)
{
    Vector<3> v(0.0, 0.0, 0.0);
    v.normalize();

    // Should remain zero (no division by zero)
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, v.z());
}

void test_dot_product(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2(4.0, 5.0, 6.0);

    double dot = v1.dot(v2);
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    TEST_ASSERT_EQUAL_FLOAT(32.0, dot);
}

void test_dot_product_orthogonal(void)
{
    Vector<3> v1(1.0, 0.0, 0.0);
    Vector<3> v2(0.0, 1.0, 0.0);

    double dot = v1.dot(v2);
    TEST_ASSERT_EQUAL_FLOAT(0.0, dot);
}

void test_cross_product(void)
{
    Vector<3> v1(1.0, 0.0, 0.0);
    Vector<3> v2(0.0, 1.0, 0.0);

    Vector<3> result = v1.cross(v2);

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.z());
}

void test_cross_product_general(void)
{
    Vector<3> v1(2.0, 3.0, 4.0);
    Vector<3> v2(5.0, 6.0, 7.0);

    Vector<3> result = v1.cross(v2);

    // Expected: (3*7 - 4*6, 4*5 - 2*7, 2*6 - 3*5) = (-3, 6, -3)
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());
}

void test_scale(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    Vector<3> result = v.scale(2.0);

    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(4.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.z());
}

void test_invert(void)
{
    Vector<3> v(1.0, -2.0, 3.0);
    Vector<3> result = v.invert();

    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());
}

// Operator Tests

void test_assignment_operator(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2;
    v2 = v1;

    TEST_ASSERT_EQUAL_FLOAT(1.0, v2.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, v2.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, v2.z());
}

void test_plus_equals_operator(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2(4.0, 5.0, 6.0);

    v1 += v2;

    TEST_ASSERT_EQUAL_FLOAT(5.0, v1.x());
    TEST_ASSERT_EQUAL_FLOAT(7.0, v1.y());
    TEST_ASSERT_EQUAL_FLOAT(9.0, v1.z());
}

void test_addition_operator(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2(4.0, 5.0, 6.0);

    Vector<3> result = v1 + v2;

    TEST_ASSERT_EQUAL_FLOAT(5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(7.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(9.0, result.z());
}

void test_subtraction_operator(void)
{
    Vector<3> v1(5.0, 7.0, 9.0);
    Vector<3> v2(1.0, 2.0, 3.0);

    Vector<3> result = v1 - v2;

    TEST_ASSERT_EQUAL_FLOAT(4.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(5.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.z());
}

void test_scalar_multiplication_operator(void)
{
    Vector<3> v(1.0, 2.0, 3.0);
    Vector<3> result = v * 3.0;

    TEST_ASSERT_EQUAL_FLOAT(3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(9.0, result.z());
}

void test_scalar_division_operator(void)
{
    Vector<3> v(6.0, 9.0, 12.0);
    Vector<3> result = v / 3.0;

    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(4.0, result.z());
}

// Angle Conversion Tests

void test_to_degrees(void)
{
    Vector<3> v(M_PI, M_PI / 2, M_PI / 4);
    v.toDegrees();

    TEST_ASSERT_FLOAT_WITHIN(1e-3, 180.0, v.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 90.0, v.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 45.0, v.z());
}

void test_to_radians(void)
{
    Vector<3> v(180.0, 90.0, 45.0);
    v.toRadians();

    TEST_ASSERT_FLOAT_WITHIN(1e-6, M_PI, v.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, M_PI / 2, v.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, M_PI / 4, v.z());
}

// Different Dimension Tests

void test_vector_2d(void)
{
    Vector<2> v(3.0, 4.0);
    TEST_ASSERT_EQUAL_FLOAT(3.0, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(4.0, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(5.0, v.magnitude());
}

void test_vector_4d(void)
{
    Vector<4> v(1.0, 2.0, 3.0, 4.0);
    TEST_ASSERT_EQUAL_FLOAT(1.0, v[0]);
    TEST_ASSERT_EQUAL_FLOAT(2.0, v[1]);
    TEST_ASSERT_EQUAL_FLOAT(3.0, v[2]);
    TEST_ASSERT_EQUAL_FLOAT(4.0, v[3]);

    double expected_mag = sqrt(1.0 + 4.0 + 9.0 + 16.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_mag, v.magnitude());
}

void test_vector_5d(void)
{
    Vector<5> v;
    v[0] = 1.0;
    v[1] = 2.0;
    v[2] = 3.0;
    v[3] = 4.0;
    v[4] = 5.0;

    double expected_mag = sqrt(1.0 + 4.0 + 9.0 + 16.0 + 25.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_mag, v.magnitude());
}

// Pointer Access Tests (xp, yp, zp)

void test_pointer_accessors(void)
{
    Vector<3> v(1.0, 2.0, 3.0);

    TEST_ASSERT_EQUAL_FLOAT(1.0, *(v.xp));
    TEST_ASSERT_EQUAL_FLOAT(2.0, *(v.yp));
    TEST_ASSERT_EQUAL_FLOAT(3.0, *(v.zp));
}

void test_pointer_accessors_write(void)
{
    Vector<3> v;
    *(v.xp) = 5.0;
    *(v.yp) = 6.0;
    *(v.zp) = 7.0;

    TEST_ASSERT_EQUAL_FLOAT(5.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(7.0, v.z());
}

// Edge Cases

void test_very_large_magnitude(void)
{
    Vector<3> v(1e100, 1e100, 1e100);
    double mag = v.magnitude();
    TEST_ASSERT_TRUE(mag > 1e100);
}

void test_very_small_magnitude(void)
{
    Vector<3> v(1e-100, 1e-100, 1e-100);
    double mag = v.magnitude();
    TEST_ASSERT_TRUE(mag < 1e-99);
}

void test_negative_values(void)
{
    Vector<3> v(-1.0, -2.0, -3.0);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, v.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, v.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, v.z());

    double expected_mag = sqrt(1.0 + 4.0 + 9.0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, expected_mag, v.magnitude());
}

void test_chain_operations(void)
{
    Vector<3> v1(1.0, 2.0, 3.0);
    Vector<3> v2(1.0, 1.0, 1.0);

    Vector<3> result = (v1 + v2) * 2.0;

    TEST_ASSERT_EQUAL_FLOAT(4.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(8.0, result.z());
}

// Main function
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Constructor tests
    RUN_TEST(test_default_constructor);
    RUN_TEST(test_single_parameter_constructor);
    RUN_TEST(test_two_parameter_constructor);
    RUN_TEST(test_three_parameter_constructor);
    RUN_TEST(test_four_parameter_constructor);
    RUN_TEST(test_copy_constructor);

    // Accessor tests
    RUN_TEST(test_bracket_operator_read);
    RUN_TEST(test_bracket_operator_write);
    RUN_TEST(test_parenthesis_operator_read);
    RUN_TEST(test_parenthesis_operator_write);
    RUN_TEST(test_xyz_accessors_read);
    RUN_TEST(test_xyz_accessors_write);
    RUN_TEST(test_n_method);
    RUN_TEST(test_pointer_accessors);
    RUN_TEST(test_pointer_accessors_write);

    // Math operation tests
    RUN_TEST(test_magnitude);
    RUN_TEST(test_magnitude_zero_vector);
    RUN_TEST(test_normalize);
    RUN_TEST(test_normalize_zero_vector);
    RUN_TEST(test_dot_product);
    RUN_TEST(test_dot_product_orthogonal);
    RUN_TEST(test_cross_product);
    RUN_TEST(test_cross_product_general);
    RUN_TEST(test_scale);
    RUN_TEST(test_invert);

    // Operator tests
    RUN_TEST(test_assignment_operator);
    RUN_TEST(test_plus_equals_operator);
    RUN_TEST(test_addition_operator);
    RUN_TEST(test_subtraction_operator);
    RUN_TEST(test_scalar_multiplication_operator);
    RUN_TEST(test_scalar_division_operator);

    // Angle conversion tests
    RUN_TEST(test_to_degrees);
    RUN_TEST(test_to_radians);

    // Different dimension tests
    RUN_TEST(test_vector_2d);
    RUN_TEST(test_vector_4d);
    RUN_TEST(test_vector_5d);

    // Edge case tests
    RUN_TEST(test_very_large_magnitude);
    RUN_TEST(test_very_small_magnitude);
    RUN_TEST(test_negative_values);
    RUN_TEST(test_chain_operations);

    UNITY_END();

    return 0;
}
