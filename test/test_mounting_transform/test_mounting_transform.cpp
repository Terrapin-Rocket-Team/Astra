#include <unity.h>
#include "Sensors/MountingTransform.h"
#include "Math/Vector.h"
#include <cmath>

using namespace astra;

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// setUp and tearDown
void setUp(void) {}
void tearDown(void) {}

// --- Constructor Tests ---

void test_default_constructor(void)
{
    MountingTransform mt;
    TEST_ASSERT_TRUE(mt.isIdentity());
    TEST_ASSERT_EQUAL(MountingOrientation::IDENTITY, mt.getOrientation());
}

void test_constructor_with_preset(void)
{
    MountingTransform mt(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_FALSE(mt.isIdentity());
    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, mt.getOrientation());
}

void test_constructor_with_axis_signs(void)
{
    MountingTransform mt(-1, 1, -1);
    TEST_ASSERT_EQUAL(MountingOrientation::CUSTOM, mt.getOrientation());

    const double *matrix = mt.getMatrix();
    TEST_ASSERT_EQUAL_FLOAT(-1.0, matrix[0]);  // X sign
    TEST_ASSERT_EQUAL_FLOAT(1.0, matrix[4]);   // Y sign
    TEST_ASSERT_EQUAL_FLOAT(-1.0, matrix[8]);  // Z sign
}

void test_constructor_with_custom_matrix(void)
{
    double customMatrix[9] = {
        0, 1, 0,
        -1, 0, 0,
        0, 0, 1
    };

    MountingTransform mt(customMatrix);
    TEST_ASSERT_EQUAL(MountingOrientation::CUSTOM, mt.getOrientation());

    const double *matrix = mt.getMatrix();
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_EQUAL_FLOAT(customMatrix[i], matrix[i]);
    }
}

// --- Transform Tests for Each Preset ---

void test_identity_transform(void)
{
    MountingTransform mt(MountingOrientation::IDENTITY);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

void test_flip_yz_transform(void)
{
    MountingTransform mt(MountingOrientation::FLIP_YZ);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // X unchanged, Y and Z flipped
    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.z());
}

void test_flip_xz_transform(void)
{
    MountingTransform mt(MountingOrientation::FLIP_XZ);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // Y unchanged, X and Z flipped
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.z());
}

void test_flip_xy_transform(void)
{
    MountingTransform mt(MountingOrientation::FLIP_XY);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // Z unchanged, X and Y flipped
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

void test_rotate_90_x_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_90_X);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // 90° about X: X unchanged, Y→Z, Z→-Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.y());  // Was Z
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.z()); // Was -Y
}

void test_rotate_neg90_x_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_NEG90_X);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // -90° about X: X unchanged, Y→-Z, Z→Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.y()); // Was -Z
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.z());  // Was Y
}

void test_rotate_90_y_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_90_Y);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // 90° about Y: Y unchanged, X→-Z, Z→X
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.x()); // Was -Z
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(1.0, output.z());  // Was X
}

void test_rotate_neg90_y_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_NEG90_Y);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // -90° about Y: Y unchanged, X→Z, Z→-X
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.x());  // Was Z
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output.z()); // Was -X
}

void test_rotate_90_z_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_90_Z);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // 90° about Z: Z unchanged, X→Y, Y→-X
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.x());  // Was Y
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output.y()); // Was -X
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

void test_rotate_neg90_z_transform(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_NEG90_Z);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    // -90° about Z: Z unchanged, X→-Y, Y→X
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.x()); // Was -Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, output.y());  // Was X
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

// --- setOrientation Tests ---

void test_setOrientation(void)
{
    MountingTransform mt(MountingOrientation::IDENTITY);
    TEST_ASSERT_TRUE(mt.isIdentity());

    mt.setOrientation(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_FALSE(mt.isIdentity());
    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, mt.getOrientation());

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.z());
}

void test_setOrientation_to_identity(void)
{
    MountingTransform mt(MountingOrientation::FLIP_XY);
    TEST_ASSERT_FALSE(mt.isIdentity());

    mt.setOrientation(MountingOrientation::IDENTITY);
    TEST_ASSERT_TRUE(mt.isIdentity());
}

// --- setMatrix Tests ---

void test_setMatrix(void)
{
    MountingTransform mt;

    double customMatrix[9] = {
        1, 0, 0,
        0, -1, 0,
        0, 0, -1
    };

    mt.setMatrix(customMatrix);
    TEST_ASSERT_EQUAL(MountingOrientation::CUSTOM, mt.getOrientation());

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.z());
}

// --- isIdentity Tests ---

void test_isIdentity_for_identity_orientation(void)
{
    MountingTransform mt(MountingOrientation::IDENTITY);
    TEST_ASSERT_TRUE(mt.isIdentity());
}

void test_isIdentity_for_custom_identity_matrix(void)
{
    double identityMatrix[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    MountingTransform mt(identityMatrix);
    TEST_ASSERT_TRUE(mt.isIdentity());
}

void test_isIdentity_for_non_identity(void)
{
    MountingTransform mt(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_FALSE(mt.isIdentity());
}

// --- getMatrix Tests ---

void test_getMatrix(void)
{
    MountingTransform mt(MountingOrientation::FLIP_XY);

    const double *matrix = mt.getMatrix();
    TEST_ASSERT_NOT_NULL(matrix);

    // FLIP_XY should have -1 on X and Y diagonal
    TEST_ASSERT_EQUAL_FLOAT(-1.0, matrix[0]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, matrix[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0, matrix[8]);
}

// --- Transform with Axis Signs Tests ---

void test_axis_signs_all_positive(void)
{
    MountingTransform mt(1, 1, 1);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

void test_axis_signs_all_negative(void)
{
    MountingTransform mt(-1, -1, -1);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(-1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, output.z());
}

void test_axis_signs_mixed(void)
{
    MountingTransform mt(1, -1, 1);

    Vector<3> input(1.0, 2.0, 3.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(1.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, output.z());
}

// --- Custom Matrix Tests ---

void test_custom_rotation_90_degrees(void)
{
    // 90° rotation about Z-axis
    double rotMatrix[9] = {
        0, -1, 0,
        1, 0, 0,
        0, 0, 1
    };

    MountingTransform mt(rotMatrix);

    Vector<3> input(1.0, 0.0, 0.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, output.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, output.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, output.z());
}

void test_custom_arbitrary_rotation(void)
{
    // Arbitrary rotation matrix
    double rotMatrix[9] = {
        0.5, -0.5, 0.707,
        0.5, 0.866, 0.0,
        -0.707, 0.0, 0.707
    };

    MountingTransform mt(rotMatrix);

    Vector<3> input(1.0, 1.0, 1.0);
    Vector<3> output = mt.transform(input);

    // Just verify it transforms (specific values depend on matrix)
    TEST_ASSERT_TRUE(output.magnitude() > 0);
}

// --- Edge Cases ---

void test_transform_zero_vector(void)
{
    MountingTransform mt(MountingOrientation::FLIP_YZ);

    Vector<3> input(0.0, 0.0, 0.0);
    Vector<3> output = mt.transform(input);

    TEST_ASSERT_EQUAL_FLOAT(0.0, output.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, output.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, output.z());
}

void test_transform_unit_vectors(void)
{
    MountingTransform mt(MountingOrientation::ROTATE_90_Z);

    // Unit X
    Vector<3> unitX(1.0, 0.0, 0.0);
    Vector<3> outX = mt.transform(unitX);
    TEST_ASSERT_EQUAL_FLOAT(0.0, outX.x());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, outX.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, outX.z());

    // Unit Y
    Vector<3> unitY(0.0, 1.0, 0.0);
    Vector<3> outY = mt.transform(unitY);
    TEST_ASSERT_EQUAL_FLOAT(1.0, outY.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, outY.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, outY.z());

    // Unit Z
    Vector<3> unitZ(0.0, 0.0, 1.0);
    Vector<3> outZ = mt.transform(unitZ);
    TEST_ASSERT_EQUAL_FLOAT(0.0, outZ.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, outZ.y());
    TEST_ASSERT_EQUAL_FLOAT(1.0, outZ.z());
}

void test_multiple_transforms(void)
{
    MountingTransform mt(MountingOrientation::IDENTITY);

    Vector<3> input(1.0, 2.0, 3.0);

    // Apply multiple transformations
    mt.setOrientation(MountingOrientation::FLIP_YZ);
    Vector<3> out1 = mt.transform(input);

    mt.setOrientation(MountingOrientation::ROTATE_90_Z);
    Vector<3> out2 = mt.transform(input);

    // Results should be different
    TEST_ASSERT_FALSE(out1.x() == out2.x() && out1.y() == out2.y() && out1.z() == out2.z());
}

void test_transform_preserves_magnitude(void)
{
    // Rotations should preserve vector magnitude
    MountingTransform mt(MountingOrientation::ROTATE_90_Y);

    Vector<3> input(3.0, 4.0, 5.0);
    double inputMag = input.magnitude();

    Vector<3> output = mt.transform(input);
    double outputMag = output.magnitude();

    TEST_ASSERT_FLOAT_WITHIN(1e-6, inputMag, outputMag);
}

void test_double_rotation_equals_flip(void)
{
    // Two 90° rotations about same axis = 180° flip
    MountingTransform mt90(MountingOrientation::ROTATE_90_Z);
    MountingTransform mt180(MountingOrientation::FLIP_XY);

    Vector<3> input(1.0, 2.0, 3.0);

    Vector<3> temp = mt90.transform(input);
    Vector<3> double90 = mt90.transform(temp);

    Vector<3> flip180 = mt180.transform(input);

    TEST_ASSERT_FLOAT_WITHIN(1e-6, flip180.x(), double90.x());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, flip180.y(), double90.y());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, flip180.z(), double90.z());
}

// Main function
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Constructor tests
    RUN_TEST(test_default_constructor);
    RUN_TEST(test_constructor_with_preset);
    RUN_TEST(test_constructor_with_axis_signs);
    RUN_TEST(test_constructor_with_custom_matrix);

    // Transform tests for each preset
    RUN_TEST(test_identity_transform);
    RUN_TEST(test_flip_yz_transform);
    RUN_TEST(test_flip_xz_transform);
    RUN_TEST(test_flip_xy_transform);
    RUN_TEST(test_rotate_90_x_transform);
    RUN_TEST(test_rotate_neg90_x_transform);
    RUN_TEST(test_rotate_90_y_transform);
    RUN_TEST(test_rotate_neg90_y_transform);
    RUN_TEST(test_rotate_90_z_transform);
    RUN_TEST(test_rotate_neg90_z_transform);

    // Setter tests
    RUN_TEST(test_setOrientation);
    RUN_TEST(test_setOrientation_to_identity);
    RUN_TEST(test_setMatrix);

    // Getter tests
    RUN_TEST(test_isIdentity_for_identity_orientation);
    RUN_TEST(test_isIdentity_for_custom_identity_matrix);
    RUN_TEST(test_isIdentity_for_non_identity);
    RUN_TEST(test_getMatrix);

    // Axis signs tests
    RUN_TEST(test_axis_signs_all_positive);
    RUN_TEST(test_axis_signs_all_negative);
    RUN_TEST(test_axis_signs_mixed);

    // Custom matrix tests
    RUN_TEST(test_custom_rotation_90_degrees);
    RUN_TEST(test_custom_arbitrary_rotation);

    // Edge cases
    RUN_TEST(test_transform_zero_vector);
    RUN_TEST(test_transform_unit_vectors);
    RUN_TEST(test_multiple_transforms);
    RUN_TEST(test_transform_preserves_magnitude);
    RUN_TEST(test_double_rotation_equals_flip);

    UNITY_END();

    return 0;
}
