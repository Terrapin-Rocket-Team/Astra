#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"
#include <Sensors/MountingTransform.h>

using namespace astra;

// Test fixture
FakeAccel accel;

void setUp(void)
{
    accel.begin();
    accel.set(Vector<3>{1.0, 2.0, 3.0});
}

void tearDown(void)
{
    // Reset to identity orientation
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
}

// Test identity orientation (no transformation)
void test_rotatable_sensor_identity()
{
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::IDENTITY, accel.getMountingOrientation());
}

// Test FLIP_YZ orientation (180° rotation about X axis)
void test_rotatable_sensor_flip_yz()
{
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y and Z flip
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, accel.getMountingOrientation());
}

// Test FLIP_XZ orientation (180° rotation about Y axis)
void test_rotatable_sensor_flip_xz()
{
    accel.setMountingOrientation(MountingOrientation::FLIP_XZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X and Z flip
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_XZ, accel.getMountingOrientation());
}

// Test FLIP_XY orientation (180° rotation about Z axis)
void test_rotatable_sensor_flip_xy()
{
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X and Y flip
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_XY, accel.getMountingOrientation());
}

// Test ROTATE_90_X orientation (90° rotation about X axis)
void test_rotatable_sensor_rotate_90_x()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_X);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y→Z, Z→-Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_X, accel.getMountingOrientation());
}

// Test ROTATE_NEG90_X orientation (-90° rotation about X axis)
void test_rotatable_sensor_rotate_neg90_x()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_X);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y→-Z, Z→Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_X, accel.getMountingOrientation());
}

// Test ROTATE_90_Y orientation (90° rotation about Y axis)
void test_rotatable_sensor_rotate_90_y()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Y);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X→-Z, Z→X
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Y, accel.getMountingOrientation());
}

// Test ROTATE_NEG90_Y orientation (-90° rotation about Y axis)
void test_rotatable_sensor_rotate_neg90_y()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_Y);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X→Z, Z→-X
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_Y, accel.getMountingOrientation());
}

// Test ROTATE_90_Z orientation (90° rotation about Z axis)
void test_rotatable_sensor_rotate_90_z()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X→Y, Y→-X
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Z, accel.getMountingOrientation());
}

// Test ROTATE_NEG90_Z orientation (-90° rotation about Z axis)
void test_rotatable_sensor_rotate_neg90_z()
{
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_Z);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X→-Y, Y→X
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_Z, accel.getMountingOrientation());
}

// Test changing orientation multiple times
void test_rotatable_sensor_change_orientation()
{
    // Start with identity
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();
    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    // Change to FLIP_YZ
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();
    result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());

    // Change back to identity
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();
    result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
}

// Test orientation with zero vector
void test_rotatable_sensor_zero_vector()
{
    accel.set(Vector<3>{0.0, 0.0, 0.0});

    // Any orientation applied to zero vector should still be zero
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test orientation with negative values
void test_rotatable_sensor_negative_values()
{
    accel.set(Vector<3>{-1.0, -2.0, -3.0});
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y and Z flip (negative becomes positive)
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
}

// Test orientation preserves magnitude
void test_rotatable_sensor_preserves_magnitude()
{
    accel.set(Vector<3>{3.0, 4.0, 0.0});
    double originalMagnitude = 5.0; // sqrt(9 + 16) = 5

    // Test several orientations
    MountingOrientation orientations[] = {
        MountingOrientation::IDENTITY,
        MountingOrientation::FLIP_YZ,
        MountingOrientation::ROTATE_90_X,
        MountingOrientation::ROTATE_90_Z
    };

    for (auto orient : orientations)
    {
        accel.setMountingOrientation(orient);
        accel.update();

        Vector<3> result = accel.getAccel();
        double magnitude = result.magnitude();

        TEST_ASSERT_FLOAT_WITHIN(0.001, originalMagnitude, magnitude);
    }
}

// Test with gyroscope to verify RotatableSensor works for other sensor types
void test_rotatable_sensor_gyro()
{
    FakeGyro gyro;
    gyro.begin();
    gyro.set(Vector<3>{0.1, 0.2, 0.3});

    gyro.setMountingOrientation(MountingOrientation::FLIP_YZ);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-0.3, result.z());
}

// Test orientation after sensor data update
void test_rotatable_sensor_orientation_after_data_change()
{
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);

    // Update data
    accel.set(Vector<3>{5.0, 6.0, 7.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(-5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(7.0, result.z());

    // Change data again
    accel.set(Vector<3>{10.0, 20.0, 30.0});
    accel.update();

    result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(-10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-20.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.z());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_rotatable_sensor_identity);
    RUN_TEST(test_rotatable_sensor_flip_yz);
    RUN_TEST(test_rotatable_sensor_flip_xz);
    RUN_TEST(test_rotatable_sensor_flip_xy);
    RUN_TEST(test_rotatable_sensor_rotate_90_x);
    RUN_TEST(test_rotatable_sensor_rotate_neg90_x);
    RUN_TEST(test_rotatable_sensor_rotate_90_y);
    RUN_TEST(test_rotatable_sensor_rotate_neg90_y);
    RUN_TEST(test_rotatable_sensor_rotate_90_z);
    RUN_TEST(test_rotatable_sensor_rotate_neg90_z);
    RUN_TEST(test_rotatable_sensor_change_orientation);
    RUN_TEST(test_rotatable_sensor_zero_vector);
    RUN_TEST(test_rotatable_sensor_negative_values);
    RUN_TEST(test_rotatable_sensor_preserves_magnitude);
    RUN_TEST(test_rotatable_sensor_gyro);
    RUN_TEST(test_rotatable_sensor_orientation_after_data_change);

    UNITY_END();
    return 0;
}
