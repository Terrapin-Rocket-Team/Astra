#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"
#include <Sensors/MountingTransform.h>

using namespace astra;

namespace test_rotatable_sensor {

FakeAccel accel;

class NullNameRotatableSensor : public RotatableSensor
{
public:
    NullNameRotatableSensor() : RotatableSensor(nullptr) {}

protected:
    int init() override { return 0; }
    int read() override { return 0; }
};

void local_setUp(void)
{
    accel.begin();
    accel.set(Vector<3>{1.0, 2.0, 3.0});
}

void local_tearDown(void)
{
    // Reset to identity orientation
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
}

void test_rotatable_sensor_identity()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::IDENTITY, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_flip_yz()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y and Z flip
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_flip_xz()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::FLIP_XZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X and Z flip
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_XZ, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_flip_xy()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X and Y flip
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_XY, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_90_x()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_X);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y→Z, Z→-Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_X, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_neg90_x()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_X);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y→-Z, Z→Y
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_X, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_90_y()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Y);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X→-Z, Z→X
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Y, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_neg90_y()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_Y);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Y stays same, X→Z, Z→-X
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_Y, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_90_z()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X→Y, Y→-X
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Z, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_rotate_neg90_z()
{
    local_setUp();
    accel.setMountingOrientation(MountingOrientation::ROTATE_NEG90_Z);
    accel.update();

    Vector<3> result = accel.getAccel();
    // Z stays same, X→-Y, Y→X
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());

    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_Z, accel.getMountingOrientation());
    local_tearDown();
}

void test_rotatable_sensor_change_orientation()
{
    local_setUp();
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
    local_tearDown();
}

void test_rotatable_sensor_zero_vector()
{
    local_setUp();
    accel.set(Vector<3>{0.0, 0.0, 0.0});

    // Any orientation applied to zero vector should still be zero
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
    local_tearDown();
}

void test_rotatable_sensor_negative_values()
{
    local_setUp();
    accel.set(Vector<3>{-1.0, -2.0, -3.0});
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    // X stays same, Y and Z flip (negative becomes positive)
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
    local_tearDown();
}

void test_rotatable_sensor_preserves_magnitude()
{
    local_setUp();
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
    local_tearDown();
}

void test_rotatable_sensor_gyro()
{
    local_setUp();
    FakeGyro gyro;
    gyro.begin();
    gyro.set(Vector<3>{0.1, 0.2, 0.3});

    gyro.setMountingOrientation(MountingOrientation::FLIP_YZ);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-0.3, result.z());
    local_tearDown();
}

void test_rotatable_sensor_orientation_after_data_change()
{
    local_setUp();
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
    local_tearDown();
}

void test_rotatable_sensor_null_name_ctor_path()
{
    local_setUp();
    NullNameRotatableSensor sensor;
    TEST_ASSERT_EQUAL(0, sensor.begin());
    TEST_ASSERT_NOT_NULL(sensor.getName());
    local_tearDown();
}

void run_test_rotatable_sensor_tests()
{
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
    RUN_TEST(test_rotatable_sensor_null_name_ctor_path);
}

} // namespace test_rotatable_sensor
