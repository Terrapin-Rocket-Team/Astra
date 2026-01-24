#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

using namespace astra;

// Test fixture
FakeGyro gyro;

void setUp(void)
{
    gyro.begin();
}

void tearDown(void)
{
    gyro.setMountingOrientation(MountingOrientation::IDENTITY);
}

// Test gyroscope initialization
void test_gyro_begin()
{
    TEST_ASSERT_TRUE(gyro.begin());
    TEST_ASSERT_TRUE(gyro.isInitialized());

    // Default angular velocity should be zero
    Vector<3> angVel = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, angVel.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, angVel.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, angVel.z());
}

// Test setting angular velocity data
void test_gyro_set()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result.z());
}

// Test getAngVel method
void test_gyro_get_ang_vel()
{
    gyro.set(Vector<3>{1.5, -2.3, 0.7});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(1.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.3, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.7, result.z());
}

// Test mounting orientation - identity
void test_gyro_orientation_identity()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    gyro.setMountingOrientation(MountingOrientation::IDENTITY);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result.z());
}

// Test mounting orientation - FLIP_YZ
void test_gyro_orientation_flip_yz()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    gyro.setMountingOrientation(MountingOrientation::FLIP_YZ);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-0.3, result.z());
}

// Test mounting orientation - ROTATE_90_Z
void test_gyro_orientation_rotate_90_z()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    gyro.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.2, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.1, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result.z());
}

// Test mounting orientation - ROTATE_90_X
void test_gyro_orientation_rotate_90_x()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    gyro.setMountingOrientation(MountingOrientation::ROTATE_90_X);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, result.z());
}

// Test zero angular velocity
void test_gyro_zero()
{
    gyro.set(Vector<3>{0.0, 0.0, 0.0});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test negative angular velocity values
void test_gyro_negative()
{
    gyro.set(Vector<3>{-0.5, -1.0, -1.5});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(-0.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-1.5, result.z());
}

// Test large angular velocity values
void test_gyro_large_values()
{
    gyro.set(Vector<3>{10.0, 20.0, 30.0});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.z());
}

// Test small angular velocity values (drift-like)
void test_gyro_small_values()
{
    gyro.set(Vector<3>{0.001, 0.002, 0.003});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.001, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.002, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.003, result.z());
}

// Test multiple updates
void test_gyro_multiple_updates()
{
    // First update
    gyro.set(Vector<3>{0.1, 0.0, 0.0});
    gyro.update();
    TEST_ASSERT_EQUAL_FLOAT(0.1, gyro.getAngVel().x());

    // Second update
    gyro.set(Vector<3>{0.0, 0.2, 0.0});
    gyro.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, gyro.getAngVel().x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, gyro.getAngVel().y());

    // Third update
    gyro.set(Vector<3>{0.0, 0.0, 0.3});
    gyro.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, gyro.getAngVel().y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, gyro.getAngVel().z());
}

// Test changing orientation after setting data
void test_gyro_change_orientation()
{
    gyro.set(Vector<3>{0.1, 0.2, 0.3});

    // First with identity
    gyro.setMountingOrientation(MountingOrientation::IDENTITY);
    gyro.update();
    Vector<3> result1 = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result1.x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, result1.y());

    // Then with flip
    gyro.setMountingOrientation(MountingOrientation::FLIP_XY);
    gyro.update();
    Vector<3> result2 = gyro.getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(-0.1, result2.x());
    TEST_ASSERT_EQUAL_FLOAT(-0.2, result2.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result2.z());
}

// Test magnitude calculation
void test_gyro_magnitude()
{
    // Set angular velocity (magnitude should be sqrt(1 + 4 + 9) = sqrt(14) ≈ 3.742)
    gyro.set(Vector<3>{1.0, 2.0, 3.0});
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 3.742, magnitude);
}

// Test that orientation preserves magnitude
void test_gyro_orientation_preserves_magnitude()
{
    gyro.set(Vector<3>{0.3, 0.4, 0.0});
    double originalMag = 0.5; // sqrt(0.09 + 0.16)

    gyro.setMountingOrientation(MountingOrientation::ROTATE_90_Y);
    gyro.update();

    Vector<3> result = gyro.getAngVel();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.001, originalMag, magnitude);
}

// Test getMountingOrientation
void test_gyro_get_mounting_orientation()
{
    gyro.setMountingOrientation(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, gyro.getMountingOrientation());

    gyro.setMountingOrientation(MountingOrientation::ROTATE_NEG90_X);
    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_X, gyro.getMountingOrientation());
}

// Test typical rotation rates (rad/s)
void test_gyro_typical_rotation_rates()
{
    // Slow rotation: ~10 deg/s = 0.174 rad/s
    gyro.set(Vector<3>{0.174, 0.0, 0.0});
    gyro.update();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.174, gyro.getAngVel().x());

    // Fast rotation: ~180 deg/s = 3.14 rad/s
    gyro.set(Vector<3>{0.0, 3.14, 0.0});
    gyro.update();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 3.14, gyro.getAngVel().y());

    // Very fast rotation: ~360 deg/s = 6.28 rad/s
    gyro.set(Vector<3>{0.0, 0.0, 6.28});
    gyro.update();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 6.28, gyro.getAngVel().z());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_gyro_begin);
    RUN_TEST(test_gyro_set);
    RUN_TEST(test_gyro_get_ang_vel);
    RUN_TEST(test_gyro_orientation_identity);
    RUN_TEST(test_gyro_orientation_flip_yz);
    RUN_TEST(test_gyro_orientation_rotate_90_z);
    RUN_TEST(test_gyro_orientation_rotate_90_x);
    RUN_TEST(test_gyro_zero);
    RUN_TEST(test_gyro_negative);
    RUN_TEST(test_gyro_large_values);
    RUN_TEST(test_gyro_small_values);
    RUN_TEST(test_gyro_multiple_updates);
    RUN_TEST(test_gyro_change_orientation);
    RUN_TEST(test_gyro_magnitude);
    RUN_TEST(test_gyro_orientation_preserves_magnitude);
    RUN_TEST(test_gyro_get_mounting_orientation);
    RUN_TEST(test_gyro_typical_rotation_rates);

    UNITY_END();
    return 0;
}
