#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

using namespace astra;

// Test fixture
FakeAccel accel;

void test_accel_setUp(void)
{
    accel.begin();
}

void test_accel_tearDown(void)
{
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
}

// Test accelerometer initialization
void test_accel_test_accel_begin()
{
    TEST_ASSERT_TRUE(accel.begin());
    TEST_ASSERT_TRUE(accel.isInitialized());

    // Default acceleration should be gravity in -Z direction
    Vector<3> acc = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, acc.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, acc.y());
    TEST_ASSERT_EQUAL_FLOAT(-9.81, acc.z());
}

// Test setting acceleration data
void test_accel_test_accel_set()
{
    accel.set(Vector<3>{1.0, 2.0, 3.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
}

// Test getAccel method
void test_accel_test_accel_get_accel()
{
    accel.set(Vector<3>{5.0, -3.0, 9.81});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(9.81, result.z());
}

// Test mounting orientation - identity
void test_accel_test_accel_orientation_identity()
{
    accel.set(Vector<3>{1.0, 2.0, 3.0});
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
}

// Test mounting orientation - FLIP_YZ
void test_accel_test_accel_orientation_flip_yz()
{
    accel.set(Vector<3>{1.0, 2.0, 3.0});
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.z());
}

// Test mounting orientation - ROTATE_90_Z
void test_accel_test_accel_orientation_rotate_90_z()
{
    accel.set(Vector<3>{1.0, 2.0, 3.0});
    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
}

// Test zero acceleration
void test_accel_test_accel_zero()
{
    accel.set(Vector<3>{0.0, 0.0, 0.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test negative acceleration values
void test_accel_test_accel_negative()
{
    accel.set(Vector<3>{-5.0, -10.0, -15.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(-5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-10.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-15.0, result.z());
}

// Test large acceleration values
void test_accel_test_accel_large_values()
{
    accel.set(Vector<3>{100.0, 200.0, 300.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(100.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(200.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(300.0, result.z());
}

// Test multiple updates
void test_accel_test_accel_multiple_updates()
{
    // First update
    accel.set(Vector<3>{1.0, 0.0, 0.0});
    accel.update();
    TEST_ASSERT_EQUAL_FLOAT(1.0, accel.getAccel().x());

    // Second update
    accel.set(Vector<3>{0.0, 2.0, 0.0});
    accel.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, accel.getAccel().x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, accel.getAccel().y());

    // Third update
    accel.set(Vector<3>{0.0, 0.0, 3.0});
    accel.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, accel.getAccel().y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, accel.getAccel().z());
}

// Test changing orientation after setting data
void test_accel_test_accel_change_orientation()
{
    accel.set(Vector<3>{1.0, 2.0, 3.0});

    // First with identity
    accel.setMountingOrientation(MountingOrientation::IDENTITY);
    accel.update();
    Vector<3> result1 = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result1.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result1.y());

    // Then with flip
    accel.setMountingOrientation(MountingOrientation::FLIP_XY);
    accel.update();
    Vector<3> result2 = accel.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(-1.0, result2.x());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result2.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result2.z());
}

// Test magnitude calculation
void test_accel_test_accel_magnitude()
{
    // Set a 3-4-5 triangle (magnitude = 5 * sqrt(2) ≈ 7.07)
    accel.set(Vector<3>{3.0, 4.0, 5.0});
    accel.update();

    Vector<3> result = accel.getAccel();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 7.071, magnitude);
}

// Test that orientation preserves magnitude
void test_accel_test_accel_orientation_preserves_magnitude()
{
    accel.set(Vector<3>{3.0, 4.0, 0.0});
    double originalMag = 5.0; // sqrt(9 + 16)

    accel.setMountingOrientation(MountingOrientation::ROTATE_90_X);
    accel.update();

    Vector<3> result = accel.getAccel();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.001, originalMag, magnitude);
}

// Test getMountingOrientation
void test_accel_test_accel_get_mounting_orientation()
{
    accel.setMountingOrientation(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, accel.getMountingOrientation());

    accel.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Z, accel.getMountingOrientation());
}
