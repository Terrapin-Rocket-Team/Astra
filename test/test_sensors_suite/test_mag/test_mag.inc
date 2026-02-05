#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

using namespace astra;

// Test fixture
FakeMag mag;

void test_mag_setUp(void)
{
    mag.begin();
}

void test_mag_tearDown(void)
{
    mag.setMountingOrientation(MountingOrientation::IDENTITY);
}

// Test magnetometer initialization
void test_mag_test_mag_begin()
{
    TEST_ASSERT_TRUE(mag.begin());
    TEST_ASSERT_TRUE(mag.isInitialized());

    // Default magnetic field should be zero
    Vector<3> magField = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(0.0, magField.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, magField.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, magField.z());
}

// Test setting magnetic field data
void test_mag_test_mag_set()
{
    mag.set(Vector<3>{20.0, 30.0, 40.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(40.0, result.z());
}

// Test getMag method
void test_mag_test_mag_get_mag()
{
    mag.set(Vector<3>{15.5, -25.3, 35.7});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(15.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-25.3, result.y());
    TEST_ASSERT_EQUAL_FLOAT(35.7, result.z());
}

// Test mounting orientation - identity
void test_mag_test_mag_orientation_identity()
{
    mag.set(Vector<3>{10.0, 20.0, 30.0});
    mag.setMountingOrientation(MountingOrientation::IDENTITY);
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.z());
}

// Test mounting orientation - FLIP_YZ
void test_mag_test_mag_orientation_flip_yz()
{
    mag.set(Vector<3>{10.0, 20.0, 30.0});
    mag.setMountingOrientation(MountingOrientation::FLIP_YZ);
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-20.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-30.0, result.z());
}

// Test mounting orientation - ROTATE_90_Z
void test_mag_test_mag_orientation_rotate_90_z()
{
    mag.set(Vector<3>{10.0, 20.0, 30.0});
    mag.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-10.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.z());
}

// Test mounting orientation - ROTATE_90_X
void test_mag_test_mag_orientation_rotate_90_x()
{
    mag.set(Vector<3>{10.0, 20.0, 30.0});
    mag.setMountingOrientation(MountingOrientation::ROTATE_90_X);
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-20.0, result.z());
}

// Test zero magnetic field
void test_mag_test_mag_zero()
{
    mag.set(Vector<3>{0.0, 0.0, 0.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test negative magnetic field values
void test_mag_test_mag_negative()
{
    mag.set(Vector<3>{-15.0, -25.0, -35.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(-15.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-25.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-35.0, result.z());
}

// Test typical Earth magnetic field values (in microTesla)
void test_mag_test_mag_earth_field_typical()
{
    // Typical Earth's magnetic field: 25-65 uT
    // Example: North America - approximately X=20, Y=0, Z=45 uT
    mag.set(Vector<3>{20.0, 0.0, 45.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(45.0, result.z());

    // Check magnitude is in reasonable range for Earth's field
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(1.0, 49.24, magnitude);
}

// Test small magnetic field values
void test_mag_test_mag_small_values()
{
    mag.set(Vector<3>{0.1, 0.2, 0.3});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(0.1, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, result.z());
}

// Test large magnetic field values (near strong magnets)
void test_mag_test_mag_large_values()
{
    mag.set(Vector<3>{100.0, 200.0, 300.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(100.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(200.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(300.0, result.z());
}

// Test multiple updates
void test_mag_test_mag_multiple_updates()
{
    // First update
    mag.set(Vector<3>{10.0, 0.0, 0.0});
    mag.update();
    TEST_ASSERT_EQUAL_FLOAT(10.0, mag.getMag().x());

    // Second update
    mag.set(Vector<3>{0.0, 20.0, 0.0});
    mag.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, mag.getMag().x());
    TEST_ASSERT_EQUAL_FLOAT(20.0, mag.getMag().y());

    // Third update
    mag.set(Vector<3>{0.0, 0.0, 30.0});
    mag.update();
    TEST_ASSERT_EQUAL_FLOAT(0.0, mag.getMag().y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, mag.getMag().z());
}

// Test changing orientation after setting data
void test_mag_test_mag_change_orientation()
{
    mag.set(Vector<3>{10.0, 20.0, 30.0});

    // First with identity
    mag.setMountingOrientation(MountingOrientation::IDENTITY);
    mag.update();
    Vector<3> result1 = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(10.0, result1.x());
    TEST_ASSERT_EQUAL_FLOAT(20.0, result1.y());

    // Then with flip
    mag.setMountingOrientation(MountingOrientation::FLIP_XY);
    mag.update();
    Vector<3> result2 = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(-10.0, result2.x());
    TEST_ASSERT_EQUAL_FLOAT(-20.0, result2.y());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result2.z());
}

// Test magnitude calculation
void test_mag_test_mag_magnitude()
{
    // Set magnetic field (magnitude should be sqrt(100 + 400 + 900) = sqrt(1400) ≈ 37.42)
    mag.set(Vector<3>{10.0, 20.0, 30.0});
    mag.update();

    Vector<3> result = mag.getMag();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 37.42, magnitude);
}

// Test that orientation preserves magnitude
void test_mag_test_mag_orientation_preserves_magnitude()
{
    mag.set(Vector<3>{30.0, 40.0, 0.0});
    double originalMag = 50.0; // sqrt(900 + 1600)

    mag.setMountingOrientation(MountingOrientation::ROTATE_90_Y);
    mag.update();

    Vector<3> result = mag.getMag();
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.001, originalMag, magnitude);
}

// Test getMountingOrientation
void test_mag_test_mag_get_mounting_orientation()
{
    mag.setMountingOrientation(MountingOrientation::FLIP_YZ);
    TEST_ASSERT_EQUAL(MountingOrientation::FLIP_YZ, mag.getMountingOrientation());

    mag.setMountingOrientation(MountingOrientation::ROTATE_NEG90_Z);
    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_NEG90_Z, mag.getMountingOrientation());
}

// Test horizontal component (useful for compass heading)
void test_mag_test_mag_horizontal_component()
{
    // Set field with known horizontal and vertical components
    // Horizontal: sqrt(20^2 + 0^2) = 20 uT
    // Vertical: 45 uT
    mag.set(Vector<3>{20.0, 0.0, 45.0});
    mag.update();

    Vector<3> result = mag.getMag();
    double horizontal = sqrt(result.x() * result.x() + result.y() * result.y());

    TEST_ASSERT_FLOAT_WITHIN(0.01, 20.0, horizontal);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 45.0, result.z());
}

// Test all axes equal (isotropic field)
void test_mag_test_mag_isotropic_field()
{
    mag.set(Vector<3>{25.0, 25.0, 25.0});
    mag.update();

    Vector<3> result = mag.getMag();
    TEST_ASSERT_EQUAL_FLOAT(25.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(25.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(25.0, result.z());

    // Magnitude = 25 * sqrt(3) ≈ 43.30
    double magnitude = result.magnitude();
    TEST_ASSERT_FLOAT_WITHIN(0.01, 43.30, magnitude);
}
