#include <unity.h>
#include "../src/Filters/Mahony.h"
#include "../src/Math/Quaternion.h"
#include "../src/Math/Vector.h"
#include "NativeTestHelper.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using astra::MahonyAHRS;
using astra::Quaternion;
using astra::Vector;

// Helper function to compare quaternions with tolerance
void assertQuaternionEqual(const Quaternion& expected, const Quaternion& actual, double tolerance = 0.01) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.w(), actual.w());
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.x(), actual.x());
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.y(), actual.y());
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.z(), actual.z());
}

// Helper function to compare vectors with tolerance
void assertVectorEqual(const Vector<3>& expected, const Vector<3>& actual, double tolerance = 0.01) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.x(), actual.x());
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.y(), actual.y());
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.z(), actual.z());
}

void setUp(void) {
    // Setup before each test
}

void tearDown(void) {
    // Cleanup after each test
}

// Test 1: Filter is always ready (no calibration mode)
void test_always_ready(void) {
    MahonyAHRS ahrs;
    TEST_ASSERT_TRUE(ahrs.isReady());
}

// Test 2: Initial quaternion is identity
void test_initial_quaternion(void) {
    MahonyAHRS ahrs;
    Quaternion q = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.01);
}

// Test 3: Static orientation holds steady
void test_static_orientation(void) {
    MahonyAHRS ahrs;

    // Gravity pointing down in body frame
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    // Run filter for 5 seconds with no movement
    double dt = 0.01;
    for (int i = 0; i < 500; i++) {
        ahrs.update(accel, gyro, dt);
    }

    // Should converge to orientation where body Z = inertial -Z (down)
    Quaternion q = ahrs.getQuaternion();

    // Check that gravity in body frame rotates to gravity in inertial frame
    Vector<3> gravity_body(0.0, 0.0, 9.81);
    Vector<3> gravity_inertial = q.rotateVector(gravity_body);
    Vector<3> expected_inertial(0.0, 0.0, 9.81);

    assertVectorEqual(expected_inertial, gravity_inertial, 0.5);
}

// Test 4: Gyro integration (90-degree rotation about Z-axis)
void test_gyro_rotation_z_axis(void) {
    MahonyAHRS ahrs;

    // Start aligned
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_init(0.0, 0.0, 0.0);

    // Let it converge to vertical
    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro_init, 0.01);
    }

    // Rotate 90 degrees about Z-axis over 1 second
    double rotation_rate = M_PI / 2.0;  // 90 deg/s in rad/s
    Vector<3> gyro(0.0, 0.0, rotation_rate);
    double dt = 0.01;

    // Use gyro-only mode to avoid accel correction
    for (int i = 0; i < 100; i++) {
        ahrs.update(gyro, dt);  // Gyro-only
    }

    Quaternion q = ahrs.getQuaternion();

    // Verify we rotated by checking a vector transformation
    Vector<3> x_body(1.0, 0.0, 0.0);
    Vector<3> x_rotated = q.rotateVector(x_body);
    Vector<3> expected(0.0, 1.0, 0.0);  // 90° rotation about Z maps X→Y

    assertVectorEqual(expected, x_rotated, 0.15);
}

// Test 5: Accelerometer correction (tilt recovery)
void test_accel_correction(void) {
    MahonyAHRS ahrs(1.0, 0.001);  // Higher Kp for faster correction

    // Start with body aligned to inertial
    Vector<3> accel_upright(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    // Converge to upright
    for (int i = 0; i < 100; i++) {
        ahrs.update(accel_upright, gyro, 0.01);
    }

    // Now tilt the sensor (simulate 45° tilt on Y-axis)
    double tilt_angle = M_PI / 4.0;
    Vector<3> accel_tilted(9.81 * sin(tilt_angle), 0.0, 9.81 * cos(tilt_angle));

    // Run for several seconds to let it converge
    for (int i = 0; i < 500; i++) {
        ahrs.update(accel_tilted, gyro, 0.01);
    }

    // The filter should align to the tilted gravity
    Quaternion q = ahrs.getQuaternion();
    Vector<3> gravity_inertial = q.rotateVector(accel_tilted);
    Vector<3> expected(0.0, 0.0, 9.81);  // Should align with vertical

    assertVectorEqual(expected, gravity_inertial, 1.0);
}

// Test 6: Reset functionality
void test_reset(void) {
    MahonyAHRS ahrs;

    // Run for a while
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.1, 0.2, 0.3);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    Quaternion q_before = ahrs.getQuaternion();

    // Reset
    ahrs.reset();

    // Should be back to identity
    Quaternion q_after = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q_after, 0.01);
}

// Test 7: Earth frame acceleration (gravity compensation)
void test_earth_frame_acceleration(void) {
    MahonyAHRS ahrs;

    // Converge to vertical
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_calib, gyro, 0.01);
    }

    // With no acceleration (just gravity), earth frame accel should be ~zero
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_calib);

    // Should be close to zero in all axes
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.z());
}

// Test 8: Earth frame acceleration with upward acceleration
void test_earth_frame_acceleration_with_motion(void) {
    MahonyAHRS ahrs;

    // Converge to upright
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_calib, gyro, 0.01);
    }

    // Simulate upward acceleration of 10 m/s² (total 19.81 m/s² measured in body Z)
    Vector<3> accel_moving(0.0, 0.0, 19.81);
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_moving);

    // Should measure approximately 10 m/s² upward (negative Z in NED = up)
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(1.0, -10.0, earthAccel.z());  // Up = negative Z
}

// Test 9: setQuaternion works correctly
void test_set_quaternion(void) {
    MahonyAHRS ahrs;

    // Create a 90° rotation about X-axis
    double half_angle = M_PI / 4.0;
    Quaternion q_set(cos(half_angle), sin(half_angle), 0.0, 0.0);

    ahrs.setQuaternion(q_set);

    Quaternion q_get = ahrs.getQuaternion();
    assertQuaternionEqual(q_set, q_get, 0.01);
}

// Test 10: Gyro-only mode doesn't drift with zero input
void test_gyro_only_no_drift(void) {
    MahonyAHRS ahrs;

    // Converge to vertical first
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    Quaternion q_initial = ahrs.getQuaternion();

    // Run gyro-only with zero input
    for (int i = 0; i < 500; i++) {
        ahrs.update(gyro, 0.01);
    }

    Quaternion q_final = ahrs.getQuaternion();

    // Should not have drifted
    assertQuaternionEqual(q_initial, q_final, 0.05);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Basic tests
    RUN_TEST(test_always_ready);
    RUN_TEST(test_initial_quaternion);
    RUN_TEST(test_reset);
    RUN_TEST(test_set_quaternion);

    // Static and stability tests
    RUN_TEST(test_static_orientation);
    RUN_TEST(test_gyro_only_no_drift);

    // Rotation tests
    RUN_TEST(test_gyro_rotation_z_axis);

    // Correction tests
    RUN_TEST(test_accel_correction);

    // Acceleration transformation tests
    RUN_TEST(test_earth_frame_acceleration);
    RUN_TEST(test_earth_frame_acceleration_with_motion);

    UNITY_END();

    return 0;
}
