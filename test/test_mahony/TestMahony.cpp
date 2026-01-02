#include <unity.h>
#include "../src/Filters/Mahony.h"
#include "../src/Math/Quaternion.h"
#include "../src/Math/Vector.h"
#include "../../lib/NativeTestMocks/NativeTestHelper.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using astra::MahonyAHRS;
using astra::MahonyMode;
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

// Test 1: Calibration and Initialization
void test_calibration_and_initialization(void) {
    MahonyAHRS ahrs;

    // Initial state should be CALIBRATING
    TEST_ASSERT_EQUAL(MahonyMode::CALIBRATING, ahrs.getMode());
    TEST_ASSERT_FALSE(ahrs.isInitialized());

    // Simulate 100 calibration samples with sensor pointing up (Z-axis aligned with gravity)
    Vector<3> accel(0.0, 0.0, 9.81);  // Gravity in Z
    Vector<3> gyro(0.01, -0.02, 0.005);  // Small constant bias

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    // Initialize the filter
    ahrs.initialize();

    TEST_ASSERT_TRUE(ahrs.isInitialized());

    // After initialization with Z-up, the quaternion should be identity (or close to it)
    // since body frame Z aligns with earth frame Z
    Quaternion q = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.1);
}

// Test 2: Calibration with tilted sensor
void test_calibration_tilted_sensor(void) {
    MahonyAHRS ahrs;

    // Sensor tilted 90 degrees (X-axis pointing down)
    Vector<3> accel(9.81, 0.0, 0.0);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    TEST_ASSERT_TRUE(ahrs.isInitialized());

    // The quaternion should represent a rotation that aligns X with Z
    Quaternion q = ahrs.getQuaternion();

    // After relative quaternion, should be close to identity
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.1);
}

// Test 3: Static orientation holds steady
void test_static_orientation(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    // Run filter for 5 seconds with no movement
    double dt = 0.01;
    for (int i = 0; i < 500; i++) {
        ahrs.update(accel, gyro, dt);
    }

    // Should remain close to identity
    Quaternion q = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.05);
}

// Test 4: Gyro integration (90-degree rotation about Z-axis)
void test_gyro_rotation_z_axis(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_calib(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro_calib);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::GYRO_ONLY);  // Pure gyro integration

    // Rotate 90 degrees about Z-axis over 1 second (π/2 rad/s)
    double rotation_rate = M_PI / 2.0;  // 90 deg/s in rad/s
    Vector<3> gyro(0.0, 0.0, -rotation_rate);  // Note: negative due to sign convention
    double dt = 0.01;

    for (int i = 0; i < 100; i++) {  // 1 second total
        ahrs.update(accel, gyro, dt);
    }

    // Expected quaternion for 90-degree rotation about Z-axis
    double half_angle = (M_PI / 2.0) / 2.0;
    Quaternion expected(cos(half_angle), 0.0, 0.0, sin(half_angle));

    Quaternion q = ahrs.getQuaternion();
    assertQuaternionEqual(expected, q, 0.1);
}

// Test 5: Accelerometer correction (tilt recovery)
void test_accel_correction(void) {
    MahonyAHRS ahrs(1.0, 0.001);  // Higher Kp for faster correction

    // Calibrate upright
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro_calib(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel_calib, gyro_calib);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    // Now simulate a tilted accelerometer (as if sensor tilted 45 degrees on X-axis)
    double tilt_angle = M_PI / 4.0;  // 45 degrees
    Vector<3> accel_tilted(0.0, 9.81 * sin(tilt_angle), 9.81 * cos(tilt_angle));
    Vector<3> gyro(0.0, 0.0, 0.0);

    double dt = 0.01;

    // Run for several seconds to let it converge
    for (int i = 0; i < 500; i++) {
        ahrs.update(accel_tilted, gyro, dt);
    }

    // The filter should have corrected to the tilted orientation
    Quaternion q = ahrs.getQuaternion();

    // Rotate gravity vector by quaternion and check it aligns with measured accel
    Vector<3> gravity_earth(0.0, 0.0, 1.0);
    Vector<3> gravity_body = q.rotateVector(gravity_earth);

    Vector<3> accel_normalized = accel_tilted;
    accel_normalized.normalize();

    printf("Accel correction test:\n");
    printf("  Quaternion: w=%f, x=%f, y=%f, z=%f\n", q.w(), q.x(), q.y(), q.z());
    printf("  Expected accel (normalized): x=%f, y=%f, z=%f\n", accel_normalized.x(), accel_normalized.y(), accel_normalized.z());
    printf("  Rotated gravity (body): x=%f, y=%f, z=%f\n", gravity_body.x(), gravity_body.y(), gravity_body.z());

    // The rotated gravity should align with the measured acceleration
    assertVectorEqual(accel_normalized, gravity_body, 0.15);
}

// Test 6: Mode switching
void test_mode_switching(void) {
    MahonyAHRS ahrs;

    TEST_ASSERT_EQUAL(MahonyMode::CALIBRATING, ahrs.getMode());

    ahrs.setMode(MahonyMode::CORRECTING);
    TEST_ASSERT_EQUAL(MahonyMode::CORRECTING, ahrs.getMode());

    ahrs.setMode(MahonyMode::GYRO_ONLY);
    TEST_ASSERT_EQUAL(MahonyMode::GYRO_ONLY, ahrs.getMode());

    ahrs.setMode(MahonyMode::CALIBRATING);
    TEST_ASSERT_EQUAL(MahonyMode::CALIBRATING, ahrs.getMode());
}

// Test 7: Reset functionality
void test_reset(void) {
    MahonyAHRS ahrs;

    // Calibrate and initialize
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.1, 0.2, 0.3);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    TEST_ASSERT_TRUE(ahrs.isInitialized());
    TEST_ASSERT_EQUAL(MahonyMode::CORRECTING, ahrs.getMode());

    // Reset
    ahrs.reset();

    // Should be back to initial state
    TEST_ASSERT_FALSE(ahrs.isInitialized());
    TEST_ASSERT_EQUAL(MahonyMode::CALIBRATING, ahrs.getMode());
}

// Test 8: Earth frame acceleration (gravity compensation)
void test_earth_frame_acceleration(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel_calib, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    // With no acceleration (just gravity), earth frame accel should be ~zero
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_calib);

    // Should be close to zero in all axes
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.z());
}

// Test 9: Earth frame acceleration with upward acceleration
void test_earth_frame_acceleration_with_motion(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel_calib, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::CORRECTING);

    // Simulate upward acceleration of 10 m/s² (total 19.81 m/s² measured)
    Vector<3> accel_moving(0.0, 0.0, 19.81);
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_moving);

    // Should measure approximately 10 m/s² upward (Z-axis)
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(1.0, 10.0, earthAccel.z());
}

// Test 10: Gyro bias removal
void test_gyro_bias_removal(void) {
    MahonyAHRS ahrs;

    // Calibrate with constant gyro bias
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_bias(0.1, -0.05, 0.08);

    for (int i = 0; i < 200; i++) {
        ahrs.calibrate(accel, gyro_bias);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::GYRO_ONLY);

    // After initialization, the same gyro reading (which is just bias)
    // should result in no rotation
    double dt = 0.01;
    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro_bias, dt);
    }

    // Should stay close to identity since bias is removed
    Quaternion q = ahrs.getQuaternion();
    printf("Gyro bias test - Quaternion: w=%f, x=%f, y=%f, z=%f\n", q.w(), q.x(), q.y(), q.z());
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.05);
}

// Test 11: GYRO_ONLY mode ignores accelerometer
void test_gyro_only_mode_ignores_accel(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel_calib(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel_calib, gyro);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::GYRO_ONLY);

    // Get initial quaternion
    ahrs.update(accel_calib, gyro, 0.01);
    Quaternion q_initial = ahrs.getQuaternion();

    // Now provide completely wrong accelerometer data (should be ignored)
    Vector<3> accel_wrong(9.81, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel_wrong, gyro, 0.01);
    }

    Quaternion q_final = ahrs.getQuaternion();

    // Quaternion should not have changed much (no gyro input)
    assertQuaternionEqual(q_initial, q_final, 0.05);
}

// Test 12: Continuous rotation tracking
void test_continuous_rotation(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_calib(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro_calib);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::GYRO_ONLY);

    // Rotate continuously at 45 deg/s for 4 seconds = 180 degrees
    double rotation_rate = M_PI / 4.0;  // 45 deg/s
    Vector<3> gyro(0.0, 0.0, -rotation_rate);
    double dt = 0.01;

    for (int i = 0; i < 400; i++) {  // 4 seconds
        ahrs.update(accel, gyro, dt);
    }

    // Expected: 180-degree rotation about Z-axis
    double half_angle = M_PI / 2.0;  // Half of 180 degrees
    Quaternion expected(cos(half_angle), 0.0, 0.0, sin(half_angle));

    Quaternion q = ahrs.getQuaternion();
    assertQuaternionEqual(expected, q, 0.15);
}

// Test 13: Full rotation (360 degrees) returns to identity
void test_full_rotation(void) {
    MahonyAHRS ahrs;

    // Calibrate upright
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_calib(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.calibrate(accel, gyro_calib);
    }

    ahrs.initialize();
    ahrs.setMode(MahonyMode::GYRO_ONLY);

    // Rotate 360 degrees about Z-axis over 4 seconds
    double rotation_rate = M_PI / 2.0;  // 90 deg/s
    Vector<3> gyro(0.0, 0.0, -rotation_rate);
    double dt = 0.01;

    for (int i = 0; i < 400; i++) {  // 4 seconds
        ahrs.update(accel, gyro, dt);
    }

    // After 360 degrees, should be back to identity (or its negative)
    Quaternion q = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    Quaternion identity_neg(-1.0, 0.0, 0.0, 0.0);

    // Check if close to either identity or -identity (both represent same rotation)
    bool close_to_identity = (fabs(q.w() - 1.0) < 0.2);
    bool close_to_neg_identity = (fabs(q.w() + 1.0) < 0.2);

    TEST_ASSERT_TRUE(close_to_identity || close_to_neg_identity);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Calibration tests
    RUN_TEST(test_calibration_and_initialization);
    RUN_TEST(test_calibration_tilted_sensor);

    // Static and stability tests
    RUN_TEST(test_static_orientation);
    RUN_TEST(test_gyro_bias_removal);

    // Rotation tests
    RUN_TEST(test_gyro_rotation_z_axis);
    RUN_TEST(test_continuous_rotation);
    RUN_TEST(test_full_rotation);

    // Correction tests
    RUN_TEST(test_accel_correction);

    // Mode tests
    RUN_TEST(test_mode_switching);
    RUN_TEST(test_gyro_only_mode_ignores_accel);

    // Utility tests
    RUN_TEST(test_reset);
    RUN_TEST(test_earth_frame_acceleration);
    RUN_TEST(test_earth_frame_acceleration_with_motion);

    UNITY_END();

    return 0;
}
