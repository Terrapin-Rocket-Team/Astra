#pragma once

#include <unity.h>
#include "Filters/Mahony.h"
#include "Math/Quaternion.h"
#include "Math/Vector.h"
#include "NativeTestHelper.h"

using astra::MahonyAHRS;
using astra::MountingOrientation;
using astra::MountingTransform;
using astra::Quaternion;
using astra::Vector;

namespace test_mahony {

double quaternionDistance(const Quaternion& a, const Quaternion& b) {
    double dist1 = sqrt(pow(a.w() - b.w(), 2) +
                        pow(a.x() - b.x(), 2) +
                        pow(a.y() - b.y(), 2) +
                        pow(a.z() - b.z(), 2));
    double dist2 = sqrt(pow(a.w() + b.w(), 2) +
                        pow(a.x() + b.x(), 2) +
                        pow(a.y() + b.y(), 2) +
                        pow(a.z() + b.z(), 2));
    return (dist1 < dist2) ? dist1 : dist2;
}

void assertQuaternionEqual(const Quaternion& expected, const Quaternion& actual, double tolerance = 0.01) {
    double dist = quaternionDistance(expected, actual);
    char msg[200];
    snprintf(msg, sizeof(msg), "Quaternion mismatch: expected(%.3f,%.3f,%.3f,%.3f) actual(%.3f,%.3f,%.3f,%.3f) dist=%.3f",
             expected.w(), expected.x(), expected.y(), expected.z(),
             actual.w(), actual.x(), actual.y(), actual.z(), dist);
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(tolerance, dist, msg);
}
void assertVectorEqual(const Vector<3>& expected, const Vector<3>& actual, double tolerance = 0.01) {
    char msg[150];
    snprintf(msg, sizeof(msg), "Vector X: expected %.3f, actual %.3f", expected.x(), actual.x());
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.x(), actual.x(), msg);
    snprintf(msg, sizeof(msg), "Vector Y: expected %.3f, actual %.3f", expected.y(), actual.y());
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.y(), actual.y(), msg);
    snprintf(msg, sizeof(msg), "Vector Z: expected %.3f, actual %.3f", expected.z(), actual.z());
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.z(), actual.z(), msg);
}

void local_setUp(void) {
    // Setup before each test
}

void local_tearDown(void) {
    // Cleanup after each test
}

void collectCalibratableMagSamples(MahonyAHRS& ahrs, int sampleCount = 150) {
    // Use a non-coplanar 3D sweep to avoid degenerate ellipsoid fits.
    for (int i = 0; i < sampleCount; i++) {
        double theta = 2.0 * M_PI * i / static_cast<double>(sampleCount);
        double phi = M_PI * (i % 30) / 30.0;
        Vector<3> mag(cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi));
        ahrs.collectMagCalibrationSample(mag * 50.0);
    }
}

void test_always_ready(void) {
    local_setUp();
    MahonyAHRS ahrs;
    TEST_ASSERT_TRUE(ahrs.isReady());
    local_tearDown();
}

void test_initial_quaternion(void) {
    local_setUp();
    MahonyAHRS ahrs;
    Quaternion q = ahrs.getQuaternion();
    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, q, 0.01);
    local_tearDown();
}

void test_constructor_with_gains(void) {
    local_setUp();
    MahonyAHRS ahrs1(0.5, 0.001);
    MahonyAHRS ahrs2(2.0, 0.01);

    // Should not crash and both should be ready
    TEST_ASSERT_TRUE(ahrs1.isReady());
    TEST_ASSERT_TRUE(ahrs2.isReady());
    local_tearDown();
}

void test_reset(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Run for a while to change state
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
    local_tearDown();
}

void test_set_quaternion(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Create a 90° rotation about X-axis
    double half_angle = M_PI / 4.0;
    Quaternion q_set(cos(half_angle), sin(half_angle), 0.0, 0.0);

    ahrs.setQuaternion(q_set);

    Quaternion q_get = ahrs.getQuaternion();
    assertQuaternionEqual(q_set, q_get, 0.01);
    local_tearDown();
}

void test_board_to_body_orientation_setter_and_getters(void) {
    local_setUp();
    MahonyAHRS ahrs;
    ahrs.setBoardToBodyOrientation(MountingOrientation::ROTATE_90_Z, false);

    MountingTransform transform = ahrs.getBoardToBodyTransform();
    TEST_ASSERT_EQUAL(MountingOrientation::ROTATE_90_Z, transform.getOrientation());

    Quaternion qMount = ahrs.getBoardToBodyQuaternion();
    Vector<3> x(1.0, 0.0, 0.0);
    Vector<3> y(0.0, 1.0, 0.0);
    Vector<3> z(0.0, 0.0, 1.0);

    assertVectorEqual(transform.transform(x), qMount.rotateVector(x), 0.01);
    assertVectorEqual(transform.transform(y), qMount.rotateVector(y), 0.01);
    assertVectorEqual(transform.transform(z), qMount.rotateVector(z), 0.01);

    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, ahrs.getBodyToEarthQuaternion(), 0.01);
    assertQuaternionEqual(qMount, ahrs.getBoardToEarthQuaternion(), 0.01);
    local_tearDown();
}

void test_board_to_body_matrix_setter(void) {
    local_setUp();
    MahonyAHRS ahrs;
    const double rotNeg90Y[9] = {
        0.0, 0.0, 1.0,
        0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0
    };
    ahrs.setBoardToBodyMatrix(rotNeg90Y, false);

    MountingTransform transform = ahrs.getBoardToBodyTransform();
    TEST_ASSERT_EQUAL(MountingOrientation::CUSTOM, transform.getOrientation());

    const double* m = transform.getMatrix();
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-9, rotNeg90Y[i], m[i]);
    }

    Quaternion qMount = ahrs.getBoardToBodyQuaternion();
    Vector<3> x(1.0, 0.0, 0.0);
    Vector<3> y(0.0, 1.0, 0.0);
    Vector<3> z(0.0, 0.0, 1.0);
    assertVectorEqual(transform.transform(x), qMount.rotateVector(x), 0.01);
    assertVectorEqual(transform.transform(y), qMount.rotateVector(y), 0.01);
    assertVectorEqual(transform.transform(z), qMount.rotateVector(z), 0.01);
    local_tearDown();
}

void test_board_to_body_quaternion_setter_normalizes(void) {
    local_setUp();
    MahonyAHRS ahrs;

    double half = M_PI / 4.0;
    Quaternion qUnnormalized(2.0 * cos(half), 0.0, 0.0, 2.0 * sin(half));
    ahrs.setBoardToBodyQuaternion(qUnnormalized, false);

    Quaternion qMount = ahrs.getBoardToBodyQuaternion();
    double norm = sqrt(qMount.w() * qMount.w() + qMount.x() * qMount.x() +
                       qMount.y() * qMount.y() + qMount.z() * qMount.z());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, norm);

    Vector<3> x(1.0, 0.0, 0.0);
    Vector<3> expectedX(0.0, 1.0, 0.0);
    assertVectorEqual(expectedX, qMount.rotateVector(x), 0.01);
    assertVectorEqual(expectedX, ahrs.getBoardToBodyTransform().transform(x), 0.01);
    local_tearDown();
}

void test_set_board_to_body_preserve_earth_true(void) {
    local_setUp();
    MahonyAHRS ahrs;

    double halfYaw = M_PI / 8.0; // 45 deg yaw
    Quaternion qInit(cos(halfYaw), 0.0, 0.0, sin(halfYaw));
    ahrs.setQuaternion(qInit);

    Quaternion boardEarthBefore = ahrs.getBoardToEarthQuaternion();
    ahrs.setBoardToBodyOrientation(MountingOrientation::ROTATE_90_X, true);

    Quaternion boardEarthAfter = ahrs.getBoardToEarthQuaternion();
    assertQuaternionEqual(boardEarthBefore, boardEarthAfter, 0.001);

    Quaternion expectedBodyToEarth = boardEarthBefore * ahrs.getBoardToBodyQuaternion().conjugate();
    assertQuaternionEqual(expectedBodyToEarth, ahrs.getBodyToEarthQuaternion(), 0.001);
    local_tearDown();
}

void test_set_board_to_body_preserve_earth_false(void) {
    local_setUp();
    MahonyAHRS ahrs;

    double halfPitch = M_PI / 12.0; // 30 deg pitch
    Quaternion qInit(cos(halfPitch), 0.0, sin(halfPitch), 0.0);
    ahrs.setQuaternion(qInit);

    Quaternion bodyEarthBefore = ahrs.getBodyToEarthQuaternion();
    Quaternion boardEarthBefore = ahrs.getBoardToEarthQuaternion();
    ahrs.setBoardToBodyOrientation(MountingOrientation::ROTATE_90_X, false);

    assertQuaternionEqual(bodyEarthBefore, ahrs.getBodyToEarthQuaternion(), 0.001);

    Quaternion expectedBoardToEarth = bodyEarthBefore * ahrs.getBoardToBodyQuaternion();
    assertQuaternionEqual(expectedBoardToEarth, ahrs.getBoardToEarthQuaternion(), 0.001);
    double dist = quaternionDistance(boardEarthBefore, ahrs.getBoardToEarthQuaternion());
    TEST_ASSERT_TRUE(dist > 0.05);
    local_tearDown();
}

void test_reset_preserves_board_to_body_transform(void) {
    local_setUp();
    MahonyAHRS ahrs;
    ahrs.setBoardToBodyOrientation(MountingOrientation::ROTATE_NEG90_Z, false);

    Quaternion mountBeforeReset = ahrs.getBoardToBodyQuaternion();
    Quaternion qInit(cos(M_PI / 10.0), sin(M_PI / 10.0), 0.0, 0.0);
    ahrs.setQuaternion(qInit);
    ahrs.reset();

    Quaternion identity(1.0, 0.0, 0.0, 0.0);
    assertQuaternionEqual(identity, ahrs.getBodyToEarthQuaternion(), 0.001);
    assertQuaternionEqual(mountBeforeReset, ahrs.getBoardToBodyQuaternion(), 0.001);
    assertQuaternionEqual(mountBeforeReset, ahrs.getBoardToEarthQuaternion(), 0.001);
    local_tearDown();
}

void test_gyro_update_respects_board_to_body_transform(void) {
    local_setUp();
    MahonyAHRS mounted;
    MahonyAHRS bodyRef;
    MountingTransform mount(MountingOrientation::ROTATE_90_Z);
    mounted.setBoardToBodyOrientation(MountingOrientation::ROTATE_90_Z, false);

    Vector<3> gyroBoard(0.6, -0.2, 0.3);
    Vector<3> gyroBody = mount.transform(gyroBoard);
    double dt = 0.01;

    for (int i = 0; i < 300; i++) {
        mounted.update(gyroBoard, dt);
        bodyRef.update(gyroBody, dt);
    }

    assertQuaternionEqual(bodyRef.getQuaternion(), mounted.getQuaternion(), 0.01);
    local_tearDown();
}

void test_accel_gyro_update_respects_board_to_body_transform(void) {
    local_setUp();
    MahonyAHRS mounted;
    MahonyAHRS bodyRef;
    MountingTransform mount(MountingOrientation::ROTATE_NEG90_Y);
    mounted.setBoardToBodyOrientation(MountingOrientation::ROTATE_NEG90_Y, false);

    Vector<3> accelBoard(1.5, -2.0, 9.5);
    Vector<3> gyroBoard(0.05, -0.03, 0.02);
    Vector<3> accelBody = mount.transform(accelBoard);
    Vector<3> gyroBody = mount.transform(gyroBoard);
    double dt = 0.01;

    for (int i = 0; i < 400; i++) {
        mounted.update(accelBoard, gyroBoard, dt);
        bodyRef.update(accelBody, gyroBody, dt);
    }

    assertQuaternionEqual(bodyRef.getQuaternion(), mounted.getQuaternion(), 0.02);
    local_tearDown();
}

void test_get_earth_acceleration_respects_board_to_body_transform(void) {
    local_setUp();
    MahonyAHRS mounted;
    MahonyAHRS bodyRef;
    MountingTransform mount(MountingOrientation::FLIP_XY);
    mounted.setBoardToBodyOrientation(MountingOrientation::FLIP_XY, false);

    Quaternion qInit(cos(M_PI / 12.0), sin(M_PI / 12.0), 0.0, 0.0); // 30 deg roll
    mounted.setQuaternion(qInit);
    bodyRef.setQuaternion(qInit);

    Vector<3> accelBoard(2.0, -3.0, 11.0);
    Vector<3> accelBody = mount.transform(accelBoard);

    Vector<3> mountedEarth = mounted.getEarthAcceleration(accelBoard);
    Vector<3> bodyEarth = bodyRef.getEarthAcceleration(accelBody);
    assertVectorEqual(bodyEarth, mountedEarth, 0.001);
    local_tearDown();
}

void test_static_orientation_upright(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Gravity pointing down in body frame (sensor upright)
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

    // Inertial frame: gravity points down (-Z), so we expect (0, 0, 9.81)
    // since the measurement is specific force (opposite of gravity)
    Vector<3> expected_inertial(0.0, 0.0, 9.81);

    assertVectorEqual(expected_inertial, gravity_inertial, 0.5);
    local_tearDown();
}

void test_static_orientation_tilted(void) {
    local_setUp();
    MahonyAHRS ahrs(2.0, 0.001); // Higher Kp for faster convergence

    // Sensor tilted 45° on Y-axis
    double tilt_angle = M_PI / 4.0;
    Vector<3> accel_tilted(9.81 * sin(tilt_angle), 0.0, 9.81 * cos(tilt_angle));
    Vector<3> gyro(0.0, 0.0, 0.0);

    // Run for several seconds to converge - increase iterations
    for (int i = 0; i < 1000; i++) {
        ahrs.update(accel_tilted, gyro, 0.01);
    }

    // The filter should align to vertical (specific force should point up in inertial frame)
    Quaternion q = ahrs.getQuaternion();
    Vector<3> gravity_inertial = q.rotateVector(accel_tilted);

    // Should align with vertical in ENU (+Z direction)
    Vector<3> expected(0.0, 0.0, 9.81);
    assertVectorEqual(expected, gravity_inertial, 0.5);
    local_tearDown();
}

void test_gyro_only_no_drift(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Converge to vertical first
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    Quaternion q_initial = ahrs.getQuaternion();

    // Run gyro-only with zero input for 5 seconds
    for (int i = 0; i < 500; i++) {
        ahrs.update(gyro, 0.01);
    }

    Quaternion q_final = ahrs.getQuaternion();

    // Should not have drifted (distance should be small)
    assertQuaternionEqual(q_initial, q_final, 0.05);
    local_tearDown();
}

void test_gyro_rotation_z_axis_90deg(void) {
    local_setUp();
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

    // Use gyro-only mode to avoid accel correction interfering
    for (int i = 0; i < 100; i++) {
        ahrs.update(gyro, dt);
    }

    Quaternion q = ahrs.getQuaternion();

    // Verify we rotated by checking a vector transformation
    Vector<3> x_body(1.0, 0.0, 0.0);
    Vector<3> x_rotated = q.rotateVector(x_body);
    Vector<3> expected(0.0, 1.0, 0.0);  // 90° rotation about Z maps X→Y

    assertVectorEqual(expected, x_rotated, 0.15);
    local_tearDown();
}

void test_gyro_rotation_x_axis_45deg(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Start at identity
    Vector<3> gyro_init(0.0, 0.0, 0.0);

    // Rotate 45 degrees about X-axis over 1 second
    double rotation_rate = M_PI / 4.0;  // 45 deg/s
    Vector<3> gyro(rotation_rate, 0.0, 0.0);
    double dt = 0.01;

    for (int i = 0; i < 100; i++) {
        ahrs.update(gyro, dt);
    }

    Quaternion q = ahrs.getQuaternion();

    // Verify rotation: Y-axis rotates toward Z-axis
    Vector<3> y_body(0.0, 1.0, 0.0);
    Vector<3> y_rotated = q.rotateVector(y_body);

    // 45° rotation about X: Y becomes (0, cos(45°), sin(45°))
    double c = cos(M_PI / 4.0);
    double s = sin(M_PI / 4.0);
    Vector<3> expected(0.0, c, s);

    assertVectorEqual(expected, y_rotated, 0.15);
    local_tearDown();
}

void test_continuous_rotation(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Converge to vertical first
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro_init(0.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro_init, 0.01);
    }

    // Rotate continuously at 45 deg/s for 4 seconds = 180 degrees
    double rotation_rate = M_PI / 4.0;
    Vector<3> gyro(0.0, 0.0, rotation_rate);
    double dt = 0.01;

    for (int i = 0; i < 400; i++) {
        ahrs.update(gyro, dt);
    }

    Quaternion q = ahrs.getQuaternion();

    // After 180°, X should flip to -X, Y should flip to -Y
    Vector<3> x_body(1.0, 0.0, 0.0);
    Vector<3> x_rotated = q.rotateVector(x_body);
    Vector<3> expected_x(-1.0, 0.0, 0.0);

    assertVectorEqual(expected_x, x_rotated, 0.2);
    local_tearDown();
}

void test_full_rotation_360deg(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Start at identity
    Vector<3> gyro_init(0.0, 0.0, 0.0);

    // Rotate 360 degrees about Z-axis over 4 seconds
    double rotation_rate = M_PI / 2.0;  // 90 deg/s
    Vector<3> gyro(0.0, 0.0, rotation_rate);
    double dt = 0.01;

    for (int i = 0; i < 400; i++) {
        ahrs.update(gyro, dt);
    }

    // After 360 degrees, should be back to identity (or -identity)
    Quaternion q = ahrs.getQuaternion();

    // Check X-axis is back to pointing in +X direction
    Vector<3> x_body(1.0, 0.0, 0.0);
    Vector<3> x_rotated = q.rotateVector(x_body);
    Vector<3> expected(1.0, 0.0, 0.0);

    assertVectorEqual(expected, x_rotated, 0.2);
    local_tearDown();
}

void test_accel_correction_from_tilt(void) {
    local_setUp();
    MahonyAHRS ahrs(2.0, 0.001);  // Higher Kp for faster correction

    // Start upright
    Vector<3> accel_upright(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    // Converge to upright
    for (int i = 0; i < 100; i++) {
        ahrs.update(accel_upright, gyro, 0.01);
    }

    // Now tilt the sensor (simulate 45° tilt on Y-axis)
    double tilt_angle = M_PI / 4.0;
    Vector<3> accel_tilted(9.81 * sin(tilt_angle), 0.0, 9.81 * cos(tilt_angle));

    // Run for several seconds to let it converge - increase iterations
    for (int i = 0; i < 1500; i++) {
        ahrs.update(accel_tilted, gyro, 0.01);
    }

    // The filter should align to vertical (specific force should point up in inertial frame)
    Quaternion q = ahrs.getQuaternion();
    Vector<3> gravity_inertial = q.rotateVector(accel_tilted);

    // Should align with vertical in ENU (+Z direction)
    Vector<3> expected(0.0, 0.0, 9.81);
    assertVectorEqual(expected, gravity_inertial, 1.0);
    local_tearDown();
}

void test_accel_correction_ignores_linear_accel(void) {
    local_setUp();
    MahonyAHRS ahrs(0.1, 0.001);  // Normal gains

    // Converge to upright
    Vector<3> accel_static(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_static, gyro, 0.01);
    }

    Quaternion q_before = ahrs.getQuaternion();

    // Add linear acceleration in X (total accel magnitude changes)
    Vector<3> accel_moving(5.0, 0.0, 9.81);

    // With accel correction, this will slightly corrupt the estimate
    // but shouldn't completely destroy it
    for (int i = 0; i < 100; i++) {
        ahrs.update(accel_moving, gyro, 0.01);
    }

    Quaternion q_after = ahrs.getQuaternion();

    // Orientation should not have changed drastically
    // (This tests that the filter is somewhat robust to linear accel)
    // Note: Linear acceleration WILL cause some drift, so use larger tolerance
    assertQuaternionEqual(q_before, q_after, 1.0);
    local_tearDown();
}

void test_earth_frame_acceleration_static(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Converge to upright
    Vector<3> accel_static(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_static, gyro, 0.01);
    }

    // With no linear acceleration (just gravity), earth frame accel should be ~zero
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_static);

    // When stationary:
    // - Body frame: accel = (0,0,9.81) - specific force opposing gravity
    // - Rotate to inertial: (0,0,9.81)
    // - Subtract gravity (0,0,-9.81): result is (0,0,9.81) - (0,0,-9.81) = (0,0,0)
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.z());
    local_tearDown();
}

void test_earth_frame_acceleration_upward(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Converge to upright
    Vector<3> accel_static(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_static, gyro, 0.01);
    }

    // Simulate upward acceleration of 10 m/s²
    // Accelerometer measures specific force: gravity (9.81) + linear accel (10) = 19.81
    Vector<3> accel_moving(0.0, 0.0, 19.81);
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_moving);

    // Body: (0,0,19.81) -> Inertial: (0,0,19.81) -> Subtract 9.81: (0,0,10.0)
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 10.0, earthAccel.z());
    local_tearDown();
}

void test_earth_frame_acceleration_lateral(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Converge to upright
    Vector<3> accel_static(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);

    for (int i = 0; i < 200; i++) {
        ahrs.update(accel_static, gyro, 0.01);
    }

    // Sensor upright, but accelerating sideways (X-direction in body frame)
    // Body measures: 5 m/s² in +X (linear), plus 9.81 in +Z (opposing gravity)
    Vector<3> accel_body(5.0, 0.0, 9.81);
    Vector<3> earthAccel = ahrs.getEarthAcceleration(accel_body);

    // Body: (5,0,9.81) -> Inertial: (5,0,9.81) -> Subtract 9.81 from Z: (5,0,0)
    TEST_ASSERT_FLOAT_WITHIN(0.5, 5.0, earthAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.5, 0.0, earthAccel.z());
    local_tearDown();
}

void test_mag_calibration_status(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Initially not calibrated
    TEST_ASSERT_FALSE(ahrs.isMagCalibrated());

    // Collect some samples (not enough for calibration)
    Vector<3> mag(1.0, 0.0, 0.0);
    for (int i = 0; i < 50; i++) {
        ahrs.collectMagCalibrationSample(mag);
    }

    // Finalize - should not calibrate with too few samples
    ahrs.finalizeMagCalibration();
    TEST_ASSERT_FALSE(ahrs.isMagCalibrated());
    local_tearDown();
}

void test_mag_calibration_sufficient_samples(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Collect 150 non-coplanar samples to ensure ellipsoid fit is solvable.
    collectCalibratableMagSamples(ahrs, 150);

    // Finalize - should calibrate
    ahrs.finalizeMagCalibration();
    TEST_ASSERT_TRUE(ahrs.isMagCalibrated());
    local_tearDown();
}

void test_mag_calibration_persists_after_reset(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Calibrate mag
    collectCalibratableMagSamples(ahrs, 150);
    ahrs.finalizeMagCalibration();

    TEST_ASSERT_TRUE(ahrs.isMagCalibrated());

    // Reset filter
    ahrs.reset();

    // Mag calibration should persist
    TEST_ASSERT_TRUE(ahrs.isMagCalibrated());
    local_tearDown();
}

void test_9dof_update(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Calibrate mag first with better sample distribution
    // Simulate rotating the sensor in 3D space
    for (int i = 0; i < 150; i++) {
        double theta = 2.0 * M_PI * i / 150.0;
        double phi = M_PI * (i % 30) / 30.0;
        Vector<3> mag(cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi));
        mag = mag * 50.0; // Scale to typical magnetometer values
        ahrs.collectMagCalibrationSample(mag);
    }
    ahrs.finalizeMagCalibration();

    // Update with all 9 DOF
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.0, 0.0, 0.0);
    // Mag pointing roughly north in horizontal plane (with some Z component for realism)
    Vector<3> mag(50.0, 0.0, 30.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, mag, 0.01);
    }

    // Should converge to some orientation (not crash or produce NaN)
    Quaternion q = ahrs.getQuaternion();
    double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    TEST_ASSERT_FALSE(isnan(norm));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, norm);
    local_tearDown();
}

void test_zero_dt(void) {
    local_setUp();
    MahonyAHRS ahrs;

    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.1, 0.2, 0.3);

    Quaternion q_before = ahrs.getQuaternion();

    // Update with zero dt - should not change state
    ahrs.update(accel, gyro, 0.0);

    Quaternion q_after = ahrs.getQuaternion();
    assertQuaternionEqual(q_before, q_after, 0.001);
    local_tearDown();
}

void test_large_dt(void) {
    local_setUp();
    MahonyAHRS ahrs;

    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.1, 0.0, 0.0);

    // Update with large dt (1 second)
    ahrs.update(accel, gyro, 1.0);

    // Should not crash or produce NaN
    Quaternion q = ahrs.getQuaternion();
    TEST_ASSERT_FALSE(isnan(q.w()));
    TEST_ASSERT_FALSE(isnan(q.x()));
    TEST_ASSERT_FALSE(isnan(q.y()));
    TEST_ASSERT_FALSE(isnan(q.z()));
    local_tearDown();
}

void test_zero_acceleration(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Zero acceleration (freefall or bad sensor)
    Vector<3> accel(0.0, 0.0, 0.0);
    Vector<3> gyro(0.0, 0.0, 0.0);

    // Should not crash
    for (int i = 0; i < 10; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    Quaternion q = ahrs.getQuaternion();
    TEST_ASSERT_FALSE(isnan(q.w()));
    local_tearDown();
}

void test_high_gyro_rate(void) {
    local_setUp();
    MahonyAHRS ahrs;

    // Very high rotation rate (10 rad/s = 573 deg/s)
    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(10.0, 0.0, 0.0);

    for (int i = 0; i < 100; i++) {
        ahrs.update(accel, gyro, 0.01);
    }

    // Should still maintain normalized quaternion
    Quaternion q = ahrs.getQuaternion();
    double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, norm);
    local_tearDown();
}

void test_quaternion_normalization_maintained(void) {
    local_setUp();
    MahonyAHRS ahrs;

    Vector<3> accel(0.0, 0.0, 9.81);
    Vector<3> gyro(0.5, 0.3, 0.2);

    // Run for extended period
    for (int i = 0; i < 1000; i++) {
        ahrs.update(accel, gyro, 0.01);

        // Check normalization periodically
        if (i % 100 == 0) {
            Quaternion q = ahrs.getQuaternion();
            double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
            TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, norm);
        }
    }
    local_tearDown();
}

void run_test_mahony_tests()
{
    RUN_TEST(test_always_ready);
    RUN_TEST(test_initial_quaternion);
    RUN_TEST(test_constructor_with_gains);
    RUN_TEST(test_reset);
    RUN_TEST(test_set_quaternion);
    RUN_TEST(test_board_to_body_orientation_setter_and_getters);
    RUN_TEST(test_board_to_body_matrix_setter);
    RUN_TEST(test_board_to_body_quaternion_setter_normalizes);
    RUN_TEST(test_set_board_to_body_preserve_earth_true);
    RUN_TEST(test_set_board_to_body_preserve_earth_false);
    RUN_TEST(test_reset_preserves_board_to_body_transform);
    RUN_TEST(test_gyro_update_respects_board_to_body_transform);
    RUN_TEST(test_accel_gyro_update_respects_board_to_body_transform);
    RUN_TEST(test_get_earth_acceleration_respects_board_to_body_transform);
    RUN_TEST(test_static_orientation_upright);
    RUN_TEST(test_static_orientation_tilted);
    RUN_TEST(test_gyro_only_no_drift);
    RUN_TEST(test_gyro_rotation_z_axis_90deg);
    RUN_TEST(test_gyro_rotation_x_axis_45deg);
    RUN_TEST(test_continuous_rotation);
    RUN_TEST(test_full_rotation_360deg);
    RUN_TEST(test_accel_correction_from_tilt);
    RUN_TEST(test_accel_correction_ignores_linear_accel);
    RUN_TEST(test_earth_frame_acceleration_static);
    RUN_TEST(test_earth_frame_acceleration_upward);
    RUN_TEST(test_earth_frame_acceleration_lateral);
    RUN_TEST(test_mag_calibration_status);
    RUN_TEST(test_mag_calibration_sufficient_samples);
    RUN_TEST(test_mag_calibration_persists_after_reset);
    RUN_TEST(test_9dof_update);
    RUN_TEST(test_zero_dt);
    RUN_TEST(test_large_dt);
    RUN_TEST(test_zero_acceleration);
    RUN_TEST(test_high_gyro_rate);
    RUN_TEST(test_quaternion_normalization_maintained);
}

} // namespace test_mahony
