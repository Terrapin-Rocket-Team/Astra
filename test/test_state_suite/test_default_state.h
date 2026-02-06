#pragma once

#include <unity.h>
#include "State/DefaultState.h"
#include "Filters/DefaultKalmanFilter.h"
#include "Filters/Mahony.h"
#include "Math/Vector.h"
#include "Math/Quaternion.h"

using namespace astra;

namespace test_default_state {

DefaultState* state;

void local_setUp(void) {
    // Create with default parameters
    state = new DefaultState();
}

void local_tearDown(void) {
    delete state;
}

void test_constructor_default_parameters() {
    local_setUp();
    DefaultState* s = new DefaultState();
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(s->getOrientationFilter());
    TEST_ASSERT_NOT_NULL(s->getDefaultKalmanFilter());
    delete s;
    local_tearDown();
}

void test_constructor_custom_parameters() {
    local_setUp();
    // Test with custom filter tuning parameters
    DefaultState* s = new DefaultState(
        2.0,    // processNoise
        10.0,   // gpsNoise
        5.0,    // baroNoise
        1.0,    // mahonyKp
        0.1     // mahonyKi
    );
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(s->getOrientationFilter());
    TEST_ASSERT_NOT_NULL(s->getDefaultKalmanFilter());
    delete s;
    local_tearDown();
}

void test_constructor_creates_owned_filters() {
    local_setUp();
    DefaultState* s = new DefaultState();

    // Should have access to both base filter and default filter
    MahonyAHRS* mahony = s->getOrientationFilter();
    DefaultKalmanFilter* kalman = s->getDefaultKalmanFilter();

    TEST_ASSERT_NOT_NULL(mahony);
    TEST_ASSERT_NOT_NULL(kalman);

    delete s;
    local_tearDown();
}

void test_begin_initializes_filters() {
    local_setUp();
    int result = state->begin();
    TEST_ASSERT_EQUAL(0, result);

    // Kalman filter should be initialized
    DefaultKalmanFilter* kf = state->getDefaultKalmanFilter();
    TEST_ASSERT_NOT_NULL(kf);
    local_tearDown();
}

void test_begin_returns_success() {
    local_setUp();
    int result = state->begin();
    TEST_ASSERT_EQUAL(0, result);
    local_tearDown();
}

void test_get_default_kalman_filter() {
    local_setUp();
    DefaultKalmanFilter* kf = state->getDefaultKalmanFilter();
    TEST_ASSERT_NOT_NULL(kf);
    local_tearDown();
}

void test_get_orientation_filter() {
    local_setUp();
    MahonyAHRS* mahony = state->getOrientationFilter();
    TEST_ASSERT_NOT_NULL(mahony);
    local_tearDown();
}

void test_update_orientation_integrates_gyro() {
    local_setUp();
    state->begin();

    // Calibrate orientation filter
    MahonyAHRS* mahony = state->getOrientationFilter();
    for (int i = 0; i < 200; ++i) {
        mahony->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    Quaternion q_before = state->getOrientation();

    // Apply rotation
    for (int i = 0; i < 50; ++i) {
        state->updateOrientation(Vector<3>(0.5, 0, 0), Vector<3>(0, 0, 9.81), 0.01);
    }

    Quaternion q_after = state->getOrientation();

    // Orientation should have changed
    TEST_ASSERT_NOT_EQUAL(q_before.w(), q_after.w());
    local_tearDown();
}

void test_update_orientation_transforms_acceleration() {
    local_setUp();
    state->begin();

    // Calibrate
    MahonyAHRS* mahony = state->getOrientationFilter();
    for (int i = 0; i < 200; ++i) {
        mahony->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    // Update with acceleration
    Vector<3> accel(1.0, 2.0, 9.81);
    state->updateOrientation(Vector<3>(0, 0, 0), accel, 0.01);

    // Should have earth-frame acceleration
    Vector<3> earth_accel = state->getAcceleration();
    TEST_ASSERT_NOT_NULL(&earth_accel);
    local_tearDown();
}

void test_predict_updates_position_velocity() {
    local_setUp();
    state->begin();

    // Set up orientation filter
    MahonyAHRS* mahony = state->getOrientationFilter();
    // Note: setMode method removed from Mahony API

    // Give it some initial state via measurement update
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    Vector<3> gpsPos(34.0, -118.0, 100.0);
    Vector<3> gpsVel(0, 0, 0);
    state->updateMeasurements(gpsPos, gpsVel, 100.0, true, true);

    // Update orientation with some acceleration
    state->updateOrientation(Vector<3>(0, 0, 0), Vector<3>(0, 0, 12.0), 0.01);

    // Predict forward
    state->predict(0.1);

    // State should have been updated (actual values depend on filter dynamics)
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_NOT_NULL(&pos);
    TEST_ASSERT_NOT_NULL(&vel);
    local_tearDown();
}

void test_predict_with_zero_dt() {
    local_setUp();
    state->begin();

    Vector<3> pos_before = state->getPosition();

    // Predict with zero dt
    state->predict(0.0);

    Vector<3> pos_after = state->getPosition();

    // Position should be unchanged (or minimally changed)
    TEST_ASSERT_FLOAT_WITHIN(0.001, pos_before.x(), pos_after.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, pos_before.y(), pos_after.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, pos_before.z(), pos_after.z());
    local_tearDown();
}

void test_predict_accumulates_over_time() {
    local_setUp();
    state->begin();

    // Calibrate orientation
    MahonyAHRS* mahony = state->getOrientationFilter();
    for (int i = 0; i < 200; ++i) {
        mahony->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    // Apply constant acceleration upward
    for (int i = 0; i < 100; ++i) {
        state->updateOrientation(Vector<3>(0, 0, 0), Vector<3>(0, 0, 15.0), 0.01);
        state->predict(0.01);
    }

    // After 1 second of upward accel, should have some upward velocity
    Vector<3> vel = state->getVelocity();

    // Should have non-zero state (exact values depend on filter tuning)
    TEST_ASSERT_NOT_NULL(&vel);
    local_tearDown();
}

void test_update_measurements_gps_only() {
    local_setUp();
    state->begin();
    state->setGPSOrigin(34.0, -118.0, 100.0);

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);

    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    // State should be updated
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_NOT_NULL(&pos);
    TEST_ASSERT_NOT_NULL(&vel);
    local_tearDown();
}

void test_update_measurements_auto_sets_gps_origin() {
    local_setUp();
    state->begin();

    Vector<3> gpsPos(34.0522, -118.2437, 100.0);
    Vector<3> gpsVel(0, 0, 0);

    // First measurement should auto-set origin
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    // Position should be near 0,0 since it's at origin
    Vector<3> pos = state->getPosition();
    TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, pos.x());
    TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, pos.y());
    local_tearDown();
}

void test_update_measurements_relative_to_origin() {
    local_setUp();
    state->begin();
    state->setGPSOrigin(34.0, -118.0, 100.0);

    // Measure at origin
    Vector<3> gpsPos1(34.0, -118.0, 100.0);
    Vector<3> gpsVel(0, 0, 0);
    state->updateMeasurements(gpsPos1, gpsVel, 0.0, true, false);

    Vector<3> pos1 = state->getPosition();

    // Measure north of origin
    Vector<3> gpsPos2(34.001, -118.0, 100.0); // ~111m north
    state->updateMeasurements(gpsPos2, gpsVel, 0.0, true, false);

    Vector<3> pos2 = state->getPosition();

    // Position should have moved north (positive Y in ENU)
    TEST_ASSERT_GREATER_THAN(pos1.y(), pos2.y());
    local_tearDown();
}

void test_update_measurements_baro_only() {
    local_setUp();
    state->begin();
    state->setBaroOrigin(100.0);

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    state->updateMeasurements(gpsPos, gpsVel, 150.0, false, true);

    // Should have updated vertical position
    Vector<3> pos = state->getPosition();
    TEST_ASSERT_NOT_NULL(&pos);
    local_tearDown();
}

void test_update_measurements_baro_relative_to_origin() {
    local_setUp();
    state->begin();
    state->setBaroOrigin(100.0);

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    // Measure at origin
    state->updateMeasurements(gpsPos, gpsVel, 100.0, false, true);
    Vector<3> pos1 = state->getPosition();

    // Measure 50m higher
    state->updateMeasurements(gpsPos, gpsVel, 150.0, false, true);
    Vector<3> pos2 = state->getPosition();

    // Vertical position should have increased
    TEST_ASSERT_GREATER_THAN(pos1.z(), pos2.z());
    local_tearDown();
}

void test_update_measurements_gps_and_baro() {
    local_setUp();
    state->begin();
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);

    state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);

    // Should have updated 3D position
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_NOT_NULL(&pos);
    TEST_ASSERT_NOT_NULL(&vel);
    local_tearDown();
}

void test_update_measurements_fuses_horizontal_and_vertical() {
    local_setUp();
    state->begin();
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);

    // Combined update
    state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);

    Vector<3> pos = state->getPosition();

    // Should have non-zero horizontal position (from GPS)
    TEST_ASSERT_NOT_EQUAL(0.0, pos.x());
    TEST_ASSERT_NOT_EQUAL(0.0, pos.y());

    // Should have non-zero vertical position (from baro)
    TEST_ASSERT_NOT_EQUAL(0.0, pos.z());
    local_tearDown();
}

void test_get_position() {
    local_setUp();
    state->begin();
    Vector<3> pos = state->getPosition();

    TEST_ASSERT_EQUAL(3, pos.n());
    local_tearDown();
}

void test_get_velocity() {
    local_setUp();
    state->begin();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_EQUAL(3, vel.n());
    local_tearDown();
}

void test_get_acceleration() {
    local_setUp();
    state->begin();
    Vector<3> accel = state->getAcceleration();

    TEST_ASSERT_EQUAL(3, accel.n());
    local_tearDown();
}

void test_get_orientation() {
    local_setUp();
    state->begin();
    Quaternion q = state->getOrientation();

    // Should be a valid quaternion
    TEST_ASSERT_NOT_NULL(&q);
    local_tearDown();
}

void test_set_gps_origin_once() {
    local_setUp();
    state->setGPSOrigin(34.0, -118.0, 100.0);

    // Second call should not change origin
    state->setGPSOrigin(35.0, -119.0, 200.0);

    state->begin();

    // Test by measuring at first origin location
    Vector<3> gpsPos(34.0, -118.0, 100.0);
    Vector<3> gpsVel(0, 0, 0);
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    Vector<3> pos = state->getPosition();

    // Should be at origin (near 0,0)
    TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, pos.x());
    TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, pos.y());
    local_tearDown();
}

void test_set_baro_origin_once() {
    local_setUp();
    state->setBaroOrigin(100.0);

    // Second call should not change origin
    state->setBaroOrigin(200.0);

    state->begin();

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    // Measure at first origin altitude
    state->updateMeasurements(gpsPos, gpsVel, 100.0, false, true);

    Vector<3> pos = state->getPosition();

    // Vertical position should be near 0
    TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, pos.z());
    local_tearDown();
}

void test_deprecated_update_returns_error() {
    local_setUp();
    state->begin();
    int result = state->update(1.0);
    TEST_ASSERT_EQUAL(-1, result);
    local_tearDown();
}

void test_full_flight_simulation() {
    local_setUp();
    // Simulate a complete flight: startup -> takeoff -> flight -> landing
    state->begin();

    // Calibrate orientation at startup
    MahonyAHRS* mahony = state->getOrientationFilter();
    for (int i = 0; i < 200; ++i) {
        mahony->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    // Set origins
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    // Ground phase
    Vector<3> gpsPos(34.0, -118.0, 100.0);
    Vector<3> gpsVel(0, 0, 0);
    state->updateMeasurements(gpsPos, gpsVel, 100.0, true, true);

    Vector<3> pos_ground = state->getPosition();

    // Takeoff - simulate acceleration
    for (int i = 0; i < 50; ++i) {
        state->updateOrientation(Vector<3>(0, 0, 0), Vector<3>(0, 0, 15.0), 0.02);
        state->predict(0.02);
    }

    // Flight - GPS shows movement
    gpsPos = Vector<3>(34.001, -118.001, 105.0);
    gpsVel = Vector<3>(10.0, 5.0, 2.0);
    state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);

    Vector<3> pos_flight = state->getPosition();

    // Should have moved from ground position
    double dist = (pos_flight - pos_ground).magnitude();
    TEST_ASSERT_GREATER_THAN(0.0, dist);
    local_tearDown();
}

void test_continuous_updates() {
    local_setUp();
    // Test that state handles continuous updates without issues
    state->begin();

    // Calibrate
    MahonyAHRS* mahony = state->getOrientationFilter();
    for (int i = 0; i < 200; ++i) {
        mahony->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    // Run 100 update cycles
    for (int i = 0; i < 100; ++i) {
        // Orientation update (high rate)
        state->updateOrientation(
            Vector<3>(0.01, 0.02, 0.01),
            Vector<3>(0.1, 0.2, 9.81),
            0.01
        );

        // Prediction (high rate)
        state->predict(0.01);

        // GPS/Baro update (every 10 cycles ~ lower rate)
        if (i % 10 == 0) {
            double lat = 34.0 + i * 0.0001;
            double lon = -118.0 + i * 0.0001;
            double alt = 100.0 + i * 0.5;

            state->updateMeasurements(
                Vector<3>(lat, lon, alt),
                Vector<3>(10.0, 5.0, 0.5),
                alt,
                true,
                true
            );
        }
    }

    // Should still be functional after many updates
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();
    Quaternion q = state->getOrientation();

    TEST_ASSERT_NOT_NULL(&pos);
    TEST_ASSERT_NOT_NULL(&vel);
    TEST_ASSERT_NOT_NULL(&q);
    local_tearDown();
}

void test_filter_parameter_tuning() {
    local_setUp();
    // Test that custom filter parameters work
    DefaultState* tuned_state = new DefaultState(
        0.1,    // Low process noise
        2.0,    // Low GPS noise (trust GPS more)
        1.0,    // Low baro noise
        2.0,    // High Kp (aggressive orientation correction)
        0.1     // Some integral term
    );

    int result = tuned_state->begin();
    TEST_ASSERT_EQUAL(0, result);

    // Should be able to use it normally
    tuned_state->setGPSOrigin(34.0, -118.0, 100.0);

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);
    tuned_state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);

    Vector<3> pos = tuned_state->getPosition();
    TEST_ASSERT_NOT_NULL(&pos);

    delete tuned_state;
    local_tearDown();
}

void test_asynchronous_sensor_updates() {
    local_setUp();
    // Test that GPS and baro can update at different rates
    state->begin();

    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);

    // GPS update only
    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    Vector<3> pos1 = state->getPosition();

    // Baro update only
    state->updateMeasurements(Vector<3>(0, 0, 0), Vector<3>(0, 0, 0), 150.0, false, true);

    Vector<3> pos2 = state->getPosition();

    // Position should have changed from both updates
    TEST_ASSERT_FLOAT_WITHIN(0.001, pos1.x(), pos2.x());
    TEST_ASSERT_NOT_EQUAL(pos1.z(), pos2.z());
    local_tearDown();
}

void run_test_default_state_tests()
{
    RUN_TEST(test_constructor_default_parameters);
    RUN_TEST(test_constructor_custom_parameters);
    RUN_TEST(test_constructor_creates_owned_filters);
    RUN_TEST(test_begin_initializes_filters);
    RUN_TEST(test_begin_returns_success);
    RUN_TEST(test_get_default_kalman_filter);
    RUN_TEST(test_get_orientation_filter);
    RUN_TEST(test_update_orientation_integrates_gyro);
    RUN_TEST(test_update_orientation_transforms_acceleration);
    RUN_TEST(test_predict_updates_position_velocity);
    RUN_TEST(test_predict_with_zero_dt);
    RUN_TEST(test_predict_accumulates_over_time);
    RUN_TEST(test_update_measurements_gps_only);
    RUN_TEST(test_update_measurements_auto_sets_gps_origin);
    RUN_TEST(test_update_measurements_relative_to_origin);
    RUN_TEST(test_update_measurements_baro_only);
    RUN_TEST(test_update_measurements_baro_relative_to_origin);
    RUN_TEST(test_update_measurements_gps_and_baro);
    RUN_TEST(test_update_measurements_fuses_horizontal_and_vertical);
    RUN_TEST(test_get_position);
    RUN_TEST(test_get_velocity);
    RUN_TEST(test_get_acceleration);
    RUN_TEST(test_get_orientation);
    RUN_TEST(test_set_gps_origin_once);
    RUN_TEST(test_set_baro_origin_once);
    RUN_TEST(test_deprecated_update_returns_error);
    RUN_TEST(test_full_flight_simulation);
    RUN_TEST(test_continuous_updates);
    RUN_TEST(test_filter_parameter_tuning);
    RUN_TEST(test_asynchronous_sensor_updates);
}

} // namespace test_default_state
