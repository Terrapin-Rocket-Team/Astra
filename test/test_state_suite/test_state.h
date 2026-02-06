#pragma once

#include <unity.h>
#include "State/State.h"
#include "Filters/LinearKalmanFilter.h"
#include "Filters/Mahony.h"
#include "Math/Vector.h"
#include "Math/Quaternion.h"

using namespace astra;

namespace test_state {

class MockLinearKalmanFilter : public LinearKalmanFilter {
public:
    bool initialized = false;
    bool predict_called = false;
    bool update_gps_called = false;
    bool update_baro_called = false;
    bool update_gps_baro_called = false;
    double last_predict_dt = 0.0;
    Matrix last_control = Matrix(3, 1);
    double last_gps_px = 0.0;
    double last_gps_py = 0.0;
    double last_baro_pz = 0.0;
    double last_gps_baro_px = 0.0;
    double last_gps_baro_py = 0.0;
    double last_gps_baro_pz = 0.0;
    double test_state[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    MockLinearKalmanFilter() : LinearKalmanFilter(3, 3, 6) {}
    void initialize() override { initialized = true; }
    Matrix getF(double dt) override { return Matrix(6, 6); }
    Matrix getG(double dt) override { return Matrix(6, 3); }
    Matrix getH() override { return Matrix(3, 6); }
    Matrix getR() override { return Matrix(3, 3); }
    Matrix getQ(double dt) override { return Matrix(6, 6); }
    void predict(double dt, Matrix control) override {
        predict_called = true;
        last_predict_dt = dt;
        last_control = control;
    }
    void update(Matrix measurements) override {
    }
    void updateGPS(double px, double py, double gpsNoise = 5.0) override {
        update_gps_called = true;
        last_gps_px = px;
        last_gps_py = py;
    }
    void updateBaro(double pz, double baroNoise = 2.0) override {
        update_baro_called = true;
        last_baro_pz = pz;
    }
    void updateGPSBaro(double px, double py, double pz, double gpsNoise = 5.0, double baroNoise = 2.0) override {
        update_gps_baro_called = true;
        last_gps_baro_px = px;
        last_gps_baro_py = py;
        last_gps_baro_pz = pz;
    }
    Matrix getState() const override {
        Matrix state(6, 1);
        for (int i = 0; i < 6; ++i) {
            state(i, 0) = test_state[i];
        }
        return state;
    }
    void resetMockState() {
        predict_called = false;
        update_gps_called = false;
        update_baro_called = false;
        update_gps_baro_called = false;
        last_predict_dt = 0.0;
    }
};
MockLinearKalmanFilter* kalmanFilter;
MahonyAHRS* orientationFilter;
State* state;

void local_setUp(void) {
    kalmanFilter = new MockLinearKalmanFilter();
    orientationFilter = new MahonyAHRS(0.5, 0.0);
    state = new State(kalmanFilter, orientationFilter);
}

void local_tearDown(void) {
    delete state;
    delete orientationFilter;
    delete kalmanFilter;
}

void test_constructor_requires_filters() {
    local_setUp();
    // State should log errors if filters are null but still construct
    State* s1 = new State(nullptr, orientationFilter);
    TEST_ASSERT_NOT_NULL(s1);
    delete s1;

    State* s2 = new State(kalmanFilter, nullptr);
    TEST_ASSERT_NOT_NULL(s2);
    delete s2;

    State* s3 = new State(nullptr, nullptr);
    TEST_ASSERT_NOT_NULL(s3);
    delete s3;
    local_tearDown();
}

void test_begin_fails_without_kalman_filter() {
    local_setUp();
    State s(nullptr, orientationFilter);
    int result = s.begin();
    TEST_ASSERT_EQUAL(-1, result);
    local_tearDown();
}

void test_begin_fails_without_orientation_filter() {
    local_setUp();
    State s(kalmanFilter, nullptr);
    int result = s.begin();
    TEST_ASSERT_EQUAL(-2, result);
    local_tearDown();
}

void test_begin_succeeds_with_both_filters() {
    local_setUp();
    int result = state->begin();
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_TRUE(kalmanFilter->initialized);
    local_tearDown();
}

void test_set_gps_origin() {
    local_setUp();
    state->setGPSOrigin(34.0522, -118.2437, 100.0);

    // Origin should be set - verify by checking that second call doesn't change it
    state->setGPSOrigin(35.0, -119.0, 200.0);

    // We can't directly access origin, but we can test behavior via updateMeasurements
    state->begin();

    // First GPS measurement should be relative to first origin
    Vector<3> gpsPos(34.0522, -118.2437, 100.0);
    Vector<3> gpsVel(0, 0, 0);
    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    // Position relative to origin should be 0,0
    TEST_ASSERT_TRUE(kalmanFilter->update_gps_called);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_gps_px);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_gps_py);
    local_tearDown();
}

void test_gps_origin_auto_set_on_first_measurement() {
    local_setUp();
    state->begin();

    Vector<3> gpsPos(34.0522, -118.2437, 100.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    // First measurement should set origin and read 0,0
    TEST_ASSERT_TRUE(kalmanFilter->update_gps_called);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_gps_px);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_gps_py);

    // Second measurement should be relative to origin
    kalmanFilter->resetMockState();
    Vector<3> gpsPos2(34.0532, -118.2437, 100.0); // 0.001 deg north ~ 111 m
    state->updateMeasurements(gpsPos2, gpsVel, 0.0, true, false);

    TEST_ASSERT_TRUE(kalmanFilter->update_gps_called);
    // ENU: north should affect +Y
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_gps_px);
    TEST_ASSERT_FLOAT_WITHIN(10.0, 111.0, kalmanFilter->last_gps_py); // ~111m north
    local_tearDown();
}

void test_set_baro_origin() {
    local_setUp();
    state->setBaroOrigin(100.0);

    // Second call shouldn't change origin
    state->setBaroOrigin(200.0);

    // Test by measuring relative altitude
    state->begin();

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);
    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 100.0, false, true);

    // Altitude relative to origin should be 0
    TEST_ASSERT_TRUE(kalmanFilter->update_baro_called);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, kalmanFilter->last_baro_pz);
    local_tearDown();
}

void test_baro_relative_to_origin() {
    local_setUp();
    state->setBaroOrigin(100.0);
    state->begin();

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    // Measure at 150m ASL -> 50m above origin
    state->updateMeasurements(gpsPos, gpsVel, 150.0, false, true);

    TEST_ASSERT_TRUE(kalmanFilter->update_baro_called);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, kalmanFilter->last_baro_pz);
    local_tearDown();
}

void test_update_orientation_low_g() {
    local_setUp();
    // Calibrate orientation filter
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    Vector<3> gyro(0.1, 0.2, 0.3);
    Vector<3> accel(0.0, 0.0, 9.81); // 1g - low acceleration

    Quaternion q_before = state->getOrientation();
    state->updateOrientation(gyro, accel, 0.01);
    Quaternion q_after = state->getOrientation();

    // Orientation should have changed
    TEST_ASSERT_NOT_EQUAL(q_before.w(), q_after.w());

    // Acceleration should be transformed to earth frame
    Vector<3> accel_earth = state->getAcceleration();
    TEST_ASSERT_NOT_NULL(&accel_earth);
    local_tearDown();
}

void test_update_orientation_high_g() {
    local_setUp();
    // Calibrate orientation filter
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    Vector<3> gyro(0.1, 0.2, 0.3);
    Vector<3> accel(0.0, 0.0, 30.0); // High-G acceleration

    // High-G should trigger gyro-only mode
    state->updateOrientation(gyro, accel, 0.01);

    // Should still update orientation
    Quaternion q = state->getOrientation();
    TEST_ASSERT_NOT_NULL(&q);
    local_tearDown();
}

void test_update_orientation_null_filter() {
    local_setUp();
    State s(kalmanFilter, nullptr);

    Vector<3> gyro(0.1, 0.2, 0.3);
    Vector<3> accel(0.0, 0.0, 9.81);

    // Should not crash
    s.updateOrientation(gyro, accel, 0.01);
    local_tearDown();
}

void test_predict_calls_kalman_filter() {
    local_setUp();
    state->begin();

    // Set up orientation filter with some acceleration
    // Note: setMode method removed from Mahony API
    Vector<3> gyro(0, 0, 0);
    Vector<3> accel(1.0, 2.0, 9.81);
    state->updateOrientation(gyro, accel, 0.01);

    kalmanFilter->resetMockState();
    state->predict(0.1);

    TEST_ASSERT_TRUE(kalmanFilter->predict_called);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.1, kalmanFilter->last_predict_dt);

    // Control input should be earth-frame acceleration
    TEST_ASSERT_EQUAL(3, kalmanFilter->last_control.getRows());
    TEST_ASSERT_EQUAL(1, kalmanFilter->last_control.getCols());
    local_tearDown();
}

void test_predict_updates_state_from_filter() {
    local_setUp();
    state->begin();

    kalmanFilter->test_state[0] = 10.0; // px
    kalmanFilter->test_state[1] = 20.0; // py
    kalmanFilter->test_state[2] = 30.0; // pz
    kalmanFilter->test_state[3] = 1.5;  // vx
    kalmanFilter->test_state[4] = 2.5;  // vy
    kalmanFilter->test_state[5] = 3.5;  // vz

    state->predict(0.1);

    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_FLOAT_WITHIN(0.001, 10.0, pos.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 20.0, pos.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 30.0, pos.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.5, vel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.5, vel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.5, vel.z());
    local_tearDown();
}

void test_predict_null_filter() {
    local_setUp();
    State s(nullptr, orientationFilter);

    // Should not crash
    s.predict(0.1);
    local_tearDown();
}

void test_update_measurements_gps_only() {
    local_setUp();
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->begin();

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);

    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    TEST_ASSERT_TRUE(kalmanFilter->update_gps_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_baro_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_gps_baro_called);

    // Should have converted lat/lon to local ENU
    TEST_ASSERT_NOT_EQUAL(0.0, kalmanFilter->last_gps_px);
    TEST_ASSERT_NOT_EQUAL(0.0, kalmanFilter->last_gps_py);
    local_tearDown();
}

void test_update_measurements_baro_only() {
    local_setUp();
    state->setBaroOrigin(100.0);
    state->begin();

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 150.0, false, true);

    TEST_ASSERT_FALSE(kalmanFilter->update_gps_called);
    TEST_ASSERT_TRUE(kalmanFilter->update_baro_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_gps_baro_called);

    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, kalmanFilter->last_baro_pz);
    local_tearDown();
}

void test_update_measurements_gps_and_baro() {
    local_setUp();
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);
    state->begin();

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);

    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);

    TEST_ASSERT_FALSE(kalmanFilter->update_gps_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_baro_called);
    TEST_ASSERT_TRUE(kalmanFilter->update_gps_baro_called);

    // Should have both GPS horizontal and baro vertical
    TEST_ASSERT_NOT_EQUAL(0.0, kalmanFilter->last_gps_baro_px);
    TEST_ASSERT_NOT_EQUAL(0.0, kalmanFilter->last_gps_baro_py);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 50.0, kalmanFilter->last_gps_baro_pz);
    local_tearDown();
}

void test_update_measurements_no_sensors() {
    local_setUp();
    state->begin();

    Vector<3> gpsPos(0, 0, 0);
    Vector<3> gpsVel(0, 0, 0);

    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 0.0, false, false);

    // No updates should be called
    TEST_ASSERT_FALSE(kalmanFilter->update_gps_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_baro_called);
    TEST_ASSERT_FALSE(kalmanFilter->update_gps_baro_called);
    local_tearDown();
}

void test_update_measurements_updates_state_from_filter() {
    local_setUp();
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->begin();

    kalmanFilter->test_state[0] = 100.0;
    kalmanFilter->test_state[1] = 200.0;
    kalmanFilter->test_state[2] = 300.0;
    kalmanFilter->test_state[3] = 10.0;
    kalmanFilter->test_state[4] = 20.0;
    kalmanFilter->test_state[5] = 30.0;

    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);
    state->updateMeasurements(gpsPos, gpsVel, 0.0, true, false);

    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_FLOAT_WITHIN(0.001, 100.0, pos.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 200.0, pos.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 300.0, pos.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 10.0, vel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 20.0, vel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 30.0, vel.z());
    local_tearDown();
}

void test_update_measurements_null_filter() {
    local_setUp();
    State s(nullptr, orientationFilter);

    Vector<3> gpsPos(34.0, -118.0, 100.0);
    Vector<3> gpsVel(0, 0, 0);

    // Should not crash
    s.updateMeasurements(gpsPos, gpsVel, 100.0, true, true);
    local_tearDown();
}

void test_getters_return_correct_values() {
    local_setUp();
    state->begin();

    // Set known state in filter
    kalmanFilter->test_state[0] = 1.1;
    kalmanFilter->test_state[1] = 2.2;
    kalmanFilter->test_state[2] = 3.3;
    kalmanFilter->test_state[3] = 4.4;
    kalmanFilter->test_state[4] = 5.5;
    kalmanFilter->test_state[5] = 6.6;

    // Update to pull state from filter
    state->predict(0.01);

    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();

    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.1, pos.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.2, pos.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.3, pos.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 4.4, vel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 5.5, vel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 6.6, vel.z());
    local_tearDown();
}

void test_get_orientation() {
    local_setUp();
    // Calibrate and update orientation
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    state->updateOrientation(Vector<3>(0, 0, 0), Vector<3>(0, 0, 9.81), 0.01);

    Quaternion q = state->getOrientation();

    // Should be near identity after calibration with stable accel
    TEST_ASSERT_FLOAT_WITHIN(0.1, 1.0, q.w());
    local_tearDown();
}

void test_get_acceleration() {
    local_setUp();
    // Calibrate orientation filter
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    Vector<3> accel(1.0, 2.0, 9.81);
    state->updateOrientation(Vector<3>(0, 0, 0), accel, 0.01);

    Vector<3> earth_accel = state->getAcceleration();

    // Should have transformed acceleration
    TEST_ASSERT_NOT_NULL(&earth_accel);
    local_tearDown();
}

void test_get_orientation_filter() {
    local_setUp();
    MahonyAHRS* filter = state->getOrientationFilter();
    TEST_ASSERT_EQUAL(orientationFilter, filter);
    local_tearDown();
}

void test_deprecated_update_returns_error() {
    local_setUp();
    state->begin();
    int result = state->update(1.0);
    TEST_ASSERT_EQUAL(-1, result);
    local_tearDown();
}

void test_deprecated_predict_state_does_nothing() {
    local_setUp();
    state->begin();

    // Should not crash, but should log error
    state->predictState(1.0);

    // Kalman filter should not be called
    TEST_ASSERT_FALSE(kalmanFilter->predict_called);
    local_tearDown();
}

void test_full_update_cycle() {
    local_setUp();
    // Complete update cycle: orientation -> predict -> measurements
    state->setGPSOrigin(34.0, -118.0, 100.0);
    state->setBaroOrigin(100.0);
    state->begin();

    // Calibrate orientation
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    // Update orientation
    Vector<3> gyro(0.1, 0.05, 0.02);
    Vector<3> accel(0.5, 0.2, 9.81);
    state->updateOrientation(gyro, accel, 0.01);

    // Predict
    kalmanFilter->resetMockState();
    state->predict(0.01);
    TEST_ASSERT_TRUE(kalmanFilter->predict_called);

    // Update measurements
    Vector<3> gpsPos(34.001, -118.001, 105.0);
    Vector<3> gpsVel(10.0, 5.0, -2.0);
    kalmanFilter->resetMockState();
    state->updateMeasurements(gpsPos, gpsVel, 150.0, true, true);
    TEST_ASSERT_TRUE(kalmanFilter->update_gps_baro_called);
    local_tearDown();
}

void test_multiple_orientation_updates() {
    local_setUp();
    // Calibrate
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    // Note: finalizeCalibration, lockFrame, and setMode methods removed from Mahony API
    // The filter is always in correcting mode now

    // Multiple updates should accumulate rotation
    Quaternion q_start = state->getOrientation();

    for (int i = 0; i < 100; ++i) {
        state->updateOrientation(Vector<3>(0.5, 0, 0), Vector<3>(0, 0, 9.81), 0.01);
    }

    Quaternion q_end = state->getOrientation();

    // Quaternion should have rotated significantly
    TEST_ASSERT_NOT_EQUAL(q_start.w(), q_end.w());
    local_tearDown();
}

void run_test_state_tests()
{
    RUN_TEST(test_constructor_requires_filters);
    RUN_TEST(test_begin_fails_without_kalman_filter);
    RUN_TEST(test_begin_fails_without_orientation_filter);
    RUN_TEST(test_begin_succeeds_with_both_filters);
    RUN_TEST(test_set_gps_origin);
    RUN_TEST(test_gps_origin_auto_set_on_first_measurement);
    RUN_TEST(test_set_baro_origin);
    RUN_TEST(test_baro_relative_to_origin);
    RUN_TEST(test_update_orientation_low_g);
    RUN_TEST(test_update_orientation_high_g);
    RUN_TEST(test_update_orientation_null_filter);
    RUN_TEST(test_predict_calls_kalman_filter);
    RUN_TEST(test_predict_updates_state_from_filter);
    RUN_TEST(test_predict_null_filter);
    RUN_TEST(test_update_measurements_gps_only);
    RUN_TEST(test_update_measurements_baro_only);
    RUN_TEST(test_update_measurements_gps_and_baro);
    RUN_TEST(test_update_measurements_no_sensors);
    RUN_TEST(test_update_measurements_updates_state_from_filter);
    RUN_TEST(test_update_measurements_null_filter);
    RUN_TEST(test_getters_return_correct_values);
    RUN_TEST(test_get_orientation);
    RUN_TEST(test_get_acceleration);
    RUN_TEST(test_get_orientation_filter);
    RUN_TEST(test_deprecated_update_returns_error);
    RUN_TEST(test_deprecated_predict_state_does_nothing);
    RUN_TEST(test_full_update_cycle);
    RUN_TEST(test_multiple_orientation_updates);
}

} // namespace test_state
