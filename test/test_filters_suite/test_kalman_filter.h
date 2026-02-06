#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "Filters/LinearKalmanFilter.h"
#include "Filters/DefaultKalmanFilter.h"
#include "Math/Matrix.h"

using namespace astra;

namespace test_kalman_filter {

class TestableKalmanFilter : public DefaultKalmanFilter {
public:
    // Constructors that forward to base class
    TestableKalmanFilter() : DefaultKalmanFilter() {}
    TestableKalmanFilter(double processNoise, double gpsNoise, double baroNoise)
        : DefaultKalmanFilter(processNoise, gpsNoise, baroNoise) {}

    void setState(const Matrix& newState) { X = newState; }
    Matrix getCovariance() const { return P; }
    void setCovariance(const Matrix& newCov) { P = newCov; }
};
void assertMatrixEqual(const Matrix& expected, const Matrix& actual, double tolerance = 0.01) {
    char msg[200];
    snprintf(msg, sizeof(msg), "Matrix dimensions: expected(%d,%d) actual(%d,%d)",
             expected.getRows(), expected.getCols(), actual.getRows(), actual.getCols());
    TEST_ASSERT_EQUAL_MESSAGE(expected.getRows(), actual.getRows(), msg);
    TEST_ASSERT_EQUAL_MESSAGE(expected.getCols(), actual.getCols(), msg);
    for (int i = 0; i < expected.getRows(); i++) {
        for (int j = 0; j < expected.getCols(); j++) {
            snprintf(msg, sizeof(msg), "Matrix[%d,%d]: expected %.4f, actual %.4f",
                     i, j, expected.get(i, j), actual.get(i, j));
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, expected.get(i, j), actual.get(i, j), msg);
        }
    }
}

void local_setUp(void) {
}

void local_tearDown(void) {
}

void test_default_kalman_constructor(void) {
    local_setUp();
    TestableKalmanFilter kf;

    // Should be constructed successfully
    TEST_ASSERT_EQUAL(6, kf.getStateSize());
    TEST_ASSERT_EQUAL(3, kf.getMeasurementSize());
    TEST_ASSERT_EQUAL(3, kf.getInputSize());
    local_tearDown();
}

void test_default_kalman_constructor_with_params(void) {
    local_setUp();
    DefaultKalmanFilter kf(2.0, 10.0, 5.0);

    TEST_ASSERT_EQUAL(6, kf.getStateSize());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, kf.getGPSNoise());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 5.0, kf.getBaroNoise());
    local_tearDown();
}

void test_default_kalman_initialize(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    Matrix state = kf.getState();

    // Initial state should be zero
    TEST_ASSERT_EQUAL(6, state.getRows());
    TEST_ASSERT_EQUAL(1, state.getCols());

    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, state.get(i, 0));
    }
    local_tearDown();
}

void test_default_kalman_state_transition_matrix(void) {
    local_setUp();
    TestableKalmanFilter kf;
    double dt = 0.1;

    Matrix F = kf.getF(dt);

    // F should be 6x6
    TEST_ASSERT_EQUAL(6, F.getRows());
    TEST_ASSERT_EQUAL(6, F.getCols());

    // Check diagonal elements (should be 1)
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, F.get(i, i));
    }

    // Check position-velocity coupling (dt in upper-right block)
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, F.get(0, 3));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, F.get(1, 4));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, F.get(2, 5));
    local_tearDown();
}

void test_default_kalman_control_matrix(void) {
    local_setUp();
    TestableKalmanFilter kf;
    double dt = 0.1;

    Matrix G = kf.getG(dt);

    // G should be 6x3
    TEST_ASSERT_EQUAL(6, G.getRows());
    TEST_ASSERT_EQUAL(3, G.getCols());

    double dt2 = 0.5 * dt * dt;

    // Check position rows (0.5*dt^2 for acceleration)
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt2, G.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt2, G.get(1, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt2, G.get(2, 2));

    // Check velocity rows (dt for acceleration)
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, G.get(3, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, G.get(4, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01, dt, G.get(5, 2));
    local_tearDown();
}

void test_default_kalman_measurement_matrix(void) {
    local_setUp();
    TestableKalmanFilter kf;

    Matrix H = kf.getH();

    // H should be 3x6 (measures position only)
    TEST_ASSERT_EQUAL(3, H.getRows());
    TEST_ASSERT_EQUAL(6, H.getCols());

    // Should be [I_3x3 | 0_3x3]
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, H.get(i, i));

        // Check zeros
        for (int j = 3; j < 6; j++) {
            TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, H.get(i, j));
        }
    }
    local_tearDown();
}

void test_default_kalman_measurement_noise_matrix(void) {
    local_setUp();
    DefaultKalmanFilter kf(1.0, 5.0, 2.0);

    Matrix R = kf.getR();

    // R should be 3x3 diagonal
    TEST_ASSERT_EQUAL(3, R.getRows());
    TEST_ASSERT_EQUAL(3, R.getCols());

    double gpsVar = 5.0 * 5.0;
    double baroVar = 2.0 * 2.0;

    TEST_ASSERT_FLOAT_WITHIN(0.01, gpsVar, R.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.01, gpsVar, R.get(1, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01, baroVar, R.get(2, 2));

    // Off-diagonals should be zero
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, R.get(0, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, R.get(1, 0));
    local_tearDown();
}

void test_default_kalman_process_noise_matrix(void) {
    local_setUp();
    TestableKalmanFilter kf;
    double dt = 0.1;

    Matrix Q = kf.getQ(dt);

    // Q should be 6x6
    TEST_ASSERT_EQUAL(6, Q.getRows());
    TEST_ASSERT_EQUAL(6, Q.getCols());

    // Q should be symmetric
    for (int i = 0; i < 6; i++) {
        for (int j = i+1; j < 6; j++) {
            TEST_ASSERT_FLOAT_WITHIN(0.01, Q.get(i, j), Q.get(j, i));
        }
    }
    local_tearDown();
}

void test_default_kalman_predict_zero_control(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Set initial state: position [10, 20, 30], velocity [1, 2, 3]
    double stateData[6] = {10.0, 20.0, 30.0, 1.0, 2.0, 3.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // Zero control input
    double controlData[3] = {0.0, 0.0, 0.0};
    Matrix U(3, 1, controlData);

    double dt = 1.0;
    kf.predict(dt, U);

    Matrix X_new = kf.getState();

    // Position should have moved by velocity * dt
    TEST_ASSERT_FLOAT_WITHIN(0.1, 11.0, X_new.get(0, 0)); // px = 10 + 1*1
    TEST_ASSERT_FLOAT_WITHIN(0.1, 22.0, X_new.get(1, 0)); // py = 20 + 2*1
    TEST_ASSERT_FLOAT_WITHIN(0.1, 33.0, X_new.get(2, 0)); // pz = 30 + 3*1

    // Velocity should be unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.1, 1.0, X_new.get(3, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 2.0, X_new.get(4, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 3.0, X_new.get(5, 0));
    local_tearDown();
}

void test_default_kalman_predict_with_acceleration(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Initial state: position [0, 0, 0], velocity [0, 0, 0]
    double stateData[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // Constant acceleration [0, 0, 9.81] (gravity)
    double controlData[3] = {0.0, 0.0, 9.81};
    Matrix U(3, 1, controlData);

    double dt = 1.0;
    kf.predict(dt, U);

    Matrix X_new = kf.getState();

    // Position: pz = 0.5 * 9.81 * 1^2 = 4.905
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, X_new.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, X_new.get(1, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 4.905, X_new.get(2, 0));

    // Velocity: vz = 9.81 * 1 = 9.81
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, X_new.get(3, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, X_new.get(4, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 9.81, X_new.get(5, 0));
    local_tearDown();
}

void test_default_kalman_update_gps_only(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Set initial state with some error
    double stateData[6] = {10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // GPS measurement: px=15, py=25 (closer to true position)
    kf.updateGPS(15.0, 25.0, 5.0);

    Matrix X_new = kf.getState();

    // State should have moved toward measurement
    // (exact value depends on Kalman gain, so we check direction)
    TEST_ASSERT_GREATER_THAN(10.0, X_new.get(0, 0)); // px increased toward 15
    TEST_ASSERT_GREATER_THAN(20.0, X_new.get(1, 0)); // py increased toward 25

    // Vertical position should be unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.1, 30.0, X_new.get(2, 0));
    local_tearDown();
}

void test_default_kalman_update_baro_only(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Set initial state
    double stateData[6] = {10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // Barometer measurement: pz=35
    kf.updateBaro(35.0, 2.0);

    Matrix X_new = kf.getState();

    // Horizontal position should be unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.1, 10.0, X_new.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.1, 20.0, X_new.get(1, 0));

    // Vertical position should move toward measurement
    TEST_ASSERT_GREATER_THAN(30.0, X_new.get(2, 0));
    local_tearDown();
}

void test_default_kalman_update_gps_baro_combined(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Set initial state
    double stateData[6] = {10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // Combined GPS+Baro measurement
    kf.updateGPSBaro(15.0, 25.0, 35.0, 5.0, 2.0);

    Matrix X_new = kf.getState();

    // All positions should move toward measurements
    TEST_ASSERT_GREATER_THAN(10.0, X_new.get(0, 0));
    TEST_ASSERT_GREATER_THAN(20.0, X_new.get(1, 0));
    TEST_ASSERT_GREATER_THAN(30.0, X_new.get(2, 0));
    local_tearDown();
}

void test_default_kalman_predict_update_cycle(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Simulate constant velocity motion with noisy measurements
    double stateData[6] = {0.0, 0.0, 0.0, 10.0, 5.0, 0.0}; // Moving at 10 m/s in X, 5 m/s in Y
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    double controlData[3] = {0.0, 0.0, 0.0};
    Matrix U(3, 1, controlData);

    // Run for 5 steps
    double dt = 0.1;
    for (int i = 0; i < 5; i++) {
        kf.predict(dt, U);

        // Simulate noisy measurement (true + noise)
        double true_px = 10.0 * (i+1) * dt;
        double true_py = 5.0 * (i+1) * dt;

        kf.updateGPS(true_px, true_py, 5.0);
    }

    Matrix X_final = kf.getState();

    // After 5 steps of 0.1s, position should be approximately [5.0, 2.5, 0.0]
    TEST_ASSERT_FLOAT_WITHIN(1.0, 5.0, X_final.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(1.0, 2.5, X_final.get(1, 0));

    // Velocity should be approximately [10, 5, 0]
    TEST_ASSERT_FLOAT_WITHIN(2.0, 10.0, X_final.get(3, 0));
    TEST_ASSERT_FLOAT_WITHIN(2.0, 5.0, X_final.get(4, 0));
    local_tearDown();
}

void test_default_kalman_free_fall_scenario(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Start at altitude 100m, zero velocity
    double stateData[6] = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    // Gravity acceleration (ENU: up is positive Z)
    // Free fall means acceleration is -9.81 in Z
    double controlData[3] = {0.0, 0.0, -9.81};
    Matrix U(3, 1, controlData);

    double dt = 0.1;

    // Simulate 1 second of free fall
    for (int i = 0; i < 10; i++) {
        kf.predict(dt, U);
    }

    Matrix X_final = kf.getState();

    // After 1 second: z = 100 + 0*1 - 0.5*9.81*1^2 = 95.095
    TEST_ASSERT_FLOAT_WITHIN(1.0, 95.095, X_final.get(2, 0));

    // Velocity: vz = 0 - 9.81*1 = -9.81
    TEST_ASSERT_FLOAT_WITHIN(1.0, -9.81, X_final.get(5, 0));
    local_tearDown();
}

void test_kalman_zero_dt(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    double stateData[6] = {10.0, 20.0, 30.0, 1.0, 2.0, 3.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    double controlData[3] = {1.0, 2.0, 3.0};
    Matrix U(3, 1, controlData);

    kf.predict(0.0, U);

    Matrix X_new = kf.getState();

    // State should not change with dt=0
    assertMatrixEqual(X_init, X_new, 0.01);
    local_tearDown();
}

void test_kalman_large_dt(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    double stateData[6] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    Matrix X_init(6, 1, stateData);
    kf.setState(X_init);

    double controlData[3] = {0.0, 0.0, 0.0};
    Matrix U(3, 1, controlData);

    // Large time step (10 seconds)
    kf.predict(10.0, U);

    Matrix X_new = kf.getState();

    // Should not crash or produce NaN
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FALSE(isnan(X_new.get(i, 0)));
    }
    local_tearDown();
}

void test_kalman_high_acceleration(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Very high acceleration (100 G's = 981 m/s^2)
    double controlData[3] = {0.0, 0.0, 981.0};
    Matrix U(3, 1, controlData);

    double dt = 0.01;
    kf.predict(dt, U);

    Matrix X_new = kf.getState();

    // Should handle high acceleration without issues
    TEST_ASSERT_FALSE(isnan(X_new.get(2, 0)));
    TEST_ASSERT_FALSE(isnan(X_new.get(5, 0)));
    local_tearDown();
}

void test_kalman_repeated_predictions(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    double controlData[3] = {1.0, 1.0, 1.0};
    Matrix U(3, 1, controlData);

    // Run many prediction steps
    for (int i = 0; i < 1000; i++) {
        kf.predict(0.01, U);
    }

    Matrix X_final = kf.getState();

    // Should maintain numerical stability
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FALSE(isnan(X_final.get(i, 0)));
        TEST_ASSERT_FALSE(isinf(X_final.get(i, 0)));
    }
    local_tearDown();
}

void test_kalman_repeated_updates(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    // Many measurement updates at same position
    for (int i = 0; i < 100; i++) {
        kf.updateGPSBaro(10.0, 20.0, 30.0);
    }

    Matrix X_final = kf.getState();

    // Should converge to measurement
    TEST_ASSERT_FLOAT_WITHIN(0.5, 10.0, X_final.get(0, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.5, 20.0, X_final.get(1, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.5, 30.0, X_final.get(2, 0));
    local_tearDown();
}

void test_kalman_alternating_predict_update(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    double controlData[3] = {0.0, 0.0, 0.0};
    Matrix U(3, 1, controlData);

    // Alternate predict and update many times
    for (int i = 0; i < 50; i++) {
        kf.predict(0.1, U);
        kf.updateGPSBaro(0.0, 0.0, 0.0);
    }

    Matrix X_final = kf.getState();

    // Should remain numerically stable
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FALSE(isnan(X_final.get(i, 0)));
    }
    local_tearDown();
}

void test_kalman_measurement_noise_effect(void) {
    local_setUp();
    TestableKalmanFilter kf1(1.0, 1.0, 1.0);   // Low noise
    TestableKalmanFilter kf2(1.0, 100.0, 100.0); // High noise

    kf1.initialize();
    kf2.initialize();

    // Set same initial state
    double stateData[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Matrix X_init(6, 1, stateData);
    kf1.setState(X_init);
    kf2.setState(X_init);

    // Need to set same initial covariance for fair comparison
    double covData[36];
    for (int i = 0; i < 36; i++) covData[i] = 0.0;
    for (int i = 0; i < 6; i++) covData[i*6 + i] = 100.0; // Large initial uncertainty
    Matrix P_init(6, 6, covData);
    kf1.setCovariance(P_init);
    kf2.setCovariance(P_init);

    // Do a single update and check immediate response
    double controlData[3] = {0.0, 0.0, 0.0};
    Matrix U(3, 1, controlData);

    // Single predict-update cycle
    kf1.predict(0.1, U);
    kf2.predict(0.1, U);
    kf1.updateGPSBaro(10.0, 10.0, 10.0);
    kf2.updateGPSBaro(10.0, 10.0, 10.0);

    Matrix X1 = kf1.getState();
    Matrix X2 = kf2.getState();

    // Low noise filter should move closer to measurement than high noise filter
    double dist1 = sqrt(pow(X1.get(0,0) - 10.0, 2) + pow(X1.get(1,0) - 10.0, 2) + pow(X1.get(2,0) - 10.0, 2));
    double dist2 = sqrt(pow(X2.get(0,0) - 10.0, 2) + pow(X2.get(1,0) - 10.0, 2) + pow(X2.get(2,0) - 10.0, 2));

    // kf1 (low noise) should have smaller distance to measurement than kf2 (high noise)
    TEST_ASSERT_LESS_THAN(dist2, dist1);
    local_tearDown();
}

void test_kalman_matrix_dimensions_consistency(void) {
    local_setUp();
    TestableKalmanFilter kf;

    double dt = 0.1;

    Matrix F = kf.getF(dt);
    Matrix G = kf.getG(dt);
    Matrix H = kf.getH();
    Matrix R = kf.getR();
    Matrix Q = kf.getQ(dt);

    int n = kf.getStateSize();      // 6
    int m = kf.getMeasurementSize(); // 3
    int p = kf.getInputSize();       // 3

    // F: n x n
    TEST_ASSERT_EQUAL(n, F.getRows());
    TEST_ASSERT_EQUAL(n, F.getCols());

    // G: n x p
    TEST_ASSERT_EQUAL(n, G.getRows());
    TEST_ASSERT_EQUAL(p, G.getCols());

    // H: m x n
    TEST_ASSERT_EQUAL(m, H.getRows());
    TEST_ASSERT_EQUAL(n, H.getCols());

    // R: m x m
    TEST_ASSERT_EQUAL(m, R.getRows());
    TEST_ASSERT_EQUAL(m, R.getCols());

    // Q: n x n
    TEST_ASSERT_EQUAL(n, Q.getRows());
    TEST_ASSERT_EQUAL(n, Q.getCols());
    local_tearDown();
}

void test_kalman_state_vector_dimension(void) {
    local_setUp();
    TestableKalmanFilter kf;
    kf.initialize();

    Matrix X = kf.getState();

    TEST_ASSERT_EQUAL(6, X.getRows());
    TEST_ASSERT_EQUAL(1, X.getCols());
    local_tearDown();
}

void run_test_kalman_filter_tests()
{
    RUN_TEST(test_default_kalman_constructor);
    RUN_TEST(test_default_kalman_constructor_with_params);
    RUN_TEST(test_default_kalman_initialize);
    RUN_TEST(test_default_kalman_state_transition_matrix);
    RUN_TEST(test_default_kalman_control_matrix);
    RUN_TEST(test_default_kalman_measurement_matrix);
    RUN_TEST(test_default_kalman_measurement_noise_matrix);
    RUN_TEST(test_default_kalman_process_noise_matrix);
    RUN_TEST(test_default_kalman_predict_zero_control);
    RUN_TEST(test_default_kalman_predict_with_acceleration);
    RUN_TEST(test_default_kalman_update_gps_only);
    RUN_TEST(test_default_kalman_update_baro_only);
    RUN_TEST(test_default_kalman_update_gps_baro_combined);
    RUN_TEST(test_default_kalman_predict_update_cycle);
    RUN_TEST(test_default_kalman_free_fall_scenario);
    RUN_TEST(test_kalman_zero_dt);
    RUN_TEST(test_kalman_large_dt);
    RUN_TEST(test_kalman_high_acceleration);
    RUN_TEST(test_kalman_repeated_predictions);
    RUN_TEST(test_kalman_repeated_updates);
    RUN_TEST(test_kalman_alternating_predict_update);
    RUN_TEST(test_kalman_measurement_noise_effect);
    RUN_TEST(test_kalman_matrix_dimensions_consistency);
    RUN_TEST(test_kalman_state_vector_dimension);
}

} // namespace test_kalman_filter
