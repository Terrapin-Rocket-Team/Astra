#include <unity.h>
#include "State/State.h"
#include "Filters/LinearKalmanFilter.h"
#include "Filters/Mahony.h"
#include "Sensors/SensorManager/SensorManager.h"
#include "UnitTestSensors.h"
#include "NativeTestHelper.h"

using namespace astra;

// Note: The tests calling state->begin() will run slowly due to a 2-second
// calibration loop with delays in the real State::begin() method.

// Mock for Kalman Filter only
class MockLinearKalmanFilter : public LinearKalmanFilter {
public:
    bool initialized = false;
    bool predict_called = false;
    bool update_called = false;
    double dt_predict = 0.0;
    double* control_vars = nullptr;
    double* measurement_vars = nullptr;

    MockLinearKalmanFilter() : LinearKalmanFilter(3, 3, 6) {}

    void initialize() override { initialized = true; }
    Matrix getF(double dt) override { return Matrix(6, 6, new double[36]()); }
    Matrix getG(double dt) override { return Matrix(6, 3, new double[18]()); }
    Matrix getH() override { return Matrix(3, 6, new double[18]()); }
    Matrix getR() override { return Matrix(3, 3, new double[9]()); }
    Matrix getQ(double dt) override { return Matrix(6, 6, new double[36]()); }

    void predict(double dt, double* control) override {
        predict_called = true;
        dt_predict = dt;
        control_vars = new double[3];
        memcpy(control_vars, control, sizeof(double) * 3);
        delete[] control;
    }

    void update(double* measurements) override {
        update_called = true;
        measurement_vars = new double[3];
        memcpy(measurement_vars, measurements, sizeof(double) * 3);
        delete[] measurements;
    }

    void getState(double* state) const override {
        for (int i = 0; i < 6; ++i) {
            state[i] = i + 1.0; // Return dummy state 1, 2, 3, 4, 5, 6
        }
    }
    
    ~MockLinearKalmanFilter() {
        delete[] control_vars;
        delete[] measurement_vars;
    }
};

// Global vars
SensorManager* sensorManager;
FakeGPS* fakeGPS;
FakeBarometer* fakeBaro;
FakeAccel* fakeAccel;
FakeGyro* fakeGyro;

MockLinearKalmanFilter* kalmanFilter;
MahonyAHRS* orientationFilter; // Using real MahonyAHRS
State* state;

void setUp(void) {
    kalmanFilter = new MockLinearKalmanFilter();
    orientationFilter = new MahonyAHRS(); // Real MahonyAHRS
    state = new State(kalmanFilter, orientationFilter);
    
    sensorManager = new SensorManager();
    fakeGPS = new FakeGPS();
    fakeBaro = new FakeBarometer();
    fakeAccel = new FakeAccel();
    fakeGyro = new FakeGyro();

    sensorManager->setPrimaryGPS(fakeGPS);
    sensorManager->setPrimaryBaro(fakeBaro);
    sensorManager->setPrimaryAccel(fakeAccel);
    sensorManager->setPrimaryGyro(fakeGyro);

    // Initialize sensors within the manager
    fakeGPS->begin();
    fakeBaro->begin();
    fakeAccel->begin();
    fakeGyro->begin();
}

void tearDown(void) {
    delete state;
    delete kalmanFilter;
    delete orientationFilter;

    delete sensorManager;
    delete fakeGPS;
    delete fakeBaro;
    delete fakeAccel;
    delete fakeGyro;
}

void test_begin_fails_without_sensor_manager() {
    State s(kalmanFilter, orientationFilter);
    TEST_ASSERT_FALSE(s.begin());
}

void test_begin_succeeds_with_sensor_manager() {
    // Set stable sensor data for calibration inside begin()
    fakeAccel->set(Vector<3>(0, 0, 9.81));
    fakeGyro->set(Vector<3>(0, 0, 0));
    
    state->withSensorManager(sensorManager);
    TEST_ASSERT_TRUE(state->begin());
    TEST_ASSERT_TRUE(kalmanFilter->initialized);
    TEST_ASSERT_EQUAL(MahonyMode::CALIBRATING, orientationFilter->getMode());
}

void test_update_orientation() {
    // Manually calibrate since we don't call begin()
    for (int i = 0; i < 200; ++i) {
        orientationFilter->update(Vector<3>(0, 0, 9.81), Vector<3>(0, 0, 0), 0.01);
    }
    orientationFilter->initialize();
    orientationFilter->setMode(MahonyMode::CORRECTING);
    TEST_ASSERT_TRUE(orientationFilter->isInitialized());

    // Give it stable data first
    Vector<3> gyro_data(0, 0, 0);
    Vector<3> accel_data(0, 0, 9.81);
    state->updateOrientation(gyro_data, accel_data, 0.01);
    
    Quaternion q_before = state->getOrientation();

    // Now apply some rotation
    gyro_data.x() = 0.5; // 0.5 rad/s ~ 28 deg/s
    for (int i=0; i<10; ++i) {
        state->updateOrientation(gyro_data, accel_data, 0.01);
    }
    
    Quaternion q_after = state->getOrientation();

    // Check that orientation has changed
    // Note: Mahony filter negates gyro input (line 115 in Mahony.h: -gyro.x())
    // So positive gyro.x() = 0.5 rad/s causes negative rotation around x-axis
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.999, q_before.w());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.996, q_after.w()); // w should decrease
    TEST_ASSERT_FLOAT_WITHIN(0.01, -0.025, q_after.x()); // x should be negative due to gyro negation
    TEST_ASSERT_NOT_EQUAL(q_before.w(), q_after.w());
}

void test_predict_state() {
    // Set stable sensor data for calibration inside begin()
    fakeAccel->set(Vector<3>(0, 0, 9.81));
    fakeGyro->set(Vector<3>(0, 0, 0));

    state->withSensorManager(sensorManager);
    state->begin(); // This will be slow
    orientationFilter->setMode(MahonyMode::CORRECTING); // Move out of calibration
    
    // Set new sensor data that will be fetched by predictState
    // The real MahonyAHRS will calculate earth-frame acceleration
    fakeAccel->set(Vector<3>(0.1, 0.2, 9.8));
    fakeGyro->set(Vector<3>(0.0, 0.0, 0.0));

    state->predictState(3.0); // time after begin() finished

    TEST_ASSERT_TRUE(kalmanFilter->predict_called);

    // Check that predict was called with earth acceleration from orientation filter
    // Note: After calibration, Mahony aligns body frame to gravity, creating a non-identity quaternion.
    // The earth frame acceleration is computed via quaternion rotation, so values differ from naive body_accel - gravity.
    // These expected values are based on the actual Mahony quaternion transformation after calibration.
    TEST_ASSERT_NOT_NULL(kalmanFilter->control_vars);
    TEST_ASSERT_FLOAT_WITHIN(0.05, 0.07, kalmanFilter->control_vars[0]); // ~0.0696
    TEST_ASSERT_FLOAT_WITHIN(0.05, 0.14, kalmanFilter->control_vars[1]); // ~0.1391
    TEST_ASSERT_FLOAT_WITHIN(0.05, -0.01, kalmanFilter->control_vars[2]); // ~-0.01

    // Check that state was updated with dummy data from mock KF's getState
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();
    TEST_ASSERT_EQUAL_FLOAT(1.0, pos.x()); // from mock getState
    TEST_ASSERT_EQUAL_FLOAT(2.0, pos.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, pos.z());
    TEST_ASSERT_EQUAL_FLOAT(4.0, vel.x());
    TEST_ASSERT_EQUAL_FLOAT(5.0, vel.y());
    TEST_ASSERT_EQUAL_FLOAT(6.0, vel.z());
}

void test_update() {
    // Set stable sensor data for calibration inside begin()
    fakeAccel->set(Vector<3>(0, 0, 9.81));
    fakeGyro->set(Vector<3>(0, 0, 0));
    fakeBaro->set(101.325, 15); // Standard pressure at sea level, 15C
    
    state->withSensorManager(sensorManager);
    state->begin(); // This will be slow
    orientationFilter->setMode(MahonyMode::CORRECTING);

    // Set up sensor data for the update
    fakeGPS->setHasFirstFix(true);
    fakeGPS->set(34.0, -118.0, 100.0);
    
    state->update(3.0); // time after begin() finished

    // Check that KF measurement update was called
    TEST_ASSERT_TRUE(kalmanFilter->update_called);

    // Check that the measurements passed to the filter are correct
    TEST_ASSERT_NOT_NULL(kalmanFilter->measurement_vars);
    // On first update with GPS fix, origin is set and displacement from origin is 0.
    TEST_ASSERT_EQUAL_FLOAT(0.0, kalmanFilter->measurement_vars[0]); 
    TEST_ASSERT_EQUAL_FLOAT(0.0, kalmanFilter->measurement_vars[1]);
    // Baro alt origin is set in `begin()`. Since value hasn't changed, measurement is 0.
    TEST_ASSERT_EQUAL_FLOAT(0.0, kalmanFilter->measurement_vars[2]);

    // Check that state was updated from mock KF
    Vector<3> pos = state->getPosition();
    Vector<3> vel = state->getVelocity();
    TEST_ASSERT_EQUAL_FLOAT(1.0, pos.x()); // from mock getState
    TEST_ASSERT_EQUAL_FLOAT(2.0, pos.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, pos.z());
    TEST_ASSERT_EQUAL_FLOAT(4.0, vel.x());
    TEST_ASSERT_EQUAL_FLOAT(5.0, vel.y());
    TEST_ASSERT_EQUAL_FLOAT(6.0, vel.z());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_begin_fails_without_sensor_manager);
    RUN_TEST(test_begin_succeeds_with_sensor_manager);
    RUN_TEST(test_update_orientation);
    RUN_TEST(test_predict_state);
    RUN_TEST(test_update);
    UNITY_END();
    return 0;
}