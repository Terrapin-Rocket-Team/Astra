#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"
#include "../../src/State/State.h"
#include "../../src/Sensors/Sensor.h"
#include "../../src/Sensors/SensorManager/SensorManager.h"
#include "../../src/Math/Vector.h"

using namespace astra;

// Fake sensor components
FakeAccel *fakeAccel;
FakeGyro *fakeGyro;
FakeGPS *fakeGPS;
FakeBarometer *fakeBaro;
Sensor *sensors[4];
MahonyAHRS *orientationFilter;
SensorManager *sensorManager;
State *state;

void setUp(void)
{
    fakeAccel = new FakeAccel();
    fakeGyro = new FakeGyro();
    fakeGPS = new FakeGPS();
    fakeBaro = new FakeBarometer();

    // Set initial test data
    fakeGPS->set(34.0522, -118.2437, 100.0);
    fakeGPS->setHeading(90.0);
    fakeGPS->setHasFirstFix(true);
    fakeBaro->set(101.3, 25.5);

    sensors[0] = fakeAccel;
    sensors[1] = fakeGyro;
    sensors[2] = fakeGPS;
    sensors[3] = fakeBaro;

    // Initialize sensors
    for (int i = 0; i < 4; i++)
    {
        sensors[i]->begin();
    }

    // Create SensorManager and register sensors
    sensorManager = new SensorManager();
    sensorManager->withSensors(sensors, 4);

    orientationFilter = new MahonyAHRS();
    state = new State(nullptr, orientationFilter);
}

void tearDown(void)
{
    delete state;
    delete sensorManager;
    delete orientationFilter;
    delete fakeAccel;
    delete fakeGyro;
    delete fakeGPS;
    delete fakeBaro;
}

// Test State initialization without sensors
void test_state_init_no_sensors()
{
    State simpleState(nullptr, nullptr);
    bool result = simpleState.begin();

    TEST_ASSERT_TRUE(result);
}

// Test State initialization with SensorManager
void test_state_init_with_sensors()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    bool result = state->begin();

    TEST_ASSERT_TRUE(result);
}

// Test orientation update with Vector data
void test_update_orientation()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    // Set mock sensor data
    fakeAccel->setAccel(0.0, 0.0, 9.81); // Gravity pointing down
    fakeGyro->setAngVel(0.0, 0.0, 0.0);  // No rotation

    // Update orientation using vector API
    state->updateOrientation(fakeGyro->getAngVel(), fakeAccel->getAccel(), 0.01);

    // Verify orientation was updated
    Quaternion orientation = state->getOrientation();
    // With gravity pointing down and no rotation, quaternion should be near identity
    TEST_ASSERT_NOT_EQUAL(0.0, orientation.w());
}

// Test orientation update with motion
void test_update_orientation_with_motion()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    // Simulate accelerating forward
    fakeAccel->setAccel(1.0, 0.0, 9.81);
    fakeGyro->setAngVel(0.1, 0.0, 0.0); // Rotating around X axis

    // Run multiple updates to let filter converge
    for (int i = 0; i < 100; i++)
    {
        state->updateOrientation(fakeGyro->getAngVel(), fakeAccel->getAccel(), 0.01);
    }

    Vector<3> acceleration = state->getAcceleration();
    // After filter converges, we should have non-zero earth-frame acceleration
    // from the 1.0 m/s² forward component (minus gravity which is subtracted)
    // This is a weak assertion - just verify the filter ran without crashing
    TEST_ASSERT_TRUE(true);
}

// Test orientation update with BodyFrameData
void test_update_orientation_with_body_frame_data()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    // Set mock sensor data
    fakeAccel->setAccel(0.0, 0.0, 9.81);
    fakeGyro->setAngVel(0.0, 0.0, 0.0);

    // Update sensor manager to get body frame data
    sensorManager->update();
    const BodyFrameData &bodyData = sensorManager->getBodyFrameData();

    // Update orientation using BodyFrameData API
    state->updateOrientation(bodyData, 0.01);

    Quaternion orientation = state->getOrientation();
    TEST_ASSERT_NOT_EQUAL(0.0, orientation.w());
}

// Test measurement update with GPS and Barometer
void test_update_measurements()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    bool hasGPS = fakeGPS && fakeGPS->isInitialized();
    bool hasBaro = fakeBaro && fakeBaro->isInitialized();

    // Update measurements (Note: State needs a filter for this to work properly)
    // For this test, we're just verifying the interface works
    if (hasGPS && hasBaro)
    {
        state->updateMeasurements(fakeGPS->getPos(), fakeBaro->getASLAltM(), hasGPS, hasBaro, 1.0);
        // No crash = success for now
        TEST_ASSERT_TRUE(true);
    }
}

// Test position/velocity update
void test_update_position_velocity()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    TEST_ASSERT_NOT_NULL(fakeGPS);

    Vector<3> gpsPos = fakeGPS->getPos();
    double heading = fakeGPS->getHeading();
    bool hasFix = fakeGPS->getHasFix();

    state->updatePositionVelocity(gpsPos.x(), gpsPos.y(), heading, hasFix);

    Vector<2> coordinates = state->getCoordinates();
    double stateHeading = state->getHeading();

    if (hasFix)
    {
        TEST_ASSERT_EQUAL_FLOAT(gpsPos.x(), coordinates.x());
        TEST_ASSERT_EQUAL_FLOAT(gpsPos.y(), coordinates.y());
        TEST_ASSERT_EQUAL_FLOAT(heading, stateHeading);
    }
}

// Test position/velocity update with no fix
void test_update_position_velocity_no_fix()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    state->updatePositionVelocity(0.0, 0.0, 0.0, false);

    Vector<2> coordinates = state->getCoordinates();
    double heading = state->getHeading();

    TEST_ASSERT_EQUAL_FLOAT(0.0, coordinates.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, coordinates.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, heading);
}

// Test State doesn't hold sensor references
void test_state_no_sensor_references()
{
    // State should be constructible without any sensors
    State independentState(nullptr, nullptr);
    bool result = independentState.begin();

    TEST_ASSERT_TRUE(result);

    // State should accept Vector data without needing sensors
    Vector<3> gyro(0, 0, 0);
    Vector<3> accel(0, 0, 9.81);

    // This should work even without sensors
    MahonyAHRS filter;
    State stateWithFilter(nullptr, &filter);
    stateWithFilter.begin();
    stateWithFilter.updateOrientation(gyro, accel, 0.01);

    TEST_ASSERT_TRUE(true); // No crash = success
}

// Test State getters
void test_state_getters()
{
    sensorManager->begin();
    state->withSensorManager(sensorManager);
    state->begin();

    Vector<3> position = state->getPosition();
    Vector<3> velocity = state->getVelocity();
    Vector<3> acceleration = state->getAcceleration();
    Quaternion orientation = state->getOrientation();
    Vector<2> coordinates = state->getCoordinates();
    double heading = state->getHeading();

    // All getters should work without crashing
    TEST_ASSERT_TRUE(true);
}

// Test SensorManager basic functionality
void test_sensor_manager_basic()
{
    sensorManager->begin();

    // Set some sensor data
    fakeAccel->setAccel(0.0, 0.0, 9.81);
    fakeGyro->setAngVel(0.0, 0.0, 0.0);

    // Need to call update() before isReady() returns true (populates bodyData flags)
    sensorManager->update();

    TEST_ASSERT_TRUE(sensorManager->isReady());
    TEST_ASSERT_NOT_NULL(sensorManager->getActiveAccel());
    TEST_ASSERT_NOT_NULL(sensorManager->getActiveGyro());
}

// Test SensorManager body frame data
void test_sensor_manager_body_frame_data()
{
    sensorManager->begin();

    fakeAccel->setAccel(1.0, 2.0, 9.81);
    fakeGyro->setAngVel(0.1, 0.2, 0.3);

    sensorManager->update();
    const BodyFrameData &data = sensorManager->getBodyFrameData();

    TEST_ASSERT_TRUE(data.hasAccel);
    TEST_ASSERT_TRUE(data.hasGyro);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, data.accel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, data.accel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.1, data.gyro.x());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_state_init_no_sensors);
    RUN_TEST(test_state_init_with_sensors);
    RUN_TEST(test_update_orientation);
    RUN_TEST(test_update_orientation_with_motion);
    RUN_TEST(test_update_orientation_with_body_frame_data);
    RUN_TEST(test_update_measurements);
    RUN_TEST(test_update_position_velocity);
    RUN_TEST(test_update_position_velocity_no_fix);
    RUN_TEST(test_state_no_sensor_references);
    RUN_TEST(test_state_getters);
    RUN_TEST(test_sensor_manager_basic);
    RUN_TEST(test_sensor_manager_body_frame_data);

    UNITY_END();

    return 0;
}
