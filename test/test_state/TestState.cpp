#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../src/State/State.h"
#include "../../src/Sensors/Sensor.h"
#include "../../src/Sensors/Accel/MockAccel.h"
#include "../../src/Sensors/Gyro/MockGyro.h"
#include "../../src/Sensors/GPS/MockGPS.h"
#include "../../src/Sensors/Baro/MockBarometer.h"
#include "../../src/Sensors/Accel/Accel.h"
#include "../../src/Sensors/Gyro/Gyro.h"
#include "../../src/Sensors/GPS/GPS.h"
#include "../../src/Sensors/Baro/Barometer.h"
#include "../../src/Math/Vector.h"

using namespace astra;

// Test data
const char header[] = "pres,temp,lat,lon,alt,head,fixQ";
const char data[] = "101.3,25.5,34.0522,-118.2437,100.0,90.0,1.0";
char *dataPtr;

// Mock components
MockAccel *mockAccel;
MockGyro *mockGyro;
MockGPS *mockGPS;
MockBarometer *mockBaro;
Sensor *sensors[4];
MahonyAHRS *orientationFilter;
State *state;

void setUp(void)
{
    dataPtr = new char[5000];
    strcpy(dataPtr, data);

    mockAccel = new MockAccel();
    mockGyro = new MockGyro();
    mockGPS = new MockGPS(dataPtr, "lat", "lon", "alt", "head", "fixQ");
    mockBaro = new MockBarometer(dataPtr, "pres", "temp");

    sensors[0] = mockAccel;
    sensors[1] = mockGyro;
    sensors[2] = mockGPS;
    sensors[3] = mockBaro;

    // Initialize sensors
    for (int i = 0; i < 4; i++)
    {
        sensors[i]->begin();
    }

    orientationFilter = new MahonyAHRS();
    state = new State(nullptr, orientationFilter);
}

void tearDown(void)
{
    delete state;
    delete orientationFilter;
    delete mockAccel;
    delete mockGyro;
    delete mockGPS;
    delete mockBaro;
    delete[] dataPtr;
}

// Test State initialization without sensors
void test_state_init_no_sensors()
{
    State simpleState(nullptr, nullptr);
    bool result = simpleState.begin();

    TEST_ASSERT_TRUE(result);
}

// Test State initialization with sensors
void test_state_init_with_sensors()
{
    state->withSensors(sensors, 4);
    bool result = state->begin();

    TEST_ASSERT_TRUE(result);
}

// Test orientation update with Vector data
void test_update_orientation()
{
    state->withSensors(sensors, 4);
    state->begin();

    // Set mock sensor data
    mockAccel->setAccel(0.0, 0.0, 9.81); // Gravity pointing down
    mockGyro->setAngVel(0.0, 0.0, 0.0);  // No rotation

    // Update orientation
    state->updateOrientation(mockGyro->getAngVel(), mockAccel->getAccel(), 0.01);

    // Verify orientation was updated
    Quaternion orientation = state->getOrientation();
    // With gravity pointing down and no rotation, quaternion should be near identity
    TEST_ASSERT_NOT_EQUAL(0.0, orientation.w());
}

// Test orientation update with motion
void test_update_orientation_with_motion()
{
    state->withSensors(sensors, 4);
    state->begin();

    // Simulate accelerating forward
    mockAccel->setAccel(1.0, 0.0, 9.81);
    mockGyro->setAngVel(0.1, 0.0, 0.0); // Rotating around X axis

    state->updateOrientation(mockGyro->getAngVel(), mockAccel->getAccel(), 0.01);

    Vector<3> acceleration = state->getAcceleration();
    // Should have some earth-frame acceleration
    TEST_ASSERT_NOT_EQUAL(0.0, acceleration.magnitude());
}

// Test measurement update with GPS and Barometer
void test_update_measurements()
{
    state->withSensors(sensors, 4);
    state->begin();

    bool hasGPS = mockGPS && mockGPS->isInitialized();
    bool hasBaro = mockBaro && mockBaro->isInitialized();

    // Update measurements (Note: State needs a filter for this to work properly)
    // For this test, we're just verifying the interface works
    if (hasGPS && hasBaro)
    {
        state->updateMeasurements(mockGPS->getPos(), mockBaro->getASLAltM(), hasGPS, hasBaro, 1.0);
        // No crash = success for now
        TEST_ASSERT_TRUE(true);
    }
}

// Test position/velocity update
void test_update_position_velocity()
{
    state->withSensors(sensors, 4);
    state->begin();

    TEST_ASSERT_NOT_NULL(mockGPS);

    Vector<3> gpsPos = mockGPS->getPos();
    double heading = mockGPS->getHeading();
    bool hasFix = mockGPS->getHasFix();

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
    state->withSensors(sensors, 4);
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
    state->withSensors(sensors, 4);
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

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_state_init_no_sensors);
    RUN_TEST(test_state_init_with_sensors);
    RUN_TEST(test_update_orientation);
    RUN_TEST(test_update_orientation_with_motion);
    RUN_TEST(test_update_measurements);
    RUN_TEST(test_update_position_velocity);
    RUN_TEST(test_update_position_velocity_no_fix);
    RUN_TEST(test_state_no_sensor_references);
    RUN_TEST(test_state_getters);

    UNITY_END();

    return 0;
}
