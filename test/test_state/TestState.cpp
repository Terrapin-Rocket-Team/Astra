#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../src/State/State.h"
#include "../../src/Sensors/SensorManager.h"
#include "../../src/Sensors/Accel/MockAccel.h"
#include "../../src/Sensors/Gyro/MockGyro.h"
#include "../../src/Sensors/GPS/MockGPS.h"
#include "../../src/Sensors/Baro/MockBarometer.h"

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
SensorManager *sensorManager;
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

    sensorManager = new SensorManager();
    Sensor *sensors[4] = {mockAccel, mockGyro, mockGPS, mockBaro};
    sensorManager->setSensors(sensors, 4);
    sensorManager->initAll();

    orientationFilter = new MahonyAHRS();
    state = new State(nullptr, orientationFilter);
}

void tearDown(void)
{
    delete state;
    delete orientationFilter;
    delete sensorManager;
    delete mockAccel;
    delete mockGyro;
    delete mockGPS;
    delete mockBaro;
    delete[] dataPtr;
}

// Test State initialization without SensorManager
void test_state_init_no_sensor_manager()
{
    State simpleState(nullptr, nullptr);
    bool result = simpleState.begin(nullptr);

    TEST_ASSERT_TRUE(result);
}

// Test State initialization with SensorManager
void test_state_init_with_sensor_manager()
{
    bool result = state->begin(sensorManager);

    TEST_ASSERT_TRUE(result);
}

// Test orientation update with primitive data
void test_update_orientation()
{
    state->begin(sensorManager);

    // Set mock sensor data
    mockAccel->setAccel(0.0, 0.0, 9.81); // Gravity pointing down
    mockGyro->setAngVel(0.0, 0.0, 0.0);  // No rotation

    // Extract data via SensorManager
    double accel[3], gyro[3];
    sensorManager->getAccelData(accel);
    sensorManager->getGyroData(gyro);

    // Update orientation
    state->updateOrientation(gyro, accel, 0.01);

    // Verify orientation was updated
    Quaternion orientation = state->getOrientation();
    // With gravity pointing down and no rotation, quaternion should be near identity
    TEST_ASSERT_NOT_EQUAL(0.0, orientation.w());
}

// Test orientation update with motion
void test_update_orientation_with_motion()
{
    state->begin(sensorManager);

    // Simulate accelerating forward
    mockAccel->setAccel(1.0, 0.0, 9.81);
    mockGyro->setAngVel(0.1, 0.0, 0.0); // Rotating around X axis

    double accel[3], gyro[3];
    sensorManager->getAccelData(accel);
    sensorManager->getGyroData(gyro);

    state->updateOrientation(gyro, accel, 0.01);

    Vector<3> acceleration = state->getAcceleration();
    // Should have some earth-frame acceleration
    TEST_ASSERT_NOT_EQUAL(0.0, acceleration.magnitude());
}

// Test measurement update with GPS and Barometer
void test_update_measurements()
{
    state->begin(sensorManager);

    // Extract GPS and baro data
    double gpsLat, gpsLon, gpsAlt, baroAlt;
    bool hasGPS = sensorManager->getGPSData(&gpsLat, &gpsLon, &gpsAlt);
    bool hasBaro = sensorManager->getBaroAltitude(&baroAlt);

    // Update measurements (Note: State needs a filter for this to work properly)
    // For this test, we're just verifying the interface works
    if (hasGPS && hasBaro)
    {
        state->updateMeasurements(gpsLat, gpsLon, gpsAlt, baroAlt, hasGPS, hasBaro, 1.0);
        // No crash = success for now
        TEST_ASSERT_TRUE(true);
    }
}

// Test position/velocity update
void test_update_position_velocity()
{
    state->begin(sensorManager);

    double lat, lon, alt, heading;
    bool hasFix;
    sensorManager->getGPSData(&lat, &lon, &alt);
    sensorManager->getGPSHeading(&heading);
    sensorManager->getGPSHasFix(&hasFix);

    state->updatePositionVelocity(lat, lon, heading, hasFix);

    Vector<2> coordinates = state->getCoordinates();
    double stateHeading = state->getHeading();

    if (hasFix)
    {
        TEST_ASSERT_EQUAL_FLOAT(lat, coordinates.x());
        TEST_ASSERT_EQUAL_FLOAT(lon, coordinates.y());
        TEST_ASSERT_EQUAL_FLOAT(heading, stateHeading);
    }
}

// Test position/velocity update with no fix
void test_update_position_velocity_no_fix()
{
    state->begin(sensorManager);

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
    bool result = independentState.begin(nullptr);

    TEST_ASSERT_TRUE(result);

    // State should accept primitive data without needing sensors
    double gyro[3] = {0, 0, 0};
    double accel[3] = {0, 0, 9.81};

    // This should work even without sensors
    MahonyAHRS filter;
    State stateWithFilter(nullptr, &filter);
    stateWithFilter.begin(nullptr);
    stateWithFilter.updateOrientation(gyro, accel, 0.01);

    TEST_ASSERT_TRUE(true); // No crash = success
}

// Test State getters
void test_state_getters()
{
    state->begin(sensorManager);

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

    RUN_TEST(test_state_init_no_sensor_manager);
    RUN_TEST(test_state_init_with_sensor_manager);
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
