#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include "../../lib/NativeTestMocks/UnitTestSensors.h"
#include "../../src/Sensors/SensorManager.h"
#include "../../src/Sensors/Accel/MockAccel.h"
#include "../../src/Sensors/Gyro/MockGyro.h"
#include "../../src/Sensors/Accel/Accel.h"
#include "../../src/Sensors/Gyro/Gyro.h"
#include "../../src/Sensors/GPS/GPS.h"
#include "../../src/Sensors/Baro/Barometer.h"
#include "../../src/Math/Vector.h"

using namespace astra;

// Mock sensors
MockAccel *mockAccel;
MockGyro *mockGyro;
FakeGPS *fakeGPS;
FakeBarometer *fakeBaro;
SensorManager *sensorManager;

void setUp(void)
{
    mockAccel = new MockAccel();
    mockGyro = new MockGyro();
    fakeGPS = new FakeGPS();
    fakeBaro = new FakeBarometer();

    // Note: Don't set GPS/Baro data here because begin() will reset it
    // Tests should set data after calling initAll()

    sensorManager = new SensorManager();
}

void tearDown(void)
{
    delete sensorManager;
    delete mockAccel;
    delete mockGyro;
    delete fakeGPS;
    delete fakeBaro;
}

// Test sensor addition
void test_add_sensors()
{
    sensorManager->addSensor(mockAccel);
    sensorManager->addSensor(mockGyro);

    TEST_ASSERT_EQUAL(2, sensorManager->getCount());
}

// Test sensor array setting
void test_set_sensors()
{
    Sensor *sensors[4] = {mockAccel, mockGyro, fakeGPS, fakeBaro};
    sensorManager->setSensors(sensors, 4);

    TEST_ASSERT_EQUAL(4, sensorManager->getCount());
}

// Test sensor initialization
void test_init_all()
{
    Sensor *sensors[4] = {mockAccel, mockGyro, fakeGPS, fakeBaro};
    sensorManager->setSensors(sensors, 4);

    bool result = sensorManager->initAll();

    TEST_ASSERT_TRUE(mockAccel->isInitialized());
    TEST_ASSERT_TRUE(mockGyro->isInitialized());
    TEST_ASSERT_TRUE(fakeGPS->isInitialized());
    TEST_ASSERT_TRUE(fakeBaro->isInitialized());
}

// Test typed sensor getters
void test_typed_sensor_getters()
{
    Sensor *sensors[4] = {mockAccel, mockGyro, fakeGPS, fakeBaro};
    sensorManager->setSensors(sensors, 4);
    sensorManager->initAll();

    TEST_ASSERT_NOT_NULL(sensorManager->getAccel());
    TEST_ASSERT_NOT_NULL(sensorManager->getGyro());
    TEST_ASSERT_NOT_NULL(sensorManager->getGPS());
    TEST_ASSERT_NOT_NULL(sensorManager->getBaro());
    TEST_ASSERT_NULL(sensorManager->getMag()); // No mag sensor added
}

// Test sensor updates
void test_update_all()
{
    Sensor *sensors[2] = {mockAccel, mockGyro};
    sensorManager->setSensors(sensors, 2);
    sensorManager->initAll();

    // Set some data
    mockAccel->setAccel(1.0, 2.0, 3.0);
    mockGyro->setAngVel(0.1, 0.2, 0.3);

    sensorManager->updateAll();

    // Verify data extraction via typed getters
    Accel *accel = sensorManager->getAccel();
    Gyro *gyro = sensorManager->getGyro();

    TEST_ASSERT_NOT_NULL(accel);
    TEST_ASSERT_NOT_NULL(gyro);

    Vector<3> accelData = accel->getAccel();
    Vector<3> gyroData = gyro->getAngVel();

    TEST_ASSERT_EQUAL_FLOAT(1.0, accelData.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, accelData.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, accelData.z());

    TEST_ASSERT_EQUAL_FLOAT(0.1, gyroData.x());
    TEST_ASSERT_EQUAL_FLOAT(0.2, gyroData.y());
    TEST_ASSERT_EQUAL_FLOAT(0.3, gyroData.z());
}

// Test accel data extraction
void test_get_accel_data()
{
    Sensor *sensors[1] = {mockAccel};
    sensorManager->setSensors(sensors, 1);
    sensorManager->initAll();

    mockAccel->setAccel(9.81, 0.0, 0.0);

    Accel *accel = sensorManager->getAccel();
    TEST_ASSERT_NOT_NULL(accel);

    Vector<3> accelData = accel->getAccel();
    TEST_ASSERT_EQUAL_FLOAT(9.81, accelData.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, accelData.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, accelData.z());
}

// Test gyro data extraction
void test_get_gyro_data()
{
    Sensor *sensors[1] = {mockGyro};
    sensorManager->setSensors(sensors, 1);
    sensorManager->initAll();

    mockGyro->setAngVel(0.5, 1.0, 1.5);

    Gyro *gyro = sensorManager->getGyro();
    TEST_ASSERT_NOT_NULL(gyro);

    Vector<3> gyroData = gyro->getAngVel();
    TEST_ASSERT_EQUAL_FLOAT(0.5, gyroData.x());
    TEST_ASSERT_EQUAL_FLOAT(1.0, gyroData.y());
    TEST_ASSERT_EQUAL_FLOAT(1.5, gyroData.z());
}

// Test GPS data extraction
void test_get_gps_data()
{
    Sensor *sensors[1] = {fakeGPS};
    sensorManager->setSensors(sensors, 1);
    sensorManager->initAll();

    // Set GPS data AFTER initAll() since begin() resets it
    fakeGPS->set(34.0522, -118.2437, 100.0);

    GPS *gps = sensorManager->getGPS();
    TEST_ASSERT_NOT_NULL(gps);

    Vector<3> gpsPos = gps->getPos();
    TEST_ASSERT_EQUAL_FLOAT(34.0522, gpsPos.x());
    TEST_ASSERT_EQUAL_FLOAT(-118.2437, gpsPos.y());
    TEST_ASSERT_EQUAL_FLOAT(100.0, gpsPos.z());
}

// Test GPS heading extraction
void test_get_gps_heading()
{
    Sensor *sensors[1] = {fakeGPS};
    sensorManager->setSensors(sensors, 1);
    sensorManager->initAll();

    // Set GPS data AFTER initAll() since begin() resets it
    fakeGPS->setHeading(90.0);

    GPS *gps = sensorManager->getGPS();
    TEST_ASSERT_NOT_NULL(gps);
    TEST_ASSERT_EQUAL_FLOAT(90.0, gps->getHeading());
}

// Test barometer altitude extraction
void test_get_baro_altitude()
{
    Sensor *sensors[1] = {fakeBaro};
    sensorManager->setSensors(sensors, 1);
    sensorManager->initAll();

    Barometer *baro = sensorManager->getBaro();
    TEST_ASSERT_NOT_NULL(baro);
    // Just verify we can call getASLAltM without crashing
    baro->getASLAltM();
}

// Test sensor retrieval by type hash
void test_get_sensor_by_type()
{
    Sensor *sensors[4] = {mockAccel, mockGyro, fakeGPS, fakeBaro};
    sensorManager->setSensors(sensors, 4);

    Sensor *accel = sensorManager->getSensor("Accelerometer"_i);
    Sensor *gyro = sensorManager->getSensor("Gyroscope"_i);
    Sensor *gps = sensorManager->getSensor("GPS"_i);
    Sensor *baro = sensorManager->getSensor("Barometer"_i);

    TEST_ASSERT_NOT_NULL(accel);
    TEST_ASSERT_NOT_NULL(gyro);
    TEST_ASSERT_NOT_NULL(gps);
    TEST_ASSERT_NOT_NULL(baro);

    TEST_ASSERT_EQUAL_PTR(mockAccel, accel);
    TEST_ASSERT_EQUAL_PTR(mockGyro, gyro);
    TEST_ASSERT_EQUAL_PTR(fakeGPS, gps);
    TEST_ASSERT_EQUAL_PTR(fakeBaro, baro);
}

// Test missing sensor returns nullptr
void test_missing_sensor_returns_nullptr()
{
    // Empty sensor manager
    sensorManager->initAll();

    TEST_ASSERT_NULL(sensorManager->getAccel());
    TEST_ASSERT_NULL(sensorManager->getGyro());
    TEST_ASSERT_NULL(sensorManager->getGPS());
    TEST_ASSERT_NULL(sensorManager->getBaro());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_add_sensors);
    RUN_TEST(test_set_sensors);
    RUN_TEST(test_init_all);
    RUN_TEST(test_typed_sensor_getters);
    RUN_TEST(test_update_all);
    RUN_TEST(test_get_accel_data);
    RUN_TEST(test_get_gyro_data);
    RUN_TEST(test_get_gps_data);
    RUN_TEST(test_get_gps_heading);
    RUN_TEST(test_get_baro_altitude);
    RUN_TEST(test_get_sensor_by_type);
    RUN_TEST(test_missing_sensor_returns_nullptr);

    UNITY_END();

    return 0;
}
