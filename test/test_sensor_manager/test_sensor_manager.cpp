#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"
#include <Sensors/SensorManager/SensorManager.h>

using namespace astra;

// Test fixtures
SensorManager sensorManager;
FakeAccel accel;
FakeGyro gyro;
FakeMag mag;
FakeBarometer baro;
FakeGPS gps;

void setUp(void)
{
    // Reset sensor manager (create new instance)
    sensorManager = SensorManager();
}

void tearDown(void)
{
    // Clean up
}

// Test that empty SensorManager is not OK
void test_sensor_manager_empty_not_ok()
{
    TEST_ASSERT_FALSE(sensorManager.isOK());
}

// Test setting and getting primary accelerometer
void test_sensor_manager_set_get_primary_accel()
{
    sensorManager.setPrimaryAccel(&accel);

    Accel* retrievedAccel = sensorManager.getPrimaryAccel();
    TEST_ASSERT_NOT_NULL(retrievedAccel);
    TEST_ASSERT_EQUAL_PTR(&accel, retrievedAccel);
}

// Test setting and getting primary gyroscope
void test_sensor_manager_set_get_primary_gyro()
{
    sensorManager.setPrimaryGyro(&gyro);

    Gyro* retrievedGyro = sensorManager.getPrimaryGyro();
    TEST_ASSERT_NOT_NULL(retrievedGyro);
    TEST_ASSERT_EQUAL_PTR(&gyro, retrievedGyro);
}

// Test setting and getting primary magnetometer
void test_sensor_manager_set_get_primary_mag()
{
    sensorManager.setPrimaryMag(&mag);

    Mag* retrievedMag = sensorManager.getPrimaryMag();
    TEST_ASSERT_NOT_NULL(retrievedMag);
    TEST_ASSERT_EQUAL_PTR(&mag, retrievedMag);
}

// Test setting and getting primary barometer
void test_sensor_manager_set_get_primary_baro()
{
    sensorManager.setPrimaryBaro(&baro);

    Barometer* retrievedBaro = sensorManager.getPrimaryBaro();
    TEST_ASSERT_NOT_NULL(retrievedBaro);
    TEST_ASSERT_EQUAL_PTR(&baro, retrievedBaro);
}

// Test setting and getting primary GPS
void test_sensor_manager_set_get_primary_gps()
{
    sensorManager.setPrimaryGPS(&gps);

    GPS* retrievedGPS = sensorManager.getPrimaryGPS();
    TEST_ASSERT_NOT_NULL(retrievedGPS);
    TEST_ASSERT_EQUAL_PTR(&gps, retrievedGPS);
}

// Test isOK with only accel (minimum requirement)
void test_sensor_manager_is_ok_with_accel_only()
{
    sensorManager.setPrimaryAccel(&accel);
    accel.begin();

    TEST_ASSERT_TRUE(sensorManager.isOK());
}

// Test isOK with all primary sensors
void test_sensor_manager_is_ok_with_all_sensors()
{
    sensorManager.setPrimaryAccel(&accel);
    sensorManager.setPrimaryGyro(&gyro);
    sensorManager.setPrimaryMag(&mag);
    sensorManager.setPrimaryBaro(&baro);
    sensorManager.setPrimaryGPS(&gps);

    accel.begin();
    gyro.begin();
    mag.begin();
    baro.begin();
    gps.begin();

    TEST_ASSERT_TRUE(sensorManager.isOK());
}

// Test isOK fails when accel not initialized
void test_sensor_manager_not_ok_accel_not_initialized()
{
    sensorManager.setPrimaryAccel(&accel);
    // Don't call begin()

    TEST_ASSERT_FALSE(sensorManager.isOK());
}

// Test begin() initializes all primary sensors
void test_sensor_manager_begin_all_sensors()
{
    sensorManager.setPrimaryAccel(&accel);
    sensorManager.setPrimaryGyro(&gyro);
    sensorManager.setPrimaryMag(&mag);
    sensorManager.setPrimaryBaro(&baro);
    sensorManager.setPrimaryGPS(&gps);

    TEST_ASSERT_TRUE(sensorManager.begin());
    TEST_ASSERT_TRUE(sensorManager.isOK());
}

// Test begin() with only some sensors
void test_sensor_manager_begin_partial_sensors()
{
    sensorManager.setPrimaryAccel(&accel);
    sensorManager.setPrimaryGyro(&gyro);
    // No mag, baro, or GPS

    TEST_ASSERT_TRUE(sensorManager.begin());
    TEST_ASSERT_TRUE(sensorManager.isOK());
}

// Test adding misc sensors
void test_sensor_manager_add_misc_sensors()
{
    FakeAccel misc1;
    FakeAccel misc2;

    misc1.begin();
    misc2.begin();

    TEST_ASSERT_TRUE(sensorManager.addMiscSensor(&misc1));
    TEST_ASSERT_TRUE(sensorManager.addMiscSensor(&misc2));
}

// Test adding null misc sensor fails
void test_sensor_manager_add_null_misc_sensor()
{
    TEST_ASSERT_FALSE(sensorManager.addMiscSensor(nullptr));
}

// Test adding too many misc sensors
void test_sensor_manager_add_too_many_misc_sensors()
{
    FakeAccel miscSensors[17]; // MAX_MISC_SENSORS is 16

    // Add 16 sensors successfully
    for (int i = 0; i < 16; i++)
    {
        miscSensors[i].begin();
        TEST_ASSERT_TRUE(sensorManager.addMiscSensor(&miscSensors[i]));
    }

    // 17th should fail
    miscSensors[16].begin();
    TEST_ASSERT_FALSE(sensorManager.addMiscSensor(&miscSensors[16]));
}

// Test update() calls all primary sensors
void test_sensor_manager_update_all_sensors()
{
    sensorManager.setPrimaryAccel(&accel);
    sensorManager.setPrimaryGyro(&gyro);
    sensorManager.setPrimaryBaro(&baro);

    accel.begin();
    gyro.begin();
    baro.begin();

    accel.set(Vector<3>{1.0, 2.0, 3.0});
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    baro.set(101325.0, 20.0);

    sensorManager.update();

    // Verify data is accessible
    Vector<3> accelData = sensorManager.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, accelData.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, accelData.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, accelData.z());
}

// Test getAccel() returns zero vector when no accel
void test_sensor_manager_get_accel_no_sensor()
{
    Vector<3> result = sensorManager.getAccel();

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test getAccel() returns correct data
void test_sensor_manager_get_accel()
{
    sensorManager.setPrimaryAccel(&accel);
    accel.begin();
    accel.set(Vector<3>{5.0, 6.0, 7.0});
    sensorManager.update();

    Vector<3> result = sensorManager.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(6.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(7.0, result.z());
}

// Test getGyro() returns zero vector when no gyro
void test_sensor_manager_get_gyro_no_sensor()
{
    Vector<3> result = sensorManager.getGyro();

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test getGyro() returns correct data
void test_sensor_manager_get_gyro()
{
    sensorManager.setPrimaryGyro(&gyro);
    gyro.begin();
    gyro.set(Vector<3>{0.5, 0.6, 0.7});
    sensorManager.update();

    Vector<3> result = sensorManager.getGyro();
    TEST_ASSERT_EQUAL_FLOAT(0.5, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.6, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.7, result.z());
}

// Test getMag() returns zero vector when no mag
void test_sensor_manager_get_mag_no_sensor()
{
    Vector<3> result = sensorManager.getMag();

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test getMag() returns correct data
void test_sensor_manager_get_mag()
{
    sensorManager.setPrimaryMag(&mag);
    mag.begin();
    mag.set(Vector<3>{25.0, 30.0, 45.0});
    sensorManager.update();

    Vector<3> result = sensorManager.getMag();
    TEST_ASSERT_EQUAL_FLOAT(25.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(30.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(45.0, result.z());
}

// Test getPressure() returns zero when no baro
void test_sensor_manager_get_pressure_no_sensor()
{
    double result = sensorManager.getPressure();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result);
}

// Test getPressure() returns correct data
void test_sensor_manager_get_pressure()
{
    sensorManager.setPrimaryBaro(&baro);
    baro.begin();
    baro.set(101325.0, 20.0);
    sensorManager.update();

    double result = sensorManager.getPressure();
    TEST_ASSERT_EQUAL_FLOAT(101325.0, result);
}

// Test getTemp() returns zero when no baro
void test_sensor_manager_get_temp_no_sensor()
{
    double result = sensorManager.getTemp();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result);
}

// Test getTemp() returns correct data
void test_sensor_manager_get_temp()
{
    sensorManager.setPrimaryBaro(&baro);
    baro.begin();
    baro.set(101325.0, 25.5);
    sensorManager.update();

    double result = sensorManager.getTemp();
    TEST_ASSERT_EQUAL_FLOAT(25.5, result);
}

// Test getGPSPos() returns zero vector when no GPS
void test_sensor_manager_get_gps_pos_no_sensor()
{
    Vector<3> result = sensorManager.getGPSPos();

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
}

// Test getGPSPos() returns correct data
void test_sensor_manager_get_gps_pos()
{
    sensorManager.setPrimaryGPS(&gps);
    gps.begin();
    gps.set(40.7128, -74.0060, 10.0); // NYC coordinates
    sensorManager.update();

    Vector<3> result = sensorManager.getGPSPos();
    TEST_ASSERT_EQUAL_FLOAT(40.7128, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-74.0060, result.y());
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.z());
}

// Test getHeading() returns zero when no GPS
void test_sensor_manager_get_heading_no_sensor()
{
    double result = sensorManager.getHeading();
    TEST_ASSERT_EQUAL_FLOAT(0.0, result);
}

// Test getHeading() returns correct data
void test_sensor_manager_get_heading()
{
    sensorManager.setPrimaryGPS(&gps);
    gps.begin();
    gps.setHeading(270.5);
    sensorManager.update();

    double result = sensorManager.getHeading();
    TEST_ASSERT_EQUAL_FLOAT(270.5, result);
}

// Test getTimeOfDay() returns empty string when no GPS
void test_sensor_manager_get_time_of_day_no_sensor()
{
    const char* result = sensorManager.getTimeOfDay();
    TEST_ASSERT_EQUAL_STRING("", result);
}

// Test getTimeOfDay() returns correct data
void test_sensor_manager_get_time_of_day()
{
    sensorManager.setPrimaryGPS(&gps);
    gps.begin();
    gps.setDateTime(2024, 1, 15, 14, 30, 45);
    sensorManager.update();

    const char* result = sensorManager.getTimeOfDay();
    TEST_ASSERT_EQUAL_STRING("14:30:45", result);
}

// Test full integration with all sensors
void test_sensor_manager_full_integration()
{
    // Configure all sensors
    sensorManager.setPrimaryAccel(&accel);
    sensorManager.setPrimaryGyro(&gyro);
    sensorManager.setPrimaryMag(&mag);
    sensorManager.setPrimaryBaro(&baro);
    sensorManager.setPrimaryGPS(&gps);

    // Initialize
    TEST_ASSERT_TRUE(sensorManager.begin());
    TEST_ASSERT_TRUE(sensorManager.isOK());

    // Set data
    accel.set(Vector<3>{1.0, 2.0, 3.0});
    gyro.set(Vector<3>{0.1, 0.2, 0.3});
    mag.set(Vector<3>{25.0, 30.0, 45.0});
    baro.set(101325.0, 20.0);
    gps.set(40.7128, -74.0060, 10.0);
    gps.setHeading(180.0);

    // Update all
    sensorManager.update();

    // Verify all data
    Vector<3> accelData = sensorManager.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, accelData.x());

    Vector<3> gyroData = sensorManager.getGyro();
    TEST_ASSERT_EQUAL_FLOAT(0.1, gyroData.x());

    Vector<3> magData = sensorManager.getMag();
    TEST_ASSERT_EQUAL_FLOAT(25.0, magData.x());

    TEST_ASSERT_EQUAL_FLOAT(101325.0, sensorManager.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(20.0, sensorManager.getTemp());

    Vector<3> gpsPos = sensorManager.getGPSPos();
    TEST_ASSERT_EQUAL_FLOAT(40.7128, gpsPos.x());

    TEST_ASSERT_EQUAL_FLOAT(180.0, sensorManager.getHeading());
}

// Test update with misc sensors
void test_sensor_manager_update_with_misc_sensors()
{
    FakeAccel misc1;
    FakeAccel misc2;

    misc1.begin();
    misc2.begin();

    sensorManager.setPrimaryAccel(&accel);
    accel.begin();

    sensorManager.addMiscSensor(&misc1);
    sensorManager.addMiscSensor(&misc2);

    misc1.set(Vector<3>{10.0, 20.0, 30.0});
    misc2.set(Vector<3>{40.0, 50.0, 60.0});

    // Update should update misc sensors too
    sensorManager.update();

    // Misc sensors should have been updated
    Vector<3> misc1Data = misc1.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(10.0, misc1Data.x());

    Vector<3> misc2Data = misc2.getAccel();
    TEST_ASSERT_EQUAL_FLOAT(40.0, misc2Data.x());
}

// Test that null sensors don't crash update
void test_sensor_manager_update_handles_nulls()
{
    // Don't set any sensors
    sensorManager.update(); // Should not crash

    // Set only some sensors
    sensorManager.setPrimaryAccel(&accel);
    accel.begin();
    sensorManager.update(); // Should not crash
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_sensor_manager_empty_not_ok);
    RUN_TEST(test_sensor_manager_set_get_primary_accel);
    RUN_TEST(test_sensor_manager_set_get_primary_gyro);
    RUN_TEST(test_sensor_manager_set_get_primary_mag);
    RUN_TEST(test_sensor_manager_set_get_primary_baro);
    RUN_TEST(test_sensor_manager_set_get_primary_gps);
    RUN_TEST(test_sensor_manager_is_ok_with_accel_only);
    RUN_TEST(test_sensor_manager_is_ok_with_all_sensors);
    RUN_TEST(test_sensor_manager_not_ok_accel_not_initialized);
    RUN_TEST(test_sensor_manager_begin_all_sensors);
    RUN_TEST(test_sensor_manager_begin_partial_sensors);
    RUN_TEST(test_sensor_manager_add_misc_sensors);
    RUN_TEST(test_sensor_manager_add_null_misc_sensor);
    RUN_TEST(test_sensor_manager_add_too_many_misc_sensors);
    RUN_TEST(test_sensor_manager_update_all_sensors);
    RUN_TEST(test_sensor_manager_get_accel_no_sensor);
    RUN_TEST(test_sensor_manager_get_accel);
    RUN_TEST(test_sensor_manager_get_gyro_no_sensor);
    RUN_TEST(test_sensor_manager_get_gyro);
    RUN_TEST(test_sensor_manager_get_mag_no_sensor);
    RUN_TEST(test_sensor_manager_get_mag);
    RUN_TEST(test_sensor_manager_get_pressure_no_sensor);
    RUN_TEST(test_sensor_manager_get_pressure);
    RUN_TEST(test_sensor_manager_get_temp_no_sensor);
    RUN_TEST(test_sensor_manager_get_temp);
    RUN_TEST(test_sensor_manager_get_gps_pos_no_sensor);
    RUN_TEST(test_sensor_manager_get_gps_pos);
    RUN_TEST(test_sensor_manager_get_heading_no_sensor);
    RUN_TEST(test_sensor_manager_get_heading);
    RUN_TEST(test_sensor_manager_get_time_of_day_no_sensor);
    RUN_TEST(test_sensor_manager_get_time_of_day);
    RUN_TEST(test_sensor_manager_full_integration);
    RUN_TEST(test_sensor_manager_update_with_misc_sensors);
    RUN_TEST(test_sensor_manager_update_handles_nulls);

    UNITY_END();
    return 0;
}
