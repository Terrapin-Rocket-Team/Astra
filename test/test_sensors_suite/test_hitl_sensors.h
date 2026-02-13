#pragma once

#include <unity.h>
#include <cstring>
#include "NativeTestHelper.h"
#include "Sensors/HITL/HITLAccel.h"
#include "Sensors/HITL/HITLBarometer.h"
#include "Sensors/HITL/HITLGPS.h"
#include "Sensors/HITL/HITLGyro.h"
#include "Sensors/HITL/HITLMag.h"
#include "Sensors/HITL/HITLSensorBuffer.h"

using namespace astra;

namespace test_hitl_sensors {

class TestableHITLAccel : public HITLAccel
{
public:
    void setInitializedForTest(bool value) { initialized = value; }
    void applyUpdateHealthForTest(int readErr) { updateHealth(readErr, 0.0); }
    bool getHealthForTest() const { return healthy; }
};

class TestableHITLGyro : public HITLGyro
{
public:
    void setInitializedForTest(bool value) { initialized = value; }
    void applyUpdateHealthForTest(int readErr) { updateHealth(readErr, 0.0); }
    bool getHealthForTest() const { return healthy; }
};

class TestableHITLMag : public HITLMag
{
public:
    void setInitializedForTest(bool value) { initialized = value; }
    void applyUpdateHealthForTest(int readErr) { updateHealth(readErr, 0.0); }
    bool getHealthForTest() const { return healthy; }
};

class TestableHITLBarometer : public HITLBarometer
{
public:
    void setInitializedForTest(bool value) { initialized = value; }
    void applyUpdateHealthForTest(int readErr) { updateHealth(readErr, 0.0); }
    bool getHealthForTest() const { return healthy; }
};

class TestableHITLGPS : public HITLGPS
{
public:
    bool hasFirstFixForTest() const { return hasFirstFix; }
};

void local_setUp(void)
{
    HITLSensorBuffer &buffer = HITLSensorBuffer::instance();
    buffer.dataReady = false;
    memset(&buffer.data, 0, sizeof(buffer.data));
}

void local_tearDown(void)
{
}

void test_hitl_accel_updateHealth_paths()
{
    local_setUp();
    TestableHITLAccel accel;

    accel.setInitializedForTest(true);
    accel.applyUpdateHealthForTest(0);
    TEST_ASSERT_TRUE(accel.getHealthForTest());

    accel.applyUpdateHealthForTest(-1);
    TEST_ASSERT_FALSE(accel.getHealthForTest());

    accel.setInitializedForTest(false);
    accel.applyUpdateHealthForTest(0);
    TEST_ASSERT_FALSE(accel.getHealthForTest());
    local_tearDown();
}

void test_hitl_gyro_updateHealth_paths()
{
    local_setUp();
    TestableHITLGyro gyro;

    gyro.setInitializedForTest(true);
    gyro.applyUpdateHealthForTest(0);
    TEST_ASSERT_TRUE(gyro.getHealthForTest());

    gyro.applyUpdateHealthForTest(-1);
    TEST_ASSERT_FALSE(gyro.getHealthForTest());

    gyro.setInitializedForTest(false);
    gyro.applyUpdateHealthForTest(0);
    TEST_ASSERT_FALSE(gyro.getHealthForTest());
    local_tearDown();
}

void test_hitl_mag_updateHealth_paths()
{
    local_setUp();
    TestableHITLMag mag;

    mag.setInitializedForTest(true);
    mag.applyUpdateHealthForTest(0);
    TEST_ASSERT_TRUE(mag.getHealthForTest());

    mag.applyUpdateHealthForTest(-1);
    TEST_ASSERT_FALSE(mag.getHealthForTest());

    mag.setInitializedForTest(false);
    mag.applyUpdateHealthForTest(0);
    TEST_ASSERT_FALSE(mag.getHealthForTest());
    local_tearDown();
}

void test_hitl_barometer_updateHealth_paths()
{
    local_setUp();
    TestableHITLBarometer baro;

    baro.setInitializedForTest(true);
    baro.applyUpdateHealthForTest(0);
    TEST_ASSERT_TRUE(baro.getHealthForTest());

    baro.applyUpdateHealthForTest(-1);
    TEST_ASSERT_FALSE(baro.getHealthForTest());

    baro.setInitializedForTest(false);
    baro.applyUpdateHealthForTest(0);
    TEST_ASSERT_FALSE(baro.getHealthForTest());
    local_tearDown();
}

void test_hitl_gps_first_fix_only_once()
{
    local_setUp();
    HITLSensorBuffer &buffer = HITLSensorBuffer::instance();
    TestableHITLGPS gps;

    TEST_ASSERT_EQUAL(0, gps.begin());
    TEST_ASSERT_FALSE(gps.hasFirstFixForTest());

    buffer.data.gps_lat = 39.9;
    buffer.data.gps_lon = -119.9;
    buffer.data.gps_alt = 90.0;
    buffer.data.gps_fix = 0;
    buffer.data.gps_fix_quality = 0;
    buffer.data.gps_heading = 0.0;
    TEST_ASSERT_EQUAL(0, gps.update());
    TEST_ASSERT_FALSE(gps.hasFirstFixForTest());
    TEST_ASSERT_FALSE(gps.getHasFix());

    buffer.data.gps_lat = 40.0;
    buffer.data.gps_lon = -120.0;
    buffer.data.gps_alt = 100.0;
    buffer.data.gps_fix = 1;
    buffer.data.gps_fix_quality = 8;
    buffer.data.gps_heading = 90.0;
    TEST_ASSERT_EQUAL(0, gps.update());
    TEST_ASSERT_TRUE(gps.hasFirstFixForTest());
    TEST_ASSERT_TRUE(gps.getHasFix());

    buffer.data.gps_lat = 40.1;
    buffer.data.gps_lon = -120.1;
    buffer.data.gps_alt = 150.0;
    buffer.data.gps_fix = 1;
    buffer.data.gps_fix_quality = 10;
    buffer.data.gps_heading = 120.0;
    TEST_ASSERT_EQUAL(0, gps.update());
    TEST_ASSERT_TRUE(gps.hasFirstFixForTest());
    TEST_ASSERT_TRUE(gps.getHasFix());
    TEST_ASSERT_EQUAL(10, gps.getFixQual());
    local_tearDown();
}

void run_test_hitl_sensors_tests()
{
    RUN_TEST(test_hitl_accel_updateHealth_paths);
    RUN_TEST(test_hitl_gyro_updateHealth_paths);
    RUN_TEST(test_hitl_mag_updateHealth_paths);
    RUN_TEST(test_hitl_barometer_updateHealth_paths);
    RUN_TEST(test_hitl_gps_first_fix_only_once);
}

} // namespace test_hitl_sensors
