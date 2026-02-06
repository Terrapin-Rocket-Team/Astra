#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

namespace test_baro {

FakeBarometer baro;
const double ALT_AT_500_HPA = 5572.1; // altitude at 500 hPa in meters
const double ALT_AT_750_HPA = 2465.2; // altitude at 750 hPa in meters
const double ALT_AT_0_HPA = 44307.69; // altitude at 0 hPa in meters

void local_setUp(void)
{
    baro.fakeP = 0;
    baro.fakeT = 0;
    baro.begin(); // reset the barometer before each test
}

void local_tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_baro_begin()
{
    local_setUp();
    TEST_ASSERT_EQUAL(0, baro.begin());

    TEST_ASSERT_EQUAL_FLOAT(0, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getTemp());
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_0_HPA, baro.getASLAltM());

    baro.set(500, 25);

    TEST_ASSERT_EQUAL(0, baro.begin());

    TEST_ASSERT_EQUAL_FLOAT(500, baro.fakeP);
    TEST_ASSERT_EQUAL_FLOAT(25, baro.fakeT);
    TEST_ASSERT_EQUAL_FLOAT(500, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(25, baro.getTemp());
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA, baro.getASLAltM());
    local_tearDown();
}

void test_baro_set()
{
    local_setUp();
    baro.set(1000, 25);
    TEST_ASSERT_EQUAL_FLOAT(1000, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(25, baro.getTemp());
    local_tearDown();
}

void test_baro_alt()
{
    local_setUp();
    baro.set(1013.25, 25);
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getASLAltM());
    baro.set(500, 25);
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA, baro.getASLAltM());
    local_tearDown();
}

void test_baro_conversions()
{
    local_setUp();
    baro.set(1013.25, 25);
    baro.begin();
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(1013.25, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(1, baro.getPressureAtm());
    TEST_ASSERT_EQUAL_FLOAT(25, baro.getTemp());
    TEST_ASSERT_EQUAL_FLOAT(77, baro.getTempF());
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getASLAltM());
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getASLAltFt());
    baro.set(500, 25);
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA, baro.getASLAltM());
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA * 3.28084, baro.getASLAltFt());
    local_tearDown();
}

void run_test_baro_tests()
{
    RUN_TEST(test_baro_begin);
    RUN_TEST(test_baro_set);
    RUN_TEST(test_baro_alt);
    RUN_TEST(test_baro_conversions);
}

} // namespace test_baro
