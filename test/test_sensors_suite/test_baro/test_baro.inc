#include <unity.h>
#include "NativeTestHelper.h"

// include other headers you need to test here

#include "UnitTestSensors.h"

// ---

// Set up and global variables or mocks for testing here

FakeBarometer baro;
const double ALT_AT_500_HPA = 5572.1; // altitude at 500 hPa in meters
const double ALT_AT_750_HPA = 2465.2; // altitude at 750 hPa in meters
const double ALT_AT_0_HPA = 44307.69; // altitude at 0 hPa in meters

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void test_baro_setUp(void)
{
    baro.fakeP = 0;
    baro.fakeT = 0;
    baro.begin(); // reset the barometer before each test
}

void test_baro_tearDown(void)
{
    // clean stuff up after each test here, if needed
}
// ---

// Test functions must be void and take no arguments, put them here

void test_baro_test_baro_begin()
{
    TEST_ASSERT_TRUE(baro.begin());

    TEST_ASSERT_EQUAL_FLOAT(0, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getTemp());
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_0_HPA, baro.getASLAltM());

    baro.set(500, 25);

    TEST_ASSERT_TRUE(baro.begin());

    TEST_ASSERT_EQUAL_FLOAT(500, baro.fakeP);
    TEST_ASSERT_EQUAL_FLOAT(25, baro.fakeT);
    TEST_ASSERT_EQUAL_FLOAT(500, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(25, baro.getTemp());
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA, baro.getASLAltM());
}

void test_baro_test_baro_set()
{
    baro.set(1000, 25);
    TEST_ASSERT_EQUAL_FLOAT(1000, baro.getPressure());
    TEST_ASSERT_EQUAL_FLOAT(25, baro.getTemp());
}

void test_baro_test_baro_alt()
{
    baro.set(1013.25, 25);
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(0, baro.getASLAltM());
    baro.set(500, 25);
    baro.update();
    TEST_ASSERT_EQUAL_FLOAT(ALT_AT_500_HPA, baro.getASLAltM());
}

void test_baro_test_baro_conversions()
{
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
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
// ---