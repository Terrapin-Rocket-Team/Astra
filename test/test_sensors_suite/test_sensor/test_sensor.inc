#include <unity.h>
#include "NativeTestHelper.h"

// include other headers you need to test here

#include "UnitTestSensors.h"

// ---

// Set up and global variables or mocks for testing here

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void test_sensor_setUp(void)
{
    // set stuff up before each test here, if needed
}

void test_sensor_tearDown(void)
{
    // clean stuff up after each test here, if needed
}
// ---

// Test functions must be void and take no arguments, put them here

void test_sensor_test_constructor() {
    FakeBarometer fake;
    TEST_ASSERT_EQUAL_STRING("FakeBarometer", fake.getName());
    // Note: Sensor typing system (getType/getTypeString) has been removed
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
// ---