#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

namespace test_sensor {


void local_setUp(void)
{
    // set stuff up before each test here, if needed
}

void local_tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_constructor() {
    local_setUp();
    FakeBarometer fake;
    TEST_ASSERT_EQUAL_STRING("FakeBarometer", fake.getName());
    // Note: Sensor typing system (getType/getTypeString) has been removed
    local_tearDown();
}

void run_test_sensor_tests()
{
    RUN_TEST(test_constructor);
}

} // namespace test_sensor
