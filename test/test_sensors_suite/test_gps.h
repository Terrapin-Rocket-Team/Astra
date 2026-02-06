#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"

namespace test_gps {

FakeGPS gps;
constexpr double lat1 = 38.9869;
constexpr double lon1 = -76.9426; // College Park, MD
constexpr double lat2 = 38.8977;
constexpr double lon2 = -77.0365; // Washington, DC

void local_setUp(void)
{
    gps.begin(); // reset the gps before each test
}

void local_tearDown(void)
{
    // clean stuff up after each test here, if needed
}

void test_gps_distance_formula()
{   
    local_setUp();
    gps.begin();
    gps.setFixQual(4);
    gps.set(lat1, lon1, 0);
    gps.update();
    gps.set(lat2, lon2, 0);
    gps.update();
    auto v = gps.getDisplacement(Vector<3>(lat1, lon1, 0));
    double distance = sqrt(v.x() * v.x() + v.y() * v.y());
    TEST_ASSERT_FLOAT_WITHIN(12819.0 / 1000.0 * 5.0, 12819, distance);
    // distance between CP and DC in meters, verified with this site:
    // https://www.gpsvisualizer.com/calculators, testing with difference of 0.05% of the "actual" Vincenty formula distance
    local_tearDown();
}

void test_gps_begin()
{
    local_setUp();
    TEST_ASSERT_EQUAL(0, gps.begin());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().z());
    TEST_ASSERT_EQUAL(0, gps.getFixQual());
    local_tearDown();
}

void test_gps_set()
{
    local_setUp();
    gps.set(lat1, lon1, 0);
    TEST_ASSERT_EQUAL_FLOAT(lat1, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(lon1, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().z());
    local_tearDown();
}

void test_gps_first_fix()
{
    local_setUp();
    gps.begin();
    TEST_ASSERT_FALSE(gps.getHasFix());
    TEST_ASSERT_EQUAL(0, gps.getFixQual());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getHeading());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().z());

    gps.set(lat1, lon1, 0);
    gps.update();

    TEST_ASSERT_FALSE(gps.getHasFix());
    TEST_ASSERT_EQUAL(0, gps.getFixQual());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getHeading());
    TEST_ASSERT_EQUAL_FLOAT(lat1, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(lon1, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().z());

    gps.setFixQual(4);
    gps.update();

    TEST_ASSERT_TRUE(gps.getHasFix());
    TEST_ASSERT_EQUAL(4, gps.getFixQual());
    TEST_ASSERT_EQUAL_FLOAT(lat1, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(lon1, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(0, gps.getPos().z());

    gps.set(lat2, lon2, 1000);
    gps.update();

    TEST_ASSERT_TRUE(gps.getHasFix());
    TEST_ASSERT_EQUAL(4, gps.getFixQual());
    TEST_ASSERT_EQUAL_FLOAT(lat2, gps.getPos().x());
    TEST_ASSERT_EQUAL_FLOAT(lon2, gps.getPos().y());
    TEST_ASSERT_EQUAL_FLOAT(1000, gps.getPos().z());
    local_tearDown();
}

void run_test_gps_tests()
{
    RUN_TEST(test_gps_distance_formula);
    RUN_TEST(test_gps_begin);
    RUN_TEST(test_gps_set);
    RUN_TEST(test_gps_first_fix);
}

} // namespace test_gps
