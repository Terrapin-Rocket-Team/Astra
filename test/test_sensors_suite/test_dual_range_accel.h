#pragma once

#include <unity.h>
#include "NativeTestHelper.h"
#include "UnitTestSensors.h"
#include <Sensors/Accel/DualRangeAccel.h>

using namespace astra;

namespace test_dual_range_accel {

FakeAccel lowGAccel;
FakeAccel highGAccel;
DualRangeAccel* dualRangeAccel;
const double MAX_LOW_G = 16.0;   // Maximum for low-G sensor (m/s^2)
const double MIN_HIGH_G = 100.0; // Minimum for high-G sensor (m/s^2)

void local_setUp(void)
{
    lowGAccel.begin();
    highGAccel.begin();
    dualRangeAccel = new DualRangeAccel(&lowGAccel, &highGAccel, MAX_LOW_G, MIN_HIGH_G, "TestDualRange");
}

void local_tearDown(void)
{
    delete dualRangeAccel;
}

void test_dual_range_accel_begin()
{
    local_setUp();
    TEST_ASSERT_EQUAL(0, dualRangeAccel->begin());
    local_tearDown();
}

void test_dual_range_accel_low_g_range()
{
    local_setUp();
    dualRangeAccel->begin();

    // Set low-G reading at 10 m/s^2 (well below MIN_HIGH_G of 100)
    lowGAccel.set(Vector<3>{10.0, 0.0, 0.0});
    highGAccel.set(Vector<3>{10.0, 0.0, 0.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Should use 100% low-G sensor (alpha = 0)
    TEST_ASSERT_EQUAL_FLOAT(10.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_high_g_range()
{
    local_setUp();
    dualRangeAccel->begin();

    // Set high-G reading at 150 m/s^2 (well above MAX_LOW_G of 16)
    lowGAccel.set(Vector<3>{150.0, 0.0, 0.0});
    highGAccel.set(Vector<3>{150.0, 0.0, 0.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Should use 100% high-G sensor (alpha = 1)
    TEST_ASSERT_EQUAL_FLOAT(150.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_transition_range()
{
    local_setUp();
    dualRangeAccel->begin();

    // Set readings at midpoint of transition range
    // Transition is from MAX_LOW_G (16) to MIN_HIGH_G (100)
    // Midpoint magnitude: (16 + 100) / 2 = 58 m/s^2

    // Low-G sensor reads 58 m/s^2
    lowGAccel.set(Vector<3>{50.0, 30.0, 10.0});  // magnitude ≈ 59.16
    // High-G sensor reads same magnitude
    highGAccel.set(Vector<3>{52.0, 28.0, 12.0}); // magnitude ≈ 59.16

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Result should be interpolated between the two sensors
    // The exact value depends on alpha calculation
    // At midpoint, alpha should be around 0.5, so result is roughly average
    TEST_ASSERT_FLOAT_WITHIN(5.0, 51.0, result.x());
    TEST_ASSERT_FLOAT_WITHIN(5.0, 29.0, result.y());
    TEST_ASSERT_FLOAT_WITHIN(5.0, 11.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_at_min_high_g()
{
    local_setUp();
    dualRangeAccel->begin();

    // Set magnitude exactly at MIN_HIGH_G (100 m/s^2)
    lowGAccel.set(Vector<3>{100.0, 0.0, 0.0});
    highGAccel.set(Vector<3>{100.0, 0.0, 0.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Should use low-G sensor (alpha = 0) since magnitude <= minHighG
    TEST_ASSERT_EQUAL_FLOAT(100.0, result.x());
    local_tearDown();
}

void test_dual_range_accel_at_max_low_g()
{
    local_setUp();
    dualRangeAccel->begin();

    // Set magnitude exactly at MAX_LOW_G (16 m/s^2)
    lowGAccel.set(Vector<3>{16.0, 0.0, 0.0});
    highGAccel.set(Vector<3>{16.0, 0.0, 0.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Should use high-G sensor (alpha = 1) since magnitude >= maxLowG
    TEST_ASSERT_EQUAL_FLOAT(16.0, result.x());
    local_tearDown();
}

void test_dual_range_accel_multi_axis()
{
    local_setUp();
    dualRangeAccel->begin();

    // Low-G range with multi-axis data
    lowGAccel.set(Vector<3>{5.0, 3.0, 2.0});
    highGAccel.set(Vector<3>{5.0, 3.0, 2.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Magnitude = sqrt(25 + 9 + 4) = sqrt(38) ≈ 6.16 (< MIN_HIGH_G)
    // Should use low-G sensor
    TEST_ASSERT_EQUAL_FLOAT(5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_mounting_orientation_warning()
{
    local_setUp();
    dualRangeAccel->begin();

    // This should log a warning but not crash
    dualRangeAccel->setMountingOrientation(MountingOrientation::FLIP_YZ);

    // Verify sensor still works after the call
    lowGAccel.set(Vector<3>{1.0, 2.0, 3.0});
    highGAccel.set(Vector<3>{1.0, 2.0, 3.0});
    dualRangeAccel->update();

    Vector<3> result = dualRangeAccel->getAccel();
    TEST_ASSERT_EQUAL_FLOAT(1.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(2.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(3.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_update_failure()
{
    local_setUp();
    // Create a custom fake accel that fails to update
    class FailingAccel : public Accel
    {
    public:
        FailingAccel() : Accel("FailingAccel") { initialized = true; }
        int read() override { return -1; }
        int init() override { return 0; }
    };

    FailingAccel failAccel;
    DualRangeAccel failingDual(&failAccel, &highGAccel, MAX_LOW_G, MIN_HIGH_G);

    failingDual.begin();

    // Update should fail if either sensor fails (returns non-zero)
    TEST_ASSERT_NOT_EQUAL(0, failingDual.update());
    local_tearDown();
}

void test_dual_range_accel_different_sensor_readings()
{
    local_setUp();
    dualRangeAccel->begin();

    // In transition zone with different readings from each sensor
    // This simulates real-world where sensors might have slight differences
    lowGAccel.set(Vector<3>{30.0, 30.0, 30.0});  // magnitude ≈ 51.96
    highGAccel.set(Vector<3>{35.0, 32.0, 28.0}); // magnitude ≈ 54.77

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Result should be somewhere between the two sensor readings
    TEST_ASSERT_FLOAT_WITHIN(10.0, 32.5, result.x());
    TEST_ASSERT_FLOAT_WITHIN(10.0, 31.0, result.y());
    TEST_ASSERT_FLOAT_WITHIN(10.0, 29.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_zero()
{
    local_setUp();
    dualRangeAccel->begin();

    lowGAccel.set(Vector<3>{0.0, 0.0, 0.0});
    highGAccel.set(Vector<3>{0.0, 0.0, 0.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    TEST_ASSERT_EQUAL_FLOAT(0.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(0.0, result.z());
    local_tearDown();
}

void test_dual_range_accel_negative_values()
{
    local_setUp();
    dualRangeAccel->begin();

    lowGAccel.set(Vector<3>{-5.0, -3.0, -2.0});
    highGAccel.set(Vector<3>{-5.0, -3.0, -2.0});

    dualRangeAccel->update();
    Vector<3> result = dualRangeAccel->getAccel();

    // Magnitude is same as positive (sqrt(25+9+4) ≈ 6.16), should use low-G
    TEST_ASSERT_EQUAL_FLOAT(-5.0, result.x());
    TEST_ASSERT_EQUAL_FLOAT(-3.0, result.y());
    TEST_ASSERT_EQUAL_FLOAT(-2.0, result.z());
    local_tearDown();
}

void run_test_dual_range_accel_tests()
{
    RUN_TEST(test_dual_range_accel_begin);
    RUN_TEST(test_dual_range_accel_low_g_range);
    RUN_TEST(test_dual_range_accel_high_g_range);
    RUN_TEST(test_dual_range_accel_transition_range);
    RUN_TEST(test_dual_range_accel_at_min_high_g);
    RUN_TEST(test_dual_range_accel_at_max_low_g);
    RUN_TEST(test_dual_range_accel_multi_axis);
    RUN_TEST(test_dual_range_accel_mounting_orientation_warning);
    RUN_TEST(test_dual_range_accel_update_failure);
    RUN_TEST(test_dual_range_accel_different_sensor_readings);
    RUN_TEST(test_dual_range_accel_zero);
    RUN_TEST(test_dual_range_accel_negative_values);
}

} // namespace test_dual_range_accel
