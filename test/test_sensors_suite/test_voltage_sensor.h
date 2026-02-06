#pragma once

#include <unity.h>
#include "UnitTestSensors.h"
#include "NativeTestHelper.h"

using namespace astra;

namespace test_voltage_sensor {

void local_setUp(void) {
    clearMockAnalogReads();
}

void local_tearDown(void) {}

void test_simple_constructor(void)
{
    local_setUp();
    MockVoltageSensor sensor(5, "TestSensor");

    // Note: pin, r1, r2, refVoltage are private members - not testing directly
    TEST_ASSERT_EQUAL_STRING("TestSensor", sensor.getName());
    local_tearDown();
}

void test_voltage_divider_constructor(void)
{
    local_setUp();
    MockVoltageSensor sensor(7, 10000, 5000, "DividerSensor", 3.3);

    // Note: pin, r1, r2, refVoltage are private members - not testing directly
    TEST_ASSERT_EQUAL_STRING("DividerSensor", sensor.getName());
    local_tearDown();
}

void test_constructor_adds_column(void)
{
    local_setUp();
    MockVoltageSensor sensor(3);

    TEST_ASSERT_EQUAL(1, sensor.getNumColumns());

    DataPoint *dp = sensor.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("Voltage (V)", dp->label);
    local_tearDown();
}

void test_begin_calls_init(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);

    int result = sensor.begin();

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_TRUE(sensor.initCalled);
    TEST_ASSERT_TRUE(sensor.isInitialized());
    local_tearDown();
}

void test_read_basic(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    // Assuming 10-bit ADC (1024 max value) and 3.3V reference
    sensor.setMockRawValue(512); // Half of max

    int result = sensor.read();

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_TRUE(sensor.readCalled);

    // For 10-bit: 512 / 1023 * 3.3 ≈ 1.65V
    #if PLATFORM_ADC_BITS > 0
    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * PLATFORM_DEFAULT_REF_VOLTAGE;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_read_zero_value(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.setMockRawValue(0);
    sensor.read();

    TEST_ASSERT_EQUAL_FLOAT(0.0, sensor.getVoltage());
    TEST_ASSERT_EQUAL(0, sensor.getRawValue());
    local_tearDown();
}

void test_read_max_value(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    int maxValue = (1 << PLATFORM_ADC_BITS) - 1;
    sensor.setMockRawValue(maxValue);
    sensor.read();

    // Should read approximately the reference voltage
    TEST_ASSERT_FLOAT_WITHIN(0.01, PLATFORM_DEFAULT_REF_VOLTAGE, sensor.getVoltage());
    TEST_ASSERT_EQUAL(maxValue, sensor.getRawValue());
    #endif
    local_tearDown();
}

void test_read_with_voltage_divider(void)
{
    local_setUp();
    // R1 = 10k, R2 = 5k (voltage divider ratio = 3)
    MockVoltageSensor sensor(5, 10000, 5000, "Divider", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    // Set to half of ADC range
    int mockValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.setMockRawValue(mockValue);
    sensor.read();

    // ADC voltage = ~1.65V
    // With divider: 1.65 * (10000 + 5000) / 5000 = 1.65 * 3 = 4.95V
    double adcVoltage = (mockValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 3.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_read_without_voltage_divider(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.setMockRawValue(512);
    sensor.read();

    // Without voltage divider, voltage should be direct ADC reading
    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * PLATFORM_DEFAULT_REF_VOLTAGE;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_getVoltage(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.setMockRawValue(100);
    sensor.read();

    double voltage = sensor.getVoltage();
    TEST_ASSERT_TRUE(voltage >= 0.0);
    local_tearDown();
}

void test_getRawValue(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.setMockRawValue(123);
    sensor.read();

    TEST_ASSERT_EQUAL(123, sensor.getRawValue());
    local_tearDown();
}

void test_update_calls_read(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.readCalled = false;
    sensor.update();

    TEST_ASSERT_TRUE(sensor.readCalled);
    local_tearDown();
}

void test_voltage_divider_ratio_2(void)
{
    local_setUp();
    // R1 = R2 = ratio of 2
    MockVoltageSensor sensor(5, 5000, 5000, "Ratio2", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    int mockValue = (1 << (PLATFORM_ADC_BITS - 1)); // Half
    sensor.setMockRawValue(mockValue);
    sensor.read();

    double adcVoltage = (mockValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 2.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_voltage_divider_ratio_4(void)
{
    local_setUp();
    // R1 = 3*R2 = ratio of 4
    MockVoltageSensor sensor(5, 15000, 5000, "Ratio4", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    int mockValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.setMockRawValue(mockValue);
    sensor.read();

    double adcVoltage = (mockValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 4.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_voltage_divider_with_zero_resistors(void)
{
    local_setUp();
    // R1 = 0, R2 = 0 should not apply divider
    MockVoltageSensor sensor(5, 0, 0, "NoDivider", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.setMockRawValue(512);
    sensor.read();

    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_multiple_reads_update_value(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.setMockRawValue(100);
    sensor.read();
    double voltage1 = sensor.getVoltage();

    sensor.setMockRawValue(200);
    sensor.read();
    double voltage2 = sensor.getVoltage();

    // Second reading should be different (roughly double)
    TEST_ASSERT_TRUE(voltage2 > voltage1);
    #endif
    local_tearDown();
}

void test_sensor_inherits_from_sensor_base(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);

    // Should have Sensor methods
    sensor.setUpdateRate(50);
    TEST_ASSERT_TRUE(true); // If we get here, inheritance works
    local_tearDown();
}

void test_default_update_rate(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);
    sensor.begin();

    // Simple constructor sets update rate to 10 Hz
    // We can't directly test the rate, but we can verify shouldUpdate works
    bool should = sensor.shouldUpdate(0.0);
    TEST_ASSERT_TRUE(should);

    should = sensor.shouldUpdate(0.05); // 50ms later
    TEST_ASSERT_FALSE(should);

    should = sensor.shouldUpdate(0.11); // 110ms later (past 100ms threshold)
    TEST_ASSERT_TRUE(should);
    local_tearDown();
}

void test_voltage_sensor_is_healthy_after_begin(void)
{
    local_setUp();
    MockVoltageSensor sensor(5);

    TEST_ASSERT_FALSE(sensor.isHealthy());

    sensor.begin();

    TEST_ASSERT_TRUE(sensor.isHealthy());
    local_tearDown();
}

void test_custom_reference_voltage(void)
{
    local_setUp();
    MockVoltageSensor sensor(5, 10000, 5000, "Custom", 5.0);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    int mockValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.setMockRawValue(mockValue);
    sensor.read();

    // With 5.0V reference, half should give ~2.5V at ADC
    double adcVoltage = (mockValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 5.0;
    double expectedVoltage = adcVoltage * 3.0; // With divider

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
    local_tearDown();
}

void test_different_pins(void)
{
    local_setUp();
    MockVoltageSensor sensor1(1);
    MockVoltageSensor sensor2(10);
    MockVoltageSensor sensor3(20);

    // Note: pin is a private member - constructor should not throw
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void run_test_voltage_sensor_tests()
{
    RUN_TEST(test_simple_constructor);
    RUN_TEST(test_voltage_divider_constructor);
    RUN_TEST(test_constructor_adds_column);
    RUN_TEST(test_begin_calls_init);
    RUN_TEST(test_read_basic);
    RUN_TEST(test_read_zero_value);
    RUN_TEST(test_read_max_value);
    RUN_TEST(test_read_with_voltage_divider);
    RUN_TEST(test_read_without_voltage_divider);
    RUN_TEST(test_getVoltage);
    RUN_TEST(test_getRawValue);
    RUN_TEST(test_update_calls_read);
    RUN_TEST(test_voltage_divider_ratio_2);
    RUN_TEST(test_voltage_divider_ratio_4);
    RUN_TEST(test_voltage_divider_with_zero_resistors);
    RUN_TEST(test_multiple_reads_update_value);
    RUN_TEST(test_sensor_inherits_from_sensor_base);
    RUN_TEST(test_default_update_rate);
    RUN_TEST(test_voltage_sensor_is_healthy_after_begin);
    RUN_TEST(test_custom_reference_voltage);
    RUN_TEST(test_different_pins);
}

} // namespace test_voltage_sensor
