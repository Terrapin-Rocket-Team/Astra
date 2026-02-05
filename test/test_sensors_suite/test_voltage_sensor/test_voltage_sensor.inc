#include <unity.h>
#include "Sensors/VoltageSensor/VoltageSensor.h"
#include "NativeTestHelper.h"

using namespace astra;

// Mock VoltageSensor for testing (since we can't test hardware)
class MockVoltageSensor : public VoltageSensor
{
public:
    bool initCalled = false;
    bool readCalled = false;
    int mockRawValue = 0;

    MockVoltageSensor(int pin, const char *name = "MockVoltage")
        : VoltageSensor(pin, name) {}

    MockVoltageSensor(int pin, int r1, int r2, const char *name = "MockVoltage", double refVoltage = 3.3)
        : VoltageSensor(pin, r1, r2, name, refVoltage) {}

    // Override to avoid actual hardware calls
    int init() override
    {
        initCalled = true;
        initialized = true;
        healthy = true;
        return 0;
    }

    int read() override
    {
        readCalled = true;
        rawValue = mockRawValue;

        // Replicate the voltage calculation logic
        int maxAdcValue = (1 << PLATFORM_ADC_BITS) - 1;
        voltage = (rawValue / (double)maxAdcValue) * refVoltage;

        if (r1 > 0 && r2 > 0)
        {
            voltage = voltage * (r1 + r2) / (double)r2;
        }

        return 0;
    }

    // Expose protected members for testing
    using VoltageSensor::voltage;
    using VoltageSensor::rawValue;
    using VoltageSensor::refVoltage;
    using VoltageSensor::r1;
    using VoltageSensor::r2;
    using VoltageSensor::pin;
};

// setUp and tearDown
void test_voltage_sensor_setUp(void) {}
void test_voltage_sensor_tearDown(void) {}

// --- Constructor Tests ---

void test_voltage_sensor_test_simple_constructor(void)
{
    MockVoltageSensor sensor(5, "TestSensor");

    TEST_ASSERT_EQUAL(5, sensor.pin);
    TEST_ASSERT_EQUAL(0, sensor.r1);
    TEST_ASSERT_EQUAL(0, sensor.r2);
    TEST_ASSERT_EQUAL_FLOAT(PLATFORM_DEFAULT_REF_VOLTAGE, sensor.refVoltage);
    TEST_ASSERT_EQUAL_STRING("TestSensor", sensor.getName());
}

void test_voltage_sensor_test_voltage_divider_constructor(void)
{
    MockVoltageSensor sensor(7, 10000, 5000, "DividerSensor", 3.3);

    TEST_ASSERT_EQUAL(7, sensor.pin);
    TEST_ASSERT_EQUAL(10000, sensor.r1);
    TEST_ASSERT_EQUAL(5000, sensor.r2);
    TEST_ASSERT_EQUAL_FLOAT(3.3, sensor.refVoltage);
    TEST_ASSERT_EQUAL_STRING("DividerSensor", sensor.getName());
}

void test_voltage_sensor_test_constructor_adds_column(void)
{
    MockVoltageSensor sensor(3);

    TEST_ASSERT_EQUAL(1, sensor.getNumColumns());

    DataPoint *dp = sensor.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("Voltage (V)", dp->label);
}

// --- Initialization Tests ---

void test_voltage_sensor_test_begin_calls_init(void)
{
    MockVoltageSensor sensor(5);

    int result = sensor.begin();

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_TRUE(sensor.initCalled);
    TEST_ASSERT_TRUE(sensor.isInitialized());
}

// --- Read Tests ---

void test_voltage_sensor_test_read_basic(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    // Assuming 10-bit ADC (1024 max value) and 3.3V reference
    sensor.mockRawValue = 512; // Half of max

    int result = sensor.read();

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_TRUE(sensor.readCalled);

    // For 10-bit: 512 / 1023 * 3.3 ≈ 1.65V
    #if PLATFORM_ADC_BITS > 0
    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * PLATFORM_DEFAULT_REF_VOLTAGE;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
}

void test_voltage_sensor_test_read_zero_value(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.mockRawValue = 0;
    sensor.read();

    TEST_ASSERT_EQUAL_FLOAT(0.0, sensor.getVoltage());
    TEST_ASSERT_EQUAL(0, sensor.getRawValue());
}

void test_voltage_sensor_test_read_max_value(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    int maxValue = (1 << PLATFORM_ADC_BITS) - 1;
    sensor.mockRawValue = maxValue;
    sensor.read();

    // Should read approximately the reference voltage
    TEST_ASSERT_FLOAT_WITHIN(0.01, PLATFORM_DEFAULT_REF_VOLTAGE, sensor.getVoltage());
    TEST_ASSERT_EQUAL(maxValue, sensor.getRawValue());
    #endif
}

void test_voltage_sensor_test_read_with_voltage_divider(void)
{
    // R1 = 10k, R2 = 5k (voltage divider ratio = 3)
    MockVoltageSensor sensor(5, 10000, 5000, "Divider", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    // Set to half of ADC range
    sensor.mockRawValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.read();

    // ADC voltage = ~1.65V
    // With divider: 1.65 * (10000 + 5000) / 5000 = 1.65 * 3 = 4.95V
    double adcVoltage = (sensor.mockRawValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 3.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
}

void test_voltage_sensor_test_read_without_voltage_divider(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = 512;
    sensor.read();

    // Without voltage divider, voltage should be direct ADC reading
    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * PLATFORM_DEFAULT_REF_VOLTAGE;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
}

// --- Getter Tests ---

void test_voltage_sensor_test_getVoltage(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.mockRawValue = 100;
    sensor.read();

    double voltage = sensor.getVoltage();
    TEST_ASSERT_TRUE(voltage >= 0.0);
}

void test_voltage_sensor_test_getRawValue(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.mockRawValue = 123;
    sensor.read();

    TEST_ASSERT_EQUAL(123, sensor.getRawValue());
}

// --- Update Tests ---

void test_voltage_sensor_test_update_calls_read(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    sensor.readCalled = false;
    sensor.update();

    TEST_ASSERT_TRUE(sensor.readCalled);
}

// --- Voltage Divider Calculation Tests ---

void test_voltage_sensor_test_voltage_divider_ratio_2(void)
{
    // R1 = R2 = ratio of 2
    MockVoltageSensor sensor(5, 5000, 5000, "Ratio2", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = (1 << (PLATFORM_ADC_BITS - 1)); // Half
    sensor.read();

    double adcVoltage = (sensor.mockRawValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 2.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
}

void test_voltage_sensor_test_voltage_divider_ratio_4(void)
{
    // R1 = 3*R2 = ratio of 4
    MockVoltageSensor sensor(5, 15000, 5000, "Ratio4", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.read();

    double adcVoltage = (sensor.mockRawValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    double expectedVoltage = adcVoltage * 4.0;

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
}

void test_voltage_sensor_test_voltage_divider_with_zero_resistors(void)
{
    // R1 = 0, R2 = 0 should not apply divider
    MockVoltageSensor sensor(5, 0, 0, "NoDivider", 3.3);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = 512;
    sensor.read();

    double expectedVoltage = (512.0 / ((1 << PLATFORM_ADC_BITS) - 1)) * 3.3;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expectedVoltage, sensor.getVoltage());
    #endif
}

// --- Edge Cases ---

void test_voltage_sensor_test_multiple_reads_update_value(void)
{
    MockVoltageSensor sensor(5);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = 100;
    sensor.read();
    double voltage1 = sensor.getVoltage();

    sensor.mockRawValue = 200;
    sensor.read();
    double voltage2 = sensor.getVoltage();

    // Second reading should be different (roughly double)
    TEST_ASSERT_TRUE(voltage2 > voltage1);
    #endif
}

void test_voltage_sensor_test_sensor_inherits_from_sensor_base(void)
{
    MockVoltageSensor sensor(5);

    // Should have Sensor methods
    sensor.setUpdateRate(50);
    TEST_ASSERT_TRUE(true); // If we get here, inheritance works
}

void test_voltage_sensor_test_default_update_rate(void)
{
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
}

void test_voltage_sensor_test_voltage_sensor_is_healthy_after_begin(void)
{
    MockVoltageSensor sensor(5);

    TEST_ASSERT_FALSE(sensor.isHealthy());

    sensor.begin();

    TEST_ASSERT_TRUE(sensor.isHealthy());
}

// --- Different Reference Voltage Tests ---

void test_voltage_sensor_test_custom_reference_voltage(void)
{
    MockVoltageSensor sensor(5, 10000, 5000, "Custom", 5.0);
    sensor.begin();

    #if PLATFORM_ADC_BITS > 0
    sensor.mockRawValue = (1 << (PLATFORM_ADC_BITS - 1));
    sensor.read();

    // With 5.0V reference, half should give ~2.5V at ADC
    double adcVoltage = (sensor.mockRawValue / (double)((1 << PLATFORM_ADC_BITS) - 1)) * 5.0;
    double expectedVoltage = adcVoltage * 3.0; // With divider

    TEST_ASSERT_FLOAT_WITHIN(0.1, expectedVoltage, sensor.getVoltage());
    #endif
}

void test_voltage_sensor_test_different_pins(void)
{
    MockVoltageSensor sensor1(1);
    MockVoltageSensor sensor2(10);
    MockVoltageSensor sensor3(20);

    TEST_ASSERT_EQUAL(1, sensor1.pin);
    TEST_ASSERT_EQUAL(10, sensor2.pin);
    TEST_ASSERT_EQUAL(20, sensor3.pin);
}

// Main function
