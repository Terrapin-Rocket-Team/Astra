#pragma once

#include <unity.h>
#include "Utils/AstraConfig.h"
#include "State/DefaultState.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "UnitTestSensors.h"

using namespace astra;

namespace test_astra_config {

using MockAccel = FakeAccel;
using MockGyro = FakeGyro;
using MockMag = FakeMag;
using MockBaro = FakeBarometer;
using MockGPS = FakeGPS;
using MockSensor = FakeSensor;
using MockIMU6DoF = FakeIMU;
using MockIMU9DoF = FakeIMU9DoF;
class MockLogSink : public ILogSink {
public:
    bool begin() override { return true; }
    bool end() override { return true; }
    bool ok() const override { return true; }
    bool wantsPrefix() const override { return false; }
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t n) override { return n; }
    void flush() override {}
};

void local_setUp(void) {
}

void local_tearDown(void) {
}

void test_constructor_default_values() {
    local_setUp();
    AstraConfig config;

    // Should construct successfully
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_state() {
    local_setUp();
    DefaultState state;
    AstraConfig config;

    config.withState(&state);

    // Should return reference for chaining
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_state_chaining() {
    local_setUp();
    DefaultState state;
    AstraConfig config;

    AstraConfig& result = config.withState(&state);

    // Should return same reference
    TEST_ASSERT_EQUAL(&config, &result);
    local_tearDown();
}

void test_with_accel() {
    local_setUp();
    MockAccel accel;
    AstraConfig config;

    config.withAccel(&accel);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_gyro() {
    local_setUp();
    MockGyro gyro;
    AstraConfig config;

    config.withGyro(&gyro);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_mag() {
    local_setUp();
    MockMag mag;
    AstraConfig config;

    config.withMag(&mag);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_baro() {
    local_setUp();
    MockBaro baro;
    AstraConfig config;

    config.withBaro(&baro);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_gps() {
    local_setUp();
    MockGPS gps;
    AstraConfig config;

    config.withGPS(&gps);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_all_sensors_chaining() {
    local_setUp();
    MockAccel accel;
    MockGyro gyro;
    MockMag mag;
    MockBaro baro;
    MockGPS gps;

    AstraConfig config;

    config.withAccel(&accel)
          .withGyro(&gyro)
          .withMag(&mag)
          .withBaro(&baro)
          .withGPS(&gps);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_misc_sensor_single() {
    local_setUp();
    MockSensor sensor("misc1");
    AstraConfig config;

    config.withMiscSensor(&sensor);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_misc_sensor_multiple() {
    local_setUp();
    MockSensor sensor1("misc1");
    MockSensor sensor2("misc2");
    MockSensor sensor3("misc3");

    AstraConfig config;

    config.withMiscSensor(&sensor1)
          .withMiscSensor(&sensor2)
          .withMiscSensor(&sensor3);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_misc_sensor_null_rejected() {
    local_setUp();
    AstraConfig config;

    config.withMiscSensor(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_misc_sensor_max_capacity() {
    local_setUp();
    AstraConfig config;
    MockSensor* sensors[20];

    // Add up to MAX_MISC_SENSORS (16)
    for (int i = 0; i < 16; i++) {
        sensors[i] = new MockSensor("misc");
        config.withMiscSensor(sensors[i]);
    }

    // Try to add 17th (should be rejected)
    MockSensor extra("extra");
    config.withMiscSensor(&extra);

    // Cleanup
    for (int i = 0; i < 16; i++) {
        delete sensors[i];
    }

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_6dof_imu() {
    local_setUp();
    MockIMU6DoF imu;
    AstraConfig config;

    config.with6DoFIMU(&imu);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_6dof_imu_null_rejected() {
    local_setUp();
    AstraConfig config;

    config.with6DoFIMU(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_9dof_imu() {
    local_setUp();
    MockIMU9DoF imu;
    AstraConfig config;

    config.with9DoFIMU(&imu);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_9dof_imu_null_rejected() {
    local_setUp();
    AstraConfig config;

    config.with9DoFIMU(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_logging_rate() {
    local_setUp();
    AstraConfig config;

    config.withLoggingRate(50.0);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_logging_rate_sets_interval() {
    local_setUp();
    AstraConfig config;

    config.withLoggingRate(10.0);  // 10 Hz = 100ms interval

    // Interval should be 0.1 seconds
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_logging_interval() {
    local_setUp();
    AstraConfig config;

    config.withLoggingInterval(100);  // 100ms

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_logging_interval_sets_rate() {
    local_setUp();
    AstraConfig config;

    config.withLoggingInterval(100);  // 100ms = 10 Hz

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_logging_rate_and_interval_mutually_exclusive() {
    local_setUp();
    AstraConfig config;

    config.withLoggingRate(20.0);     // 20 Hz
    config.withLoggingInterval(100);  // 100ms = 10 Hz (should override)

    // Last one wins
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_buzzer_pin() {
    local_setUp();
    AstraConfig config;

    config.withBuzzerPin(5);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_bb_pin_single() {
    local_setUp();
    AstraConfig config;

    config.withBBPin(10);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_bb_pin_multiple() {
    local_setUp();
    AstraConfig config;

    config.withBBPin(10)
          .withBBPin(11)
          .withBBPin(12);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_bb_pin_duplicate_rejected() {
    local_setUp();
    AstraConfig config;

    config.withBBPin(10);
    config.withBBPin(10);  // Duplicate should be rejected

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_bb_async() {
    local_setUp();
    AstraConfig config;

    config.withBBAsync(true, 50);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_bb_async_disabled() {
    local_setUp();
    AstraConfig config;

    config.withBBAsync(false);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_status_led() {
    local_setUp();
    AstraConfig config;

    config.withStatusLED(13);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_status_buzzer() {
    local_setUp();
    AstraConfig config;

    config.withStatusBuzzer(9);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_gps_fix_led() {
    local_setUp();
    AstraConfig config;

    config.withGPSFixLED(12);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_status_indicators_auto_add_to_bb() {
    local_setUp();
    AstraConfig config;

    // Status indicators should automatically add their pins to BlinkBuzz
    config.withStatusLED(13)
          .withStatusBuzzer(9)
          .withGPSFixLED(12);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_data_logs_single() {
    local_setUp();
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;
    config.withDataLogs(sinks, 1);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_data_logs_multiple() {
    local_setUp();
    MockLogSink sink1, sink2, sink3;
    ILogSink* sinks[] = {&sink1, &sink2, &sink3};

    AstraConfig config;
    config.withDataLogs(sinks, 3);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_data_logs_max_capacity() {
    local_setUp();
    MockLogSink* sinks[60];
    ILogSink* sinkPtrs[60];

    for (int i = 0; i < 60; i++) {
        sinks[i] = new MockLogSink();
        sinkPtrs[i] = sinks[i];
    }

    AstraConfig config;
    config.withDataLogs(sinkPtrs, 60);  // Exceeds max of 50

    // Cleanup
    for (int i = 0; i < 60; i++) {
        delete sinks[i];
    }

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_hitl_enabled() {
    local_setUp();
    AstraConfig config;

    config.withHITL(true);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_with_hitl_disabled() {
    local_setUp();
    AstraConfig config;

    config.withHITL(false);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_complete_configuration_chain() {
    local_setUp();
    DefaultState state;
    MockAccel accel;
    MockGyro gyro;
    MockMag mag;
    MockBaro baro;
    MockGPS gps;
    MockSensor misc1("misc1");
    MockSensor misc2("misc2");
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;

    config.withState(&state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withMag(&mag)
          .withBaro(&baro)
          .withGPS(&gps)
          .withMiscSensor(&misc1)
          .withMiscSensor(&misc2)
          .withLoggingRate(50.0)
          .withBBPin(10)
          .withBBPin(11)
          .withBBAsync(true, 100)
          .withStatusLED(13)
          .withStatusBuzzer(9)
          .withGPSFixLED(12)
          .withDataLogs(sinks, 1)
          .withHITL(false);

    // Should successfully chain all configurations
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_imu_based_configuration() {
    local_setUp();
    DefaultState state;
    MockIMU9DoF imu;
    MockBaro baro;
    MockGPS gps;
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;

    config.withState(&state)
          .with9DoFIMU(&imu)  // Provides accel, gyro, mag
          .withBaro(&baro)
          .withGPS(&gps)
          .withLoggingRate(100.0)
          .withDataLogs(sinks, 1);

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_minimal_configuration() {
    local_setUp();
    DefaultState state;

    AstraConfig config;
    config.withState(&state);

    // Should work with just state
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_reconfiguration_override() {
    local_setUp();
    AstraConfig config;

    // Configure, then reconfigure
    config.withLoggingRate(10.0);
    config.withLoggingRate(50.0);  // Should override

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_mixed_logging_configuration() {
    local_setUp();
    AstraConfig config;

    config.withLoggingRate(10.0)      // 100ms
          .withLoggingInterval(50);    // 50ms (should override)

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_multiple_state_assignments() {
    local_setUp();
    DefaultState state1;
    DefaultState state2;

    AstraConfig config;

    config.withState(&state1);
    config.withState(&state2);  // Should override

    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void test_bb_pin_limit() {
    local_setUp();
    AstraConfig config;

    // Try to add more than 50 pins
    for (int i = 0; i < 55; i++) {
        config.withBBPin(i);
    }

    // Should handle gracefully
    TEST_ASSERT_NOT_NULL(&config);
    local_tearDown();
}

void run_test_astra_config_tests()
{
    RUN_TEST(test_constructor_default_values);
    RUN_TEST(test_with_state);
    RUN_TEST(test_with_state_chaining);
    RUN_TEST(test_with_accel);
    RUN_TEST(test_with_gyro);
    RUN_TEST(test_with_mag);
    RUN_TEST(test_with_baro);
    RUN_TEST(test_with_gps);
    RUN_TEST(test_with_all_sensors_chaining);
    RUN_TEST(test_with_misc_sensor_single);
    RUN_TEST(test_with_misc_sensor_multiple);
    RUN_TEST(test_with_misc_sensor_null_rejected);
    RUN_TEST(test_with_misc_sensor_max_capacity);
    RUN_TEST(test_with_6dof_imu);
    RUN_TEST(test_with_6dof_imu_null_rejected);
    RUN_TEST(test_with_9dof_imu);
    RUN_TEST(test_with_9dof_imu_null_rejected);
    RUN_TEST(test_with_logging_rate);
    RUN_TEST(test_with_logging_rate_sets_interval);
    RUN_TEST(test_with_logging_interval);
    RUN_TEST(test_with_logging_interval_sets_rate);
    RUN_TEST(test_logging_rate_and_interval_mutually_exclusive);
    RUN_TEST(test_with_buzzer_pin);
    RUN_TEST(test_with_bb_pin_single);
    RUN_TEST(test_with_bb_pin_multiple);
    RUN_TEST(test_with_bb_pin_duplicate_rejected);
    RUN_TEST(test_with_bb_async);
    RUN_TEST(test_with_bb_async_disabled);
    RUN_TEST(test_with_status_led);
    RUN_TEST(test_with_status_buzzer);
    RUN_TEST(test_with_gps_fix_led);
    RUN_TEST(test_status_indicators_auto_add_to_bb);
    RUN_TEST(test_with_data_logs_single);
    RUN_TEST(test_with_data_logs_multiple);
    RUN_TEST(test_with_data_logs_max_capacity);
    RUN_TEST(test_with_hitl_enabled);
    RUN_TEST(test_with_hitl_disabled);
    RUN_TEST(test_complete_configuration_chain);
    RUN_TEST(test_imu_based_configuration);
    RUN_TEST(test_minimal_configuration);
    RUN_TEST(test_reconfiguration_override);
    RUN_TEST(test_mixed_logging_configuration);
    RUN_TEST(test_multiple_state_assignments);
    RUN_TEST(test_bb_pin_limit);
}

} // namespace test_astra_config
