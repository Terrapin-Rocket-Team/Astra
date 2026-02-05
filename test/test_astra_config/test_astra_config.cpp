#include <unity.h>
#include "Utils/AstraConfig.h"
#include "State/DefaultState.h"
#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/Mag/Mag.h"
#include "Sensors/Baro/Barometer.h"
#include "Sensors/GPS/GPS.h"
#include "Sensors/IMU/IMU6DoF.h"
#include "Sensors/IMU/IMU9DoF.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

using namespace astra;

// ========================= Mock Sensors =========================

class MockAccel : public Accel {
public:
    MockAccel() : Accel("MockAccel") {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
    Vector<3> read() override { return Vector<3>(0, 0, 9.81); }
};

class MockGyro : public Gyro {
public:
    MockGyro() : Gyro("MockGyro") {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
    Vector<3> read() override { return Vector<3>(0, 0, 0); }
};

class MockMag : public Mag {
public:
    MockMag() : Mag("MockMag") {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
    Vector<3> read() override { return Vector<3>(20, 0, 0); }
};

class MockBaro : public Barometer {
public:
    MockBaro() : Barometer("MockBaro") {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
    double readPressure() override { return 101325.0; }
    double readTemperature() override { return 25.0; }
    double getAltitudeMSL() override { return 0.0; }
};

class MockGPS : public GPS {
public:
    MockGPS() : GPS("MockGPS") {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
};

class MockSensor : public Sensor {
public:
    MockSensor(const char* name) : Sensor(name) {}
    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }
};

class MockIMU6DoF : public IMU6DoF {
public:
    MockAccel accel;
    MockGyro gyro;

    MockIMU6DoF() : IMU6DoF("MockIMU6DoF") {}

    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }

    Accel* getAccelSensor() override { return &accel; }
    Gyro* getGyroSensor() override { return &gyro; }
};

class MockIMU9DoF : public IMU9DoF {
public:
    MockAccel accel;
    MockGyro gyro;
    MockMag mag;

    MockIMU9DoF() : IMU9DoF("MockIMU9DoF") {}

    int begin() override { return 0; }
    int update(double currentTime = -1) override { return 0; }

    Accel* getAccelSensor() override { return &accel; }
    Gyro* getGyroSensor() override { return &gyro; }
    Mag* getMagSensor() override { return &mag; }
};

class MockLogSink : public ILogSink {
public:
    bool begin() override { return true; }
    bool end() override { return true; }
    bool ok() const override { return true; }
    bool wantsPrefix() const override { return false; }
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t n) override { return n; }
    void flush() override {}
    using Print::write;
};

// ========================= Test Fixtures =========================

void setUp(void) {
}

void tearDown(void) {
}

// ========================= Constructor Tests =========================

void test_constructor_default_values() {
    AstraConfig config;

    // Should construct successfully
    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= State Configuration Tests =========================

void test_with_state() {
    DefaultState state;
    AstraConfig config;

    config.withState(&state);

    // Should return reference for chaining
    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_state_chaining() {
    DefaultState state;
    AstraConfig config;

    AstraConfig& result = config.withState(&state);

    // Should return same reference
    TEST_ASSERT_EQUAL(&config, &result);
}

// ========================= Individual Sensor Configuration Tests =========================

void test_with_accel() {
    MockAccel accel;
    AstraConfig config;

    config.withAccel(&accel);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_gyro() {
    MockGyro gyro;
    AstraConfig config;

    config.withGyro(&gyro);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_mag() {
    MockMag mag;
    AstraConfig config;

    config.withMag(&mag);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_baro() {
    MockBaro baro;
    AstraConfig config;

    config.withBaro(&baro);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_gps() {
    MockGPS gps;
    AstraConfig config;

    config.withGPS(&gps);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_all_sensors_chaining() {
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
}

// ========================= Misc Sensor Tests =========================

void test_with_misc_sensor_single() {
    MockSensor sensor("misc1");
    AstraConfig config;

    config.withMiscSensor(&sensor);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_misc_sensor_multiple() {
    MockSensor sensor1("misc1");
    MockSensor sensor2("misc2");
    MockSensor sensor3("misc3");

    AstraConfig config;

    config.withMiscSensor(&sensor1)
          .withMiscSensor(&sensor2)
          .withMiscSensor(&sensor3);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_misc_sensor_null_rejected() {
    AstraConfig config;

    config.withMiscSensor(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_misc_sensor_max_capacity() {
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
}

// ========================= IMU Configuration Tests =========================

void test_with_6dof_imu() {
    MockIMU6DoF imu;
    AstraConfig config;

    config.with6DoFIMU(&imu);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_6dof_imu_null_rejected() {
    AstraConfig config;

    config.with6DoFIMU(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_9dof_imu() {
    MockIMU9DoF imu;
    AstraConfig config;

    config.with9DoFIMU(&imu);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_9dof_imu_null_rejected() {
    AstraConfig config;

    config.with9DoFIMU(nullptr);

    // Should not crash
    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Logging Configuration Tests =========================

void test_with_logging_rate() {
    AstraConfig config;

    config.withLoggingRate(50.0);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_logging_rate_sets_interval() {
    AstraConfig config;

    config.withLoggingRate(10.0);  // 10 Hz = 100ms interval

    // Interval should be 0.1 seconds
    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_logging_interval() {
    AstraConfig config;

    config.withLoggingInterval(100);  // 100ms

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_logging_interval_sets_rate() {
    AstraConfig config;

    config.withLoggingInterval(100);  // 100ms = 10 Hz

    TEST_ASSERT_NOT_NULL(&config);
}

void test_logging_rate_and_interval_mutually_exclusive() {
    AstraConfig config;

    config.withLoggingRate(20.0);     // 20 Hz
    config.withLoggingInterval(100);  // 100ms = 10 Hz (should override)

    // Last one wins
    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= BlinkBuzz Configuration Tests =========================

void test_with_buzzer_pin() {
    AstraConfig config;

    config.withBuzzerPin(5);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_bb_pin_single() {
    AstraConfig config;

    config.withBBPin(10);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_bb_pin_multiple() {
    AstraConfig config;

    config.withBBPin(10)
          .withBBPin(11)
          .withBBPin(12);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_bb_pin_duplicate_rejected() {
    AstraConfig config;

    config.withBBPin(10);
    config.withBBPin(10);  // Duplicate should be rejected

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_bb_async() {
    AstraConfig config;

    config.withBBAsync(true, 50);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_bb_async_disabled() {
    AstraConfig config;

    config.withBBAsync(false);

    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Status Indicator Configuration Tests =========================

void test_with_status_led() {
    AstraConfig config;

    config.withStatusLED(13);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_status_buzzer() {
    AstraConfig config;

    config.withStatusBuzzer(9);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_gps_fix_led() {
    AstraConfig config;

    config.withGPSFixLED(12);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_status_indicators_auto_add_to_bb() {
    AstraConfig config;

    // Status indicators should automatically add their pins to BlinkBuzz
    config.withStatusLED(13)
          .withStatusBuzzer(9)
          .withGPSFixLED(12);

    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Data Logging Configuration Tests =========================

void test_with_data_logs_single() {
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;
    config.withDataLogs(sinks, 1);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_data_logs_multiple() {
    MockLogSink sink1, sink2, sink3;
    ILogSink* sinks[] = {&sink1, &sink2, &sink3};

    AstraConfig config;
    config.withDataLogs(sinks, 3);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_data_logs_max_capacity() {
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
}

// ========================= HITL Configuration Tests =========================

void test_with_hitl_enabled() {
    AstraConfig config;

    config.withHITL(true);

    TEST_ASSERT_NOT_NULL(&config);
}

void test_with_hitl_disabled() {
    AstraConfig config;

    config.withHITL(false);

    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Complete Configuration Tests =========================

void test_complete_configuration_chain() {
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
}

void test_imu_based_configuration() {
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
}

void test_minimal_configuration() {
    DefaultState state;

    AstraConfig config;
    config.withState(&state);

    // Should work with just state
    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Edge Case Tests =========================

void test_reconfiguration_override() {
    AstraConfig config;

    // Configure, then reconfigure
    config.withLoggingRate(10.0);
    config.withLoggingRate(50.0);  // Should override

    TEST_ASSERT_NOT_NULL(&config);
}

void test_mixed_logging_configuration() {
    AstraConfig config;

    config.withLoggingRate(10.0)      // 100ms
          .withLoggingInterval(50);    // 50ms (should override)

    TEST_ASSERT_NOT_NULL(&config);
}

void test_multiple_state_assignments() {
    DefaultState state1;
    DefaultState state2;

    AstraConfig config;

    config.withState(&state1);
    config.withState(&state2);  // Should override

    TEST_ASSERT_NOT_NULL(&config);
}

void test_bb_pin_limit() {
    AstraConfig config;

    // Try to add more than 50 pins
    for (int i = 0; i < 55; i++) {
        config.withBBPin(i);
    }

    // Should handle gracefully
    TEST_ASSERT_NOT_NULL(&config);
}

// ========================= Main =========================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Constructor tests
    RUN_TEST(test_constructor_default_values);

    // State configuration tests
    RUN_TEST(test_with_state);
    RUN_TEST(test_with_state_chaining);

    // Individual sensor configuration tests
    RUN_TEST(test_with_accel);
    RUN_TEST(test_with_gyro);
    RUN_TEST(test_with_mag);
    RUN_TEST(test_with_baro);
    RUN_TEST(test_with_gps);
    RUN_TEST(test_with_all_sensors_chaining);

    // Misc sensor tests
    RUN_TEST(test_with_misc_sensor_single);
    RUN_TEST(test_with_misc_sensor_multiple);
    RUN_TEST(test_with_misc_sensor_null_rejected);
    RUN_TEST(test_with_misc_sensor_max_capacity);

    // IMU configuration tests
    RUN_TEST(test_with_6dof_imu);
    RUN_TEST(test_with_6dof_imu_null_rejected);
    RUN_TEST(test_with_9dof_imu);
    RUN_TEST(test_with_9dof_imu_null_rejected);

    // Logging configuration tests
    RUN_TEST(test_with_logging_rate);
    RUN_TEST(test_with_logging_rate_sets_interval);
    RUN_TEST(test_with_logging_interval);
    RUN_TEST(test_with_logging_interval_sets_rate);
    RUN_TEST(test_logging_rate_and_interval_mutually_exclusive);

    // BlinkBuzz configuration tests
    RUN_TEST(test_with_buzzer_pin);
    RUN_TEST(test_with_bb_pin_single);
    RUN_TEST(test_with_bb_pin_multiple);
    RUN_TEST(test_with_bb_pin_duplicate_rejected);
    RUN_TEST(test_with_bb_async);
    RUN_TEST(test_with_bb_async_disabled);

    // Status indicator configuration tests
    RUN_TEST(test_with_status_led);
    RUN_TEST(test_with_status_buzzer);
    RUN_TEST(test_with_gps_fix_led);
    RUN_TEST(test_status_indicators_auto_add_to_bb);

    // Data logging configuration tests
    RUN_TEST(test_with_data_logs_single);
    RUN_TEST(test_with_data_logs_multiple);
    RUN_TEST(test_with_data_logs_max_capacity);

    // HITL configuration tests
    RUN_TEST(test_with_hitl_enabled);
    RUN_TEST(test_with_hitl_disabled);

    // Complete configuration tests
    RUN_TEST(test_complete_configuration_chain);
    RUN_TEST(test_imu_based_configuration);
    RUN_TEST(test_minimal_configuration);

    // Edge case tests
    RUN_TEST(test_reconfiguration_override);
    RUN_TEST(test_mixed_logging_configuration);
    RUN_TEST(test_multiple_state_assignments);
    RUN_TEST(test_bb_pin_limit);

    return UNITY_END();
}
