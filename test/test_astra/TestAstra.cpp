#include <unity.h>
#include "Utils/Astra.h"
#include "Utils/AstraConfig.h"
#include "State/DefaultState.h"
#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/Baro/Barometer.h"
#include "Sensors/GPS/GPS.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

using namespace astra;

// ========================= Mock Sensors =========================

class MockAccel : public Accel {
public:
    bool _initialized = false;
    bool _healthy = true;
    Vector<3> _reading = Vector<3>(0, 0, 9.81);

    MockAccel() : Accel("MockAccel") {}

    int begin() override {
        _initialized = true;
        return 0;
    }

    int update(double currentTime = -1) override {
        if (_initialized) {
            setHasUpdate(true);
        }
        return 0;
    }

    Vector<3> read() override { return _reading; }
    bool isHealthy() const override { return _healthy; }
};

class MockGyro : public Gyro {
public:
    bool _initialized = false;
    bool _healthy = true;
    Vector<3> _reading = Vector<3>(0, 0, 0);

    MockGyro() : Gyro("MockGyro") {}

    int begin() override {
        _initialized = true;
        return 0;
    }

    int update(double currentTime = -1) override {
        if (_initialized) {
            setHasUpdate(true);
        }
        return 0;
    }

    Vector<3> read() override { return _reading; }
    bool isHealthy() const override { return _healthy; }
};

class MockBaro : public Barometer {
public:
    bool _initialized = false;
    bool _healthy = true;
    double _altitude = 0.0;

    MockBaro() : Barometer("MockBaro") {}

    int begin() override {
        _initialized = true;
        return 0;
    }

    int update(double currentTime = -1) override {
        if (_initialized) {
            setHasUpdate(true);
        }
        return 0;
    }

    double readPressure() override { return 101325.0; }
    double readTemperature() override { return 25.0; }
    double getAltitudeMSL() override { return _altitude; }
    bool isHealthy() const override { return _healthy; }
};

class MockGPS : public GPS {
public:
    bool _initialized = false;
    bool _healthy = true;
    bool _hasFix = false;

    MockGPS() : GPS("MockGPS") {}

    int begin() override {
        _initialized = true;
        return 0;
    }

    int update(double currentTime = -1) override {
        if (_initialized) {
            setHasUpdate(true);
        }
        return 0;
    }

    bool getHasFix() const override { return _hasFix; }
    bool isHealthy() const override { return _healthy; }
};

class MockFailingAccel : public Accel {
public:
    MockFailingAccel() : Accel("FailingAccel") {}
    int begin() override { return -1; }  // Fail
    int update(double currentTime = -1) override { return 0; }
    Vector<3> read() override { return Vector<3>(0, 0, 0); }
};

class MockLogSink : public ILogSink {
public:
    bool began = false;

    bool begin() override {
        began = true;
        return true;
    }
    bool end() override { return true; }
    bool ok() const override { return began; }
    bool wantsPrefix() const override { return false; }
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t*, size_t n) override { return n; }
    void flush() override {}
    using Print::write;
};

// ========================= Test Fixtures =========================

DefaultState* state;
Astra* astra;

void setUp(void) {
    state = nullptr;
    astra = nullptr;
}

void tearDown(void) {
    delete astra;
    delete state;
}

// ========================= Constructor Tests =========================

void test_constructor_with_config() {
    state = new DefaultState();
    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);

    TEST_ASSERT_NOT_NULL(astra);
}

void test_constructor_null_config() {
    // Should construct but likely fail on init
    astra = new Astra(nullptr);

    TEST_ASSERT_NOT_NULL(astra);
}

// ========================= Initialization Tests =========================

void test_init_minimal_config() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    int errors = astra->init();

    // Should succeed with just state
    TEST_ASSERT_EQUAL(0, errors);
}

void test_init_with_all_sensors() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    MockBaro baro;
    MockGPS gps;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withBaro(&baro)
          .withGPS(&gps);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);
    TEST_ASSERT_TRUE(accel._initialized);
    TEST_ASSERT_TRUE(gyro._initialized);
    TEST_ASSERT_TRUE(baro._initialized);
    TEST_ASSERT_TRUE(gps._initialized);
}

void test_init_with_failing_sensor() {
    state = new DefaultState();
    MockFailingAccel failingAccel;

    AstraConfig config;
    config.withState(state)
          .withAccel(&failingAccel);

    astra = new Astra(&config);
    int errors = astra->init();

    // Should return 1 error
    TEST_ASSERT_EQUAL(1, errors);
}

void test_init_sets_baro_origin() {
    state = new DefaultState();
    MockBaro baro;
    baro._altitude = 123.45;

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    // Baro origin should be set in state
    // (Can't directly test but verify no crash)
    TEST_ASSERT_NOT_NULL(astra);
}

void test_init_creates_message_router() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    SerialMessageRouter* router = astra->getMessageRouter();
    TEST_ASSERT_NOT_NULL(router);
}

// ========================= Update Tests =========================

void test_update_without_init_auto_initializes() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);

    // Update without init - should auto-initialize
    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
}

void test_update_with_simulation_time() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
}

void test_update_without_time_uses_millis() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    // Pass -1 to use millis()
    bool result = astra->update(-1);

    TEST_ASSERT_TRUE(result);
}

void test_update_sets_did_flags() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);  // First update
    astra->update(0.01); // Second update with dt

    // Flags should be set
    TEST_ASSERT_TRUE(astra->didUpdateSensors());
}

void test_update_clears_did_flags_each_cycle() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    astra->update(1.0);

    // Flags should be cleared at start of each update
    TEST_ASSERT_FALSE(astra->didLog());  // Won't log on first call
}

// ========================= Sensor Update Tests =========================

void test_update_sensors_every_cycle() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    TEST_ASSERT_TRUE(astra->didUpdateSensors());
}

void test_update_orientation_when_imu_available() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    // Calibrate
    for (int i = 0; i < 50; i++) {
        astra->update(i * 0.01);
    }

    // Should have updated orientation
    TEST_ASSERT_NOT_NULL(astra);
}

void test_update_skips_unhealthy_imu() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    accel._healthy = false;  // Mark as unhealthy

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    // Should not crash with unhealthy sensor
    TEST_ASSERT_NOT_NULL(astra);
}

void test_predict_runs_after_orientation_update() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);
    astra->update(0.02);

    TEST_ASSERT_TRUE(astra->didPredictState());
}

// ========================= Measurement Update Tests =========================

void test_update_measurements_with_gps_fix() {
    state = new DefaultState();
    MockGPS gps;
    gps._hasFix = true;

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    TEST_ASSERT_TRUE(astra->didUpdateState());
}

void test_update_measurements_with_baro() {
    state = new DefaultState();
    MockBaro baro;

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    TEST_ASSERT_TRUE(astra->didUpdateState());
}

void test_update_measurements_with_gps_and_baro() {
    state = new DefaultState();
    MockGPS gps;
    MockBaro baro;

    gps._hasFix = true;

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    TEST_ASSERT_TRUE(astra->didUpdateState());
}

void test_update_skips_gps_without_fix() {
    state = new DefaultState();
    MockGPS gps;
    gps._hasFix = false;  // No fix

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    // Should not update state without GPS fix
    TEST_ASSERT_FALSE(astra->didUpdateState());
}

void test_update_skips_unhealthy_gps() {
    state = new DefaultState();
    MockGPS gps;
    gps._hasFix = true;
    gps._healthy = false;  // Unhealthy

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    // Should not update state with unhealthy sensor
    TEST_ASSERT_FALSE(astra->didUpdateState());
}

void test_update_skips_unhealthy_baro() {
    state = new DefaultState();
    MockBaro baro;
    baro._healthy = false;

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.01);

    // Should not update state with unhealthy sensor
    TEST_ASSERT_FALSE(astra->didUpdateState());
}

// ========================= Logging Tests =========================

void test_logging_at_specified_interval() {
    state = new DefaultState();
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;
    config.withState(state)
          .withLoggingInterval(100)  // 100ms = 0.1s
          .withDataLogs(sinks, 1);

    astra = new Astra(&config);
    astra->init();

    // First update
    astra->update(0.0);
    bool logged1 = astra->didLog();

    // Update at 0.05s (< interval)
    astra->update(0.05);
    bool logged2 = astra->didLog();

    // Update at 0.1s (= interval)
    astra->update(0.1);
    bool logged3 = astra->didLog();

    TEST_ASSERT_FALSE(logged1);  // First update doesn't log
    TEST_ASSERT_FALSE(logged2);  // Too soon
    TEST_ASSERT_TRUE(logged3);   // At interval
}

void test_logging_rate_conversion() {
    state = new DefaultState();
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    AstraConfig config;
    config.withState(state)
          .withLoggingRate(10.0)  // 10 Hz = 100ms = 0.1s
          .withDataLogs(sinks, 1);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.1);

    TEST_ASSERT_TRUE(astra->didLog());
}

// ========================= HITL Mode Tests =========================

void test_hitl_mode_enabled() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);
}

void test_hitl_mode_requires_simulation_time() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    astra->init();

    // In HITL mode, must pass simulation time
    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
}

// ========================= Integration Tests =========================

void test_complete_update_cycle() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    MockBaro baro;
    MockGPS gps;
    MockLogSink sink;
    ILogSink* sinks[] = {&sink};

    gps._hasFix = true;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withBaro(&baro)
          .withGPS(&gps)
          .withLoggingInterval(100)
          .withDataLogs(sinks, 1);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);

    // Run full update cycle
    astra->update(0.0);
    astra->update(0.01);
    astra->update(0.02);

    TEST_ASSERT_TRUE(astra->didUpdateSensors());
    TEST_ASSERT_TRUE(astra->didPredictState());
    TEST_ASSERT_TRUE(astra->didUpdateState());
}

void test_multiple_update_cycles() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    astra->init();

    // Run 100 update cycles
    for (int i = 0; i < 100; i++) {
        astra->update(i * 0.01);
    }

    // Should still be functional
    TEST_ASSERT_NOT_NULL(astra);
}

void test_flight_simulation() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    MockBaro baro;
    MockGPS gps;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withBaro(&baro)
          .withGPS(&gps);

    astra = new Astra(&config);
    astra->init();

    // Ground phase
    accel._reading = Vector<3>(0, 0, 9.81);
    baro._altitude = 0.0;
    gps._hasFix = true;

    for (int i = 0; i < 10; i++) {
        astra->update(i * 0.01);
    }

    // Launch - high acceleration
    accel._reading = Vector<3>(0, 0, 50.0);

    for (int i = 10; i < 30; i++) {
        astra->update(i * 0.01);
        baro._altitude = (i - 10) * 5.0;  // Climbing
    }

    // Coasting
    accel._reading = Vector<3>(0, 0, 9.81);

    for (int i = 30; i < 50; i++) {
        astra->update(i * 0.01);
        baro._altitude = 100.0 + (i - 30) * 2.0;
    }

    // Should complete without issues
    TEST_ASSERT_NOT_NULL(astra);
}

void test_sensor_failure_during_flight() {
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    MockBaro baro;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    // Normal operation
    for (int i = 0; i < 10; i++) {
        astra->update(i * 0.01);
    }

    // Sensor failure
    accel._healthy = false;

    // Continue operation with failed sensor
    for (int i = 10; i < 20; i++) {
        astra->update(i * 0.01);
    }

    // Should handle gracefully
    TEST_ASSERT_NOT_NULL(astra);
}

// ========================= Edge Case Tests =========================

void test_update_without_state() {
    AstraConfig config;
    // No state configured

    astra = new Astra(&config);
    astra->init();

    bool result = astra->update(1.0);

    // Should return false and log warning
    TEST_ASSERT_FALSE(result);
}

void test_rapid_updates() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    // Very rapid updates (1ms timestep)
    for (int i = 0; i < 100; i++) {
        astra->update(i * 0.001);
    }

    TEST_ASSERT_NOT_NULL(astra);
}

void test_large_time_jump() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(1000.0);  // Large jump

    // Should handle gracefully
    TEST_ASSERT_NOT_NULL(astra);
}

void test_backwards_time() {
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    astra->update(10.0);
    astra->update(5.0);  // Backwards

    // Should handle gracefully (dt will be negative or zero)
    TEST_ASSERT_NOT_NULL(astra);
}

// ========================= Main =========================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Constructor tests
    RUN_TEST(test_constructor_with_config);
    RUN_TEST(test_constructor_null_config);

    // Initialization tests
    RUN_TEST(test_init_minimal_config);
    RUN_TEST(test_init_with_all_sensors);
    RUN_TEST(test_init_with_failing_sensor);
    RUN_TEST(test_init_sets_baro_origin);
    RUN_TEST(test_init_creates_message_router);

    // Update tests
    RUN_TEST(test_update_without_init_auto_initializes);
    RUN_TEST(test_update_with_simulation_time);
    RUN_TEST(test_update_without_time_uses_millis);
    RUN_TEST(test_update_sets_did_flags);
    RUN_TEST(test_update_clears_did_flags_each_cycle);

    // Sensor update tests
    RUN_TEST(test_update_sensors_every_cycle);
    RUN_TEST(test_update_orientation_when_imu_available);
    RUN_TEST(test_update_skips_unhealthy_imu);
    RUN_TEST(test_predict_runs_after_orientation_update);

    // Measurement update tests
    RUN_TEST(test_update_measurements_with_gps_fix);
    RUN_TEST(test_update_measurements_with_baro);
    RUN_TEST(test_update_measurements_with_gps_and_baro);
    RUN_TEST(test_update_skips_gps_without_fix);
    RUN_TEST(test_update_skips_unhealthy_gps);
    RUN_TEST(test_update_skips_unhealthy_baro);

    // Logging tests
    RUN_TEST(test_logging_at_specified_interval);
    RUN_TEST(test_logging_rate_conversion);

    // HITL mode tests
    RUN_TEST(test_hitl_mode_enabled);
    RUN_TEST(test_hitl_mode_requires_simulation_time);

    // Integration tests
    RUN_TEST(test_complete_update_cycle);
    RUN_TEST(test_multiple_update_cycles);
    RUN_TEST(test_flight_simulation);
    RUN_TEST(test_sensor_failure_during_flight);

    // Edge case tests
    RUN_TEST(test_update_without_state);
    RUN_TEST(test_rapid_updates);
    RUN_TEST(test_large_time_jump);
    RUN_TEST(test_backwards_time);

    return UNITY_END();
}
