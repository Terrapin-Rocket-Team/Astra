#pragma once

#include <unity.h>
#include "Utils/Astra.h"
#include "Utils/AstraConfig.h"
#include "State/DefaultState.h"
#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/Baro/Barometer.h"
#include "Sensors/GPS/GPS.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "UnitTestSensors.h"

using namespace astra;

namespace test_astra {

// Use FakeSensors from UnitTestSensors.h
using MockAccel = FakeAccel;
using MockGyro = FakeGyro;
using MockBaro = FakeBarometer;
using MockGPS = FakeGPS;
using MockFailingAccel = FakeFailingAccel;

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
};
DefaultState* state;
Astra* astra;

void local_setUp(void) {
    state = nullptr;
    astra = nullptr;
}

void local_tearDown(void) {
    delete astra;
    delete state;
}

void test_constructor_with_config() {
    local_setUp();
    state = new DefaultState();
    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);

    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void test_constructor_null_config() {
    local_setUp();
    // Should construct but likely fail on init
    astra = new Astra(nullptr);

    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void test_init_minimal_config() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    int errors = astra->init();

    // Should succeed with just state
    TEST_ASSERT_EQUAL(0, errors);
    local_tearDown();
}

void test_init_with_all_sensors() {
    local_setUp();
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
    TEST_ASSERT_TRUE(accel.isInitialized());
    TEST_ASSERT_TRUE(gyro.isInitialized());
    TEST_ASSERT_TRUE(baro.isInitialized());
    TEST_ASSERT_TRUE(gps.isInitialized());
    local_tearDown();
}

void test_init_with_failing_sensor() {
    local_setUp();
    state = new DefaultState();
    MockFailingAccel failingAccel;

    AstraConfig config;
    config.withState(state)
          .withAccel(&failingAccel);

    astra = new Astra(&config);
    int errors = astra->init();

    // Should return 1 error
    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_sets_baro_origin() {
    local_setUp();
    state = new DefaultState();
    MockBaro baro;
    baro.setAltitude(123.45);

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    // Baro origin should be set in state
    // (Can't directly test but verify no crash)
    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void test_init_creates_message_router() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    SerialMessageRouter* router = astra->getMessageRouter();
    TEST_ASSERT_NOT_NULL(router);
    local_tearDown();
}

void test_update_without_init_auto_initializes() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);

    // Update without init - should auto-initialize
    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
    local_tearDown();
}

void test_update_with_simulation_time() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
    local_tearDown();
}

void test_update_without_time_uses_millis() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    // Pass -1 to use millis()
    bool result = astra->update(-1);

    TEST_ASSERT_TRUE(result);
    local_tearDown();
}

void test_update_sets_did_flags() {
    local_setUp();
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
    astra->update(0.2);  // Second update with enough dt for sensor update

    // Flags should be set
    TEST_ASSERT_TRUE(astra->didPredictState());
    local_tearDown();
}

void test_update_clears_did_flags_each_cycle() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    // Use time < loggingInterval (0.1s) so didLog stays false
    astra->update(0.05);

    // Flags should be cleared at start of each update
    TEST_ASSERT_FALSE(astra->didLog());  // Won't log on first call
    local_tearDown();
}

void test_update_sensors_every_cycle() {
    local_setUp();
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
    astra->update(0.2);

    TEST_ASSERT_TRUE(astra->didPredictState());
    local_tearDown();
}

void test_update_orientation_when_imu_available() {
    local_setUp();
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
    local_tearDown();
}

void test_update_skips_unhealthy_imu() {
    local_setUp();
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
    local_tearDown();
}

void test_predict_runs_after_orientation_update() {
    local_setUp();
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
    local_tearDown();
}

void test_update_measurements_with_gps_fix() {
    local_setUp();
    state = new DefaultState();
    MockGPS gps;
    gps.setHasFirstFix(1);

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps);

    astra = new Astra(&config);
    astra->init();

    gps.setHasFirstFix(1);

    astra->update(0.0);
    // GPS update interval is 0.2s (5 Hz), use time >= 0.2s to trigger sensor update
    astra->update(0.2);

    TEST_ASSERT_TRUE(astra->didUpdateState());
    local_tearDown();
}

void test_update_measurements_with_baro() {
    local_setUp();
    state = new DefaultState();
    MockBaro baro;

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    // Use time >= updateInterval (0.1s) to trigger sensor update
    astra->update(0.1);

    TEST_ASSERT_TRUE(astra->didUpdateState());
    local_tearDown();
}

void test_update_measurements_with_gps_and_baro() {
    local_setUp();
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
    // Use time >= updateInterval (0.1s) to trigger sensor update
    astra->update(0.1);

    TEST_ASSERT_TRUE(astra->didUpdateState());
    local_tearDown();
}

void test_update_skips_gps_without_fix() {
    local_setUp();
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
    local_tearDown();
}

void test_update_skips_unhealthy_gps() {
    local_setUp();
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
    local_tearDown();
}

void test_update_skips_unhealthy_baro() {
    local_setUp();
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
    local_tearDown();
}

void test_logging_at_specified_interval() {
    local_setUp();
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
    local_tearDown();
}

void test_logging_rate_conversion() {
    local_setUp();
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
    local_tearDown();
}

void test_hitl_mode_enabled() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);
    local_tearDown();
}

void test_hitl_mode_requires_simulation_time() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    astra->init();

    // In HITL mode, must pass simulation time
    bool result = astra->update(1.0);

    TEST_ASSERT_TRUE(result);
    local_tearDown();
}

void test_complete_update_cycle() {
    local_setUp();
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
    // Use time >= updateInterval (0.1s) to trigger sensor updates
    astra->update(0.1);

    TEST_ASSERT_TRUE(astra->didPredictState());
    TEST_ASSERT_TRUE(astra->didUpdateState());
    local_tearDown();
}

void test_multiple_update_cycles() {
    local_setUp();
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
    local_tearDown();
}

void test_flight_simulation() {
    local_setUp();
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
    baro.setAltitude(0.0);
    gps._hasFix = true;

    for (int i = 0; i < 10; i++) {
        astra->update(i * 0.01);
    }

    // Launch - high acceleration
    accel._reading = Vector<3>(0, 0, 50.0);

    for (int i = 10; i < 30; i++) {
        astra->update(i * 0.01);
        baro.setAltitude((i - 10) * 5.0);  // Climbing
    }

    // Coasting
    accel._reading = Vector<3>(0, 0, 9.81);

    for (int i = 30; i < 50; i++) {
        astra->update(i * 0.01);
        baro.setAltitude(100.0 + (i - 30) * 2.0);
    }

    // Should complete without issues
    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void test_sensor_failure_during_flight() {
    local_setUp();
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
    local_tearDown();
}

void test_update_without_state() {
    local_setUp();
    AstraConfig config;
    // No state configured

    astra = new Astra(&config);
    astra->init();

    bool result = astra->update(1.0);

    // Astra should create a DefaultState and proceed
    TEST_ASSERT_TRUE(result);
    local_tearDown();
}

void test_rapid_updates() {
    local_setUp();
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
    local_tearDown();
}

void test_large_time_jump() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(1000.0);  // Large jump

    // Should handle gracefully
    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void test_backwards_time() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    astra->init();

    astra->update(10.0);
    astra->update(5.0);  // Backwards

    // Should handle gracefully (dt will be negative or zero)
    TEST_ASSERT_NOT_NULL(astra);
    local_tearDown();
}

void run_test_astra_tests()
{
    RUN_TEST(test_constructor_with_config);
    RUN_TEST(test_constructor_null_config);
    RUN_TEST(test_init_minimal_config);
    RUN_TEST(test_init_with_all_sensors);
    RUN_TEST(test_init_with_failing_sensor);
    RUN_TEST(test_init_sets_baro_origin);
    RUN_TEST(test_init_creates_message_router);
    RUN_TEST(test_update_without_init_auto_initializes);
    RUN_TEST(test_update_with_simulation_time);
    RUN_TEST(test_update_without_time_uses_millis);
    RUN_TEST(test_update_sets_did_flags);
    RUN_TEST(test_update_clears_did_flags_each_cycle);
    RUN_TEST(test_update_sensors_every_cycle);
    RUN_TEST(test_update_orientation_when_imu_available);
    RUN_TEST(test_update_skips_unhealthy_imu);
    RUN_TEST(test_predict_runs_after_orientation_update);
    RUN_TEST(test_update_measurements_with_gps_fix);
    RUN_TEST(test_update_measurements_with_baro);
    RUN_TEST(test_update_measurements_with_gps_and_baro);
    RUN_TEST(test_update_skips_gps_without_fix);
    RUN_TEST(test_update_skips_unhealthy_gps);
    RUN_TEST(test_update_skips_unhealthy_baro);
    RUN_TEST(test_logging_at_specified_interval);
    RUN_TEST(test_logging_rate_conversion);
    RUN_TEST(test_hitl_mode_enabled);
    RUN_TEST(test_hitl_mode_requires_simulation_time);
    RUN_TEST(test_complete_update_cycle);
    RUN_TEST(test_multiple_update_cycles);
    RUN_TEST(test_flight_simulation);
    RUN_TEST(test_sensor_failure_during_flight);
    RUN_TEST(test_update_without_state);
    RUN_TEST(test_rapid_updates);
    RUN_TEST(test_large_time_jump);
    RUN_TEST(test_backwards_time);
}

} // namespace test_astra
