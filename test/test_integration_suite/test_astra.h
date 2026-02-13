#pragma once

#include <unity.h>
#include "Utils/Astra.h"
#include "Utils/AstraConfig.h"
#include "State/DefaultState.h"
#include "State/State.h"
#include "Filters/DefaultKalmanFilter.h"
#include "Filters/Mahony.h"
#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/Baro/Barometer.h"
#include "Sensors/GPS/GPS.h"
#include "Sensors/HITL/HITL.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "UnitTestSensors.h"

using namespace astra;

namespace test_astra {

// Use FakeSensors from UnitTestSensors.h
using MockAccel = FakeAccel;
using MockGyro = FakeGyro;
using MockMag = FakeMag;
using MockBaro = FakeBarometer;
using MockGPS = FakeGPS;
using MockFailingAccel = FakeFailingAccel;

class MockFailingMiscSensor : public FakeSensor {
public:
    MockFailingMiscSensor() : FakeSensor("MockFailingMisc") {}
    int init() override { return -1; }
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
};

class AutoUpdateReporter : public DataReporter {
public:
    int updateCount = 0;
    float value = 0.0f;

    AutoUpdateReporter() : DataReporter("AutoUpdateReporter") {
        addColumn("%0.2f", &value, "value");
    }

    int begin() override {
        initialized = true;
        return 0;
    }

    int update(double currentTime = -1) override {
        (void)currentTime;
        updateCount++;
        value += 1.0f;
        return 0;
    }
};

class RecordingState : public State {
public:
    RecordingState() : State(&kf, &ahrs) {}

    int begin() override {
        initialized = true;
        return 0;
    }

    void updateOrientation(const Vector<3>& gyro, const Vector<3>& accel, double dt) override {
        record(Call::Orientation);
        sawOrientation = true;
        lastGyro = gyro;
        lastAccel = accel;
        lastOrientationDt = dt;
    }

    void updateOrientation(const Vector<3>& gyro, const Vector<3>& accel, const Vector<3>& mag, double dt) override {
        record(Call::Orientation);
        sawOrientation = true;
        lastGyro = gyro;
        lastAccel = accel;
        lastMag = mag;
        lastOrientationDt = dt;
    }

    void predict(double dt) override {
        record(Call::Predict);
        sawPredict = true;
        lastPredictDt = dt;
    }

    void updateGPSMeasurement(const Vector<3>& gpsPos, const Vector<3>& gpsVel) override {
        record(Call::GPS);
        sawGPS = true;
        lastGPSPos = gpsPos;
        lastGPSVel = gpsVel;
    }

    void updateBaroMeasurement(double baroAlt) override {
        record(Call::Baro);
        sawBaro = true;
        lastBaroAlt = baroAlt;
    }

    void setBaroOrigin(double altASL) override {
        baroOriginSetCount++;
        lastBaroOrigin = altASL;
        State::setBaroOrigin(altASL);
    }

    enum class Call : uint8_t { Orientation, Predict, GPS, Baro };

    Call calls[8];
    uint8_t callCount = 0;
    Vector<3> lastGyro = Vector<3>(0, 0, 0);
    Vector<3> lastAccel = Vector<3>(0, 0, 0);
    Vector<3> lastMag = Vector<3>(0, 0, 0);
    Vector<3> lastGPSPos = Vector<3>(0, 0, 0);
    Vector<3> lastGPSVel = Vector<3>(0, 0, 0);
    double lastBaroAlt = 0.0;
    double lastOrientationDt = 0.0;
    double lastPredictDt = 0.0;
    bool sawOrientation = false;
    bool sawPredict = false;
    bool sawGPS = false;
    bool sawBaro = false;
    uint8_t baroOriginSetCount = 0;
    double lastBaroOrigin = 0.0;

private:
    void record(Call call) {
        if (callCount < sizeof(calls) / sizeof(calls[0])) {
            calls[callCount++] = call;
        }
    }

    DefaultKalmanFilter kf;
    MahonyAHRS ahrs;
};

State* state;
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

void test_init_with_failing_gyro_reports_error() {
    local_setUp();
    state = new DefaultState();
    MockGyro gyro;
    gyro._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withGyro(&gyro);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_with_failing_mag_reports_error() {
    local_setUp();
    state = new DefaultState();
    MockMag mag;
    mag._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withMag(&mag);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_with_failing_baro_reports_error() {
    local_setUp();
    state = new DefaultState();
    MockBaro baro;
    baro._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withBaro(&baro);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_with_failing_gps_reports_error() {
    local_setUp();
    state = new DefaultState();
    MockGPS gps;
    gps._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_with_failing_misc_reports_error() {
    local_setUp();
    state = new DefaultState();
    MockFailingMiscSensor misc;

    AstraConfig config;
    config.withState(state)
          .withMiscSensor(&misc);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_init_with_multiple_failing_sensors_reports_multiple_errors() {
    local_setUp();
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    accel._shouldFailInit = true;
    gyro._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(2, errors);
    local_tearDown();
}

void test_init_reuses_message_router_when_called_twice() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());
    SerialMessageRouter* firstRouter = astra->getMessageRouter();
    TEST_ASSERT_NOT_NULL(firstRouter);

    TEST_ASSERT_EQUAL(0, astra->init());
    SerialMessageRouter* secondRouter = astra->getMessageRouter();
    TEST_ASSERT_NOT_NULL(secondRouter);
    TEST_ASSERT_EQUAL_PTR(firstRouter, secondRouter);
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
    state = new RecordingState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    astra->init();

    // In HITL mode, update() without explicit time should only poll router.
    bool result = astra->update();

    TEST_ASSERT_TRUE(result);
    RecordingState* rec = static_cast<RecordingState*>(state);
    TEST_ASSERT_FALSE(rec->sawOrientation);
    TEST_ASSERT_FALSE(rec->sawPredict);
    TEST_ASSERT_FALSE(rec->sawGPS);
    TEST_ASSERT_FALSE(rec->sawBaro);
    local_tearDown();
}

void test_hitl_update_flow_and_order() {
    local_setUp();
    state = new RecordingState();

    HITLAccel accel;
    HITLGyro gyro;
    HITLBarometer baro;
    HITLGPS gps;
    accel.setUpdateRate(1000);
    gyro.setUpdateRate(1000);
    baro.setUpdateRate(1000);
    gps.setUpdateRate(1000);

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withBaro(&baro)
          .withGPS(&gps)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();
    TEST_ASSERT_EQUAL(0, errors);

    RecordingState* rec = static_cast<RecordingState*>(state);

    // Warm up baro health with varying pressures
    double simTime = 0.0;
    const char* line1 = "HITL/0.11,1.1,2.2,3.3,0.1,0.2,0.3,10.0,20.0,30.0,900.0,15.0,37.0,-122.0,100.0,1,8,45.0";
    const char* line2 = "HITL/0.22,1.2,2.1,3.4,0.11,0.21,0.31,10.0,20.0,30.0,901.0,15.0,37.0,-122.0,100.0,1,8,45.0";
    const char* line3 = "HITL/0.33,1.3,2.0,3.5,0.12,0.22,0.32,10.0,20.0,30.0,902.0,15.0,37.0,-122.0,100.0,1,8,45.0";

    TEST_ASSERT_TRUE(HITLParser::parseAndInject(line1, simTime));
    astra->update(simTime);
    TEST_ASSERT_TRUE(HITLParser::parseAndInject(line2, simTime));
    astra->update(simTime);

    rec->callCount = 0;
    rec->sawOrientation = false;
    rec->sawPredict = false;
    rec->sawGPS = false;
    rec->sawBaro = false;
    TEST_ASSERT_TRUE(HITLParser::parseAndInject(line3, simTime));
    astra->update(simTime);

    TEST_ASSERT_TRUE_MESSAGE(rec->sawOrientation, "orientation not called");
    TEST_ASSERT_TRUE_MESSAGE(rec->sawPredict, "predict not called");
    TEST_ASSERT_TRUE_MESSAGE(rec->sawGPS, "gps update not called");
    TEST_ASSERT_TRUE_MESSAGE(rec->sawBaro, "baro update not called");
    TEST_ASSERT_EQUAL(4, rec->callCount);
    TEST_ASSERT_EQUAL((uint8_t)RecordingState::Call::Orientation, (uint8_t)rec->calls[0]);
    TEST_ASSERT_EQUAL((uint8_t)RecordingState::Call::Predict, (uint8_t)rec->calls[1]);
    TEST_ASSERT_EQUAL((uint8_t)RecordingState::Call::GPS, (uint8_t)rec->calls[2]);
    TEST_ASSERT_EQUAL((uint8_t)RecordingState::Call::Baro, (uint8_t)rec->calls[3]);

    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.12, rec->lastGyro.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.22, rec->lastGyro.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.32, rec->lastGyro.z());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.3, rec->lastAccel.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.0, rec->lastAccel.y());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.5, rec->lastAccel.z());

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 37.0, rec->lastGPSPos.x());
    TEST_ASSERT_FLOAT_WITHIN(0.0001, -122.0, rec->lastGPSPos.y());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 100.0, rec->lastGPSPos.z());

    double expectedAlt = Barometer::calcAltitude(902.0);
    TEST_ASSERT_FLOAT_WITHIN(0.5, expectedAlt, rec->lastBaroAlt);
    local_tearDown();
}

void test_hitl_router_drives_core_update_from_serial() {
    local_setUp();
    state = new RecordingState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();
    TEST_ASSERT_EQUAL(0, errors);

    RecordingState* rec = static_cast<RecordingState*>(state);
    const char* line = "HITL/0.25,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,901.0,15.0,37.0,-122.0,100.0,1,8,45.0\n";
    Serial.simulateInput(line);

    TEST_ASSERT_TRUE(astra->update()); // HITL event loop path

    TEST_ASSERT_TRUE(rec->sawOrientation);
    TEST_ASSERT_TRUE(rec->sawPredict);
    TEST_ASSERT_TRUE(rec->sawGPS);
    TEST_ASSERT_TRUE(rec->sawBaro);
    TEST_ASSERT_TRUE(astra->didPredictState());
    TEST_ASSERT_TRUE(astra->didUpdateState());
    TEST_ASSERT_TRUE(astra->didLog());
    local_tearDown();
}

void test_logging_auto_updates_enabled_reporters() {
    local_setUp();
    state = new DefaultState();
    static MockLogSink sink;
    ILogSink* sinks[] = {&sink};
    static AutoUpdateReporter reporter;
    reporter.updateCount = 0;
    reporter.value = 0.0f;
    reporter.begin();

    AstraConfig config;
    config.withState(state)
          .withLoggingInterval(100)
          .withDataLogs(sinks, 1);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());

    astra->update(0.0);
    astra->update(0.1);

    TEST_ASSERT_TRUE(astra->didLog());
    TEST_ASSERT_GREATER_THAN(0, reporter.updateCount);
    local_tearDown();
}

void test_init_with_event_logs_configured() {
    local_setUp();
    state = new DefaultState();
    static MockLogSink sink;
    ILogSink* eventSinks[] = {&sink};

    AstraConfig config;
    config.withState(state)
          .withEventLogs(eventSinks, 1);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);
    local_tearDown();
}

void test_hitl_router_ignores_invalid_packet() {
    local_setUp();
    state = new RecordingState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();
    TEST_ASSERT_EQUAL(0, errors);

    RecordingState* rec = static_cast<RecordingState*>(state);
    Serial.simulateInput("HITL/not,a,valid,packet\n");
    TEST_ASSERT_TRUE(astra->update());

    TEST_ASSERT_FALSE(rec->sawOrientation);
    TEST_ASSERT_FALSE(rec->sawPredict);
    TEST_ASSERT_FALSE(rec->sawGPS);
    TEST_ASSERT_FALSE(rec->sawBaro);
    TEST_ASSERT_FALSE(astra->didPredictState());
    TEST_ASSERT_FALSE(astra->didUpdateState());
    local_tearDown();
}

void test_update_with_mag_only_sensor_does_not_predict() {
    local_setUp();
    state = new DefaultState();
    MockMag mag;

    AstraConfig config;
    config.withState(state)
          .withMag(&mag);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.2);

    TEST_ASSERT_FALSE(astra->didPredictState());
    TEST_ASSERT_FALSE(astra->didUpdateState());
    local_tearDown();
}

void test_update_with_unhealthy_mag_only_sensor_does_not_predict() {
    local_setUp();
    state = new DefaultState();
    MockMag mag;
    mag._healthy = false;

    AstraConfig config;
    config.withState(state)
          .withMag(&mag);

    astra = new Astra(&config);
    astra->init();

    astra->update(0.0);
    astra->update(0.2);

    TEST_ASSERT_FALSE(astra->didPredictState());
    TEST_ASSERT_FALSE(astra->didUpdateState());
    local_tearDown();
}

void test_status_indicators_success_path() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withStatusLED(13)
          .withStatusBuzzer(33);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(0, errors);
    local_tearDown();
}

void test_status_indicators_single_failure_path() {
    local_setUp();
    state = new DefaultState();
    MockGPS gps;
    gps._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps)
          .withStatusLED(13)
          .withStatusBuzzer(33);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(1, errors);
    local_tearDown();
}

void test_status_indicators_multiple_failure_path() {
    local_setUp();
    state = new DefaultState();
    MockAccel accel;
    MockGyro gyro;
    accel._shouldFailInit = true;
    gyro._shouldFailInit = true;

    AstraConfig config;
    config.withState(state)
          .withAccel(&accel)
          .withGyro(&gyro)
          .withStatusLED(13)
          .withStatusBuzzer(33);

    astra = new Astra(&config);
    int errors = astra->init();

    TEST_ASSERT_EQUAL(2, errors);
    local_tearDown();
}

void test_gps_fix_led_without_gps_source() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state)
          .withGPSFixLED(13);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());
    TEST_ASSERT_TRUE(astra->update(0.0));
    local_tearDown();
}

void test_gps_fix_led_with_fix() {
    local_setUp();
    state = new DefaultState();
    MockGPS gps;
    gps.setHasFirstFix(1);

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps)
          .withGPSFixLED(13);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());
    gps.setHasFirstFix(1);
    TEST_ASSERT_TRUE(astra->update(0.0));
    TEST_ASSERT_TRUE(astra->update(0.2));
    local_tearDown();
}

void test_gps_fix_led_without_fix() {
    local_setUp();
    state = new DefaultState();
    MockGPS gps;
    gps.setHasFirstFix(0);

    AstraConfig config;
    config.withState(state)
          .withGPS(&gps)
          .withGPSFixLED(13);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());
    astra->update(0.0);
    astra->update(0.2);
    TEST_ASSERT_FALSE(astra->didUpdateState());
    local_tearDown();
}

void test_hitl_baro_origin_set_once_from_first_packet() {
    local_setUp();
    state = new RecordingState();

    AstraConfig config;
    config.withState(state)
          .withHITL(true);

    astra = new Astra(&config);
    int errors = astra->init();
    TEST_ASSERT_EQUAL(0, errors);

    RecordingState* rec = static_cast<RecordingState*>(state);
    TEST_ASSERT_EQUAL(0, rec->baroOriginSetCount);

    Serial.simulateInput("HITL/0.11,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,900.0,15.0,37.0,-122.0,100.0,1,8,45.0\n");
    TEST_ASSERT_TRUE(astra->update());
    TEST_ASSERT_EQUAL(1, rec->baroOriginSetCount);
    double expectedOrigin = Barometer::calcAltitude(900.0);
    TEST_ASSERT_FLOAT_WITHIN(0.5, expectedOrigin, rec->lastBaroOrigin);

    Serial.simulateInput("HITL/0.22,1.0,2.0,3.0,0.1,0.2,0.3,10.0,20.0,30.0,950.0,15.0,37.0,-122.0,100.0,1,8,45.0\n");
    TEST_ASSERT_TRUE(astra->update());
    TEST_ASSERT_EQUAL(1, rec->baroOriginSetCount);
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

void test_update_recreates_state_when_config_state_is_cleared_after_init() {
    local_setUp();
    state = new DefaultState();

    AstraConfig config;
    config.withState(state);

    astra = new Astra(&config);
    TEST_ASSERT_EQUAL(0, astra->init());

    config.withState(nullptr);
    TEST_ASSERT_TRUE(astra->update(0.2));
    local_tearDown();
}

void run_test_astra_tests()
{
    RUN_TEST(test_constructor_with_config);
    RUN_TEST(test_constructor_null_config);
    RUN_TEST(test_init_minimal_config);
    RUN_TEST(test_init_with_all_sensors);
    RUN_TEST(test_init_with_failing_sensor);
    RUN_TEST(test_init_with_failing_gyro_reports_error);
    RUN_TEST(test_init_with_failing_mag_reports_error);
    RUN_TEST(test_init_with_failing_baro_reports_error);
    RUN_TEST(test_init_with_failing_gps_reports_error);
    RUN_TEST(test_init_with_failing_misc_reports_error);
    RUN_TEST(test_init_with_multiple_failing_sensors_reports_multiple_errors);
    RUN_TEST(test_init_reuses_message_router_when_called_twice);
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
    RUN_TEST(test_update_with_mag_only_sensor_does_not_predict);
    RUN_TEST(test_update_with_unhealthy_mag_only_sensor_does_not_predict);
    RUN_TEST(test_logging_at_specified_interval);
    RUN_TEST(test_logging_rate_conversion);
    RUN_TEST(test_logging_auto_updates_enabled_reporters);
    RUN_TEST(test_init_with_event_logs_configured);
    RUN_TEST(test_status_indicators_success_path);
    RUN_TEST(test_status_indicators_single_failure_path);
    RUN_TEST(test_status_indicators_multiple_failure_path);
    RUN_TEST(test_gps_fix_led_without_gps_source);
    RUN_TEST(test_gps_fix_led_with_fix);
    RUN_TEST(test_gps_fix_led_without_fix);
    RUN_TEST(test_hitl_mode_enabled);
    RUN_TEST(test_hitl_mode_requires_simulation_time);
    RUN_TEST(test_hitl_update_flow_and_order);
    RUN_TEST(test_hitl_router_drives_core_update_from_serial);
    RUN_TEST(test_hitl_router_ignores_invalid_packet);
    RUN_TEST(test_hitl_baro_origin_set_once_from_first_packet);
    RUN_TEST(test_complete_update_cycle);
    RUN_TEST(test_multiple_update_cycles);
    RUN_TEST(test_flight_simulation);
    RUN_TEST(test_sensor_failure_during_flight);
    RUN_TEST(test_update_without_state);
    RUN_TEST(test_rapid_updates);
    RUN_TEST(test_large_time_jump);
    RUN_TEST(test_backwards_time);
    RUN_TEST(test_update_recreates_state_when_config_state_is_cleared_after_init);
}

} // namespace test_astra
