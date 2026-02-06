#ifndef Astra_CONFIG_H
#define Astra_CONFIG_H

#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "Sensors/SensorManager/SensorManager.h"
#include <stdint.h>

namespace astra
{
    class State;
    class DataReporter;
    class Sensor;
    class Accel;
    class Gyro;
    class Mag;
    class Barometer;
    class GPS;
    class IMU6DoF;
    class IMU9DoF;

    class AstraConfig
    {
        friend class Astra;

    public:
        // Add state to Astra's knowledge.
        // If omitted, Astra will create a DefaultState.
        AstraConfig &withState(State *state);

        // ===== Individual Sensor Configuration =====
        // These methods configure sensors without requiring manual SensorManager creation
        // Astra will create and manage the SensorManager internally

        // Set accelerometer sensor
        // Astra will manage this sensor internally
        AstraConfig &withAccel(Accel *accel);

        // Set gyroscope sensor
        // Astra will manage this sensor internally
        AstraConfig &withGyro(Gyro *gyro);

        // Set magnetometer sensor (optional for most applications)
        // Astra will manage this sensor internally
        AstraConfig &withMag(Mag *mag);

        // Set barometer sensor
        // Astra will manage this sensor internally
        AstraConfig &withBaro(Barometer *baro);

        // Set GPS sensor
        // Astra will manage this sensor internally
        AstraConfig &withGPS(GPS *gps);

        // Add miscellaneous sensor for logging only (not used in state estimation)
        // Can be called multiple times to add multiple misc sensors
        // Astra will manage these sensors internally
        AstraConfig &withMiscSensor(Sensor *sensor);

        // ===== IMU Convenience Methods =====
        // These methods automatically extract accelerometer and gyroscope from IMU

        // Set 6-DOF IMU (accelerometer + gyroscope)
        // Automatically extracts and configures accel and gyro sensors
        // The IMU itself is added as a misc sensor for data logging
        AstraConfig &with6DoFIMU(IMU6DoF *imu);

        // Set 9-DOF IMU (accelerometer + gyroscope + magnetometer)
        // Automatically extracts and configures accel, gyro, and mag sensors
        // The IMU itself is added as a misc sensor for data logging
        AstraConfig &with9DoFIMU(IMU9DoF *imu);

        // Set the rate at which logs will be written to the SD card (in hz).
        // Mutually exclusive with `withLoggingInterval()`. Last one called will take effect.
        // Default `10`.
        AstraConfig &withLoggingRate(double loggingRate);

        // Set the interval at which logs will be written to the SD card (in ms).
        // Mutually exclusive with `withLoggingRate()`. Last one called will take effect.
        // Default `100`.
        AstraConfig &withLoggingInterval(unsigned int loggingInterval);

        // Set the duration that sensors will average over to correct for drift (in s)
        // Affected by update rate/update interval.
        // Default `2`.
        AstraConfig &withSensorBiasCorrectionDataLength(unsigned int sensorBiasCorrectionDataLength);

        // Set the duration of the most recent data to ignore for drift correction averaging.(in s)
        // Affected by update rate/update interval.
        // Default `1`.
        AstraConfig &withSensorBiasCorrectionDataIgnore(unsigned int sensorBiasCorrectionDataIgnore);

        // Determines if the sensors will continuously re-zero themselves while on the ground.
        // Default `false`.
        // ***DANGER** Must have working launch detection or data will not be accurate.
        AstraConfig &withUsingSensorBiasCorrection(bool useBiasCorrection);

        // Set the named `BUZZER_PIN` for use with `BlinkBuzz`.
        // Default `-3`.
        AstraConfig &withBuzzerPin(unsigned int buzzerPin);

        // Add a pin to `BlinkBuzz`. Without this, BB will refuse to toggle the pin.
        // No pins added by default.
        AstraConfig &withBBPin(unsigned int bbPin);

        // Allow `BlinkBuzz` to use Async features.
        // Incurs moderate memory overhead based on `queueSize`, which is the number of state changes a pin can queue up at once.
        // Default `true`, `50`.
        AstraConfig &withBBAsync(bool bbAsync, unsigned int queueSize = 50);

        // ===== Status Indicator Configuration =====

        // Set the status LED pin for init diagnostics
        // LED will show init status via blink patterns:
        //   Solid ON = All sensors initialized successfully
        //   N blinks = Specific sensor failed (1=Accel, 2=Gyro, 3=Mag, 4=Baro, 5=GPS, 6=Misc)
        //   Fast continuous = Multiple critical failures
        // Default: not configured
        AstraConfig &withStatusLED(int pin);

        // Set the status buzzer pin for init diagnostics
        // Buzzer will play beep codes at init matching the LED pattern
        // Default: not configured
        AstraConfig &withStatusBuzzer(int pin);

        // Set the GPS fix indicator LED
        // LED states:
        //   OFF = No GPS configured
        //   Slow blink = GPS configured but no fix
        //   Solid ON = GPS has fix
        // Default: not configured
        AstraConfig &withGPSFixLED(int pin);

        // Setup which telemetry logs will be written to on update
        // No default
        AstraConfig &withDataLogs(ILogSink **logs, uint8_t numLogs);

        // Setup which event logs will be written to on update
        // No default. If not set, EventLogger must be configured manually.
        AstraConfig &withEventLogs(ILogSink **logs, uint8_t numLogs);

        // Enable HITL (Hardware-In-The-Loop) mode
        // When enabled, Astra configures all update intervals to 0 for maximum simulation speed
        // The user must pass simulation time to update(simTimeMs)
        // Default `false`
        AstraConfig &withHITL(bool hitlEnabled);

        AstraConfig();

    private:
        // Internal method called by Astra to populate SensorManager from individual sensor pointers
        void populateSensorManager();

    protected:
        State *state = nullptr;

        // SensorManager owned by config, populated from individual sensor pointers
        SensorManager sensorManager;  // OWNED by config

        // Individual sensor pointers for configuration
        Accel *accel = nullptr;
        Gyro *gyro = nullptr;
        Mag *mag = nullptr;
        Barometer *baro = nullptr;
        GPS *gps = nullptr;

        static constexpr uint8_t MAX_MISC_SENSORS = 16;
        Sensor *miscSensors[MAX_MISC_SENSORS] = {nullptr};
        uint8_t numMiscSensors = 0;

        int pins[50];
        ILogSink *logs[50];
        uint8_t numLogs = 0;
        ILogSink *eventLogs[50];
        uint8_t numEventLogs = 0;
        bool bbAsync;
        unsigned int maxQueueSize = 50;
        double loggingInterval = 0.100; // in seconds (100ms = 10Hz)
        double loggingRate = 10;        // in hz

        bool hitlMode = false;          // HITL mode enabled

        // Status indicator pins
        int statusLED = -1;      // Main status LED for init diagnostics
        int statusBuzzer = -1;   // Buzzer for init feedback
        int gpsFixLED = -1;      // GPS fix indicator LED

        uint8_t numReporters = 0;
    };
}
#endif
