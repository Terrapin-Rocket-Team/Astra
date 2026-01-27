#ifndef Astra_CONFIG_H
#define Astra_CONFIG_H

#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include <stdint.h>

namespace astra
{
    class State;
    class DataReporter;
    class Sensor;
    class SensorManager;

    class AstraConfig
    {
        friend class Astra;

    public:
        // Add state to Astra's knowledge.
        // No default.
        AstraConfig &withState(State *state);

        // Add sensors to Astra's knowledge. Astra will manage updating them.
        // No default.
        AstraConfig &withSensorManager(SensorManager *sensorManager);

        // Set sensor read rate (in hz). Sensors will be polled at this rate.
        // Default `30`.
        AstraConfig &withSensorUpdateRate(double sensorUpdateRate);

        // Set sensor read interval (in ms). Sensors will be polled at this interval.
        // Mutually exclusive with `withSensorUpdateRate()`. Last one called will take effect.
        // Default `33` (~30 Hz).
        AstraConfig &withSensorUpdateInterval(unsigned int sensorUpdateInterval);

        // Set Kalman filter prediction rate (in hz). Filter prediction step runs at this rate.
        // Default `30`.
        AstraConfig &withPredictRate(double predictRate);

        // Set Kalman filter prediction interval (in ms). Filter prediction step runs at this interval.
        // Mutually exclusive with `withPredictRate()`. Last one called will take effect.
        // Default `33` (~30 Hz).
        AstraConfig &withPredictInterval(unsigned int predictInterval);

        // Set measurement update rate (in hz). Filter measurement update step runs at this rate.
        // Default `20`.
        AstraConfig &withMeasurementUpdateRate(double measurementUpdateRate);

        // Set measurement update interval (in ms). Filter measurement update step runs at this interval.
        // Mutually exclusive with `withMeasurementUpdateRate()`. Last one called will take effect.
        // Default `50` (20 Hz).
        AstraConfig &withMeasurementUpdateInterval(unsigned int measurementUpdateInterval);

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

        // Setup which telemetry logs will be written to on update
        // No default
        AstraConfig &withDataLogs(ILogSink **logs, uint8_t numLogs);

        // Enable HITL (Hardware-In-The-Loop) mode
        // When enabled, Astra configures all update intervals to 0 for maximum simulation speed
        // The user must pass simulation time to update(simTimeMs)
        // Default `false`
        AstraConfig &withHITL(bool hitlEnabled);

        AstraConfig();

    protected:
        State *state = nullptr;
        SensorManager *sensorManager = nullptr;
        int pins[50];
        ILogSink *logs[50];
        uint8_t numLogs = 0;
        bool bbAsync;
        unsigned int maxQueueSize = 50;
        double loggingInterval = 0.100; // in seconds (100ms = 10Hz)
        double loggingRate = 10;        // in hz

        double sensorUpdateInterval = 0.033;      // in seconds (~30 Hz)
        double predictInterval = 0.033;           // in seconds (~30 Hz)
        double measurementUpdateInterval = 0.050; // in seconds (20 Hz)
        double sensorUpdateRate = 30;             // in hz
        double predictRate = 30;                  // in hz
        double measurementUpdateRate = 20;        // in hz

        bool hitlMode = false;                    // HITL mode enabled

        uint8_t numReporters = 0;
    };
}
#endif