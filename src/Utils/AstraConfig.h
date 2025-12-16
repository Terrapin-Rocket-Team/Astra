#ifndef Astra_CONFIG_H
#define Astra_CONFIG_H

#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include <stdint.h>

namespace astra
{
    class State;
    class DataReporter;
    class AstraConfig
    {
        friend class Astra;

    public:
        // Add state (and its sensors) to Astra's knowledge.
        // No default.
        AstraConfig &withState(State *state);

        // Set a sensor/state update rate (in hz).
        // Mutually exclusive with `withUpdateInterval()`. Last one called will take effect.
        // Default `10`.
        AstraConfig &withUpdateRate(double updateRate);

        // Set a sensor/state update interval (in ms).
        // Mutually exclusive with `withUpdateRate()`. Last one called will take effect.
        // Default `100`.
        AstraConfig &withUpdateInterval(unsigned int updateInterval);

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

        // Add more `DataReporter` objects for Logger to record flight data from.
        // Passing in a State (via `withState()`) will also capture the state's sensors for logging, so adding them here is redundant.
        // No other `DataReporter`s added by default.
        AstraConfig &withOtherDataReporters(DataReporter **others, uint8_t numOthers);

        // Setup which telemetry logs will be written to on update
        // No default
        AstraConfig &withDataLogs(ILogSink **logs, uint8_t numLogs);

        AstraConfig();

    private:
        State *state = nullptr;
        int pins[50];
        ILogSink *logs[50];
        uint8_t numLogs = 0;
        bool bbAsync;
        unsigned int maxQueueSize = 50;
        DataReporter *reporters[50];
        unsigned int updateInterval = 100;  // in ms
        unsigned int loggingInterval = 100; // in ms
        double loggingRate = 10;            // in hz
        double updateRate = 10;             // in hz

        uint8_t numReporters = 0;
    };
}
#endif