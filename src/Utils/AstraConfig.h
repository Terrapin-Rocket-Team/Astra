#ifndef Astra_CONFIG_H
#define Astra_CONFIG_H

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
        AstraConfig &withUpdateRate(unsigned int updateRate);

        // Set a sensor/state update interval (in ms).
        // Mutually exclusive with `withUpdateRate()`. Last one called will take effect.
        // Default `100`.
        AstraConfig &withUpdateInterval(unsigned int updateInterval);

        // Set the rate at which logs will be written to the SD card (in hz).
        // Mutually exclusive with `withLoggingInterval()`. Last one called will take effect.
        // Default `10`.
        AstraConfig &withLoggingRate(unsigned int loggingRate);

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
        AstraConfig &withOtherDataReporters(DataReporter **others);

        // Remove the default event handler from the event manager.
        // Useful if you have your own that alters the behavior.
        AstraConfig &withNoDefaultEventListener();

        // Change the formatting of the log prefix.
        // use $time and $logType to access the time and log type of the current log.
        // Default: `"$time - [$logType] "`
        AstraConfig &withLogPrefixFormatting(const char *prefix);

        AstraConfig();

    private:
        State *state = nullptr;
        int pins[50];
        bool bbAsync;
        unsigned int maxQueueSize = 50;
        DataReporter *reporters[50];
        bool useBiasCorrection;
        // IEventListener *defaultEventListener;

        uint8_t numReporters = 0;
    };
}
#endif