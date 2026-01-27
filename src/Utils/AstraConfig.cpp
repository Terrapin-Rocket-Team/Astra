#include "AstraConfig.h"
#include "../State/State.h"
#include "../BlinkBuzz/BlinkBuzz.h"

namespace astra
{

    AstraConfig::AstraConfig()
    {
        for (int i = 0; i < 50; i++)
        {
            pins[i] = -1;
        }
    }
    AstraConfig &AstraConfig::withState(State *state)
    {
        this->state = state;
        return *this;
    }

    AstraConfig &AstraConfig::withSensorManager(SensorManager *sensorManager)
    {
        this->sensorManager = sensorManager;
        LOGI("Custom SensorManager configured.");
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingRate(double loggingRate)
    {
        if (this->loggingRate == loggingRate)
            return *this;
        LOGI("Logging rate modified from %d to %d hz.", loggingRate, loggingRate);
        this->loggingRate = loggingRate;
        this->loggingInterval = 1000.0 / loggingRate;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingInterval(unsigned int loggingIntervalMs)
    {
        double loggingIntervalSec = loggingIntervalMs / 1000.0;
        if (this->loggingInterval == loggingIntervalSec)
            return *this;
        LOGI("Logging interval modified to %d ms", loggingIntervalMs);
        this->loggingInterval = loggingIntervalSec;
        this->loggingRate = 1000.0 / loggingIntervalMs;
        return *this;
    }
    AstraConfig &AstraConfig::withBuzzerPin(unsigned int buzzerPin)
    {
        if ((unsigned int)BUZZER == buzzerPin)
            return *this;
        BUZZER = buzzerPin;
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int)pins[i] == buzzerPin)
            {
                LOGW("Attempted to set buzzer pin %d to BlinkBuzz, but it is already in use.", buzzerPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = buzzerPin;
                return *this;
            }
        }
        LOGW("Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", buzzerPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBPin(unsigned int bbPin)
    {
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int)pins[i] == bbPin)
            {
                LOGW("Attempted to add pin %d to BlinkBuzz, but it is already in use.", bbPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = bbPin;
                return *this;
            }
        }
        LOGW("Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", bbPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBAsync(bool bbAsync, unsigned int queueSize)
    {
        if (this->bbAsync == bbAsync)
            return *this;
        LOGI("BlinkBuzz async modified from %s to %s.", this->bbAsync ? "true" : "false", bbAsync ? "true" : "false");
        this->bbAsync = bbAsync;
        return *this;
    }

    AstraConfig &AstraConfig::withDataLogs(ILogSink **logs, uint8_t numLogs)
    {
        if (numLogs > 50)
        {
            LOGW("Attempted to add %d log sinks, but the maximum number of log sinks is 50. That's too many logs. Why.", numLogs);
            numLogs = 50;
        }
        for (uint8_t i = 0; i < numLogs; i++)
            this->logs[i] = logs[i];
        this->numLogs = numLogs;
        return *this;
    }

    AstraConfig &AstraConfig::withSensorUpdateRate(double sensorUpdateRate)
    {
        this->sensorUpdateRate = sensorUpdateRate;
        this->sensorUpdateInterval = 1000.0 / sensorUpdateRate;
        LOGI("Sensor update rate set to %.2f Hz (%.2f ms interval)", sensorUpdateRate, this->sensorUpdateInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withSensorUpdateInterval(unsigned int sensorUpdateIntervalMs)
    {
        this->sensorUpdateInterval = sensorUpdateIntervalMs / 1000.0;
        this->sensorUpdateRate = 1000.0 / sensorUpdateIntervalMs;
        LOGI("Sensor update interval set to %u ms (%.2f Hz)", sensorUpdateIntervalMs, this->sensorUpdateRate);
        return *this;
    }

    AstraConfig &AstraConfig::withPredictRate(double predictRate)
    {
        this->predictRate = predictRate;
        this->predictInterval = 1000.0 / predictRate;
        LOGI("Predict rate set to %.2f Hz (%.2f ms interval)", predictRate, this->predictInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withPredictInterval(unsigned int predictIntervalMs)
    {
        this->predictInterval = predictIntervalMs / 1000.0;
        this->predictRate = 1000.0 / predictIntervalMs;
        LOGI("Predict interval set to %u ms (%.2f Hz)", predictIntervalMs, this->predictRate);
        return *this;
    }

    AstraConfig &AstraConfig::withMeasurementUpdateRate(double measurementUpdateRate)
    {
        this->measurementUpdateRate = measurementUpdateRate;
        this->measurementUpdateInterval = 1000.0 / measurementUpdateRate;
        LOGI("Measurement update rate set to %.2f Hz (%.2f ms interval)", measurementUpdateRate, this->measurementUpdateInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withMeasurementUpdateInterval(unsigned int measurementUpdateIntervalMs)
    {
        this->measurementUpdateInterval = measurementUpdateIntervalMs / 1000.0;
        this->measurementUpdateRate = 1000.0 / measurementUpdateIntervalMs;
        LOGI("Measurement update interval set to %u ms (%.2f Hz)", measurementUpdateIntervalMs, this->measurementUpdateRate);
        return *this;
    }

    AstraConfig &AstraConfig::withHITL(bool hitlEnabled)
    {
        this->hitlMode = hitlEnabled;

        return *this;
    }

}