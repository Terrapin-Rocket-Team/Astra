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
            reporters[i] = nullptr;
        }
        // defaultEventListener = new DefaultEventHandler();
    }
    AstraConfig &AstraConfig::withState(State *state)
    {
        this->state = state;
        return *this;
    }
    AstraConfig &AstraConfig::withUpdateRate(unsigned int updateRate)
    {
        if (UPDATE_RATE == updateRate)
            return *this;
        //getLogger().recordLogData(LOG_, "Update rate modified from %d to %d hz.", UPDATE_RATE, updateRate);
        UPDATE_RATE = updateRate;
        UPDATE_INTERVAL = 1000.0 / updateRate;
        return *this;
    }
    AstraConfig &AstraConfig::withUpdateInterval(unsigned int updateInterval)
    {
        if (UPDATE_INTERVAL == updateInterval)
            return *this;
        //getLogger().recordLogData(LOG_, "Update interval modified from %d to %d ms.", UPDATE_INTERVAL, updateInterval);
        UPDATE_INTERVAL = updateInterval;
        UPDATE_RATE = 1000.0 / updateInterval;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingRate(unsigned int loggingRate)
    {
        if (LOGGING_RATE == loggingRate)
            return *this;
        //getLogger().recordLogData(LOG_, "Logging rate modified from %d to %d hz.", LOGGING_RATE, loggingRate);
        LOGGING_RATE = loggingRate;
        LOGGING_INTERVAL = 1000.0 / loggingRate;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingInterval(unsigned int loggingInterval)
    {
        if (LOGGING_INTERVAL == loggingInterval)
            return *this;
        //getLogger().recordLogData(LOG_, "Logging interval modified from %d to %d ms.", LOGGING_INTERVAL, loggingInterval);
        LOGGING_INTERVAL = loggingInterval;
        LOGGING_RATE = 1000.0 / loggingInterval;
        return *this;
    }


    AstraConfig &AstraConfig::withSensorBiasCorrectionDataLength(unsigned int sensorBiasCorrectionDataLength)
    {
        if (SENSOR_BIAS_CORRECTION_DATA_LENGTH == sensorBiasCorrectionDataLength)
            return *this;
        //getLogger().recordLogData(LOG_, "Sensor bias correction data length modified from %d to %d s.", SENSOR_BIAS_CORRECTION_DATA_LENGTH, sensorBiasCorrectionDataLength);
        SENSOR_BIAS_CORRECTION_DATA_LENGTH = sensorBiasCorrectionDataLength;
        return *this;
    }
    AstraConfig &AstraConfig::withSensorBiasCorrectionDataIgnore(unsigned int sensorBiasCorrectionDataIgnore)
    {
        if (SENSOR_BIAS_CORRECTION_DATA_IGNORE == sensorBiasCorrectionDataIgnore)
            return *this;
        //getLogger().recordLogData(LOG_, "Sensor bias correction data ignore modified from %d to %d s.", SENSOR_BIAS_CORRECTION_DATA_IGNORE, sensorBiasCorrectionDataIgnore);
        SENSOR_BIAS_CORRECTION_DATA_IGNORE = sensorBiasCorrectionDataIgnore;
        return *this;
    }
    AstraConfig &AstraConfig::withUsingSensorBiasCorrection(bool useBiasCorrection)
    {
        if (this->useBiasCorrection == useBiasCorrection)
            return *this;
        //getLogger().recordLogData(LOG_, "Sensor bias correction modified from %s to %s.", useBiasCorrection ? "true" : "false", useBiasCorrection ? "true" : "false");
        this->useBiasCorrection = useBiasCorrection;
        return *this;
    }
    AstraConfig &AstraConfig::withBuzzerPin(unsigned int buzzerPin)
    {
        if ((unsigned int) BUZZER == buzzerPin)
            return *this;
        BUZZER = buzzerPin;
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int) pins[i] == buzzerPin)
            {
                //getLogger().recordLogData(WARNING_, "Attempted to set buzzer pin %d to BlinkBuzz, but it is already in use.", buzzerPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = buzzerPin;
                return *this;
            }
        }
        //getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", buzzerPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBPin(unsigned int bbPin)
    {
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int) pins[i] == bbPin)
            {
                //getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but it is already in use.", bbPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = bbPin;
                return *this;
            }
        }
        //getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", bbPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBAsync(bool bbAsync, unsigned int queueSize)
    {
        if (this->bbAsync == bbAsync)
            return *this;
        //getLogger().recordLogData(LOG_, "BlinkBuzz async modified from %s to %s.", this->bbAsync ? "true" : "false", bbAsync ? "true" : "false");
        this->bbAsync = bbAsync;
        return *this;
    }
    AstraConfig &AstraConfig::withOtherDataReporters(DataReporter **others)
    {
        for (int i = 0; i < 50; i++)
        {
            if (reporters[i] == others[i])
            {
                //getLogger().recordLogData(WARNING_, "Attempted to add DataReporter %s to logger, but it was already there", others[i]->getName());
                return *this;
            }
            if (reporters[i] == nullptr)
            {
                reporters[i] = others[i];
                //getLogger().recordLogData(LOG_, "Added DataReporter %s to logger", others[i]->getName());
                numReporters++;
                return *this;
            }
        }
        //getLogger().recordLogData(WARNING_, "Attempted to add DataReporter %s to logger, but the maximum number of reporters (50) has been reached. That's too many reporters. Why.", others[0]->getName());
        return *this;
    }

    AstraConfig &AstraConfig::withNoDefaultEventListener()
    {
        // delete defaultEventListener;
        // defaultEventListener = nullptr;
        return *this;
    }

    AstraConfig &AstraConfig::withLogPrefixFormatting(const char *prefix)
    {
        //getLogger().setLogPrefixFormatting(prefix);
        return *this;
    }

}