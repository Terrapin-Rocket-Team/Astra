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
    AstraConfig &AstraConfig::withUpdateRate(double updateRate)
    {
        if (this->updateRate == updateRate)
            return *this;
        // getLogger().recordLogData(LOG_, "Update rate modified from %d to %d hz.", updateRate, updateRate);
        this->updateRate = updateRate;
        this->updateInterval = 1000.0 / updateRate;
        return *this;
    }
    AstraConfig &AstraConfig::withUpdateInterval(unsigned int updateInterval)
    {
        if (this->updateInterval == updateInterval)
            return *this;
        // getLogger().recordLogData(LOG_, "Update interval modified from %d to %d ms.", updateInterval, updateInterval);
        this->updateInterval = updateInterval;
        this->updateRate = 1000.0 / updateInterval;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingRate(double loggingRate)
    {
        if (this->loggingRate == loggingRate)
            return *this;
        // getLogger().recordLogData(LOG_, "Logging rate modified from %d to %d hz.", loggingRate, loggingRate);
        this->loggingRate = loggingRate;
        this->loggingInterval = 1000.0 / loggingRate;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingInterval(unsigned int loggingInterval)
    {
        if (this->loggingInterval == loggingInterval)
            return *this;
        // getLogger().recordLogData(LOG_, "Logging interval modified from %d to %d ms.", loggingInterval, loggingInterval);
        this->loggingInterval = loggingInterval;
        this->loggingRate = 1000.0 / loggingInterval;
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
                // getLogger().recordLogData(WARNING_, "Attempted to set buzzer pin %d to BlinkBuzz, but it is already in use.", buzzerPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = buzzerPin;
                return *this;
            }
        }
        // getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", buzzerPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBPin(unsigned int bbPin)
    {
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int)pins[i] == bbPin)
            {
                // getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but it is already in use.", bbPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = bbPin;
                return *this;
            }
        }
        // getLogger().recordLogData(WARNING_, "Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", bbPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBAsync(bool bbAsync, unsigned int queueSize)
    {
        if (this->bbAsync == bbAsync)
            return *this;
        // getLogger().recordLogData(LOG_, "BlinkBuzz async modified from %s to %s.", this->bbAsync ? "true" : "false", bbAsync ? "true" : "false");
        this->bbAsync = bbAsync;
        return *this;
    }
    AstraConfig &AstraConfig::withOtherDataReporters(DataReporter **others)
    {
        for (int i = 0; i < 50; i++)
        {
            if (reporters[i] == others[i])
            {
                // getLogger().recordLogData(WARNING_, "Attempted to add DataReporter %s to logger, but it was already there", others[i]->getName());
                return *this;
            }
            if (reporters[i] == nullptr)
            {
                reporters[i] = others[i];
                // getLogger().recordLogData(LOG_, "Added DataReporter %s to logger", others[i]->getName());
                numReporters++;
                return *this;
            }
        }
        // getLogger().recordLogData(WARNING_, "Attempted to add DataReporter %s to logger, but the maximum number of reporters (50) has been reached. That's too many reporters. Why.", others[0]->getName());
        return *this;
    }

}