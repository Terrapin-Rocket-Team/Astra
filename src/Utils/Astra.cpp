#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "Wire.h"
#include "RetrieveData/SerialHandler.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;
BlinkBuzz bb;
Astra::Astra(AstraConfig *config) : config(config)
{
}
void Astra::init()
{
    // getLogger().recordCrashReport();
    LOGI("Initializing Astra.");
    Wire.begin();
    // BlinkBuzz first
    int pins = 0;
    for (int i = 0; i < 50; i++)
    {
        if (config->pins[i] == -1)
            break;
        pins++;
    }
    bb.init(config->pins, pins, config->bbAsync, config->maxQueueSize);

    // then Logger
    DataReporter **reporters = new DataReporter *[config->numReporters + config->state->getNumMaxSensors() + 1];

    reporters[0] = config->state;
    int i = 1;
    for (; i < config->state->getNumMaxSensors() + 1; i++)
        reporters[i] = config->state->getSensors()[i - 1];
    int j = i;
    for (i = 0; i < config->numReporters; i++)
        reporters[j++] = config->reporters[i];

    DataLogger::configure(config->logs, config->numLogs, reporters, j);

    // bool log = //getLogger().init(reporters, j);
    // getEventManager().invoke(BoolEvent{"LOGGER_INIT"_i, log});

    delay(10);
    // then State
    bool state = config->state->begin();
    // getEventManager().invoke(BoolEvent{"STATE_INIT"_i, state});
    ready = true;

    // getLogger().writeCsvHeader();
    LOGI("Astra initialized.");
}
bool Astra::update(double ms)
{
    bool didUpdate = false;
    if (!ready)
    {
        LOGW("Attempted to update Astra before it was initialized. Initializing it now...");
        init();
    }
    getSerialHandler().handle();
    // loop based on time and interval and update bb.
    bb.update();
    if (ms == -1)
        ms = millis();

    if (ms - lastStateUpdate > config->updateInterval)
    {
        lastStateUpdate = ms;
        if (config->state)
            config->state->update();
        else
            LOGW("Astra Attempted to update State without a reference to it! (use AstraConfig.withState(&stateVar))");
        didUpdate = true;
    }

    if (ms - lastLoggingUpdate > config->loggingInterval)
    {
        lastLoggingUpdate = ms;
        // getLogger().recordFlightData();
    }

    return didUpdate;
}