#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "Wire.h"
#ifndef NATIVE
#include "RetrieveData/RetrieveSDCardData.h"
#endif
#include "RecordData/Logging/DataLogger.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include <cstring>
using namespace astra;
BlinkBuzz bb;

#ifndef ASTRA_VERSION
#define ASTRA_VERSION "UNKNOWN"
#endif

Astra::Astra(AstraConfig *config) : config(config), messageRouter(nullptr)
{
}

void Astra::handleCommandMessage(const char* message, const char* prefix, Stream* source)
{
    if (!message || !source)
        return;

    // Check if command is "HEADER"
    if (strcmp(message, "HEADER") == 0)
    {
        // Create a temporary PrintLog wrapper to send header to the requesting stream
        PrintLog tempLog(*source, true);  // true = wants prefix
        if (tempLog.begin())
        {
            if (DataLogger::available())
            {
                DataLogger::instance().printHeaderTo(&tempLog);
            }
            tempLog.end();
        }
    }
}

void Astra::init()
{
    LOGI("Initializing Astra version %s", ASTRA_VERSION);
#ifdef ENV_STM
    Wire.begin(PB9, PB8); // stm32
#else
    Wire.begin();
#endif
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

    // Setup SerialMessageRouter for command handling
    messageRouter = new SerialMessageRouter(4, 8, 256);
    messageRouter->withInterface(&Serial)
                  .withListener("CMD/", handleCommandMessage);

    delay(10);
    // then State
    config->state->begin();
    ready = true;
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

    // Update SerialMessageRouter to handle incoming commands
    if (messageRouter)
    {
        messageRouter->update();
    }

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
        if (DataLogger::available())
        {
            DataLogger::instance().appendLine();
        }
    }

    return didUpdate;
}