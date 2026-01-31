#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "Sensors/Sensor.h"
#include "Sensors/SensorManager/SensorManager.h"
#include "Sensors/GPS/GPS.h"
#include "Sensors/Baro/Barometer.h"
#include "Math/Vector.h"
#include "Wire.h"
#include "Sensors/SensorManager/SensorManager.h"
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

Astra::~Astra()
{
}

void Astra::handleCommandMessage(const char *message, const char *prefix, Stream *source)
{
    if (!message || !source)
        return;

    // Check if command is "HEADER"
    if (strcmp(message, "HEADER") == 0)
    {
        // Create a temporary PrintLog wrapper to send header to the requesting stream
        PrintLog tempLog(*source, true); // true = wants prefix
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

    // Loggign next
    DataLogger::configure(config->logs, config->numLogs);
    // setup for HITL
    if (config->hitlMode)
    {
        LOGI("HITL mode enabled - configuring for simulation.");
        // Set all intervals to 0 for maximum simulation speed
        // User must pass simulation time to update()
        config->sensorUpdateInterval = 0;
        config->loggingInterval = 0;
        config->measurementUpdateInterval = 0;
        config->predictInterval = 0;
    }

    // Initialize SensorManager
    if (config->sensorManager)
    {
        config->sensorManager->begin();
        config->state->withSensorManager(config->sensorManager);
    }

    // Setup SerialMessageRouter for command handling
    messageRouter = new SerialMessageRouter(4, 8, 256);
    messageRouter->withInterface(&Serial)
        .withListener("CMD/", handleCommandMessage);

    delay(10);
    // then State
    if (config->state)
    {
        config->state->begin();
    }
    ready = true;
    LOGI("Astra initialized.");
}
bool Astra::update(double timeSeconds)
{

    _didLog = false;
    _didUpdateSensors = false;
    _didUpdateState = false;
    _didPredictState = false;

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

    // If no time provided, use system time in seconds
    if (timeSeconds == -1)
        timeSeconds = millis() / 1000.0;

    double currentTime = timeSeconds;
    

    if (!config->state)
    {
        LOGW("Astra Attempted to update State without a reference to it! (use AstraConfig.withState(&stateVar))");
        return false;
    }
    if (!config->sensorManager)
    {
        LOGW("Astra Attempted to update SensorManager without a reference to it! (use AstraConfig.withSensorManager(&sensorManager))");
        return false;
    }

    // Sensor update - highest frequency

    if (timeSeconds - lastSensorUpdate >= config->sensorUpdateInterval)
    {

        lastSensorUpdate = timeSeconds;
        if (config->sensorManager)
        {
            config->sensorManager->update(timeSeconds);
        }
        _didUpdateSensors = true;

    }

    // Prediction step - run at predict rate

    if (timeSeconds - lastPredictUpdate >= config->predictInterval)
    {

        lastPredictUpdate = timeSeconds;
        config->state->predictState(currentTime);
        _didPredictState = true;

    }

    // Measurement update - run at measurement update rate

    if (timeSeconds - lastMeasurementUpdate >= config->measurementUpdateInterval)
    {

        lastMeasurementUpdate = timeSeconds;
        config->state->update(currentTime);
        _didUpdateState = true;

    }

    // Logging update

    if (timeSeconds - lastLoggingUpdate >= config->loggingInterval)
    {

        lastLoggingUpdate = timeSeconds;
        if (DataLogger::available())
        {
            for (uint8_t i = 0; i < DataLogger::instance().getNumReporters(); i++)
            {
                auto d = DataLogger::instance().getReporters()[i];
                if (d && d->getAutoUpdate())
                {
                    d->update();
                }
            }
            DataLogger::instance().appendLine();
            LOGI("DEBUG Astra: appendLine() complete");
        }
        else
        {
            LOGE("DEBUG Astra: DataLogger NOT available!");
        }
        _didLog = true;
        
    }
    
    return true; // bool here is deprecated
}