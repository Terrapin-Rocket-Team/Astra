#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "Sensors/Sensor.h"
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
    // Setup sensor manager with sensors from config
    if (config->sensors && config->numSensors > 0)
    {
        sensorManager.setSensors(config->sensors, config->numSensors);
    }
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

    // Initialize sensors via SensorManager
    sensorManager.initAll();

    // then Logger - combine sensors and other reporters
    Sensor **sensors = sensorManager.getSensors();
    int numSensors = sensorManager.getCount();
    DataReporter **reporters = new DataReporter *[config->numReporters + numSensors + 1];

    reporters[0] = config->state;
    int i = 1;
    for (int s = 0; s < numSensors; s++)
        reporters[i++] = sensors[s];
    for (int r = 0; r < config->numReporters; r++)
        reporters[i++] = config->reporters[r];

    DataLogger::configure(config->logs, config->numLogs, reporters, i);

    // Setup SerialMessageRouter for command handling
    messageRouter = new SerialMessageRouter(4, 8, 256);
    messageRouter->withInterface(&Serial)
                  .withListener("CMD/", handleCommandMessage);

    delay(10);
    // then State
    if (config->state)
    {
        config->state->begin(&sensorManager);
    }
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

    // Convert ms to seconds for State time tracking (HITL compatibility)
    double currentTime = ms / 1000.0;

    if (!config->state)
    {
        LOGW("Astra Attempted to update State without a reference to it! (use AstraConfig.withState(&stateVar))");
        return didUpdate;
    }

    // Sensor update - highest frequency
    if (ms - lastSensorUpdate > config->sensorUpdateInterval)
    {
        lastSensorUpdate = ms;
        sensorManager.updateAll();
        didUpdate = true;
    }

    // Orientation update - runs after sensors
    if (didUpdate && config->state)
    {
        double gyro[3], accel[3];
        bool hasAccel = sensorManager.getAccelData(accel);
        bool hasGyro = sensorManager.getGyroData(gyro);

        if (hasAccel && hasGyro)
        {
            double dt = currentTime - (lastOrientationUpdate / 1000.0);
            config->state->updateOrientation(gyro, accel, dt);
            lastOrientationUpdate = ms;
        }
    }

    // Prediction step - run at predict rate
    if (ms - lastPredictUpdate > config->predictInterval)
    {
        lastPredictUpdate = ms;
        config->state->predictState(currentTime);
    }

    // Measurement update - run at measurement update rate
    if (ms - lastMeasurementUpdate > config->measurementUpdateInterval)
    {
        lastMeasurementUpdate = ms;

        // Extract GPS and barometer data
        double gpsLat, gpsLon, gpsAlt, baroAlt;
        bool hasGPS = sensorManager.getGPSData(&gpsLat, &gpsLon, &gpsAlt);
        bool hasBaro = sensorManager.getBaroAltitude(&baroAlt);

        config->state->updateMeasurements(gpsLat, gpsLon, gpsAlt, baroAlt, hasGPS, hasBaro, currentTime);

        // Update position/velocity tracking
        double heading;
        bool hasFix;
        if (sensorManager.getGPSHeading(&heading) && sensorManager.getGPSHasFix(&hasFix))
        {
            config->state->updatePositionVelocity(gpsLat, gpsLon, heading, hasFix);
        }
    }

    // Logging update
    if (ms - lastLoggingUpdate > config->loggingInterval)
    {
        lastLoggingUpdate = ms;
        if (DataLogger::available())
        {
            DataLogger::instance().appendLine();
        }
    }

    // Keep backward compatibility with old updateInterval (deprecated)
    if (ms - lastStateUpdate > config->updateInterval)
    {
        lastStateUpdate = ms;
    }

    return didUpdate;
}