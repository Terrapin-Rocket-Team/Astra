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

    // Populate and initialize SensorManager from config
    config->populateSensorManager();
    config->sensorManager.begin();

    // Setup SerialMessageRouter for command handling
    messageRouter = new SerialMessageRouter(4, 8, 256);
    messageRouter->withInterface(&Serial)
        .withListener("CMD/", handleCommandMessage);

    delay(10);

    // Initialize State
    if (config->state)
    {
        config->state->begin();
    }

    // Set baro origin if available (sensors have settled by now)
    if (config->sensorManager.getBaroSource() && config->sensorManager.getBaroSource()->isInitialized())
    {
        double baroAlt = config->sensorManager.getBarometricAltitude();
        if (config->state)
            config->state->setBaroOrigin(baroAlt);
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

    // =================== Sensor Update ===================
    // Update sensors at configured rate
    if (timeSeconds - lastSensorUpdate >= config->sensorUpdateInterval)
    {
        lastSensorUpdate = timeSeconds;
        config->sensorManager.update(timeSeconds);
        _didUpdateSensors = true;

        // Update orientation immediately after sensor update
        // This should run at high rate (gyro rate)
        Vector<3> gyro = config->sensorManager.getAngularVelocity();
        Vector<3> accel = config->sensorManager.getAcceleration();
        double dt = timeSeconds - lastTime;
        if (dt > 0 && config->state)
        {
            config->state->updateOrientation(gyro, accel, dt);
        }
        lastTime = timeSeconds;
    }

    // =================== Prediction Step ===================
    // Run KF prediction at configured rate
    if (timeSeconds - lastPredictUpdate >= config->predictInterval)
    {
        double predictDt = timeSeconds - lastPredictUpdate;
        lastPredictUpdate = timeSeconds;

        if (config->state && predictDt > 0)
        {
            config->state->predict(predictDt);
        }
        _didPredictState = true;
    }

    // =================== Measurement Update ===================
    // Update KF with GPS/baro measurements at configured rate
    if (timeSeconds - lastMeasurementUpdate >= config->measurementUpdateInterval)
    {
        lastMeasurementUpdate = timeSeconds;

        // Fetch sensor data
        Vector<3> gpsPos = config->sensorManager.getGPSPosition();
        Vector<3> gpsVel = config->sensorManager.getGPSVelocity();
        double baroAlt = config->sensorManager.getBarometricAltitude();

        bool hasGPS = config->sensorManager.hasGPSUpdate() &&
                     config->sensorManager.getGPSSource() &&
                     config->sensorManager.getGPSSource()->getHasFix();

        bool hasBaro = config->sensorManager.hasBaroUpdate() &&
                      config->sensorManager.getBaroSource() &&
                      config->sensorManager.getBaroSource()->isInitialized();

        // Update state with measurements
        if (config->state && (hasGPS || hasBaro))
        {
            config->state->updateMeasurements(gpsPos, gpsVel, baroAlt, hasGPS, hasBaro);
        }

        // Clear update flags after consuming
        if (hasGPS)
            config->sensorManager.clearGPSUpdate();
        if (hasBaro)
            config->sensorManager.clearBaroUpdate();

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