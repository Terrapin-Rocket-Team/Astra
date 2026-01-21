#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "Sensors/Sensor.h"
#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/GPS/GPS.h"
#include "Sensors/Baro/Barometer.h"
#include "Math/Vector.h"
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
    // Store sensors from config
    if (config->sensors && config->numSensors > 0)
    {
        sensors = config->sensors;
        numSensors = config->numSensors;
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

bool Astra::initAllSensors()
{
    int good = 0;
    for (int i = 0; i < numSensors; i++)
    {
        if (sensors[i])
        {
            if (sensors[i]->begin())
            {
                good++;
                LOGI("%s [%s] initialized.", sensors[i]->getTypeString(), sensors[i]->getName());
            }
            else
            {
                LOGE("%s [%s] failed to initialize.", sensors[i]->getTypeString(), sensors[i]->getName());
            }
        }
        else
        {
            LOGE("Sensor index %d in the array was null!", i);
        }
    }
    LOGI("Initialized %d of %d sensors.", good, numSensors);
    return good == numSensors;
}

void Astra::updateAllSensors()
{
    for (int i = 0; i < numSensors; i++)
    {
        if (sensors[i] && sensors[i]->isInitialized())
        {
            sensors[i]->update();
        }
    }
}

Sensor *Astra::findSensor(uint32_t type, int sensorNum) const
{
    for (int i = 0; i < numSensors; i++)
    {
        if (sensors[i] && type == sensors[i]->getType() && --sensorNum == 0)
            return sensors[i];
    }
    return nullptr;
}

Accel *Astra::findAccel(int sensorNum) const
{
    // Try standalone accelerometer first
    Sensor *sensor = findSensor("Accelerometer"_i, sensorNum);
    if (sensor)
        return static_cast<Accel *>(sensor);

    // Fall back to IMU6DoF
    sensor = findSensor("IMU6DoF"_i, sensorNum);
    if (sensor)
        return reinterpret_cast<Accel *>(sensor);

    // Fall back to IMU9DoF
    sensor = findSensor("IMU9DoF"_i, sensorNum);
    if (sensor)
        return reinterpret_cast<Accel *>(sensor);

    return nullptr;
}

Gyro *Astra::findGyro(int sensorNum) const
{
    // Try standalone gyroscope first
    Sensor *sensor = findSensor("Gyroscope"_i, sensorNum);
    if (sensor)
        return static_cast<Gyro *>(sensor);

    // Fall back to IMU6DoF
    sensor = findSensor("IMU6DoF"_i, sensorNum);
    if (sensor)
        return reinterpret_cast<Gyro *>(sensor);

    // Fall back to IMU9DoF
    sensor = findSensor("IMU9DoF"_i, sensorNum);
    if (sensor)
        return reinterpret_cast<Gyro *>(sensor);

    return nullptr;
}

GPS *Astra::findGPS(int sensorNum) const
{
    return static_cast<GPS *>(findSensor("GPS"_i, sensorNum));
}

Barometer *Astra::findBaro(int sensorNum) const
{
    return static_cast<Barometer *>(findSensor("Barometer"_i, sensorNum));
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

    // Initialize sensors directly
    initAllSensors();

    // then Logger - combine sensors and other reporters
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
        config->state->withSensors(sensors, numSensors);
        config->state->begin();
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
        updateAllSensors();
        didUpdate = true;
    }

    // Orientation update - runs after sensors
    if (didUpdate && config->state)
    {
        Accel *accel = findAccel();
        Gyro *gyro = findGyro();

        if (accel && accel->isInitialized() && gyro && gyro->isInitialized())
        {
            double dt = currentTime - (lastOrientationUpdate / 1000.0);
            config->state->updateOrientation(gyro->getAngVel(), accel->getAccel(), dt);
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

        // Get sensors
        GPS *gps = findGPS();
        Barometer *baro = findBaro();

        bool hasGPS = gps && gps->isInitialized();
        bool hasBaro = baro && baro->isInitialized();

        Vector<3> gpsPos = hasGPS ? gps->getPos() : Vector<3>();
        double baroAlt = hasBaro ? baro->getASLAltM() : 0.0;

        config->state->updateMeasurements(gpsPos, baroAlt, hasGPS, hasBaro, currentTime);

        // Update position/velocity tracking
        if (hasGPS)
        {
            config->state->updatePositionVelocity(gpsPos.x(), gpsPos.y(), gps->getHeading(), gps->getHasFix());
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