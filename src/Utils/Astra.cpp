#include "Astra.h"
#include "BlinkBuzz/BlinkBuzz.h"
#include "State/State.h"
#include "State/DefaultState.h"
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
#include "RecordData/Logging/EventLogger.h"
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
    if (ownsState && config && config->state)
    {
        delete config->state;
        config->state = nullptr;
    }
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

int Astra::init()
{
    // Configure event logger before any LOGI/LOGW output
    if (config->numEventLogs > 0)
    {
        EventLogger::configure(config->eventLogs, config->numEventLogs);
    }

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

    // Logging next
    DataLogger::configure(config->logs, config->numLogs);
    // setup for HITL
    if (config->hitlMode)
    {
        LOGI("HITL mode enabled - sensors and state updates run event-driven. User must pass simulation time to update().");
    }

    // Populate and initialize SensorManager from config
    config->populateSensorManager();
    int sensorErrors = config->sensorManager.begin();

    // Determine error code for status indicators
    // Error codes: 0=success, 1=accel, 2=gyro, 3=mag, 4=baro, 5=gps, 6=misc, 7+=multiple
    if (sensorErrors == 0)
    {
        initErrorCode = 0;  // Success
    }
    else if (sensorErrors == 1)
    {
        // Single sensor failure - determine which one
        if (config->sensorManager.didAccelInitFail())
            initErrorCode = 1;
        else if (config->sensorManager.didGyroInitFail())
            initErrorCode = 2;
        else if (config->sensorManager.didMagInitFail())
            initErrorCode = 3;
        else if (config->sensorManager.didBaroInitFail())
            initErrorCode = 4;
        else if (config->sensorManager.didGPSInitFail())
            initErrorCode = 5;
        else if (config->sensorManager.didMiscInitFail())
            initErrorCode = 6;
    }
    else
    {
        // Multiple failures
        initErrorCode = 7;
    }

    // Setup SerialMessageRouter for command handling
    messageRouter = new SerialMessageRouter(4, 8, 256);
    messageRouter->withInterface(&Serial)
        .withListener("CMD/", handleCommandMessage);

    delay(10);

    // Initialize State (create DefaultState if not provided)
    if (!config->state)
    {
        LOGW("No State provided; using DefaultState.");
        config->state = new DefaultState();
        ownsState = true;
    }
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

    // Play init feedback on status indicators
    playInitFeedback(initErrorCode);

    if (sensorErrors == 0)
        LOGI("Astra initialized successfully.");
    else
        LOGW("Astra initialized with %d sensor error(s).", sensorErrors);

    return sensorErrors;
}
bool Astra::update(double timeSeconds)
{

    _didLog = false;
    _didUpdateState = false;
    _didPredictState = false;

    if (!ready)
    {
        LOGW("Attempted to update Astra before it was initialized. Initializing it now...");
        init();
    }

    // Update SerialMessageRouter to handle incoming commands
    // Guard against re-entrant calls (e.g. HITL handler calling sys.update())
    if (messageRouter && !inRouterUpdate)
    {
        inRouterUpdate = true;
        messageRouter->update();
        inRouterUpdate = false;
    }

    bb.update();

    // If no time provided, use system time in seconds
    if (timeSeconds == -1)
        timeSeconds = millis() / 1000.0;

    double currentTime = timeSeconds;
    

    if (!config->state)
    {
        LOGW("No State provided; using DefaultState.");
        config->state = new DefaultState();
        ownsState = true;
        config->state->begin();
    }

    // =================== Sensor Update ===================
    // Update sensors every loop - each sensor's shouldUpdate() decides when to read
    config->sensorManager.update(timeSeconds);

    // =================== Orientation & Prediction ===================
    // Only update orientation and predict when new IMU data is available AND sensors are healthy
    bool hasAccel = config->sensorManager.hasAccelUpdate();
    bool hasGyro = config->sensorManager.hasGyroUpdate();
    bool hasMag = config->sensorManager.hasMagUpdate();

    // Check sensor health before using data
    bool accelHealthy = !config->sensorManager.getAccelSource() || config->sensorManager.getAccelSource()->isHealthy();
    bool gyroHealthy = !config->sensorManager.getGyroSource() || config->sensorManager.getGyroSource()->isHealthy();
    bool magHealthy = !config->sensorManager.getMagSource() || config->sensorManager.getMagSource()->isHealthy();

    if ((hasAccel || hasGyro) && accelHealthy && gyroHealthy)
    {
        // Update orientation with new IMU data
        Vector<3> gyro = config->sensorManager.getAngularVelocity();
        Vector<3> accel = config->sensorManager.getAcceleration();
        double dt = timeSeconds - lastTime;

        if (dt > 0 && config->state)
        {
            if (hasMag && magHealthy)
            {
                Vector<3> mag = config->sensorManager.getMagneticField();
                config->state->updateOrientation(gyro, accel, mag, dt);
            }
            else
            {
                config->state->updateOrientation(gyro, accel, dt);
            }

            // Run KF prediction immediately after orientation update
            // Prediction uses the updated earth-frame acceleration
            double predictDt = timeSeconds - lastPredictUpdate;
            if (predictDt > 0)
            {
                config->state->predict(predictDt);
                _didPredictState = true;
                lastPredictUpdate = timeSeconds;
            }
        }
        lastTime = timeSeconds;

        // Clear IMU flags after consuming
        if (hasAccel)
            config->sensorManager.clearAccelUpdate();
        if (hasGyro)
            config->sensorManager.clearGyroUpdate();
        if (hasMag)
            config->sensorManager.clearMagUpdate();
    }
    else if ((hasAccel || hasGyro || hasMag) && (!accelHealthy || !gyroHealthy || !magHealthy))
    {
        // Clear flags even if unhealthy to prevent stale data from being used later
        if (hasAccel)
            config->sensorManager.clearAccelUpdate();
        if (hasGyro)
            config->sensorManager.clearGyroUpdate();
        if (hasMag)
            config->sensorManager.clearMagUpdate();
    }
    else if (hasMag)
    {
        // Clear mag update if no accel/gyro update this cycle
        config->sensorManager.clearMagUpdate();
    }

    // =================== Measurement Update ===================
    // Event-driven: Update KF only when new GPS or baro data is available AND healthy
    GPS* gps = config->sensorManager.getGPSSource();
    Barometer* baro = config->sensorManager.getBaroSource();

    bool hasGPS = config->sensorManager.hasGPSUpdate() &&
                  gps &&
                  gps->getHasFix() &&
                  gps->isHealthy();

    bool hasBaro = config->sensorManager.hasBaroUpdate() &&
                   baro &&
                   baro->isHealthy();

    // Update only the sensor that has new data
    if (config->state && hasGPS)
    {
        Vector<3> gpsPos = config->sensorManager.getGPSPosition();
        Vector<3> gpsVel = config->sensorManager.getGPSVelocity();
        config->state->updateGPSMeasurement(gpsPos, gpsVel);
        _didUpdateState = true;
    }
    if (config->state && hasBaro)
    {
        double baroAlt = config->sensorManager.getBarometricAltitude();
        config->state->updateBaroMeasurement(baroAlt);
        _didUpdateState = true;
    }

    // Clear update flags after consuming (even if unhealthy)
    if (config->sensorManager.hasGPSUpdate())
        config->sensorManager.clearGPSUpdate();
    if (config->sensorManager.hasBaroUpdate())
        config->sensorManager.clearBaroUpdate();

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
    // Update GPS fix LED
    updateStatusLEDs();

    return true;
}

void Astra::playInitFeedback(int errorCode)
{
    // Pattern definitions
    const int BEEP_DURATION = 100;   // Short beep duration
    const int BEEP_PAUSE = 150;      // Pause between beeps
    const int PATTERN_PAUSE = 800;   // Pause before repeating pattern
    const int EMERGENCY_BLINK = 100; // Fast blink for multiple failures

    // Success pattern: 2 quick beeps, solid LED
    if (errorCode == 0)
    {
        if (config->statusBuzzer != -1)
        {
            // 2 quick cheerful beeps (synchronous)
            bb.onoff(config->statusBuzzer, BEEP_DURATION, 2, BEEP_PAUSE);
        }
        if (config->statusLED != -1)
        {
            // Solid ON
            bb.on(config->statusLED);
        }
        return;
    }

    // Multiple failures: fast continuous pattern (emergency indicator)
    if (errorCode >= 7)
    {
        if (config->statusBuzzer != -1)
        {
            // Rapid alternating beep pattern (async, continuous)
            bb.aonoff(config->statusBuzzer, EMERGENCY_BLINK, 0, EMERGENCY_BLINK);
        }
        if (config->statusLED != -1)
        {
            // Fast continuous blink (async)
            bb.aonoff(config->statusLED, EMERGENCY_BLINK, 0, EMERGENCY_BLINK);
        }
        return;
    }

    // Single sensor failure: N blinks/beeps pattern
    // errorCode 1-6 indicates which sensor failed
    if (errorCode >= 1 && errorCode <= 6)
    {
        if (config->statusBuzzer != -1)
        {
            // Play N beeps (synchronous, once)
            bb.onoff(config->statusBuzzer, BEEP_DURATION, errorCode, BEEP_PAUSE);
        }
        if (config->statusLED != -1)
        {
            // N blinks, repeating pattern (async, continuous)
            BBPattern pattern(BEEP_DURATION, errorCode, BEEP_PAUSE);
            pattern.r(PATTERN_PAUSE);  // Add pause before repeating
            bb.aonoff(config->statusLED, pattern, true);  // Indefinite repeat
        }
    }
}

void Astra::updateStatusLEDs()
{
    // GPS fix indicator LED
    if (config->gpsFixLED != -1)
    {
        GPS *gps = config->sensorManager.getGPSSource();

        if (!gps)
        {
            // No GPS configured - LED off
            bb.off(config->gpsFixLED);
        }
        else if (gps->getHasFix())
        {
            // Has fix - solid ON
            bb.on(config->gpsFixLED);
        }
        else
        {
            // No fix - slow blink (async, continuous)
            // Only start the pattern if not already blinking
            if (!bb.isOn(config->gpsFixLED))
            {
                bb.aonoff(config->gpsFixLED, 500, 0, 500);  // 500ms on/off, indefinite
            }
        }
    }
}
