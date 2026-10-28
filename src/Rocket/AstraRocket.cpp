#include "AstraRocket.h"
#include "Sensors/SensorManager/SensorManager.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/DataLogger.h>
#include <BlinkBuzz/BlinkBuzz.h>

#ifndef ASTRA_ROCKET_VERSION
#define ASTRA_ROCKET_VERSION "UNKNOWN"
#endif

namespace astra_rocket
{
    namespace
    {
        AstraRocketConfig &defaultConfig()
        {
            static AstraRocketConfig config;
            return config;
        }
    }

    AstraRocket::AstraRocket() : AstraRocket(defaultConfig())
    {
    }

    AstraRocket::AstraRocket(AstraRocketConfig &cfg)
        : config(cfg),
          astraSys(nullptr),
          rocketState(nullptr),
          kalmanFilter(nullptr),
          orientationFilter(nullptr),
          arcBridge(nullptr),
          dataSinks(nullptr),
          eventSinks(nullptr),
          numDataSinks(0),
          numEventSinks(0),
          groundLevelAltitude(0)
    {
    }

    AstraRocket::~AstraRocket()
    {
        if (astraSys)
            delete astraSys;
        if (ownsRocketState && rocketState)
            delete rocketState;
        if (ownsKalmanFilter && kalmanFilter)
            delete kalmanFilter;
        if (ownsOrientationFilter && orientationFilter)
            delete orientationFilter;
        if (arcBridge)
            delete arcBridge;
        if (dataSinks)
            delete[] dataSinks;
        if (eventSinks)
            delete[] eventSinks;
    }

    bool AstraRocket::init()
    {
        LOGI("Initializing Astra-Rocket version %s", ASTRA_ROCKET_VERSION);

        // Setup logging first so we can log initialization progress
        setupLogging();

        rocketState = config.getConfiguredRocketState();
        if (rocketState)
        {
            LOGI("Using caller-provided RocketState.");
        }
        else
        {
            if (config.getConfiguredState() != nullptr)
            {
                LOGW("Configured state is not a RocketState. Replacing it with an internal RocketState.");
            }

            kalmanFilter = new DefaultKalmanFilter();
            ownsKalmanFilter = true;
            LOGI("DefaultKalmanFilter created");

            orientationFilter = new MahonyAHRS(0.1, 0.0005);
            ownsOrientationFilter = true;
            LOGI("MahonyAHRS orientation filter created");

            rocketState = new RocketState(kalmanFilter, orientationFilter, &config);
            ownsRocketState = true;
            LOGI("RocketState created with Kalman filter and orientation filter");
            config.withState(rocketState);
        }

        // Configure mode-dependent runtime behavior (HITL vs hardware)
        configureRuntimeMode();

        // Create Astra system
        astraSys = new Astra(&config);
        int initErrors = astraSys->init();
        if (initErrors > 0)
        {
            LOGW("Astra initialized with %d sensor error(s).", initErrors);
        }
        LOGI("Astra system initialized successfully. Reading sensors to establish baseline.");

        if (config.getArcEnabled())
        {
            arcBridge = new ArcCommandBridge(config.getArcAddress(), config.getArcStream(), this);
            arcBridge->begin();
            LOGI("ARC command bridge enabled at address 0x%02X", config.getArcAddress());
        }

        // Establish ground level reference
        if (config.getSimulationEnabled())
        {
            // Astra core now owns HITL/SITL routing, timing, and baseline setup.
            LOGI("Simulation mode enabled. Astra core will process simulator packets.");
        }
        else
        {
            // Hardware mode: Wait for barometer to stabilize
            delay(500);
            astraSys->update();
            if (config.getSensorManager() && config.getSensorManager()->getBaroSource() && config.getSensorManager()->getBaroSource()->isInitialized())
            {
                groundLevelAltitude = config.getSensorManager()->getBaroSource()->getASLAltM();
                rocketState->setGroundLevel(groundLevelAltitude);
                LOGI("Ground level established: %0.2f m MSL", groundLevelAltitude);
            }
            else
            {
                LOGE("Barometer not available!");
                return false;
            }
        }

        // Initialization complete
        LOGI("AstraRocket initialization complete!");
        LOGI("Flight computer ready. Flight stage: %s", flightStageToString(PAD_IDLE));

        return ready = true;
    }

    void AstraRocket::configureRuntimeMode()
    {
        if (config.getHITLEnabled())
        {
            configureHITLMode();
        }
        else
        {
            // Hardware mode uses preflight log rate until liftoff logic changes it.
            config.withLoggingRate(config.getPreflightLogRate());
        }
    }

    void AstraRocket::configureHITLMode()
    {
        // Astra core now owns HITL routing and default sensor installation.
        config.withHITL();
    }

    void AstraRocket::update()
    {

        if (!ready)
        {
            LOGE("AstraRocket not initialized!");
            return;
        }

        // Astra::update() handles both hardware and HITL modes.
        astraSys->update();
        if (arcBridge)
            arcBridge->update();
    }

    // ===== Private Helper Methods =====

    void AstraRocket::setupLogging()
    {
        const bool needsDataDefaults = config.getConfiguredDataLogCount() == 0;
        const bool needsEventDefaults = config.getConfiguredEventLogCount() == 0;

        if (!needsDataDefaults && !needsEventDefaults)
            return;

        if (needsDataDefaults)
            dataSinks = new ILogSink *[ASTRA_ROCKET_MAX_LOG_SINKS]();
        if (needsEventDefaults)
            eventSinks = new ILogSink *[ASTRA_ROCKET_MAX_LOG_SINKS]();

#if defined(NATIVE)
        if (needsEventDefaults)
        {
            ILogSink *serialEventLog = new PrintLog(Serial, true);
            rememberOwnedSink(serialEventLog);
            eventSinks[numEventSinks++] = serialEventLog;
        }
        if (needsDataDefaults)
        {
            ILogSink *serialDataLog = new PrintLog(Serial, true);
            rememberOwnedSink(serialDataLog);
            dataSinks[numDataSinks++] = serialDataLog;
        }
#else
        if (needsEventDefaults)
        {
            ILogSink *usbEventLog = new PrintLog(Serial, true);
            rememberOwnedSink(usbEventLog);
            eventSinks[numEventSinks++] = usbEventLog;
        }

#if defined(ENV_STM)
        if (needsEventDefaults)
        {
            ILogSink *stmEventLog = new FileLogSink("events.log", config.getStorageBackend(), false);
            rememberOwnedSink(stmEventLog);
            eventSinks[numEventSinks++] = stmEventLog;
        }
        if (needsDataDefaults)
        {
            ILogSink *stmDataLog = new FileLogSink("data.csv", config.getStorageBackend(), false);
            rememberOwnedSink(stmDataLog);
            dataSinks[numDataSinks++] = stmDataLog;
        }
#elif defined(ENV_TEENSY)
        if (needsEventDefaults)
        {
            ILogSink *sdEventLog = new FileLogSink("events.log", config.getStorageBackend(), false);
            rememberOwnedSink(sdEventLog);
            eventSinks[numEventSinks++] = sdEventLog;
        }
        if (needsDataDefaults)
        {
            ILogSink *sdDataLog = new FileLogSink("data.csv", config.getStorageBackend(), false);
            rememberOwnedSink(sdDataLog);
            dataSinks[numDataSinks++] = sdDataLog;
        }
#endif
#endif

        if (needsEventDefaults && numEventSinks > 0)
            config.withEventLogs(eventSinks, numEventSinks);
        if (needsDataDefaults && numDataSinks > 0)
            config.withDataLogs(dataSinks, numDataSinks);
    }

    void AstraRocket::rememberOwnedSink(ILogSink *sink)
    {
        if (!sink || numOwnedLogSinks >= ASTRA_ROCKET_MAX_OWNED_LOG_SINKS)
            return;

        ownedLogSinks[numOwnedLogSinks++] = sink;
    }

} // namespace astra_rocket
