#ifndef ASTRA_ROCKET_H
#define ASTRA_ROCKET_H

#include <Utils/Astra.h>
#include <RecordData/Logging/DataLogger.h>
#include <RecordData/Logging/EventLogger.h>
#include <Sensors/Baro/Barometer.h>
#include <Sensors/Accel/Accel.h>
#include <Sensors/GPS/GPS.h>
#include <Sensors/Gyro/Gyro.h>
#include <Sensors/Mag/Mag.h>
#include <Sensors/IMU/IMU6DoF.h>
#include <Sensors/IMU/IMU9DoF.h>

#include "AstraRocketConfig.h"
#include "ArcCommandBridge.h"
#include "RocketState.h"
#include <Filters/DefaultKalmanFilter.h>
#include "FlightStage.h"

using namespace astra;

namespace astra_rocket
{

    /**
     * AstraRocket: High-level rocketry flight computer wrapper
     *
     * Provides a simple interface for rocket flight computers built on Astra.
     * Features:
     * - Configured sensor initialization and update orchestration
     * - Default serial and storage logging sinks
     * - Flight stage detection and tracking
     * - Recovery event detection (based on descent rate changes)
     *
     * Basic usage:
     *   AstraRocket rocket;
     *   rocket.init();
     *   while(1) rocket.update();
     */
    class AstraRocket
    {
    public:
        /**
         * Default constructor
         * Uses default configuration; mission sensors must still be configured
         */
        AstraRocket();

        /**
         * Constructor with custom configuration
         * @param config AstraRocketConfig object with custom settings
         */
        AstraRocket(AstraRocketConfig &config);

        /**
         * Destructor
         */
        ~AstraRocket();

        /**
         * Initialize the flight computer
         * - Initializes configured sensors
         * - Sets up logging
         * - Performs pre-flight checks
         *
         * @return true if initialization successful, false otherwise
         */
        bool init();

        /**
         * Update the flight computer state
         * Should be called in main loop()
         * - Updates all sensors
         * - Updates state estimation
         * - Handles flight stage transitions
         * - Manages logging
         */
        void update();

        /**
         * Get the rocket state object
         */
        RocketState *getRocketState() { return rocketState; }

        /**
         * Get the underlying Astra system object
         */
        Astra *getAstraSystem() { return astraSys; }
        SerialMessageRouter *getMessageRouter() { return astraSys ? astraSys->getMessageRouter() : nullptr; }
        AstraRocketConfig &getConfig() { return config; }
        bool isReady() const { return ready; }
        ArcCommandBridge *getArcCommandBridge() { return arcBridge; }

    private:
        bool ready = false;
        // Configuration
        AstraRocketConfig &config;

        // Core Astra system
        Astra *astraSys;
        RocketState *rocketState;
        DefaultKalmanFilter *kalmanFilter;
        MahonyAHRS *orientationFilter;
        ArcCommandBridge *arcBridge;

        // Logging
        ILogSink **dataSinks;
        ILogSink **eventSinks;
        int numDataSinks;
        int numEventSinks;
        bool ownsRocketState = false;
        bool ownsKalmanFilter = false;
        bool ownsOrientationFilter = false;
        static constexpr int ASTRA_ROCKET_MAX_OWNED_LOG_SINKS = 10;
        ILogSink *ownedLogSinks[ASTRA_ROCKET_MAX_OWNED_LOG_SINKS] = {};
        uint8_t numOwnedLogSinks = 0;

        // Flight tracking
        unsigned long liftoffTime;
        double groundLevelAltitude;

        // Helper methods
        void setupLogging();
        void configureRuntimeMode();
        void configureHITLMode();
        void rememberOwnedSink(ILogSink *sink);

        // Constants
        static constexpr int ASTRA_ROCKET_MAX_SENSORS = 10;
        static constexpr int ASTRA_ROCKET_MAX_LOG_SINKS = 5;
    };

} // namespace astra_rocket

#endif // ASTRA_ROCKET_H
