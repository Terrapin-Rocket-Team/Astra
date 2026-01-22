#ifndef Astra_SYSTEM_H
#define Astra_SYSTEM_H

#include "AstraConfig.h"
#include "../Sensors/Sensor.h"
#include "../Sensors/SensorManager/ISensorManager.h"
#include "Communication/SerialMessageRouter.h"

namespace astra
{
    // Forward declarations
    class GPS;
    class Barometer;
    class SensorManager;

    class Astra
    {
    public:
        Astra(AstraConfig *config);
        ~Astra();

        void init();
        bool update(double ms = -1); // returns true if state update occurred
        SerialMessageRouter* getMessageRouter() { return messageRouter; }

        // Sensor access
        Sensor **getSensors() { return sensors; }
        int getNumSensors() const { return numSensors; }

        // SensorManager access
        ISensorManager *getSensorManager() { return sensorManager; }

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastStateUpdate = 0, lastLoggingUpdate = 0;
        double lastSensorUpdate = 0, lastPredictUpdate = 0, lastMeasurementUpdate = 0;
        double lastOrientationUpdate = 0;

        // Sensor management
        Sensor **sensors = nullptr;
        int numSensors = 0;

        // SensorManager handles sensor selection, body-frame transformation, and health
        ISensorManager *sensorManager = nullptr;
        bool ownsSensorManager = false;

        SerialMessageRouter *messageRouter = nullptr;

        bool initAllSensors();
        void updateAllSensors();

        // GPS/Baro lookup (still needed for measurement updates)
        Sensor *findSensor(uint32_t type, int sensorNum = 1) const;
        GPS *findGPS(int sensorNum = 1) const;
        Barometer *findBaro(int sensorNum = 1) const;

        static void handleCommandMessage(const char* message, const char* prefix, Stream* source);
    };
}
#endif