#ifndef Astra_SYSTEM_H
#define Astra_SYSTEM_H

#include "AstraConfig.h"
#include "../Sensors/Sensor.h"
#include "Communication/SerialMessageRouter.h"

namespace astra
{
    // Forward declarations
    class Accel;
    class Gyro;
    class GPS;
    class Barometer;

    class Astra
    {
    public:
        Astra(AstraConfig *config);
        void init();
        bool update(double ms = -1); // returns true if state update occurred
        SerialMessageRouter* getMessageRouter() { return messageRouter; }

        // Sensor access
        Sensor **getSensors() { return sensors; }
        int getNumSensors() const { return numSensors; }

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastStateUpdate = 0, lastLoggingUpdate = 0;
        double lastSensorUpdate = 0, lastPredictUpdate = 0, lastMeasurementUpdate = 0;
        double lastOrientationUpdate = 0;

        // Sensor management - direct storage
        Sensor **sensors = nullptr;
        int numSensors = 0;
        SerialMessageRouter *messageRouter = nullptr;

        bool initAllSensors();
        void updateAllSensors();

        // Sensor lookup helpers
        Sensor *findSensor(uint32_t type, int sensorNum = 1) const;
        Accel *findAccel(int sensorNum = 1) const;
        Gyro *findGyro(int sensorNum = 1) const;
        GPS *findGPS(int sensorNum = 1) const;
        Barometer *findBaro(int sensorNum = 1) const;

        static void handleCommandMessage(const char* message, const char* prefix, Stream* source);
    };
}
#endif