#ifndef Astra_SYSTEM_H
#define Astra_SYSTEM_H

#include "AstraConfig.h"
#include "../Sensors/SensorManager.h"
#include "Communication/SerialMessageRouter.h"

namespace astra
{
    class Astra
    {
    public:
        Astra(AstraConfig *config);
        void init();
        bool update(double ms = -1); // returns true if state update occurred
        SerialMessageRouter* getMessageRouter() { return messageRouter; }

        // Sensor access
        SensorManager *getSensorManager() { return &sensorManager; }

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastStateUpdate = 0, lastLoggingUpdate = 0;
        double lastSensorUpdate = 0, lastPredictUpdate = 0, lastMeasurementUpdate = 0;
        double lastOrientationUpdate = 0;

        // Sensor management
        SensorManager sensorManager;
        SerialMessageRouter *messageRouter = nullptr;

        static void handleCommandMessage(const char* message, const char* prefix, Stream* source);
    };
}
#endif