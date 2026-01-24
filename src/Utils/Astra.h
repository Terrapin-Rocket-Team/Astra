#ifndef Astra_SYSTEM_H
#define Astra_SYSTEM_H

#include "AstraConfig.h"
#include "../Sensors/Sensor.h"
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

        bool didLog() { return didLog; }
        bool didUpdateSensors() { return didUpdateSensors; }
        bool didUpdateState() { return didUpdateState; }
        bool didPredictState() { return didPredictState; }

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastLoggingUpdate = 0;
        double lastSensorUpdate = 0, lastPredictUpdate = 0, lastMeasurementUpdate = 0;
        bool didLog = false, didUpdateSensors = false, didUpdateState = false, didPredictState = false;
        SerialMessageRouter *messageRouter = nullptr;

        static void handleCommandMessage(const char* message, const char* prefix, Stream* source);
    };
}
#endif