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

        // Returns 0 on success, non-zero error count on failure
        int init();
        bool update(double timeSeconds = -1); // Pass simulation time in seconds, or -1 to use millis()
        SerialMessageRouter* getMessageRouter() { return messageRouter; }

        bool didLog() { return _didLog; }
        bool didUpdateState() { return _didUpdateState; }
        bool didPredictState() { return _didPredictState; }

    private:
        bool ready = false;
        AstraConfig *config = nullptr;
        double lastLoggingUpdate = 0;
        double lastPredictUpdate = 0;  // Still needed for computing prediction dt
        double lastTime = 0;  // For computing dt in orientation updates
        bool _didLog = false, _didUpdateState = false, _didPredictState = false;
        SerialMessageRouter *messageRouter = nullptr;
        bool ownsState = false;
        bool inRouterUpdate = false;  // Re-entrancy guard for router update

        // Status indicator state
        int initErrorCode = 0;  // 0=success, 1-6=specific sensor, 7+=multiple failures

        // Internal methods for status feedback
        void playInitFeedback(int errorCode);
        void updateStatusLEDs();

        static void handleCommandMessage(const char* message, const char* prefix, Stream* source);
    };
}
#endif
