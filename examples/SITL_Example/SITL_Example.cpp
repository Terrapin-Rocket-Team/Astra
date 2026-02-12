/**
 * SITL (Software-In-The-Loop) Example
 *
 * This example runs Astra on your PC and connects to an external simulator
 * over TCP using the HITL message format.
 *
 * Usage:
 * 1. Start the Python simulator: python sitl_simulator.py --sim parabolic
 * 2. Build and run this example for native: pio run -e native
 */

#include <Arduino.h>
#include <Sensors/HITL/HITL.h>
#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>
#include <RecordData/Logging/EventLogger.h>
#include <Testing/HITLParser.h>

using namespace astra;

// HITL Sensors
HITLBarometer baro;
HITLAccel accel;
HITLGyro gyro;
HITLMag mag;
HITLGPS gps;

// Default state estimation
DefaultState state;

// Telemetry + event logs go to the SITL socket
PrintLog telemLog(Serial, true);
ILogSink *telemSinks[] = {&telemLog};
PrintLog eventLog(Serial, true);
ILogSink *eventSinks[] = {&eventLog};

AstraConfig config = AstraConfig()
                         .withHITL(true)
                         .withAccel(&accel)
                         .withGyro(&gyro)
                         .withMag(&mag)
                         .withBaro(&baro)
                         .withGPS(&gps)
                         .withState(&state)
                         .withDataLogs(telemSinks, 1)
                         .withEventLogs(eventSinks, 1);

Astra sys(&config);

static void handleHITL(const char *message, const char *prefix, Stream *source)
{
    (void)prefix;
    (void)source;

    double simTime;
    if (HITLParser::parse(message, simTime))
    {
        // Update Astra with simulation time (seconds)
        sys.update(simTime);
    }
}

void setup()
{
    // Connect to SITL simulator
    // The simulator should be running on localhost:5555
    if (Serial.connectSITL("localhost", 5555))
    {
        // Initialize Astra after the socket is ready (logs/TELEM go to the socket)
        int err = sys.init();
        if (err != 0)
        {
            LOGE("Astra init failed with %d error(s)", err);
        }
        else
        {
            LOGI("SITL mode initialized");
        }
    }
    else
    {
        return;
    }

    // Register HITL listener on Astra's internal message router
    SerialMessageRouter *router = sys.getMessageRouter();
    if (router)
    {
        router->withListener("HITL/", handleHITL);
    }
    else
    {
        LOGE("Astra message router not available - SITL cannot continue.");
    }
}

void loop()
{
    // Process incoming messages via Astra's internal router
    // (CMD/ is already registered; HITL/ registered in setup())
    SerialMessageRouter *router = sys.getMessageRouter();
    if (router)
        router->update();

    // DataLogger automatically outputs TELEM/ CSV at the configured logging rate
    delay(1);
}

// For native SITL runs, provide main(); unit tests provide their own main().
#if defined(NATIVE) && !defined(PIO_UNIT_TESTING)
int main()
{
    setup();
    while (Serial.isSITLConnected())
    {
        loop();
    }
    LOGI("SITL connection closed");
    return 0;
}
#endif
