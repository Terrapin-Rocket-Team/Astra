#if !defined(PIO_UNIT_TESTING) && !defined(UNIT_TEST)

#include <Arduino.h>
#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

using namespace astra;

static DefaultState g_state;

static PrintLog g_telemLog(Serial, true);
static ILogSink *g_telemSinks[] = {&g_telemLog};
static PrintLog g_eventLog(Serial, true);
static ILogSink *g_eventSinks[] = {&g_eventLog};

static AstraConfig g_config;
static Astra g_sys(&g_config);

void setup()
{
    Serial.begin(115200);
    if (Serial.connectSITL("localhost", 5555))
    {
        Serial.println("Connected to SITL");
    }
    else
    {
        Serial.println("Failed to connect to SITL");
        return;
    }

    while (!Serial && millis() < 5000)
    {
        delay(10);
    }

    Serial.println("===========================================");
    Serial.println("  Astra SITL Mode");
    Serial.println("  Hardware-In-The-Loop Simulation");
    Serial.println("===========================================");
    Serial.println();

    g_config.withHITL(true)
        .withState(&g_state)
        .withDataLogs(g_telemSinks, 1)
        .withEventLogs(g_eventSinks, 1);

    Serial.println("Initializing HITL mode...");
    int err = g_sys.init();
    if (err != 0)
    {
        Serial.println("ERROR: Astra initialization failed!");
        return;
    }

    Serial.println("HITL mode initialized successfully!");
    Serial.println();
    Serial.println("Ready for simulation. Waiting for HITL packets...");
    Serial.println("Expected format:");
    Serial.println("  HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading");
    Serial.println();
}

void loop()
{
    // In HITL mode, Astra core handles HITL/ packet parsing and update(simTime)
    // through its internal SerialMessageRouter listener.
    g_sys.update();
}

#if defined(NATIVE)
int main()
{
    setup();
    while (Serial.isSITLConnected())
    {
        loop();
    }
    return 0;
}
#endif

#endif // !PIO_UNIT_TESTING && !UNIT_TEST
