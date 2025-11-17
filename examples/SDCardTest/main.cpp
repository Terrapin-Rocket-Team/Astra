// Minimal SD logging test: writes two columns (TimeMs, Count) once per second.
#include <Arduino.h>
#include "../../src/RecordData/Logging/SDCard/SDCardLog.h"
#include "../../src/RecordData/Logging/DataLogger.h"
#include "../../src/RecordData/Logging/EventLogger.h"
#include "../../src/RecordData/DataReporter/DataReporter.h"

using namespace astra;

// Two values to log
unsigned long timeMs = 0;
int count = 0;

// Define a small reporter with two columns
class MinimalReporter : public DataReporter
{
public:
    MinimalReporter() : DataReporter("Test")
    {
        addColumn("%lu", &timeMs, "TimeMs");
        addColumn("%d", &count, "Count");
    }
};

MinimalReporter reporter;


SDCardLog dataLog("test.csv", false);
SDCardLog eventLog("events.log", true);

// Track test start
unsigned long startMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (!dataLog.begin() || !eventLog.begin()) {
        Serial.println("SD init failed");
        while (true) delay(1000);
    }

    ILogSink* sinks[] = { &dataLog };
    DataReporter* rps[] = { &reporter };
    DataLogger::configure(sinks, 1, rps, 1); // writes CSV header
    ILogSink* eventSinks[] = { &eventLog };
    EventLogger::configure(eventSinks, 1);
    LOGI("Test started"); // Event #1
    Serial.println("[EVENT] Test started");

    Serial.println("Logging to test.csv");
    startMs = millis();
}

void loop()
{
    static unsigned long last = 0;
    if (millis() - last >= 1000) {
        timeMs = millis();
        count++;
        if (DataLogger::available()) {
            DataLogger::instance().appendLine(); // flushes each row
            Serial.printf("[DATA] %lu,%d\n", timeMs, count);
        }
        if (count % 5 == 0) {
            LOGW("Reached multiple of 5=%d", count); // Event #2
            Serial.printf("[EVENT] Reached multiple of 5=%d\n", count);
        }
        last = millis();
    }
    // Stop after 10 seconds
    if (millis() - startMs >= 10000) {
        LOGI("Stopping test");
        Serial.println("Done. Stopping.");
        dataLog.flush();
        eventLog.flush();
        dataLog.end();
        eventLog.end();
        while (true) delay(1000);
    }
    delay(10);
}