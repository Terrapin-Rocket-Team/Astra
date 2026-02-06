#include <Arduino.h>
#include "RecordData/Logging/DataLogger.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "Sensors/VoltageSensor/VoltageSensor.h"

using namespace astra;

// Data-logger-only example:
// - No Astra
// - No State estimation
// - Manual sensor updates + DataLogger CSV output

const int VOLTAGE_PIN = A0;

VoltageSensor battery(VOLTAGE_PIN, "Battery");

// PrintLog writes TELEM/ CSV to Serial. We call Serial.begin in setup.
PrintLog telemLog(Serial, true);
ILogSink *telemSinks[] = {&telemLog};

double lastLogTime = 0.0;
const double logInterval = 0.1; // 10 Hz

void setup()
{
    Serial.begin(115200);
    delay(500);

    int err = battery.begin();
    if (err != 0)
    {
        Serial.print("Battery sensor init failed: ");
        Serial.println(err);
    }

    // Configure DataLogger AFTER creating reporters so the header includes them.
    DataLogger::configure(telemSinks, 1);

    // Optional: set update rate for the sensor
    battery.setUpdateRate(10);
}

void loop()
{
    double now = millis() / 1000.0;

    if (battery.shouldUpdate(now))
    {
        battery.update();
    }

    if (now - lastLogTime >= logInterval)
    {
        DataLogger::instance().appendLine();
        lastLogTime = now;
    }
}
