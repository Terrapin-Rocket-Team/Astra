#include <Arduino.h>
#include "Utils/Astra.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

#include "RecordData/DataReporter/SimpleDataReporter.h"

using namespace astra;

// The functions below are passed to the callback
float readBatteryVoltage()
{
    return analogRead(A0);
}


auto batteryReporter = SimpleDataReporter<float>(
    "Battery",
    "%.2f",
    "voltage",
    // you can inline the begin() function using lambdas
    []() -> bool
    {
        pinMode(A0, INPUT);
        return true;
    },
    //
    readBatteryVoltage,
    0.0f);

PrintLog serialLog(Serial, true);
ILogSink *logs[] = {&serialLog};

AstraConfig config = AstraConfig()
                         .withDataLogs(logs, 1)
                         .withLoggingRate(10);

Astra sys(&config);

void setup()
{
    Serial.begin(115200);
    sys.init();
}

void loop()
{
    // Manually update pyro channel status (simulated)

    sys.update();
}
