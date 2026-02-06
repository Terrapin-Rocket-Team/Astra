#include <Arduino.h>
#include "Utils/Astra.h"
#include "State/DefaultState.h"
#include "Sensors/HW/IMU/BMI088.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

using namespace astra;

// Minimal sensor setup: accel + gyro via a 6-DoF IMU.
// This example uses BMI088, but any IMU6DoF works with with6DoFIMU().

BMI088 imu;
DefaultState state;

// Telemetry sink to Serial (TELEM/ CSV)
PrintLog telemLog(Serial, true);
ILogSink *telemSinks[] = {&telemLog};

AstraConfig config = AstraConfig()
                         .with6DoFIMU(&imu)   // extracts accel + gyro
                         .withState(&state)  // optional; Astra can create DefaultState if omitted
                         .withDataLogs(telemSinks, 1);

Astra sys(&config);

void setup()
{
    Serial.begin(115200);
    delay(500);

    // Optional: tune IMU update rate (Hz)
    imu.setUpdateRate(200);

    int err = sys.init();
    if (err != 0)
    {
        Serial.print("Astra init failed with ");
        Serial.print(err);
        Serial.println(" error(s).");
    }
}

void loop()
{
    sys.update(); // uses millis() internally
    delay(5);
}
