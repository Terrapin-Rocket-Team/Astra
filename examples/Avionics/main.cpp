#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#include "AvionicsKF.h"
#include "Sensors/HW/GPS/SAM_M10Q.h"
#include "Sensors/HW/Baro/DPS368.h"
#include "Sensors/HW/IMU/BMI088.h"
#include "Sensors/HW/Mag/MMC5603NJ.h"
#include "Sensors/SensorManager/SensorManager.h"
#include "Filters/Mahony.h"
#include "State/State.h"
#include "Utils/Astra.h"
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;
const int BUZZER_PIN = 33;
const int STATUS_LED = 25;       // Status LED for init diagnostics
const int GPS_FIX_LED = 26;      // GPS fix indicator LED

#ifdef STM32
HardwareSerial Serial1(PA10, PA9);
#endif

SAM_M10Q gps;
DPS368 baro;
BMI088 imu("BMI088");
MMC5603NJ mag("MMC5603NJ");
AvionicsKF kfilter;
MahonyAHRS orientationFilter;
State avionicsState(&kfilter, &orientationFilter);

AstraConfig config = AstraConfig()
                         .withBBPin(LED_BUILTIN)
                         .withBuzzerPin(BUZZER_PIN)
                         .withStatusLED(STATUS_LED)          // Init diagnostics LED
                         .withStatusBuzzer(BUZZER_PIN)       // Init beep codes
                         .withGPSFixLED(GPS_FIX_LED)        // GPS fix indicator
                         .with6DoFIMU(&imu)
                         .withGPS(&gps)
                         .withBaro(&baro)
                         .withMag(&mag)
                         .withState(&avionicsState);

Astra sys(&config);

CircBufferLog buf(5000, true);
ILogSink *bufLogs[] = {&buf};
#ifdef ENV_STM
UARTLog uLog(Serial, 115200, true);
#else
UARTLog uLog(Serial1, 115200, true);
#endif
ILogSink *logs[] = {&uLog};
void setup()
{
    Serial.begin(9600);
    Serial.println("Starting up");

    EventLogger::configure(bufLogs, 1);

    int err = sys.init();
    if (err != 0) {
        Serial.print("Astra init failed with ");
        Serial.print(err);
        Serial.println(" error(s). Check logs.");
    }
    // Serial.println(); // getLogger().isReady());
}
bool handshake = false;
void loop()
{
    if (!handshake)
#ifdef ENV_STM
        if (Serial.available())
#else
        if (Serial1.available())
#endif
        {
#ifdef ENV_STM
            String s = Serial.readStringUntil('\n');
#else
            String s = Serial1.readStringUntil('\n');
#endif

            s.trim();
            if (s == "PING")
            {
#ifdef ENV_STM
                Serial.println("PONG");
#else
                Serial1.println("PONG");
#endif
            }
            EventLogger::configure(logs, 1);
            buf.transfer(uLog);
            LOGI("Pi Handshake Complete.");
        }
    sys.update();
}

#endif  // PIO_UNIT_TESTING