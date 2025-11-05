#include <Arduino.h>
#include "AvionicsKF.h"
#include "Sensors/GPS/MAX_M10S.h"
#include "Sensors/IMU/BMI088andLIS3MDL.h"
#include "Sensors/Baro/DPS368.h"
#include "State/State.h"
#include "Utils/Astra.h"
#include "RetrieveData/SerialHandler.h"
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;
const int BUZZER_PIN = 33;

#ifdef STM32
HardwareSerial Serial1(PA10, PA9);
#endif

MAX_M10S gps;
BMI088andLIS3MDL mmfsimu;
DPS368 baro;
Sensor *sensors[3] = {&gps, &mmfsimu, &baro};
AvionicsKF kfilter;
State avionicsState(sensors, 3, &kfilter);

AstraConfig config = AstraConfig()
                         .withBBPin(LED_BUILTIN)
                         .withBuzzerPin(BUZZER_PIN)
                         .withState(&avionicsState);

Astra sys(&config);

CircBufferLog buf(5000, true);
ILogSink *bufLogs[] = {&buf};
UARTLog uLog(Serial1, 115200, true);
ILogSink *logs[] = {&uLog};
void setup()
{
    Serial.begin(9600);
    Serial.println("Starting up");

    EventLogger::configure(bufLogs, 1);

    sys.init();
    // Serial.println(); // getLogger().isReady());
}
bool handshake = false;
void loop()
{
    if (!handshake)
        if (Serial1.available())
        {
            String s = Serial1.readStringUntil('\n');
            s.trim();
            if (s == "PING")
                Serial1.println("PONG");
            EventLogger::configure(logs, 1);
            buf.transfer(uLog);
            LOGI("Pi Handshake Complete.");
        }
    sys.update();
}