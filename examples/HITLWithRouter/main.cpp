#include <Arduino.h>
#include "Utils/Astra.h"
#include "State/DefaultState.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "RecordData/Logging/EventLogger.h"

using namespace astra;

/**
 * Plug-and-play HITL example.
 *
 * Astra owns the default HITL sensor bundle and the internal HITL/ router.
 * Desktop simulation sends: HITL/timestamp,ax,ay,az,...\n
 * Flight computer responds: TELEM/<csv>\n via DataLogger.
 */

// Default state estimation (uses built-in filters)
DefaultState hitlState;

// Telemetry + event logs go to Serial
PrintLog telemLog(Serial, true);
ILogSink *telemSinks[] = {&telemLog};
PrintLog eventLog(Serial, true);
ILogSink *eventSinks[] = {&eventLog};

// Astra system configuration
AstraConfig config = AstraConfig()
                         .withHITL()
                         .withState(&hitlState)
                         .withDataLogs(telemSinks, 1)
                         .withEventLogs(eventSinks, 1)
                         .withBBPin(LED_BUILTIN);

Astra sys(&config);

//------------------------------------------------------------------------------
// Setup & Loop
//------------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    // Wait for serial connection (helpful for USB serial)
    delay(1000);

    LOGI("===========================================");
    LOGI(" Plug-and-play HITL Example");
    LOGI("===========================================");
    LOGI("");
    LOGI("Astra now owns HITL sensor creation and HITL/ routing.");
    LOGI("Send HITL/timestamp,... packets to Serial and call sys.update() in loop().");

    // Initialize Astra system
    int err = sys.init();
    if (err != 0)
    {
        LOGE("Astra init failed with %d error(s)", err);
    }
    else
    {
        LOGI("Astra system initialized");
    }
    LOGI("Ready! Waiting for HITL data...");
}

void loop()
{
    // Astra handles HITL routing internally and emits TELEM/ for each valid packet.
    sys.update();
}

/*
 * Desktop Simulation Example (Python)
 *
 * import serial
 * import time
 *
 * ser = serial.Serial('/dev/ttyACM0', 115200)
 *
 * # Simulation loop
 * sim_time = 0.0
 * dt = 0.02  # 50 Hz
 *
 * while sim_time < 10.0:
 *     # Generate sensor data
 *     ax, ay, az = 0.0, 0.0, 9.81  # Acceleration (m/s^2)
 *     gx, gy, gz = 0.0, 0.0, 0.0   # Gyro (rad/s)
 *     mx, my, mz = 20.0, 10.0, -45.0  # Mag (uT)
 *     pressure = 1013.25 - sim_time * 10  # Decreasing pressure
 *     temp = 25.0
 *     lat, lon, alt = 38.0, -122.0, sim_time * 50  # GPS
 *     fix, fixqual, heading = 1, 8, 90.0
 *
 *     # Send HITL packet
 *     packet = f"HITL/{sim_time},{ax},{ay},{az},{gx},{gy},{gz},"
 *     packet += f"{mx},{my},{mz},{pressure},{temp},"
 *     packet += f"{lat},{lon},{alt},{fix},{fixqual},{heading}\n"
 *     ser.write(packet.encode())
 *
 *     # Read TELEM response
 *     if ser.in_waiting:
 *         line = ser.readline().decode().strip()
 *         print(f"FC: {line}")
 *
 *     time.sleep(dt)
 *     sim_time += dt
 */
