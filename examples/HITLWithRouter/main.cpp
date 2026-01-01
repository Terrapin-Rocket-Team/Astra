#include <Arduino.h>
#include "Sensors/HITL/HITL.h"
#include "Communication/SerialMessageRouter.h"
#include "Utils/Astra.h"
#include "State/State.h"
#include "RecordData/Logging/EventLogger.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;

/**
 * HITL Example with SerialMessageRouter
 *
 * This example demonstrates how to use the HITL framework with the
 * SerialMessageRouter for clean, centralized serial message handling.
 *
 * The router automatically:
 * - Listens for HITL/ messages on Serial
 * - Strips the prefix and passes data to handleHITL
 * - Allows easy addition of other message types (CMD/, RAD/, etc.)
 *
 * Desktop simulation sends: HITL/timestamp,ax,ay,az,...\n
 * Flight computer responds: TELEM/stage,altitude,...\n
 */

// Create HITL sensors (drop-in replacements for hardware sensors)
HITLBarometer baro;
HITLAccel accel;
HITLGyro gyro;
HITLMag mag;
HITLGPS gps;

Sensor *sensors[] = {&baro, &accel, &gyro, &mag, &gps};

// Simple state estimation (replace with your own Kalman filter)
State hitlState(sensors, 5, nullptr);

// Astra system configuration
AstraConfig config = AstraConfig()
                         .withState(&hitlState)
                         .withBBPin(LED_BUILTIN);

Astra sys(&config);

// SerialMessageRouter for handling all incoming messages
SerialMessageRouter router;

//------------------------------------------------------------------------------
// Message Handlers
//------------------------------------------------------------------------------

/**
 * Handle incoming HITL simulation data
 * Called automatically by router when "HITL/" message arrives
 */
void handleHITL(const char* message, const char* prefix, Stream* source) {
    double simTime;

    // Parse the CSV data (prefix already stripped by router)
    if (HITLParser::parse(message, simTime)) {
        // Update system with simulation time (NOT millis()!)
        sys.update(simTime);

        // DataLogger automatically outputs TELEM/ data
        LOGI("HITL: Processed timestep %.3f", simTime);
    } else {
        LOGE("HITL: Failed to parse packet");
    }
}

/**
 * Handle ground station commands
 * Called when "CMD/" message arrives
 */
void handleCommand(const char* message, const char* prefix, Stream* source) {
    LOGI("CMD: %s", message);

    if (strcmp(message, "STATUS") == 0) {
        // Send status back on the same interface
        source->printf("STATUS: Sim running, alt=%.2fm\n", hitlState.getAltitude());
    }
    else if (strcmp(message, "PING") == 0) {
        source->println("PONG");
    }
    else if (strcmp(message, "RESET") == 0) {
        LOGW("Reset command received - resetting HITL buffer");
        HITLSensorBuffer::instance().dataReady = false;
    }
    else {
        LOGW("Unknown command: %s", message);
    }
}

/**
 * Default handler for unrecognized messages
 */
void handleUnknown(const char* message, const char* prefix, Stream* source) {
    LOGW("Unknown message type: %s", message);
}

//------------------------------------------------------------------------------
// Setup & Loop
//------------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Wait for serial connection (helpful for USB serial)
    delay(1000);

    LOGI("===========================================");
    LOGI(" HITL Example with SerialMessageRouter");
    LOGI("===========================================");
    LOGI("");
    LOGI("Listening for messages:");
    LOGI("  HITL/timestamp,ax,ay,az,...  - Simulation data");
    LOGI("  CMD/STATUS                   - Get status");
    LOGI("  CMD/PING                     - Ping test");
    LOGI("  CMD/RESET                    - Reset HITL buffer");
    LOGI("");

    // Initialize Astra system
    sys.init();
    LOGI("Astra system initialized");

    // Configure SerialMessageRouter
    router.withInterface(&Serial)
          .withListener("HITL/", handleHITL)
          .withListener("CMD/", handleCommand)
          .withDefaultHandler(handleUnknown);

    LOGI("Router configured with %d interfaces and %d listeners",
         router.getInterfaceCount(), router.getListenerCount());
    LOGI("");
    LOGI("Ready! Waiting for HITL data...");
}

void loop() {
    // Update the router - handles all serial message routing
    router.update();

    // That's it! The router automatically:
    // - Polls Serial for new data
    // - Buffers until newline
    // - Matches prefixes
    // - Calls appropriate handlers
    //
    // No manual Serial.available() checks needed!
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
