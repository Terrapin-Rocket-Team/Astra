#include <Arduino.h>
#include "Communication/SerialMessageRouter.h"
#include "RecordData/Logging/EventLogger.h"

using namespace astra;

// Create router instance
SerialMessageRouter router;

// HITL message handler
void handleHITL(const char* message, const char* prefix, Stream* source) {
    LOGI("HITL data received: %s", message);

    // Parse HITL packet format:
    // timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading

    // Example: Pass to HITL parser
    // double simTime;
    // if (HITLParser::parseAndInject(message, simTime)) {
    //     astraSys->update(simTime);
    // }
}

// Radio telemetry handler
void handleRadio(const char* message, const char* prefix, Stream* source) {
    LOGI("Radio message: %s", message);

    // Example: Parse radio-specific data
    // Could be RSSI info, remote commands, etc.
}

// Ground station command handler
void handleCommand(const char* message, const char* prefix, Stream* source) {
    LOGI("Command received: %s", message);

    if (strcmp(message, "STATUS") == 0) {
        // Send status back on the same interface
        source->println("STATUS: OK, Altitude: 123.45m, Stage: BOOST");
    }
    else if (strcmp(message, "REBOOT") == 0) {
        LOGW("Reboot command received!");
        // Handle reboot...
    }
    else if (strcmp(message, "PING") == 0) {
        source->println("PONG");
    }
    else {
        LOGW("Unknown command: %s", message);
    }
}

// Catch-all for unrecognized messages
void handleUnknown(const char* message, const char* prefix, Stream* source) {
    LOGW("Unrecognized message: %s", message);
}

void setup() {
    // Initialize serial interfaces
    Serial.begin(115200);   // USB

#ifdef ENV_TEENSY
    Serial8.begin(9600);    // Radio on Serial8
#endif

    // Configure event logger
    LOGI("SerialMessageRouter Example");
    LOGI("Listening for messages with prefixes:");
    LOGI("  - HITL/   : Hardware-in-the-loop data");
    LOGI("  - RAD/    : Radio telemetry");
    LOGI("  - CMD/    : Ground station commands");

    // Configure the router with builder pattern
    router
        .withInterface(&Serial)        // Monitor USB Serial
#ifdef ENV_TEENSY
        .withInterface(&Serial8)       // Monitor Radio Serial
#endif
        .withListener("HITL/", handleHITL)
        .withListener("RAD/", handleRadio)
        .withListener("CMD/", handleCommand)
        .withDefaultHandler(handleUnknown);  // Catch unmatched messages

    LOGI("Router configured with %d interfaces and %d listeners",
         router.getInterfaceCount(), router.getListenerCount());

    LOGI("Ready! Send messages like:");
    LOGI("  CMD/STATUS");
    LOGI("  CMD/PING");
    LOGI("  HITL/1.234,0,0,9.81,...");
    LOGI("  RAD/RSSI:-45dBm");
}

void loop() {
    // Update the router - this polls all registered serial interfaces
    // and dispatches messages to callbacks based on prefix matching
    router.update();

    // The rest of your flight computer logic goes here...
    // The router is non-blocking and plays nicely with other code

    delay(10);  // Small delay for demonstration
}

/*
 * Example Usage:
 *
 * Send these commands over Serial Monitor:
 *
 *   CMD/STATUS        -> Calls handleCommand, sends back status
 *   CMD/PING          -> Calls handleCommand, sends "PONG"
 *   CMD/REBOOT        -> Calls handleCommand, logs reboot
 *   HITL/1.23,0,0,9.8 -> Calls handleHITL, processes HITL data
 *   RAD/RSSI:-45dBm   -> Calls handleRadio, logs radio data
 *   UNKNOWN/test      -> Calls handleUnknown, logs warning
 *
 * Hot-swapping example:
 *   You can send "CMD/STATUS" from either Serial (USB) or Serial8 (Radio)
 *   and the response will go back on the same interface!
 */
