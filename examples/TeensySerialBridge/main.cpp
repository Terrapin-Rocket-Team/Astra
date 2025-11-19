/*
 * Teensy Serial Bridge
 *
 * Routes Serial1 (hardware UART) to USB Serial for monitoring
 * Useful for debugging external devices connected to Serial1
 *
 * Connections:
 * - Serial1 RX (Pin 0) - Connect to external device TX
 * - Serial1 TX (Pin 1) - Connect to external device RX
 * - USB - Connect to computer for monitoring
 *
 * Baud rates are configurable below
 */

#include <Arduino.h>

// Serial1 baud rate (adjust to match your external device)
#define SERIAL1_BAUD 115200

// USB Serial baud rate (doesn't really matter for Teensy, but set for consistency)
#define USB_BAUD 115200

void setup() {
    // Initialize USB Serial
    Serial.begin(USB_BAUD);

    // Initialize Serial1 (hardware UART on pins 0/1)
    Serial1.begin(SERIAL1_BAUD);

    // Wait a moment for Serial to initialize
    delay(1000);

    Serial.println("=================================");
    Serial.println("  Teensy Serial1 <-> USB Bridge");
    Serial.println("=================================");
    Serial.print("Serial1 Baud: ");
    Serial.println(SERIAL1_BAUD);
    Serial.println("Bridge active - forwarding data...");
    Serial.println();
}

void loop() {
    // Forward data from Serial1 to USB Serial
    while (Serial1.available()) {
        char c = Serial1.read();
        Serial.write(c);
    }

    // Forward data from USB Serial to Serial1 (for two-way communication)
    while (Serial.available()) {
        char c = Serial.read();
        Serial1.write(c);
    }
}
