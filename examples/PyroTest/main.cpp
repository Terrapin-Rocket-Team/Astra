/*
 * Pyro Channel Test for STM32
 * Tests arming circuit and pyro channel continuity/firing
 *
 * Pin Configuration:
 * - PA2: Arm Status Input (reads armed state from arming circuit)
 * - PA3: Arm Control Output (sets armed state)
 * - PB0: Continuity Status Input (reads continuity: HIGH = good, LOW = no continuity)
 * - PB1: Fire Control Output (triggers pyro channel)
 *
 * Serial: PB12 (TX), PB13 (RX) at 115200 baud
 *
 * Commands via Serial:
 * - 'a' : Arm the system (set PA3 HIGH)
 * - 'd' : Disarm the system (set PA3 LOW)
 * - 'c' : Check continuity status (read PB0)
 * - 'f' : FIRE pyro channel (set PB1 HIGH for 1 second) - REQUIRES ARM
 * - 's' : Show status
 * - 'h' : Show help
 */

#include <Arduino.h>

// Pin definitions
#define ARM_STATUS_IN   PA2   // Input: Read armed state from arming circuit
#define ARM_CONTROL_OUT PA3   // Output: Set armed state
#define CONTINUITY_IN   PB0   // Input: Read continuity status (HIGH = good, LOW = open)
#define FIRE_OUT        PB1   // Output: Fire control

// Serial on PB12 (TX) and PB13 (RX)
HardwareSerial Serial5(PB13, PB12);

// State variables
bool isArmed = false;
unsigned long lastStatusPrint = 0;

void printDivider() {
    Serial5.println(F("========================================"));
}

void printHelp() {
    printDivider();
    Serial5.println(F("PYRO CHANNEL TEST COMMANDS"));
    printDivider();
    Serial5.println(F("a - ARM system (set PA3 HIGH)"));
    Serial5.println(F("d - DISARM system (set PA3 LOW)"));
    Serial5.println(F("c - Check CONTINUITY (read PB0)"));
    Serial5.println(F("f - FIRE pyro channel (REQUIRES ARM!)"));
    Serial5.println(F("s - Show STATUS"));
    Serial5.println(F("h - Show this HELP"));
    printDivider();
}

void printStatus() {
    Serial5.println(F("\n--- System Status ---"));

    // Read arm status input
    bool armStatusRead = digitalRead(ARM_STATUS_IN);
    Serial5.print(F("ARM_STATUS_IN   (PA2): "));
    Serial5.println(armStatusRead ? F("HIGH (ARMED)") : F("LOW (DISARMED)"));

    // Show arm control output state
    Serial5.print(F("ARM_CONTROL_OUT (PA3): "));
    Serial5.println(isArmed ? F("HIGH (ARMED)") : F("LOW (DISARMED)"));

    // Read continuity input
    bool continuity = digitalRead(CONTINUITY_IN);
    Serial5.print(F("CONTINUITY_IN   (PB0): "));
    if (continuity) {
        Serial5.println(F("HIGH (GOOD CONTINUITY)"));
    } else {
        Serial5.println(F("LOW (NO CONTINUITY / OPEN CIRCUIT)"));
    }

    // Read fire output
    Serial5.print(F("FIRE_OUT        (PB1): "));
    Serial5.println(digitalRead(FIRE_OUT) ? F("HIGH (FIRING!)") : F("LOW"));

    Serial5.println();

    // Warn if arm status doesn't match control
    if (armStatusRead != isArmed) {
        Serial5.println(F("WARNING: ARM_STATUS_IN doesn't match ARM_CONTROL_OUT!"));
        Serial5.println(F("         Check arming circuit connections."));
    }
}

void armSystem() {
    Serial5.println(F("\n>>> ARMING SYSTEM <<<"));
    digitalWrite(ARM_CONTROL_OUT, HIGH);
    isArmed = true;
    delay(50); // Give circuit time to respond

    bool armStatusRead = digitalRead(ARM_STATUS_IN);
    if (armStatusRead) {
        Serial5.println(F("SUCCESS: System armed (PA2 reads HIGH)"));
    } else {
        Serial5.println(F("WARNING: ARM_CONTROL set HIGH but ARM_STATUS reads LOW!"));
        Serial5.println(F("         Check arming circuit."));
    }
}

void disarmSystem() {
    Serial5.println(F("\n>>> DISARMING SYSTEM <<<"));
    digitalWrite(ARM_CONTROL_OUT, LOW);
    digitalWrite(FIRE_OUT, LOW); // Ensure fire is off
    isArmed = false;
    delay(50);

    bool armStatusRead = digitalRead(ARM_STATUS_IN);
    if (!armStatusRead) {
        Serial5.println(F("SUCCESS: System disarmed (PA2 reads LOW)"));
    } else {
        Serial5.println(F("WARNING: ARM_CONTROL set LOW but ARM_STATUS reads HIGH!"));
    }
}

void checkContinuity() {
    Serial5.println(F("\n>>> CONTINUITY CHECK <<<"));

    bool continuity = digitalRead(CONTINUITY_IN);

    Serial5.print(F("CONTINUITY_IN (PB0): "));
    if (continuity) {
        Serial5.println(F("HIGH - GOOD CONTINUITY"));
        Serial5.println(F("E-match is connected and has proper resistance."));
    } else {
        Serial5.println(F("LOW - NO CONTINUITY"));
        Serial5.println(F("E-match is missing, disconnected, or open circuit!"));
        Serial5.println(F("WARNING: Do NOT fire with no continuity!"));
    }
}

void firePyro() {
    // Safety check: must be armed
    bool armStatusRead = digitalRead(ARM_STATUS_IN);
    if (!isArmed || !armStatusRead) {
        Serial5.println(F("\n!!! FIRE ABORTED !!!"));
        Serial5.println(F("System is NOT armed. Arm system first with 'a' command."));
        return;
    }

    // Check continuity before firing
    bool continuity = digitalRead(CONTINUITY_IN);
    if (!continuity) {
        Serial5.println(F("\n!!! FIRE ABORTED !!!"));
        Serial5.println(F("NO CONTINUITY detected on PB0!"));
        Serial5.println(F("Check e-match connection before firing."));
        return;
    }

    Serial5.println(F("\n!!! FIRING PYRO CHANNEL !!!"));
    Serial5.println(F("Continuity: GOOD"));
    Serial5.println(F("Setting FIRE_OUT (PB1) HIGH for 1 second..."));

    // Fire for 1 second
    digitalWrite(FIRE_OUT, HIGH);

    for (int i = 1; i <= 10; i++) {
        Serial5.print(F("."));
        delay(100);
    }
    Serial5.println();

    digitalWrite(FIRE_OUT, LOW);
    Serial5.println(F("FIRE complete. FIRE_OUT set LOW."));

    // Recommend disarming after fire
    Serial5.println(F("RECOMMENDATION: Disarm system with 'd' command."));
}

void setup() {
    // Initialize serial
    Serial5.begin(115200);
    delay(1000);

    // Configure pins
    pinMode(ARM_STATUS_IN, INPUT);       // Read arm status
    pinMode(ARM_CONTROL_OUT, OUTPUT);    // Control arming
    pinMode(CONTINUITY_IN, INPUT);       // Read continuity status
    pinMode(FIRE_OUT, OUTPUT);           // Fire control

    // Initialize outputs to safe state
    digitalWrite(ARM_CONTROL_OUT, LOW);  // Disarmed
    digitalWrite(FIRE_OUT, LOW);         // No fire
    isArmed = false;

    // Print startup message
    Serial5.println(F("\n\n"));
    printDivider();
    Serial5.println(F("  STM32 PYRO CHANNEL TEST"));
    Serial5.println(F("  Arm Circuit & Continuity/Fire Test"));
    printDivider();
    Serial5.println();

    Serial5.println(F("Pin Configuration:"));
    Serial5.println(F("  PA2 - ARM_STATUS_IN   (Input: Read armed state)"));
    Serial5.println(F("  PA3 - ARM_CONTROL_OUT (Output: Set armed state)"));
    Serial5.println(F("  PB0 - CONTINUITY_IN   (Input: Read continuity status)"));
    Serial5.println(F("  PB1 - FIRE_OUT        (Output: Fire control)"));
    Serial5.println();

    Serial5.println(F("System initialized in DISARMED state."));
    Serial5.println(F("All outputs set LOW."));
    Serial5.println();

    printHelp();
    printStatus();
}

void loop() {
    // Check for serial commands
    if (Serial5.available()) {
        char cmd = Serial5.read();

        switch (cmd) {
            case 'a':
            case 'A':
                armSystem();
                printStatus();
                break;

            case 'd':
            case 'D':
                disarmSystem();
                printStatus();
                break;

            case 'c':
            case 'C':
                checkContinuity();
                break;

            case 'f':
            case 'F':
                firePyro();
                printStatus();
                break;

            case 's':
            case 'S':
                printStatus();
                break;

            case 'h':
            case 'H':
                printHelp();
                break;

            default:
                // Ignore whitespace and newlines
                if (cmd != '\n' && cmd != '\r' && cmd != ' ') {
                    Serial5.print(F("Unknown command: "));
                    Serial5.println(cmd);
                    Serial5.println(F("Type 'h' for help."));
                }
                break;
        }
    }

    // Print periodic status (every 5 seconds)
    if (millis() - lastStatusPrint > 5000) {
        lastStatusPrint = millis();

        // Show armed and continuity status briefly
        Serial5.print(F("[Status] "));
        bool armStatusRead = digitalRead(ARM_STATUS_IN);
        bool continuity = digitalRead(CONTINUITY_IN);

        Serial5.print(isArmed ? F("ARMED") : F("DISARMED"));
        Serial5.print(F(" | ARM_IN: "));
        Serial5.print(armStatusRead ? F("HIGH") : F("LOW"));
        Serial5.print(F(" | CONT: "));
        Serial5.println(continuity ? F("GOOD") : F("OPEN"));
    }
}
