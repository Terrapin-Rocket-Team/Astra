/**
 * Quick SITL connection test
 * Build with: pio run -e native
 * Run with: .pio/build/native/program.exe
 */

#include <Arduino.h>
#include <Sensors/HITL/HITL.h>
#include <Communication/SerialMessageRouter.h>
#include <RecordData/Logging/EventLogger.h>

using namespace astra;

SerialMessageRouter router;
HITLBarometer* baro;

int main() {
    printf("=== Astra SITL Test ===\n");

    // Try to connect to SITL simulator on localhost:5555
    printf("Attempting to connect to SITL simulator on localhost:5555...\n");

    if (Serial.connectSITL("localhost", 5555)) {
        printf("✓ Connected to SITL simulator!\n\n");

        // Create HITL sensors
        baro = new HITLBarometer();

        // Setup message router
        router.withInterface(&Serial)
              .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
                  double simTime;
                  if (HITLParser::parse(msg, simTime)) {
                      printf("← Received HITL data at t=%.3f\n", simTime);
                  }
              });

        printf("Running for 5 seconds...\n\n");

        // Initialize and run for 5 seconds
        baro->begin();
        uint64_t startTime = millis();
        int msgCount = 0;

        while (millis() - startTime < 5000) {
            router.update();

            // Update sensor
            baro->update();

            // Send telemetry every 100ms
            if (msgCount % 10 == 0) {
                float pressure = baro->getPressure();
                float altitude = baro->getASLAltM();
                Serial.printf("→ TELEM/baro,pressure=%.2f,alt=%.2f\n", pressure, altitude);
            }

            msgCount++;
            delay(10);
        }

        printf("\n✓ Test complete! SITL is working!\n");
        Serial.disconnectSITL();

    } else {
        printf("✗ Failed to connect to SITL simulator\n");
        printf("\nMake sure the simulator is running:\n");
        printf("  python sitl_simulator.py --sim static\n");
        return 1;
    }

    return 0;
}
