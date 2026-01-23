/**
 * SITL (Software-In-The-Loop) Example
 *
 * This example demonstrates how to run Astra flight software in SITL mode,
 * connecting to an external simulator via TCP sockets.
 *
 * Usage:
 * 1. Start the Python simulator: python sitl_simulator.py --sim parabolic
 * 2. Build and run this example for native: pio run -e native
 * 3. The flight software will connect to the simulator and exchange data
 *
 * The example uses HITL sensors which read from the HITLSensorBuffer,
 * which is populated by the HITLParser from data received via Serial.
 */

#include <Arduino.h>
#include <Sensors/HITL/HITL.h>
#include <Communication/SerialMessageRouter.h>
#include <RecordData/Logging/EventLogger.h>

using namespace astra;

// HITL Sensors
HITLBarometer* baro;
HITLAccel* accel;
HITLGyro* gyro;
HITLMag* mag;
HITLGPS* gps;

SerialMessageRouter router;

void setup()
{
    // Connect to SITL simulator
    // The simulator should be running on localhost:5555
    LOGI("Connecting to SITL simulator...");
    if (Serial.connectSITL("localhost", 5555)) {
        LOGI("Connected to SITL simulator!");
    } else {
        LOGE("Failed to connect to SITL simulator");
        LOGE("Make sure the simulator is running: python sitl_simulator.py");
        return;
    }

    // Create HITL sensors
    baro = new HITLBarometer();
    accel = new HITLAccel();
    gyro = new HITLGyro();
    mag = new HITLMag();
    gps = new HITLGPS();

    // Setup serial message router to parse HITL messages
    router.withInterface(&Serial)
          .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
              double simTime;
              if (HITLParser::parse(msg, simTime)) {
                  // Successfully parsed HITL data
                  // In a real application, you would call astraSys->update(simTime) here
              }
          });

    LOGI("SITL mode initialized");
}

void loop()
{
    // Process incoming HITL messages
    router.update();

    // Read sensor data (from HITLSensorBuffer populated by parser)
    if (baro->dataReady()) {
        float pressure = baro->getPressure();
        float altitude = baro->getAltitude();
        float temp = baro->getTemperature();

        // Send telemetry back to simulator
        Serial.printf("TELEM/baro,pressure=%.2f,alt=%.2f,temp=%.2f\n",
                      pressure, altitude, temp);
    }

    if (accel->dataReady()) {
        Vector<3> a = accel->getAcceleration();
        Serial.printf("TELEM/accel,x=%.3f,y=%.3f,z=%.3f\n",
                      a.x(), a.y(), a.z());
    }

    if (gps->dataReady()) {
        double lat = gps->getLatitude();
        double lon = gps->getLongitude();
        float alt = gps->getAltitude();

        Serial.printf("TELEM/gps,lat=%.6f,lon=%.6f,alt=%.2f\n",
                      lat, lon, alt);
    }

    // Small delay to prevent overwhelming the simulator
    delay(10);
}

// For native builds, provide main()
#ifdef NATIVE
int main()
{
    setup();
    while (Serial.isSITLConnected()) {
        loop();
    }
    LOGI("SITL connection closed");
    return 0;
}
#endif
