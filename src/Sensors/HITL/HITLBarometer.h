#ifndef HITL_BAROMETER_H
#define HITL_BAROMETER_H

#include "../Baro/Barometer.h"
#include "HITLSensorBuffer.h"

namespace astra
{
    /**
     * HITLBarometer: Hardware-In-The-Loop barometer sensor
     *
     * Simulated barometer that reads pressure and temperature data from
     * the HITLSensorBuffer instead of physical hardware. Used for desktop
     * simulation testing with real flight computer code.
     *
     * Usage:
     *   HITLBarometer* baro = new HITLBarometer();
     *   baro->begin();
     *   // ... in update loop ...
     *   baro->update();  // Reads from HITLSensorBuffer
     *   double altitude = baro->getASLAltM();
     */
    class HITLBarometer : public Barometer
    {
    public:
        HITLBarometer(const char *name = "HITL_Barometer")
            : Barometer(name)
        {
            updateInterval = 0;
        }

        virtual ~HITLBarometer() {}

    protected:
        int init() override
        {
            // No hardware initialization needed
            return 0;
        }

        void updateHealth(int readErr, double currentTime) override
        {
            (void)currentTime;
            // In HITL, static pressure can be valid; only comms/read errors are unhealthy.
            healthy = initialized && (readErr == 0);
        }

        int read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

            // Update pressure and temperature
            pressure = buffer.data.pressure;
            temp = buffer.data.temperature;

            // Calculate altitude from pressure
            altitudeASL = calcAltitude(pressure);

            return 0;
        }
    };

} // namespace astra

#endif // HITL_BAROMETER_H
