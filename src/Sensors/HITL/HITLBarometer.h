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
        }

        virtual ~HITLBarometer() {}

    protected:
        bool init() override
        {
            // No hardware initialization needed
            return true;
        }

        bool read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

            // Update pressure and temperature
            pressure = buffer.data.pressure;
            temp = buffer.data.temperature;

            // Calculate altitude from pressure
            altitudeASL = calcAltitude(pressure);

            return true;
        }
    };

} // namespace astra

#endif // HITL_BAROMETER_H
