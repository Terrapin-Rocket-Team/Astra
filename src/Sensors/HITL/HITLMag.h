#ifndef HITL_MAG_H
#define HITL_MAG_H

#include "../Mag/Mag.h"
#include "HITLSensorBuffer.h"

namespace astra
{
    /**
     * HITLMag: Hardware-In-The-Loop magnetometer sensor
     *
     * Simulated magnetometer that reads magnetic field data from
     * the HITLSensorBuffer instead of physical hardware.
     *
     * Usage:
     *   HITLMag* mag = new HITLMag();
     *   mag->begin();
     *   // ... in update loop ...
     *   mag->update();  // Reads from HITLSensorBuffer
     *   Vector<3> magneticField = mag->getMag();
     */
    class HITLMag : public Mag
    {
    public:
        HITLMag(const char* name = "HITL_Magnetometer")
            : Mag(name)
        {
        }

        virtual ~HITLMag() {}

    protected:
        bool init() override
        {
            // No hardware initialization needed
            return true;
        }

        bool read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer& buffer = HITLSensorBuffer::instance();

            // Update magnetic field vector
            mag = buffer.data.mag;

            return true;
        }
    };

} // namespace astra

#endif // HITL_MAG_H
