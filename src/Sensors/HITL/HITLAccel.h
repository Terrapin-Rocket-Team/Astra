#ifndef HITL_ACCEL_H
#define HITL_ACCEL_H

#include "../Accel/Accel.h"
#include "HITLSensorBuffer.h"

namespace astra
{
    /**
     * HITLAccel: Hardware-In-The-Loop accelerometer sensor
     *
     * Simulated accelerometer that reads acceleration data from
     * the HITLSensorBuffer instead of physical hardware.
     *
     * Usage:
     *   HITLAccel* accel = new HITLAccel();
     *   accel->begin();
     *   // ... in update loop ...
     *   accel->update();  // Reads from HITLSensorBuffer
     *   Vector<3> acceleration = accel->getAccel();
     */
    class HITLAccel : public Accel
    {
    public:
        HITLAccel(const char *name = "HITL_Accelerometer")
            : Accel(name)
        {
            updateInterval = 0;
        }

        virtual ~HITLAccel() {}

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

            // Update acceleration vector
            acc = buffer.data.accel;

            return true;
        }
    };

} // namespace astra

#endif // HITL_ACCEL_H
