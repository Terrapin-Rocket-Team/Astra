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
        int init() override
        {
            // No hardware initialization needed
            return 0;
        }

        void updateHealth(int readErr, double currentTime) override
        {
            (void)currentTime;
            // In HITL, static values can be valid; only comms/read errors are unhealthy.
            healthy = initialized && (readErr == 0);
        }

        int read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

            // Update acceleration vector
            acc = buffer.data.accel;

            return 0;
        }
    };

} // namespace astra

#endif // HITL_ACCEL_H
