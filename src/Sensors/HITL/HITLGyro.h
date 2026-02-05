#ifndef HITL_GYRO_H
#define HITL_GYRO_H

#include "../Gyro/Gyro.h"
#include "HITLSensorBuffer.h"

namespace astra
{
    /**
     * HITLGyro: Hardware-In-The-Loop gyroscope sensor
     *
     * Simulated gyroscope that reads angular velocity data from
     * the HITLSensorBuffer instead of physical hardware.
     *
     * Usage:
     *   HITLGyro* gyro = new HITLGyro();
     *   gyro->begin();
     *   // ... in update loop ...
     *   gyro->update();  // Reads from HITLSensorBuffer
     *   Vector<3> angularVelocity = gyro->getAngVel();
     */
    class HITLGyro : public Gyro
    {
    public:
        HITLGyro(const char *name = "HITL_Gyroscope")
            : Gyro(name)
        {
            updateInterval = 0;
        }

        virtual ~HITLGyro() {}

    protected:
        int init() override
        {
            // No hardware initialization needed
            return 0;
        }

        int read() override
        {
            // Read from HITL sensor buffer
            HITLSensorBuffer &buffer = HITLSensorBuffer::instance();

            // Update angular velocity vector
            angVel = buffer.data.gyro;

            return 0;
        }
    };

} // namespace astra

#endif // HITL_GYRO_H
