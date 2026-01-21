#ifndef IMU9DOF_H
#define IMU9DOF_H

#include "../Accel/Accel.h"
#include "../Gyro/Gyro.h"
#include "../Mag/Mag.h"

namespace astra
{
    /**
     * IMU9DoF - 9 Degrees of Freedom IMU (Accelerometer + Gyroscope + Magnetometer)
     *
     * This class combines accelerometer, gyroscope, and magnetometer functionality in a single sensor.
     * Astra and State can find this sensor by searching for "IMU9DoF" type, or fall back to it when
     * searching for "Accelerometer", "Gyroscope", or "Magnetometer" types.
     *
     * Note: This uses virtual inheritance to resolve the diamond problem (all three inherit from Sensor).
     */
    class IMU9DoF : public Accel, public Gyro, public Mag
    {
    public:
        virtual ~IMU9DoF() {}

        // Override getType to return a composite type
        virtual const SensorType getType() const override
        {
            // Return Accelerometer type by default
            return Accel::getType();
        }

        virtual const char *getTypeString() const override
        {
            return "IMU9DoF";
        }

        // Expose all three interfaces
        using Accel::getAccel;
        using Gyro::getAngVel;
        using Mag::getMag;

    protected:
        IMU9DoF(const char *name = "IMU9DoF")
            : Sensor("IMU9DoF", name), Accel(name), Gyro(name), Mag(name) {}

        // Derived classes implement init() and read() to update acc, angVel, and mag
        virtual bool init() = 0;
        virtual bool read() = 0;
    };
}

#endif // IMU9DOF_H
