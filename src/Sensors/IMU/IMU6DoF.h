#ifndef IMU6DOF_H
#define IMU6DOF_H

#include "../Accel/Accel.h"
#include "../Gyro/Gyro.h"

namespace astra
{
    /**
     * IMU6DoF - 6 Degrees of Freedom IMU (Accelerometer + Gyroscope)
     *
     * This class combines accelerometer and gyroscope functionality in a single sensor.
     * SensorManager can find this sensor by searching for either "Accelerometer" or "Gyroscope" types,
     * making it seamless to work with combo sensors.
     *
     * Note: This uses virtual inheritance to resolve the diamond problem (both Accel and Gyro inherit from Sensor).
     */
    class IMU6DoF : public Accel, public Gyro
    {
    public:
        virtual ~IMU6DoF() {}

        // Override getType to return a composite type
        // SensorManager will need to check both Accel and Gyro types
        virtual const SensorType getType() const override
        {
            // Return Accelerometer type by default
            return Accel::getType();
        }

        virtual const char *getTypeString() const override
        {
            return "IMU6DoF";
        }

        // Expose both Accel and Gyro interfaces
        using Accel::getAccel;
        using Gyro::getAngVel;

    protected:
        IMU6DoF(const char *name = "IMU6DoF")
            : Sensor("IMU6DoF", name), Accel(name), Gyro(name) {}

        // Derived classes implement init() and read() to update both acc and angVel
        virtual bool init() = 0;
        virtual bool read() = 0;
    };
}

#endif // IMU6DOF_H
