#ifndef MOCK_IMU9DOF_H
#define MOCK_IMU9DOF_H

#include "IMU9DoF.h"

namespace astra
{
    class MockIMU9DoF : public IMU9DoF
    {
    public:
        MockIMU9DoF(const char *name = "MockIMU9DoF") : IMU9DoF(name) {}

        void setAccel(double x, double y, double z)
        {
            acc = Vector<3>(x, y, z);
        }

        void setAngVel(double x, double y, double z)
        {
            angVel = Vector<3>(x, y, z);
        }

        void setMag(double x, double y, double z)
        {
            mag = Vector<3>(x, y, z);
        }

        void setIMUData(double ax, double ay, double az,
                       double gx, double gy, double gz,
                       double mx, double my, double mz)
        {
            acc = Vector<3>(ax, ay, az);
            angVel = Vector<3>(gx, gy, gz);
            mag = Vector<3>(mx, my, mz);
        }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }
    };
}

#endif // MOCK_IMU9DOF_H
