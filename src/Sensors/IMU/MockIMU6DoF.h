#ifndef MOCK_IMU6DOF_H
#define MOCK_IMU6DOF_H

#include "IMU6DoF.h"

namespace astra
{
    class MockIMU6DoF : public IMU6DoF
    {
    public:
        MockIMU6DoF(const char *name = "MockIMU6DoF") : IMU6DoF(name) {}

        void setAccel(double x, double y, double z)
        {
            acc = Vector<3>(x, y, z);
        }

        void setAngVel(double x, double y, double z)
        {
            angVel = Vector<3>(x, y, z);
        }

        void setIMUData(double ax, double ay, double az, double gx, double gy, double gz)
        {
            acc = Vector<3>(ax, ay, az);
            angVel = Vector<3>(gx, gy, gz);
        }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }
    };
}

#endif // MOCK_IMU6DOF_H
