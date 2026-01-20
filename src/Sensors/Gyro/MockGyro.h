#ifndef MOCK_GYRO_H
#define MOCK_GYRO_H

#include "Gyro.h"

namespace astra
{
    class MockGyro : public Gyro
    {
    public:
        MockGyro(const char *name = "MockGyroscope") : Gyro(name) {}

        void setAngVel(double x, double y, double z)
        {
            angVel = Vector<3>(x, y, z);
        }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }
    };
}

#endif // MOCK_GYRO_H
