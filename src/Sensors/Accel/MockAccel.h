#ifndef MOCK_ACCEL_H
#define MOCK_ACCEL_H

#include "Accel.h"

namespace astra
{
    class MockAccel : public Accel
    {
    public:
        MockAccel(const char *name = "MockAccelerometer") : Accel(name) {}

        void setAccel(double x, double y, double z)
        {
            acc = Vector<3>(x, y, z);
        }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }
    };
}

#endif // MOCK_ACCEL_H
