#include "Accel.h"

namespace mmfs
{
    Accel::Accel(const char *name) : Sensor("Accelerometer", name)
    {
        addColumn("%0.3f", &acc.x(), "Acc X (m/s^2)");
        addColumn("%0.3f", &acc.y(), "Acc Y (m/s^2)");
        addColumn("%0.3f", &acc.z(), "Acc Z (m/s^2)");
    }

    Accel::~Accel()
    {
    }

    Vector<3> Accel::getAccel() const
    {
        return acc;
    }
} // namespace mmfs