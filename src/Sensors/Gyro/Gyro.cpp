#include "Gyro.h"

namespace astra
{
    Gyro::Gyro(const char *name) : Sensor("Gyroscope", name)
    {
        addColumn("%0.3f", &angVel.x(), "Gyro X (rad/s)");
        addColumn("%0.3f", &angVel.y(), "Gyro Y (rad/s)");
        addColumn("%0.3f", &angVel.z(), "Gyro Z (rad/s)");
    }

    Gyro::~Gyro()
    {
    }

    Vector<3> Gyro::getAngVel() const
    {
        return angVel;
    }
} // namespace astra