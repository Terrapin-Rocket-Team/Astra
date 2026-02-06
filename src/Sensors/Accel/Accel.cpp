#include "Accel.h"

namespace astra
{
    Accel::Accel(const char *name) : RotatableSensor(name), lastReadings(HEALTH_BUFFER_SIZE)
    {
        addColumn("%0.3f", &acc.x(), "Acc X (m/s^2)");
        addColumn("%0.3f", &acc.y(), "Acc Y (m/s^2)");
        addColumn("%0.3f", &acc.z(), "Acc Z (m/s^2)");
        setUpdateRate(100);
    }

    Accel::~Accel()
    {
    }

    Vector<3> Accel::getAccel() const
    {
        return orient.transform(acc);
    }

} // namespace astra
