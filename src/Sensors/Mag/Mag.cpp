#include "Mag.h"

namespace astra
{
    Mag::Mag(const char *name) : RotatableSensor(name), lastReadings(HEALTH_BUFFER_SIZE)
    {
        addColumn("%0.3f", &mag.x(), "Mag X (uT)");
        addColumn("%0.3f", &mag.y(), "Mag Y (uT)");
        addColumn("%0.3f", &mag.z(), "Mag Z (uT)");
        setUpdateRate(100);
    }

    Mag::~Mag()
    {
    }

    Vector<3> Mag::getMag() const
    {
        return orient.transform(mag);
    }
} // namespace astra
