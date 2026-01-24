#ifndef MAG_H
#define MAG_H

#include "../Sensor.h"
#include "../SensorManager/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Mag : public RotatableSensor
    {
    public:
        virtual ~Mag();
        virtual Vector<3> getMag() const;

    protected:
        Mag(const char *name = "Magnetometer");
        Vector<3> mag = Vector<3>(0, 0, 0);
    };
}
#endif // MAG_H