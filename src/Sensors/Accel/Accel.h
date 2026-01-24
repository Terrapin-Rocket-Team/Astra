#ifndef ACCEL_H
#define ACCEL_H

#include "../Sensor.h"
#include "../SensorManager/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Accel : public RotatableSensor
    {
    public:
        virtual ~Accel();
        virtual Vector<3> getAccel() const;

    protected:
        Accel(const char *name = "Accelerometer");
        Vector<3> acc = Vector<3>(0, 0, 0);
        MountingTransform orient;
    };
}
#endif // ACCEL_H