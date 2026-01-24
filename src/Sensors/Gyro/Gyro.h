#ifndef GYRO_H
#define GYRO_H

#include "../Sensor.h"
#include "../SensorManager/RotatableSensor.h"
#include "../../Math/Vector.h"

namespace astra
{
    class Gyro : public RotatableSensor
    {
    public:
        virtual ~Gyro();
        virtual Vector<3> getAngVel() const;

    protected:
        Gyro(const char *name = "Gyroscope");
        Vector<3> angVel = Vector<3>(0, 0, 0);
    };
}
#endif // GYRO_H