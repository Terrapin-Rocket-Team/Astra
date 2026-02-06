#ifndef ROTATABLE_SENSOR_H
#define ROTATABLE_SENSOR_H

#include "MountingTransform.h"
#include "Sensors/Sensor.h"

namespace astra
{
    /*
        A rotatable sensor is any sensor whose board-mounted orientation matters.

        For example, if you have two accelerometers (high and low g) you are fusing on your baord, 
        but they are mounted differently, then one's X axis could be the other's Y.

        This follows for gyros, IMUs, and magnetometers as well. 
        
        If you know how the sensor is mounted in relation to the "board" then you can use this to align everything.
    */
    class RotatableSensor : public Sensor
    {
    public:
        virtual ~RotatableSensor() {}
        virtual void setMountingOrientation(MountingOrientation orientation)
        {
            orient.setOrientation(orientation);
        }
        virtual MountingOrientation getMountingOrientation() const
        {
            return orient.getOrientation();
        }

    protected:
        RotatableSensor(const char *name) : Sensor(name) {}
        MountingTransform orient;
    };
}

#endif // ROTATABLE_SENSOR_H