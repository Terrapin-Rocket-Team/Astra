#include "MMC5603NJ.h"

namespace astra
{
    MMC5603NJ::MMC5603NJ(const char *name, TwoWire &bus, uint8_t addr) : Mag(name), magmtr(addr), i2c_bus(&bus), i2c_addr(addr)
    {
    }

    MMC5603NJ::MMC5603NJ(TwoWire &bus, uint8_t addr) : Mag("MMC5603NJ"), magmtr(addr), i2c_bus(&bus), i2c_addr(addr)
    {
    }
    bool MMC5603NJ::init()
    {
        if(!magmtr.begin(i2c_addr, i2c_bus))
        {
            return false;
        }
        // I think continuous would be better for our use case
        magmtr.setContinuousMode(true);
        return true;
    }
    bool MMC5603NJ::read()
    {
        sensors_event_t event;
        magmtr.getEvent(&event);

        // TODO: figure out what hard iron calibration is
        mag = astra::Vector<3>(
            (float)event.magnetic.x,
            (float)event.magnetic.y,
            (float)event.magnetic.z);
        return true;
    }
}