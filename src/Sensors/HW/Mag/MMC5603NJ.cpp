#include "MMC5603NJ.h"

namespace astra
{
    MMC5603NJ::MMC5603NJ(const char *name, TwoWire *bus, uint8_t addr) :  Mag(name), magmtr(addr), i2c_bus(bus), i2c_addr(addr)
    {
    }

    MMC5603NJ::MMC5603NJ(TwoWire *bus, uint8_t addr) : MMC5603NJ("MMC5603NJ", bus, addr)
    {
    }
    int MMC5603NJ::init()
    {
        if (!magmtr.begin(i2c_addr, i2c_bus))
        {
            return -1;
        }
        magmtr.setDataRate(255);
        // I think continuous would be better for our use case
        magmtr.setContinuousMode(true);
        // clear any offsets in the magnetometer
        magmtr.magnetSetReset();
        return 0;
    }
    int MMC5603NJ::read()
    {
        sensors_event_t event;
        magmtr.getEvent(&event);

        mag = {event.magnetic.x,
               event.magnetic.y,
               event.magnetic.z};
        return 0;
    }
}
