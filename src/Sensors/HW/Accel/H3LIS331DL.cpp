#include "H3LIS331DL.h"

namespace astra
{
    H3LIS331DL::H3LIS331DL(const char *name, TwoWire *bus, uint8_t address) :  Accel(name), bus(bus), address(address)
    {
    }
    H3LIS331DL::H3LIS331DL(TwoWire *bus, uint8_t address) : H3LIS331DL("H3LIS331DL", bus, address)
    {
    }
    int H3LIS331DL::init()
    {
        if (!lis.begin_I2C(address, bus))
        {
            return -1;
        }
        lis.setRange(H3LIS331_RANGE_100_G);
        lis.setDataRate(LIS331_DATARATE_1000_HZ);
        return 0;
    }
    int H3LIS331DL::read()
    {
        sensors_event_t event;
        if (!lis.getEvent(&event))
        {
            return -1;
        }
        acc = {event.acceleration.x,
               event.acceleration.y,
               event.acceleration.z};
        return 0;
    }
}
