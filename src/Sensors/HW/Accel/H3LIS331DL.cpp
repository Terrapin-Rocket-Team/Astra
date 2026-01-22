#include "H3LIS331DL.h"

namespace astra
{
    H3LIS331DL::H3LIS331DL(const char *name, TwoWire &bus, uint8_t address) :  Accel(name), bus(&bus), address(address)
    {
    }
    H3LIS331DL::H3LIS331DL(uint8_t address) : Accel("H3LIS331DL"), address(address)
    {
    }
    bool H3LIS331DL::init()
    {
        if (!lis.begin_I2C(address, bus))
        {
            return false;
        }
        lis.setRange(H3LIS331_RANGE_100_G);
        lis.setDataRate(LIS331_DATARATE_1000_HZ);
        return true;
    }
    bool H3LIS331DL::read()
    {
        sensors_event_t event;
        if (!lis.getEvent(&event))
        {
            return false;
        }
        acc = {event.acceleration.x,
               event.acceleration.y,
               event.acceleration.z};
        return true;
    }
}