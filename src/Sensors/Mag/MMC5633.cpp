#include "MMC5633.h"

using namespace astra;

MMC5633::MMC5633(const char *name, TwoWire &bus, uint8_t address) : Mag(name), magnet(0), addr(address), bus(bus)
{
}

bool MMC5633::init()
{
    return (this->magnet.begin(addr, &bus));
}

bool MMC5633::read()
{
    sensors_event_t event;
    if (this->magnet.getEvent(&event))
    {
        mag = Vector<3>((double)event.magnetic.x, (double)event.magnetic.y, (double)event.magnetic.z);
        return true;
    }
    else
        return false;
}
