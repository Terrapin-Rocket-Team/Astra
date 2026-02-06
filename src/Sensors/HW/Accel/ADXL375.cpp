#include "ADXL375.h"

using namespace astra;

ADXL375::ADXL375(const char *name, TwoWire *bus, uint8_t address) : Accel(name), accel(0, bus), addr(address)
{
}

ADXL375::ADXL375(TwoWire *bus, uint8_t address) : ADXL375("ADXL375", bus, address)
{
}

int ADXL375::init()
{
    if (this->accel.begin(addr))
    {
        accel.setTrimOffsets(-3, -3, -1); // Z should be '20' at 1g (49mg per bit)
        return 0;
    }
    return -1;
}

int ADXL375::read()
{
    sensors_event_t event;
    if (this->accel.getEvent(&event))
    {
        acc = Vector<3>((double)event.acceleration.x, (double)event.acceleration.y, (double)event.acceleration.z);
        return 0;
    }
    else
        return -1;
}
