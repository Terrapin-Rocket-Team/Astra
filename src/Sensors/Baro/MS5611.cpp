
#include "MS5611.h"

astra::MS5611::MS5611(const char *name, uint8_t addr, TwoWire *bus) : Barometer(name), ms(addr, bus) {}

astra::MS5611::MS5611(uint8_t addr, TwoWire *bus) : Barometer("MS5611"), ms(addr, bus) {}

bool astra::MS5611::init()
{
    if (!ms.begin())
    {
        printf("Failed to initialize MS5611 sensor\n");
        return false;
    }
    ms.setOversampling(OSR_ULTRA_HIGH);

    return true;
}
bool astra::MS5611::read()
{
    ms.read();
    temp = ms.getTemperature();
    pressure = ms.getPressure();
    return true;
}
