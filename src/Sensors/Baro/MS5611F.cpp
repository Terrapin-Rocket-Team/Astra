
#include "MS5611F.h"

using namespace astra;

astra::MS5611::MS5611(const char *name, uint8_t addr, TwoWire *bus) : Barometer(name), ms(addr, bus) {}

astra::MS5611::MS5611(uint8_t addr, TwoWire *bus) : Barometer("MS5611"), ms(addr, bus) {}

bool astra::MS5611::init()
{
    if (!ms.begin())
    {
        printf("Failed to initialize MS5611 sensor\n");
        return initialized = false;
    }
    ms.setOversampling(OSR_ULTRA_HIGH);

    return initialized = true;
}
bool astra::MS5611::read()
{
    ms.read();
    temp = ms.getTemperature();
    pressure = ms.getPressure();
    return true;
}
