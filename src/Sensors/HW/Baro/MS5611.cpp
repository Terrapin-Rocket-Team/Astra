
#include "MS5611.h"

astra::MS5611::MS5611(const char *name, TwoWire *bus, uint8_t addr) : Barometer(name), ms(addr, bus) {}

astra::MS5611::MS5611(TwoWire *bus, uint8_t addr) : MS5611("MS5611", bus, addr) {}

int astra::MS5611::init()
{
    if (!ms.begin())
    {
        printf("Failed to initialize MS5611 sensor\n");
        return -1;
    }
    ms.setOversampling(OSR_ULTRA_HIGH);

    return 0;
}
int astra::MS5611::read()
{
    ms.read();
    temp = ms.getTemperature();
    pressure = ms.getPressure();
    return 0;
}
