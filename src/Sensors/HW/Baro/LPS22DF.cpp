#include "LPS22DF.h"

astra::LPS22DF::LPS22DF(const char *name, uint8_t addr) : Barometer(name), addr(addr)
{
}

astra::LPS22DF::LPS22DF(uint8_t addr) : Barometer("LPS22DF"), addr(addr)
{
}

bool astra::LPS22DF::init()
{
    if (!lps22.init(LPS::deviceType::device_22DF, addr))
    {
        return false;
    }
    lps22.enableDefault();
    return true;
}

bool astra::LPS22DF::read()
{
    pressure = lps22.readPressureMillibars();
    temp = lps22.readTemperatureC();
    return true;
}