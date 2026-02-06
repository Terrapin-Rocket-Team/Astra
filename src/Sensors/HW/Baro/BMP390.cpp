#include "BMP390.h"

using namespace astra;

BMP390::BMP390(const char *name, TwoWire *theWire, uint8_t address) : Barometer(name), wire(theWire), addr(address), bmp()
{
}

BMP390::BMP390(TwoWire *theWire, uint8_t address) : BMP390("BMP390", theWire, address)
{
}

int BMP390::init()
{
    if (!bmp.begin_I2C(addr, wire))
    {
        return -1;
    }

    // delay(1000);

    // Set up oversampling and filter initialization
    int good = 0;
    good += bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    good += bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    good += bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    good += bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (good != 4) // If any of the above failed
        return -1;

    return 0;
}

int BMP390::read()
{
    pressure = bmp.readPressure() / 100.0; // hPa
    temp = bmp.readTemperature();          // C
    return 0;
}
