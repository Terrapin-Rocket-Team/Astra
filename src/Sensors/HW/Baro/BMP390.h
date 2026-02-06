#ifndef BMP390_H
#define BMP390_H

#include <Adafruit_BMP3XX.h>
#include <Sensors/Baro/Barometer.h>

namespace astra
{
    class BMP390 : public Barometer
    {
    private:
        TwoWire *wire;
        uint8_t addr;
        Adafruit_BMP3XX bmp;

    public:
        BMP390(const char *name = "BMP390", TwoWire *theWire = &Wire, uint8_t address = BMP3XX_DEFAULT_ADDRESS);
        BMP390(TwoWire *theWire, uint8_t address = BMP3XX_DEFAULT_ADDRESS);
        virtual int init() override;
        virtual int read() override;
    };
}

#endif // BMP390_H
