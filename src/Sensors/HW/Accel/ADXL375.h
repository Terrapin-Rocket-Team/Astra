#ifndef ADXL375_H
#define ADXL375_H
#include <Adafruit_ADXL375.h>
#include <Wire.h>
#include <Sensors/Accel/Accel.h>

namespace astra
{
    class ADXL375 : public Accel
    {
    public:
        ADXL375(const char *name = "ADXL375", TwoWire *bus = &Wire, uint8_t address = 0x1D);
        ADXL375(TwoWire *bus, uint8_t address = 0x1D);

        int init() override;
        int read() override;

    private:
        Adafruit_ADXL375 accel;
        uint8_t addr; // Default I2C address for ADXL375
    };
}
#endif
