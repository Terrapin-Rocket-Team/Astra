#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include <Adafruit_H3LIS331.h>
#include <Wire.h>
#include <Sensors/Accel/Accel.h>

namespace astra
{
    class H3LIS331DL : public Accel
    {
    public:
        H3LIS331DL(const char *name = "H3LIS331DL", TwoWire &bus = Wire, uint8_t address = 0x18);
        H3LIS331DL(uint8_t address);
        virtual int init() override;
        virtual int read() override;

    private:
        Adafruit_H3LIS331 lis;
        TwoWire *bus;
        uint8_t address;
    };
}
#endif