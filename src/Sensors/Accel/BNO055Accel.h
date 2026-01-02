#ifndef BNO055ACCEL_H
#define BNO055ACCEL_H

#include <Adafruit_BNO055.h>
#include "Accel.h"

namespace astra
{
    class BNO055Accel : public astra::Accel
    {
    public:
        bool read() override;
        bool init() override;
        BNO055Accel(const char *name = "BNO055", uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire);
        BNO055Accel(uint8_t address, TwoWire *theWire = &Wire);

    protected:
        Adafruit_BNO055 bno;
        uint8_t address;
    };
}
#endif // BNO055ACCEL_H
