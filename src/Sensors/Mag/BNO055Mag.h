#ifndef BNO055MAG_H
#define BNO055MAG_H

#include <Adafruit_BNO055.h>
#include "Mag.h"

namespace astra
{
    class BNO055Mag : public astra::Mag
    {
    public:
        bool read() override;
        bool init() override;
        BNO055Mag(const char *name = "BNO055", uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire);
        BNO055Mag(uint8_t address, TwoWire *theWire = &Wire);

    protected:
        Adafruit_BNO055 bno;
        uint8_t address;
    };
}
#endif // BNO055MAG_H
