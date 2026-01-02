#ifndef BNO055GYRO_H
#define BNO055GYRO_H

#include <Adafruit_BNO055.h>
#include "Gyro.h"

namespace astra
{
    class BNO055Gyro : public astra::Gyro
    {
    public:
        bool read() override;
        bool init() override;
        BNO055Gyro(const char *name = "BNO055", uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire);
        BNO055Gyro(uint8_t address, TwoWire *theWire = &Wire);

    protected:
        Adafruit_BNO055 bno;
        uint8_t address;
    };
}
#endif // BNO055GYRO_H
