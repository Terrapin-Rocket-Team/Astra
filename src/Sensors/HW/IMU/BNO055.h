#ifndef BNO055_HW_H
#define BNO055_HW_H

#include <Adafruit_BNO055.h>
#include <Sensors/IMU/IMU9DoF.h>

namespace astra
{
    class BNO055 : public IMU9DoF
    {
    public:
        BNO055(const char *name = "BNO055", uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire);
        BNO055(uint8_t address, TwoWire *theWire = &Wire);

        int init() override;
        int read() override;

    protected:
        Adafruit_BNO055 bno;
        uint8_t address;
        TwoWire *wire;
    };
}
#endif // BNO055_HW_H
