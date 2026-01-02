#include "BNO055Mag.h"

namespace astra
{
    BNO055Mag::BNO055Mag(const char *name, uint8_t address, TwoWire *theWire)
        : Mag(name),
          bno(-1, address, theWire),
          address(address)
    {
    }

    BNO055Mag::BNO055Mag(uint8_t address, TwoWire *theWire)
        : Mag("BNO055"),
          bno(-1, address, theWire),
          address(address)
    {
    }

    bool BNO055Mag::init()
    {
        if (!bno.begin())
            return false;

        bno.setExtCrystalUse(true);
        return true;
    }

    bool BNO055Mag::read()
    {
        imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        mag = Vector<3>(magData.x(), magData.y(), magData.z());

        // Check the i2c bus to make sure the BNO didn't misbehave
        Wire.beginTransmission(address);
        byte b = Wire.endTransmission();
        if (b != 0x00)
        {
            Wire.end();
            Wire.begin();
            LOGE("BNO055 I2C Error");
        }
        return true;
    }
} // namespace astra
