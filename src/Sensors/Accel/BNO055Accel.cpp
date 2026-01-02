#include "BNO055Accel.h"

namespace astra
{
    BNO055Accel::BNO055Accel(const char *name, uint8_t address, TwoWire *theWire)
        : Accel(name),
          bno(-1, address, theWire),
          address(address)
    {
    }

    BNO055Accel::BNO055Accel(uint8_t address, TwoWire *theWire)
        : Accel("BNO055"),
          bno(-1, address, theWire),
          address(address)
    {
    }

    bool BNO055Accel::init()
    {
        if (!bno.begin())
            return false;

        bno.setExtCrystalUse(true);
        return true;
    }

    bool BNO055Accel::read()
    {
        imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        acc = Vector<3>(accelData.x(), accelData.y(), accelData.z());

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
