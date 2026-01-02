#include "BNO055Gyro.h"

namespace astra
{
    BNO055Gyro::BNO055Gyro(const char *name, uint8_t address, TwoWire *theWire)
        : Gyro(name),
          bno(-1, address, theWire),
          address(address)
    {
    }

    BNO055Gyro::BNO055Gyro(uint8_t address, TwoWire *theWire)
        : Gyro("BNO055"),
          bno(-1, address, theWire),
          address(address)
    {
    }

    bool BNO055Gyro::init()
    {
        if (!bno.begin())
            return false;

        bno.setExtCrystalUse(true);
        return true;
    }

    bool BNO055Gyro::read()
    {
        imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        angVel = Vector<3>(gyroData.x(), gyroData.y(), gyroData.z());

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
