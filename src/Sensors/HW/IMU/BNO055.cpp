#include "BNO055.h"

namespace astra
{
    BNO055::BNO055(const char *name, uint8_t address, TwoWire *theWire)
        : IMU9DoF(name),
          bno(-1, address, theWire),
          address(address),
          wire(theWire)
    {
    }

    BNO055::BNO055(uint8_t address, TwoWire *theWire)
        : IMU9DoF("BNO055"),
          bno(-1, address, theWire),
          address(address),
          wire(theWire)
    {
    }

    int BNO055::init()
    {
        if (!bno.begin())
            return -1;

        bno.setExtCrystalUse(true);
        return 0;
    }

    int BNO055::read()
    {
        imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        acc = Vector<3>(accelData.x(), accelData.y(), accelData.z());
        angVel = Vector<3>(gyroData.x(), gyroData.y(), gyroData.z());
        mag = Vector<3>(magData.x(), magData.y(), magData.z());

        // Check the i2c bus to make sure the BNO didn't misbehave
        wire->beginTransmission(address);
        byte b = wire->endTransmission();
        if (b != 0x00)
        {
            wire->end();
            wire->begin();
            LOGE("BNO055 I2C Error");
        }

        return 0;
    }
} // namespace astra
