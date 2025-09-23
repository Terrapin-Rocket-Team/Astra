#include "BNO055.h"

namespace astra
{

    BNO055::BNO055(const char *name, uint8_t address, TwoWire *theWire)
        : IMU(name),
          bno(-1, address, theWire), //-1 is the default sensor ID
          address(address)
    {
    }

    BNO055::BNO055(uint8_t address, TwoWire *theWire)
        : IMU("BNO055"),
          bno(-1, address, theWire), //-1 is the default sensor ID
          address(address)
    {
    }

    bool BNO055::init()
    {
        if (!bno.begin())
            return initialized = false;

        bno.setExtCrystalUse(true);

        // initialMagField = convertIMUtoAstra(bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));
        return initialized = true;
    }

    bool BNO055::read()    {
        measuredAcc = convertIMUtoAstra(bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        measuredGyro = convertIMUtoAstra(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));
        measuredMag = convertIMUtoAstra(bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));

        orientation = convertIMUtoAstra(bno.getQuat());
        // check the i2c bus to make sure the BNO didn't misbehave
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

    void BNO055::calibrateBno() // not used in flight, used with a separate main file to calibrate the BNO055. BNO does not store these values between power cycles.
    {
        uint8_t system, gyro, accel, mag, i = 0;
        while ((system != 3) || (gyro != 3) || (accel != 3) || (mag != 3))
        {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            i = i + 1;
            if (i == 10)
            {
                i = 0;
            }
            delay(10);
        }
    }
}