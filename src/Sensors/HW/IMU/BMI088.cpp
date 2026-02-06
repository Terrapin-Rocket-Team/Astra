#include "BMI088.h"

namespace astra
{
    BMI088::BMI088(const char *name, TwoWire *bus, uint8_t accelAddr, uint8_t gyroAddr)
        : IMU6DoF(name), accel(*bus, accelAddr), gyro(*bus, gyroAddr)
    {
    }

    BMI088::BMI088(TwoWire *bus, uint8_t accelAddr, uint8_t gyroAddr)
        : BMI088("BMI088", bus, accelAddr, gyroAddr)
    {
    }

    int BMI088::init()
    {
        int accelErr = accel.begin();
        if (accelErr <= 0)
        {
            LOGE("BMI088 Accel ERROR: %d", accelErr);
            return accelErr;
        }

        int gyroErr = gyro.begin();
        if (gyroErr <= 0)
        {
            LOGE("BMI088 Gyro ERROR: %d", gyroErr);
            return gyroErr;
        }

        return 0;
    }

    int BMI088::read()
    {
        accel.readSensor();
        gyro.readSensor();

        acc = Vector<3>(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
        angVel = Vector<3>(gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());

        return 0;
    }
} // namespace astra
