#ifndef BMI088_H
#define BMI088_H

#include <BMI088.h>
#include <Sensors/IMU/IMU6DoF.h>

namespace astra
{
    class BMI088 : public IMU6DoF
    {
    public:
        BMI088(const char *name = "BMI088", TwoWire *bus = &Wire, uint8_t accelAddr = 0x18, uint8_t gyroAddr = 0x68);
        BMI088(TwoWire *bus, uint8_t accelAddr = 0x18, uint8_t gyroAddr = 0x68);

        int init() override;
        int read() override;

    protected:
        Bmi088Accel accel;
        Bmi088Gyro gyro;
    };
}
#endif // BMI088_H
