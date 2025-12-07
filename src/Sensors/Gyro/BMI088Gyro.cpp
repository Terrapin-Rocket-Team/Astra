#include "BMI088Gyro.h"

namespace astra
{
    BMI088Gyro::BMI088Gyro(const char *name, TwoWire &bus, uint8_t addr) : Gyro(name), gyro(bus, addr)
    {
        addColumn("%0.3f", &angVel.x(), "Gyro X (rad/s)");
        addColumn("%0.3f", &angVel.y(), "Gyro Y (rad/s)");
        addColumn("%0.3f", &angVel.z(), "Gyro Z (rad/s)");
    }

    BMI088Gyro::BMI088Gyro(TwoWire &bus, uint8_t addr) : Gyro("BMI088"), gyro(bus, addr)
    {
    }

    bool BMI088Gyro::init()
    {
        int err = gyro.begin();
        if (err <= 0)
            LOGE("BMI ERROR: %d", err);
        return err > 0;
    }

    bool BMI088Gyro::read()
    {
        gyro.readSensor();
        angVel = Vector<3>(gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());
        return true;
    }
} // namespace astra