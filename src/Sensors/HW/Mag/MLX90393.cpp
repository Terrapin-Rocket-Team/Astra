#include "MLX90393.h"

namespace astra
{

    MLX90393::MLX90393(const char *name, TwoWire &bus, uint8_t addr) : Mag(name), bus(&bus), addr(addr) {}
    MLX90393::MLX90393(TwoWire &bus, uint8_t addr) : Mag("MLX90393"), bus(&bus), addr(addr) {}
    int MLX90393::init()
    {
        if (!magmtr.begin_I2C(addr, bus))
        {
            return -1;
        }
        magmtr.enableAutoRange(true);
        magmtr.setOversampling(MLX90393_OSR_3);
        magmtr.setGain(MLX90393_GAIN_1X);
        magmtr.setResolution(MLX90393_X, MLX90393_RES_19);
        magmtr.setResolution(MLX90393_Y, MLX90393_RES_19);
        magmtr.setResolution(MLX90393_Z, MLX90393_RES_19);
        // this was in the example so im js going with it
        magmtr.setFilter(MLX90393_FILTER_5);
        return true;
    }
    int MLX90393::read()
    {
        sensors_event_t event;
        bool status = magmtr.getEvent(&event);
        mag = {
            event.magnetic.x,
            event.magnetic.y,
            event.magnetic.z,
        };
        return status ? 0 : -1;
    }
}