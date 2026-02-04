#ifndef MLX90393_H
#define MLX90393_H

#include <Adafruit_MLX90393.h>
#include "Sensors/Mag/Mag.h"

namespace astra
{
    class MLX90393 : public Mag
    {
    public:
        MLX90393(const char *name = "MLX90393", TwoWire &bus = Wire, uint8_t addr = MLX90393_DEFAULT_ADDR);
        MLX90393(TwoWire &bus = Wire, uint8_t addr = MLX90393_DEFAULT_ADDR);
        bool read() override;
        bool init() override;

    private:
        TwoWire *bus = &Wire;
        uint8_t addr = MLX90393_DEFAULT_ADDR;
        Adafruit_MLX90393 magmtr;
    };
}

#endif