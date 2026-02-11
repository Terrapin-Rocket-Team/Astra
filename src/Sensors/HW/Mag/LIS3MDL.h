#ifndef LIS3MDL_MAG_H
#define LIS3MDL_MAG_H

#include <LIS3MDL.h>
#include <Wire.h>
#include "Sensors/Mag/Mag.h"

namespace astra
{
    class LIS3MDL : public Mag
    {
    public:
        LIS3MDL(const char *name = "LIS3MDL");
        int init() override;
        int read() override;

    private:
        ::LIS3MDL magmtr;
        static constexpr float LSB_PER_GAUSS = 6842.0f; // +/-4 gauss
        static constexpr float GAUSS_TO_UT = 100.0f;
    };
}

#endif // LIS3MDL_MAG_H
