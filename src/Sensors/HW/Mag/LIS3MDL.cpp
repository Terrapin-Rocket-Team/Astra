#include "LIS3MDL.h"

namespace astra
{
    LIS3MDL::LIS3MDL(const char *name) : Mag(name) {}

    int LIS3MDL::init()
    {
        if (!magmtr.init())
        {
            return -1;
        }

        magmtr.enableDefault();
        return 0;
    }

    int LIS3MDL::read()
    {
        magmtr.read();

        const float scale = GAUSS_TO_UT / LSB_PER_GAUSS;
        mag = Vector<3>(
            magmtr.m.x * scale,
            magmtr.m.y * scale,
            magmtr.m.z * scale
        );

        return 0;
    }
}
