#ifndef MS5611F_H
#define MS5611F_H

#include <MS5611.h>
#include "Sensors/Baro/Barometer.h"

namespace astra
{
    class MS5611 : public Barometer
    {
    private:
        ::MS5611 ms; // uses the same name, so namespace specification is needed

    public:
        MS5611(const char *name = "MS5611", TwoWire *bus = &Wire, uint8_t addr = 0x76);
        MS5611(TwoWire *bus, uint8_t addr = 0x76);
        virtual int init() override;
        virtual int read() override;
    };

}
#endif
