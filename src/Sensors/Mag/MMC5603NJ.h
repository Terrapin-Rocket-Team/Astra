#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include <Adafruit_MMC56x3.h>
#include <Wire.h>
#include "Mag.h"

namespace astra
{
    class MMC5603NJ : public astra::Mag
    {
    public:
        bool read() override;
        bool init() override;
        MMC5603NJ(const char *name = "MMC5603NJ", TwoWire &bus = Wire, uint8_t addr = MMC56X3_DEFAULT_ADDRESS);
        MMC5603NJ(TwoWire &bus = Wire, uint8_t addr = MMC56X3_DEFAULT_ADDRESS);
    protected:
        ::Adafruit_MMC5603 magmtr;

        // Store the I2C bus pointer so we can use the bus later (e.g. in init())
        
        ::astra::Vector<3> m_min = {0, 0, 0}; // TODO: find these
        ::astra::Vector<3> m_max = {0, 0, 0}; // TODO: find these
        private:
        TwoWire *i2c_bus = &Wire;
        uint8_t i2c_addr = MMC56X3_DEFAULT_ADDRESS;
    };
    
}

#endif