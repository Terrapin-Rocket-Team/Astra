#ifndef MMC5633_H
#define MMC5633_H
#include <Adafruit_MMC56x3.h>
#include <Wire.h>
#include "Mag.h"

using namespace astra;
class MMC5633 : public Mag
{
public:
    MMC5633(const char *name = "MMC5633", TwoWire &bus = Wire, uint8_t address = 0x30);

    bool init() override;
    bool read() override;

private:
    Adafruit_MMC5603 magnet;
    uint8_t addr;
    TwoWire &bus;
};

#endif
