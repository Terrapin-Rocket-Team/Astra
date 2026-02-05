#ifndef MAX_M10S_H
#define MAX_M10S_H

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include <Sensors/GPS/GPS.h>

namespace astra
{
    class MAX_M10S : public GPS
    {
    private:
        SFE_UBLOX_GNSS m10s;
        TwoWire *wire;
        uint8_t address;

    public:
        MAX_M10S(const char *name = "MAX-M10S", TwoWire &wire = Wire, uint8_t address = 0x42); // 0x42 is the default address
        virtual ~MAX_M10S() {}
        int init() override;
        int read() override;
    };
}
#endif // MAX_M10S_H
