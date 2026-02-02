#ifndef LPS22DF_H
#define LPS22DF_H

#include <Sensors/Baro/Barometer.h>
#include <Wire.h>
#include <LPS.h>

namespace astra 
{
  class LPS22DF: public Barometer
  {
    public:
      LPS22DF(const char *name = "LPS22DF", uint8_t addr = 0x5D);
      LPS22DF(uint8_t addr);
      virtual bool init() override;
      virtual bool read() override;

    private:
      LPS lps22;
      uint8_t addr;
  };
}

#endif