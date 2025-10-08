#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include <SparkFun_LIS331.h>
#include "IMU.h"

namespace astra {
    class H3LIS331DL : public IMU {
    public:
        H3LIS331DL(const char *name = "H3LIS331DL", uint8_t address = 0x18);
        H3LIS331DL(uint8_t address);
        virtual ~H3LIS331DL(){};
        virtual void calibrate();
        virtual void configureInterrupt();
        virtual bool init() override;
        virtual bool read() override;

    private:
        LIS331 H3LI;
        uint8_t address;

    };
}
#endif