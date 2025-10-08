#include "H3LIS331DL.h"

namespace astra
{
    H3LIS331DL::H3LIS331DL(const char *name, uint8_t address) : IMU(name), address(address)
    {
    }
    H3LIS331DL::H3LIS331DL(uint8_t address) : IMU("H3LIS331DL"), address(address)
    {
    }
    bool H3LIS331DL::init()
    {
        H3LI.setI2CAddr(address); // must be called before begin()
        H3LI.begin(LIS331::USE_I2C);
        int16_t x, y, z;
        H3LI.readAxes(x, y, z);
        if (x == 0.0 && y == 0.0 && z == 0.0)
        { // since it is very unlikely for all 3 axes to be zero, we can return false
            return false;
        }
        // no need to set interrupts
        return true;
    }
    bool H3LIS331DL::read()
    {

        int16_t x = 0, y = 0, z = 0, old_x = 0, old_y = 0, old_z = 0;
        H3LI.readAxes(x, y, z);
        if(x == old_x && y == old_y && z == old_z){
            return false;
        } else {
            measuredAcc = {x, y, z};
            return true;
        }
    }
}