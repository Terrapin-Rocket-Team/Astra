#include "BMI088andLIS3MDL.h"

using namespace astra;

bool BMI088andLIS3MDL::init()
{
    int accelStatus = accel.begin();
    int gyroStatus = gyro.begin();

    int magStatus = mag.init();
    if (magStatus != 0)
    {
        mag.enableDefault();
    }
    else{
        LOGW("LIS3MDL Failed to initialize.");
    }

    initialized = (accelStatus > 0 && gyroStatus > 0);

    //read all data once to setup complementary filter
    accel.readSensor();
    gyro.readSensor();
    mag.read();
    measuredMag = astra::Vector<3>(mag.m.x, mag.m.y, mag.m.z);
    measuredAcc = astra::Vector<3>(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
    measuredGyro = astra::Vector<3>(gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());

    // quaternionBasedComplimentaryFilterSetup();
    // setAccelBestFilteringAtStatic(.5);
    // setMagBestFilteringAtStatic(.5);
    return initialized;
}

bool BMI088andLIS3MDL::read(){
    accel.readSensor();
    gyro.readSensor();
    mag.read();

    measuredMag = astra::Vector<3>(mag.m.x, mag.m.y, mag.m.z);
    measuredAcc = astra::Vector<3>(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
    measuredGyro = astra::Vector<3>(gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());
    
    // quaternionBasedComplimentaryFilter(updateInterval / 1000.0);
    return true;
}


