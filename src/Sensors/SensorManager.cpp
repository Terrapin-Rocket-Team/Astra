#include "SensorManager.h"
#include "IMU/IMU.h"
#include "Accel/Accel.h"
#include "Gyro/Gyro.h"
#include "Mag/Mag.h"
#include "GPS/GPS.h"
#include "Baro/Barometer.h"
#include "RecordData/Logging/EventLogger.h"

namespace astra
{

    SensorManager::SensorManager()
    {
        for (int i = 0; i < MAX_SENSORS; i++)
        {
            sensors[i] = nullptr;
        }
        numSensors = 0;
    }

    void SensorManager::addSensor(Sensor *sensor)
    {
        if (numSensors >= MAX_SENSORS)
        {
            LOGW("Cannot add sensor - maximum of %d sensors reached", MAX_SENSORS);
            return;
        }
        sensors[numSensors++] = sensor;
    }

    void SensorManager::setSensors(Sensor **sensorArray, int count)
    {
        if (count > MAX_SENSORS)
        {
            LOGW("Sensor count %d exceeds maximum %d, capping to maximum", count, MAX_SENSORS);
            count = MAX_SENSORS;
        }

        for (int i = 0; i < count; i++)
        {
            sensors[i] = sensorArray[i];
        }
        numSensors = count;
    }

    bool SensorManager::initAll()
    {
        int good = 0;
        for (int i = 0; i < numSensors; i++)
        {
            if (sensors[i])
            {
                if (sensors[i]->begin())
                {
                    good++;
                    LOGI("%s [%s] initialized.", sensors[i]->getTypeString(), sensors[i]->getName());
                }
                else
                {
                    LOGE("%s [%s] failed to initialize.", sensors[i]->getTypeString(), sensors[i]->getName());
                }
            }
            else
            {
                LOGE("Sensor index %d in the array was null!", i);
            }
        }
        LOGI("Initialized %d of %d sensors.", good, numSensors);
        return good == numSensors;
    }

    void SensorManager::updateAll()
    {
        for (int i = 0; i < numSensors; i++)
        {
            if (sensors[i] && sensors[i]->isInitialized())
            {
                sensors[i]->update();
            }
        }
    }

    Sensor *SensorManager::getSensor(uint32_t type, int sensorNum) const
    {
        for (int i = 0; i < numSensors; i++)
        {
            if (sensors[i] && type == sensors[i]->getType() && --sensorNum == 0)
                return sensors[i];
        }
        return nullptr;
    }

    // Data extraction methods
    bool SensorManager::getIMUData(double *gyro, double *accel, double *mag)
    {
        // IMU support not implemented yet
        // IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
        // if (imu && imu->isInitialized())
        // {
        //     Vector<3> gyroVec = imu->getAngularVelocity();
        //     Vector<3> accelVec = imu->getAcceleration();
        //     gyro[0] = gyroVec.x();
        //     gyro[1] = gyroVec.y();
        //     gyro[2] = gyroVec.z();
        //     accel[0] = accelVec.x();
        //     accel[1] = accelVec.y();
        //     accel[2] = accelVec.z();
        //     if (mag)
        //     {
        //         Vector<3> magVec = imu->getMagneticField();
        //         mag[0] = magVec.x();
        //         mag[1] = magVec.y();
        //         mag[2] = magVec.z();
        //     }
        //     return true;
        // }
        return false;
    }

    bool SensorManager::getAccelData(double *accel)
    {
        // Try to find standalone Accel sensor first
        Accel *accel_sensor = reinterpret_cast<Accel *>(getSensor("Accelerometer"_i));

        // If not found, check for IMU6DoF or IMU9DoF
        if (!accel_sensor)
        {
            accel_sensor = reinterpret_cast<Accel *>(getSensor("IMU6DoF"_i));
        }
        if (!accel_sensor)
        {
            accel_sensor = reinterpret_cast<Accel *>(getSensor("IMU9DoF"_i));
        }

        if (accel_sensor && accel_sensor->isInitialized())
        {
            Vector<3> accelVec = accel_sensor->getAccel();
            accel[0] = accelVec.x();
            accel[1] = accelVec.y();
            accel[2] = accelVec.z();
            return true;
        }
        return false;
    }

    bool SensorManager::getGyroData(double *gyro)
    {
        // Try to find standalone Gyro sensor first
        Gyro *gyro_sensor = reinterpret_cast<Gyro *>(getSensor("Gyroscope"_i));

        // If not found, check for IMU6DoF or IMU9DoF
        if (!gyro_sensor)
        {
            gyro_sensor = reinterpret_cast<Gyro *>(getSensor("IMU6DoF"_i));
        }
        if (!gyro_sensor)
        {
            gyro_sensor = reinterpret_cast<Gyro *>(getSensor("IMU9DoF"_i));
        }

        if (gyro_sensor && gyro_sensor->isInitialized())
        {
            Vector<3> gyroVec = gyro_sensor->getAngVel();
            gyro[0] = gyroVec.x();
            gyro[1] = gyroVec.y();
            gyro[2] = gyroVec.z();
            return true;
        }
        return false;
    }

    bool SensorManager::getMagData(double *mag)
    {
        // Try to find standalone Mag sensor first
        Mag *mag_sensor = reinterpret_cast<Mag *>(getSensor("Magnetometer"_i));

        // If not found, check for IMU9DoF
        if (!mag_sensor)
        {
            mag_sensor = reinterpret_cast<Mag *>(getSensor("IMU9DoF"_i));
        }

        if (mag_sensor && mag_sensor->isInitialized())
        {
            Vector<3> magVec = mag_sensor->getMag();
            mag[0] = magVec.x();
            mag[1] = magVec.y();
            mag[2] = magVec.z();
            return true;
        }
        return false;
    }

    bool SensorManager::getGPSData(double *lat, double *lon, double *alt)
    {
        GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
        if (gps && gps->isInitialized())
        {
            Vector<3> pos = gps->getPos();
            *lat = pos.x();
            *lon = pos.y();
            *alt = pos.z();
            return true;
        }
        return false;
    }

    bool SensorManager::getGPSVelocity(double *vn, double *ve, double *vd)
    {
        GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
        if (gps && gps->isInitialized())
        {
            Vector<3> vel = gps->getVel();
            *vn = vel.x();
            *ve = vel.y();
            *vd = vel.z();
            return true;
        }
        return false;
    }

    bool SensorManager::getGPSHeading(double *heading)
    {
        GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
        if (gps && gps->isInitialized())
        {
            *heading = gps->getHeading();
            return true;
        }
        return false;
    }

    bool SensorManager::getGPSHasFix(bool *hasFix)
    {
        GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
        if (gps && gps->isInitialized())
        {
            *hasFix = gps->getHasFix();
            return true;
        }
        return false;
    }

    bool SensorManager::getBaroData(double *pressure, double *temp)
    {
        Barometer *baro = reinterpret_cast<Barometer *>(getSensor("Barometer"_i));
        if (baro && baro->isInitialized())
        {
            *pressure = baro->getPressure();
            if (temp)
                *temp = baro->getTemp();
            return true;
        }
        return false;
    }

    bool SensorManager::getBaroAltitude(double *altM)
    {
        Barometer *baro = reinterpret_cast<Barometer *>(getSensor("Barometer"_i));
        if (baro && baro->isInitialized())
        {
            *altM = baro->getASLAltM();
            return true;
        }
        return false;
    }

} // namespace astra
