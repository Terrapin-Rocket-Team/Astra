#include "SensorManager.h"
#include "Accel/Accel.h"
#include "Gyro/Gyro.h"
#include "Mag/Mag.h"
#include "GPS/GPS.h"
#include "Baro/Barometer.h"
#include "VoltageSensor/VoltageSensor.h"
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

    // Typed sensor getters
    // Note: For IMU types, use getAccel()/getGyro()/getMag() which handle the fallback automatically

    Accel *SensorManager::getAccel(int sensorNum) const
    {
        // Try standalone accelerometer first
        Sensor *sensor = getSensor("Accelerometer"_i, sensorNum);
        if (sensor)
            return static_cast<Accel *>(sensor);

        // Fall back to IMU6DoF (which inherits from Accel)
        sensor = getSensor("IMU6DoF"_i, sensorNum);
        if (sensor)
            return reinterpret_cast<Accel *>(sensor);

        // Fall back to IMU9DoF (which inherits from Accel)
        sensor = getSensor("IMU9DoF"_i, sensorNum);
        if (sensor)
            return reinterpret_cast<Accel *>(sensor);

        return nullptr;
    }

    Gyro *SensorManager::getGyro(int sensorNum) const
    {
        // Try standalone gyroscope first
        Sensor *sensor = getSensor("Gyroscope"_i, sensorNum);
        if (sensor)
            return static_cast<Gyro *>(sensor);

        // Fall back to IMU6DoF (which inherits from Gyro)
        sensor = getSensor("IMU6DoF"_i, sensorNum);
        if (sensor)
            return reinterpret_cast<Gyro *>(sensor);

        // Fall back to IMU9DoF (which inherits from Gyro)
        sensor = getSensor("IMU9DoF"_i, sensorNum);
        if (sensor)
            return reinterpret_cast<Gyro *>(sensor);

        return nullptr;
    }

    Mag *SensorManager::getMag(int sensorNum) const
    {
        // Try standalone magnetometer first
        Sensor *sensor = getSensor("Magnetometer"_i, sensorNum);
        if (sensor)
            return static_cast<Mag *>(sensor);

        // Fall back to IMU9DoF (which has magnetometer)
        sensor = getSensor("IMU9DoF"_i, sensorNum);
        if (sensor)
            return reinterpret_cast<Mag *>(sensor);

        return nullptr;
    }

    GPS *SensorManager::getGPS(int sensorNum) const
    {
        return static_cast<GPS *>(getSensor("GPS"_i, sensorNum));
    }

    Barometer *SensorManager::getBaro(int sensorNum) const
    {
        return static_cast<Barometer *>(getSensor("Barometer"_i, sensorNum));
    }

    VoltageSensor *SensorManager::getVoltageSensor(int sensorNum) const
    {
        return static_cast<VoltageSensor *>(getSensor("Voltage Sensor"_i, sensorNum));
    }

} // namespace astra
