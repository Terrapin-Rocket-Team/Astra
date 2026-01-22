#include "SensorManager.h"
#include "../Accel/Accel.h"
#include "../Gyro/Gyro.h"
#include "../Mag/Mag.h"
#include "../Baro/Barometer.h"
#include "../IMU/IMU6DoF.h"
#include "../IMU/IMU9DoF.h"
#include "Utils/Hash.h"
#include "RecordData/Logging/EventLogger.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>

namespace astra
{
    // Static member
    SensorHealthInfo SensorManager::unknownHealth;

    SensorManager::SensorManager()
        : accelMount(MountingOrientation::IDENTITY),
          gyroMount(MountingOrientation::IDENTITY),
          magMount(MountingOrientation::IDENTITY)
    {
    }

    SensorManager::~SensorManager()
    {
        // Sensors are owned externally, don't delete them
    }

    void SensorManager::withSensors(Sensor** sensors, int numSensors)
    {
        this->sensors = sensors;
        this->numSensors = numSensors;
    }

    bool SensorManager::begin()
    {
        if (!sensors || numSensors == 0)
        {
            LOGE("SensorManager: No sensors registered");
            return false;
        }

        LOGI("SensorManager: Selecting sensors...");

        // Select primary sensors
        activeAccel = selectAccel();
        activeGyro = selectGyro();
        activeMag = selectMag();
        activeBaro = selectBaro();

        // Log what we found
        if (activeAccel)
            LOGI("  Accel: %s", activeAccel->getName());
        else
            LOGW("  Accel: NONE");

        if (activeGyro)
            LOGI("  Gyro: %s", activeGyro->getName());
        else
            LOGW("  Gyro: NONE");

        if (activeMag)
            LOGI("  Mag: %s", activeMag->getName());
        else
            LOGI("  Mag: NONE (optional)");

        if (activeBaro)
            LOGI("  Baro: %s", activeBaro->getName());
        else
            LOGW("  Baro: NONE");

        // Initialize health tracking
        uint32_t now = millis();
        if (activeAccel)
        {
            accelHealth.status = SensorHealth::HEALTHY;
            accelHealth.lastUpdateTime = now;
        }
        if (activeGyro)
        {
            gyroHealth.status = SensorHealth::HEALTHY;
            gyroHealth.lastUpdateTime = now;
        }
        if (activeMag)
        {
            magHealth.status = SensorHealth::HEALTHY;
            magHealth.lastUpdateTime = now;
        }
        if (activeBaro)
        {
            baroHealth.status = SensorHealth::HEALTHY;
            baroHealth.lastUpdateTime = now;
        }

        initialized = true;
        LOGI("SensorManager: Initialized");
        return activeAccel != nullptr || activeGyro != nullptr;
    }

    bool SensorManager::update()
    {
        if (!initialized)
            return false;

        uint32_t now = millis();
        bodyData.timestamp = now / 1000.0;

        // Read and transform accelerometer
        if (activeAccel && checkSensorHealth(activeAccel, accelHealth, now))
        {
            Vector<3> rawAccel = activeAccel->getAccel();

            // Validate data (check for NaN, infinite, or unreasonable values)
            if (!std::isnan(rawAccel.x()) && !std::isnan(rawAccel.y()) && !std::isnan(rawAccel.z()) &&
                std::isfinite(rawAccel.x()) && std::isfinite(rawAccel.y()) && std::isfinite(rawAccel.z()) &&
                rawAccel.magnitude() < 2000.0)  // Sanity check: < 200g
            {
                bodyData.accel = accelMount.transform(rawAccel);
                bodyData.hasAccel = true;
                accelHealth.lastUpdateTime = now;
                accelHealth.failureCount = 0;
            }
            else
            {
                accelHealth.failureCount++;
                accelHealth.totalFailures++;
                accelHealth.failureReason = "Invalid data (NaN or out of range)";
                if (accelHealth.failureCount >= maxFailures)
                {
                    handleSensorFailure("accel");
                }
            }
        }
        else
        {
            bodyData.hasAccel = false;
        }

        // Read and transform gyroscope
        if (activeGyro && checkSensorHealth(activeGyro, gyroHealth, now))
        {
            Vector<3> rawGyro = activeGyro->getAngVel();

            // Validate data
            if (!std::isnan(rawGyro.x()) && !std::isnan(rawGyro.y()) && !std::isnan(rawGyro.z()) &&
                std::isfinite(rawGyro.x()) && std::isfinite(rawGyro.y()) && std::isfinite(rawGyro.z()) &&
                rawGyro.magnitude() < 100.0)  // Sanity check: < ~5700 deg/s
            {
                bodyData.gyro = gyroMount.transform(rawGyro);
                bodyData.hasGyro = true;
                gyroHealth.lastUpdateTime = now;
                gyroHealth.failureCount = 0;
            }
            else
            {
                gyroHealth.failureCount++;
                gyroHealth.totalFailures++;
                gyroHealth.failureReason = "Invalid data (NaN or out of range)";
                if (gyroHealth.failureCount >= maxFailures)
                {
                    handleSensorFailure("gyro");
                }
            }
        }
        else
        {
            bodyData.hasGyro = false;
        }

        // Read and transform magnetometer
        if (activeMag && checkSensorHealth(activeMag, magHealth, now))
        {
            Vector<3> rawMag = activeMag->getMag();

            // Validate data
            if (!std::isnan(rawMag.x()) && !std::isnan(rawMag.y()) && !std::isnan(rawMag.z()) &&
                std::isfinite(rawMag.x()) && std::isfinite(rawMag.y()) && std::isfinite(rawMag.z()) &&
                rawMag.magnitude() < 10000.0)  // Sanity check: < 10000 μT
            {
                bodyData.mag = magMount.transform(rawMag);
                bodyData.hasMag = true;
                magHealth.lastUpdateTime = now;
                magHealth.failureCount = 0;
            }
            else
            {
                magHealth.failureCount++;
                magHealth.totalFailures++;
                magHealth.failureReason = "Invalid data (NaN or out of range)";
                if (magHealth.failureCount >= maxFailures)
                {
                    handleSensorFailure("mag");
                }
            }
        }
        else
        {
            bodyData.hasMag = false;
        }

        // Read barometer (no transform needed - scalar data)
        if (activeBaro && checkSensorHealth(activeBaro, baroHealth, now))
        {
            double pressure = activeBaro->getPressure();
            double temp = activeBaro->getTemp();
            double alt = activeBaro->getASLAltM();

            // Validate data
            if (!std::isnan(pressure) && std::isfinite(pressure) &&
                pressure > 100.0 && pressure < 1200.0)  // Reasonable pressure range
            {
                bodyData.pressure = pressure;
                bodyData.temperature = temp;
                bodyData.baroAltASL = alt;
                bodyData.hasBaro = true;
                baroHealth.lastUpdateTime = now;
                baroHealth.failureCount = 0;
            }
            else
            {
                baroHealth.failureCount++;
                baroHealth.totalFailures++;
                baroHealth.failureReason = "Invalid data (NaN or out of range)";
                if (baroHealth.failureCount >= maxFailures)
                {
                    handleSensorFailure("baro");
                }
            }
        }
        else
        {
            bodyData.hasBaro = false;
        }

        return bodyData.hasAccel || bodyData.hasGyro;
    }

    const SensorHealthInfo& SensorManager::getHealthInfo(const char* type) const
    {
        if (strcmp(type, "accel") == 0)
            return accelHealth;
        if (strcmp(type, "gyro") == 0)
            return gyroHealth;
        if (strcmp(type, "mag") == 0)
            return magHealth;
        if (strcmp(type, "baro") == 0)
            return baroHealth;
        return unknownHealth;
    }

    Sensor* SensorManager::findSensor(uint32_t type, int sensorNum) const
    {
        for (int i = 0; i < numSensors; i++)
        {
            if (sensors[i] && type == sensors[i]->getType() && --sensorNum == 0)
                return sensors[i];
        }
        return nullptr;
    }

    Accel* SensorManager::selectAccel()
    {
        // Priority: Standalone Accelerometer > IMU6DoF > IMU9DoF
        // This allows high-g accelerometers to be preferred over IMU accelerometers

        Sensor* sensor = findSensor("Accelerometer"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            // Found standalone accel - also look for backup
            backupAccel = nullptr;
            Sensor* imu6 = findSensor("IMU6DoF"_i, 1);
            if (imu6 && imu6->isInitialized())
                backupAccel = static_cast<IMU6DoF*>(imu6)->getAccelSensor();
            else
            {
                Sensor* imu9 = findSensor("IMU9DoF"_i, 1);
                if (imu9 && imu9->isInitialized())
                    backupAccel = static_cast<IMU9DoF*>(imu9)->getAccelSensor();
            }
            return static_cast<Accel*>(sensor);
        }

        // Try IMU6DoF
        sensor = findSensor("IMU6DoF"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupAccel = nullptr;
            // Check for IMU9DoF as backup
            Sensor* imu9 = findSensor("IMU9DoF"_i, 1);
            if (imu9 && imu9->isInitialized())
                backupAccel = static_cast<IMU9DoF*>(imu9)->getAccelSensor();
            return static_cast<IMU6DoF*>(sensor)->getAccelSensor();
        }

        // Try IMU9DoF
        sensor = findSensor("IMU9DoF"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupAccel = nullptr;
            return static_cast<IMU9DoF*>(sensor)->getAccelSensor();
        }

        return nullptr;
    }

    Gyro* SensorManager::selectGyro()
    {
        // Priority: Standalone Gyroscope > IMU6DoF > IMU9DoF

        Sensor* sensor = findSensor("Gyroscope"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupGyro = nullptr;
            Sensor* imu6 = findSensor("IMU6DoF"_i, 1);
            if (imu6 && imu6->isInitialized())
                backupGyro = static_cast<IMU6DoF*>(imu6)->getGyroSensor();
            else
            {
                Sensor* imu9 = findSensor("IMU9DoF"_i, 1);
                if (imu9 && imu9->isInitialized())
                    backupGyro = static_cast<IMU9DoF*>(imu9)->getGyroSensor();
            }
            return static_cast<Gyro*>(sensor);
        }

        // Try IMU6DoF
        sensor = findSensor("IMU6DoF"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupGyro = nullptr;
            Sensor* imu9 = findSensor("IMU9DoF"_i, 1);
            if (imu9 && imu9->isInitialized())
                backupGyro = static_cast<IMU9DoF*>(imu9)->getGyroSensor();
            return static_cast<IMU6DoF*>(sensor)->getGyroSensor();
        }

        // Try IMU9DoF
        sensor = findSensor("IMU9DoF"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupGyro = nullptr;
            return static_cast<IMU9DoF*>(sensor)->getGyroSensor();
        }

        return nullptr;
    }

    Mag* SensorManager::selectMag()
    {
        // Priority: Standalone Magnetometer > IMU9DoF

        Sensor* sensor = findSensor("Magnetometer"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupMag = nullptr;
            Sensor* imu9 = findSensor("IMU9DoF"_i, 1);
            if (imu9 && imu9->isInitialized())
                backupMag = static_cast<IMU9DoF*>(imu9)->getMagSensor();
            return static_cast<Mag*>(sensor);
        }

        // Try IMU9DoF
        sensor = findSensor("IMU9DoF"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            backupMag = nullptr;
            return static_cast<IMU9DoF*>(sensor)->getMagSensor();
        }

        return nullptr;
    }

    Barometer* SensorManager::selectBaro()
    {
        // Barometers: just take the first one, look for second as backup
        Sensor* sensor = findSensor("Barometer"_i, 1);
        if (sensor && sensor->isInitialized())
        {
            // Look for backup
            backupBaro = static_cast<Barometer*>(findSensor("Barometer"_i, 2));
            return static_cast<Barometer*>(sensor);
        }
        return nullptr;
    }

    bool SensorManager::checkSensorHealth(Sensor* sensor, SensorHealthInfo& health, uint32_t currentTimeMs)
    {
        if (!sensor)
        {
            health.status = SensorHealth::FAILED;
            health.failureReason = "Sensor is null";
            return false;
        }

        if (!sensor->isInitialized())
        {
            health.status = SensorHealth::FAILED;
            health.failureReason = "Sensor not initialized";
            return false;
        }

        // Check for stale data
        if (health.lastUpdateTime > 0 && (currentTimeMs - health.lastUpdateTime) > staleTimeoutMs)
        {
            health.status = SensorHealth::DEGRADED;
            health.failureReason = "Stale data (timeout)";
            health.failureCount++;
            health.totalFailures++;

            if (health.failureCount >= maxFailures)
            {
                health.status = SensorHealth::FAILED;
                return false;
            }
        }

        // If we've had recent failures but not enough to fail, mark as degraded
        if (health.failureCount > 0 && health.failureCount < maxFailures)
        {
            health.status = SensorHealth::DEGRADED;
        }

        return health.status == SensorHealth::HEALTHY || health.status == SensorHealth::DEGRADED;
    }

    void SensorManager::handleSensorFailure(const char* sensorType)
    {
        if (strcmp(sensorType, "accel") == 0)
        {
            LOGW("SensorManager: Accelerometer failed, attempting fallback...");
            if (backupAccel && backupAccel->isInitialized())
            {
                activeAccel = backupAccel;
                backupAccel = nullptr;
                accelHealth.status = SensorHealth::HEALTHY;
                accelHealth.failureCount = 0;
                accelHealth.failureReason = nullptr;
                LOGI("SensorManager: Switched to backup accel: %s", activeAccel->getName());
            }
            else
            {
                accelHealth.status = SensorHealth::FAILED;
                LOGE("SensorManager: No backup accelerometer available");
            }
        }
        else if (strcmp(sensorType, "gyro") == 0)
        {
            LOGW("SensorManager: Gyroscope failed, attempting fallback...");
            if (backupGyro && backupGyro->isInitialized())
            {
                activeGyro = backupGyro;
                backupGyro = nullptr;
                gyroHealth.status = SensorHealth::HEALTHY;
                gyroHealth.failureCount = 0;
                gyroHealth.failureReason = nullptr;
                LOGI("SensorManager: Switched to backup gyro: %s", activeGyro->getName());
            }
            else
            {
                gyroHealth.status = SensorHealth::FAILED;
                LOGE("SensorManager: No backup gyroscope available");
            }
        }
        else if (strcmp(sensorType, "mag") == 0)
        {
            LOGW("SensorManager: Magnetometer failed, attempting fallback...");
            if (backupMag && backupMag->isInitialized())
            {
                activeMag = backupMag;
                backupMag = nullptr;
                magHealth.status = SensorHealth::HEALTHY;
                magHealth.failureCount = 0;
                magHealth.failureReason = nullptr;
                LOGI("SensorManager: Switched to backup mag: %s", activeMag->getName());
            }
            else
            {
                magHealth.status = SensorHealth::FAILED;
                LOGE("SensorManager: No backup magnetometer available");
            }
        }
        else if (strcmp(sensorType, "baro") == 0)
        {
            LOGW("SensorManager: Barometer failed, attempting fallback...");
            if (backupBaro && backupBaro->isInitialized())
            {
                activeBaro = backupBaro;
                backupBaro = nullptr;
                baroHealth.status = SensorHealth::HEALTHY;
                baroHealth.failureCount = 0;
                baroHealth.failureReason = nullptr;
                LOGI("SensorManager: Switched to backup baro: %s", activeBaro->getName());
            }
            else
            {
                baroHealth.status = SensorHealth::FAILED;
                LOGE("SensorManager: No backup barometer available");
            }
        }
    }

} // namespace astra
