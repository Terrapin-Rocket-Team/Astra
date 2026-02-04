#include "AstraConfig.h"
#include "../State/State.h"
#include "../BlinkBuzz/BlinkBuzz.h"
#include "../Sensors/Accel/Accel.h"
#include "../Sensors/Gyro/Gyro.h"
#include "../Sensors/Mag/Mag.h"
#include "../Sensors/Baro/Barometer.h"
#include "../Sensors/GPS/GPS.h"
#include "../Sensors/Sensor.h"
#include "../Sensors/IMU/IMU6DoF.h"
#include "../Sensors/IMU/IMU9DoF.h"
#include "../Sensors/SensorManager/SensorManager.h"
#include "../RecordData/Logging/EventLogger.h"

namespace astra
{

    AstraConfig::AstraConfig()
    {
        for (int i = 0; i < 50; i++)
        {
            pins[i] = -1;
        }
    }
    AstraConfig &AstraConfig::withState(State *state)
    {
        this->state = state;
        return *this;
    }

    AstraConfig &AstraConfig::withLoggingRate(double loggingRate)
    {
        if (this->loggingRate == loggingRate)
            return *this;
        LOGI("Logging rate modified from %d to %d hz.", loggingRate, loggingRate);
        this->loggingRate = loggingRate;
        this->loggingInterval = 1000.0 / loggingRate;
        return *this;
    }
    AstraConfig &AstraConfig::withLoggingInterval(unsigned int loggingIntervalMs)
    {
        double loggingIntervalSec = loggingIntervalMs / 1000.0;
        if (this->loggingInterval == loggingIntervalSec)
            return *this;
        LOGI("Logging interval modified to %d ms", loggingIntervalMs);
        this->loggingInterval = loggingIntervalSec;
        this->loggingRate = 1000.0 / loggingIntervalMs;
        return *this;
    }
    AstraConfig &AstraConfig::withBuzzerPin(unsigned int buzzerPin)
    {
        if ((unsigned int)BUZZER == buzzerPin)
            return *this;
        BUZZER = buzzerPin;
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int)pins[i] == buzzerPin)
            {
                LOGW("Attempted to set buzzer pin %d to BlinkBuzz, but it is already in use.", buzzerPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = buzzerPin;
                return *this;
            }
        }
        LOGW("Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", buzzerPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBPin(unsigned int bbPin)
    {
        for (int i = 0; i < 50; i++)
        {
            if ((unsigned int)pins[i] == bbPin)
            {
                LOGW("Attempted to add pin %d to BlinkBuzz, but it is already in use.", bbPin);
                return *this;
            }
            if (pins[i] == -1)
            {
                pins[i] = bbPin;
                return *this;
            }
        }
        LOGW("Attempted to add pin %d to BlinkBuzz, but the maximum number of pins (50) has been reached. That's too many pins. Why.", bbPin);
        return *this;
    }
    AstraConfig &AstraConfig::withBBAsync(bool bbAsync, unsigned int queueSize)
    {
        if (this->bbAsync == bbAsync)
            return *this;
        LOGI("BlinkBuzz async modified from %s to %s.", this->bbAsync ? "true" : "false", bbAsync ? "true" : "false");
        this->bbAsync = bbAsync;
        return *this;
    }

    AstraConfig &AstraConfig::withDataLogs(ILogSink **logs, uint8_t numLogs)
    {
        if (numLogs > 50)
        {
            LOGW("Attempted to add %d log sinks, but the maximum number of log sinks is 50. That's too many logs. Why.", numLogs);
            numLogs = 50;
        }
        for (uint8_t i = 0; i < numLogs; i++)
            this->logs[i] = logs[i];
        this->numLogs = numLogs;
        return *this;
    }

    AstraConfig &AstraConfig::withSensorUpdateRate(double sensorUpdateRate)
    {
        this->sensorUpdateRate = sensorUpdateRate;
        this->sensorUpdateInterval = 1000.0 / sensorUpdateRate;
        LOGI("Sensor update rate set to %.2f Hz (%.2f ms interval)", sensorUpdateRate, this->sensorUpdateInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withSensorUpdateInterval(unsigned int sensorUpdateIntervalMs)
    {
        this->sensorUpdateInterval = sensorUpdateIntervalMs / 1000.0;
        this->sensorUpdateRate = 1000.0 / sensorUpdateIntervalMs;
        LOGI("Sensor update interval set to %u ms (%.2f Hz)", sensorUpdateIntervalMs, this->sensorUpdateRate);
        return *this;
    }

    AstraConfig &AstraConfig::withPredictRate(double predictRate)
    {
        this->predictRate = predictRate;
        this->predictInterval = 1000.0 / predictRate;
        LOGI("Predict rate set to %.2f Hz (%.2f ms interval)", predictRate, this->predictInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withPredictInterval(unsigned int predictIntervalMs)
    {
        this->predictInterval = predictIntervalMs / 1000.0;
        this->predictRate = 1000.0 / predictIntervalMs;
        LOGI("Predict interval set to %u ms (%.2f Hz)", predictIntervalMs, this->predictRate);
        return *this;
    }

    AstraConfig &AstraConfig::withMeasurementUpdateRate(double measurementUpdateRate)
    {
        this->measurementUpdateRate = measurementUpdateRate;
        this->measurementUpdateInterval = 1000.0 / measurementUpdateRate;
        LOGI("Measurement update rate set to %.2f Hz (%.2f ms interval)", measurementUpdateRate, this->measurementUpdateInterval);
        return *this;
    }

    AstraConfig &AstraConfig::withMeasurementUpdateInterval(unsigned int measurementUpdateIntervalMs)
    {
        this->measurementUpdateInterval = measurementUpdateIntervalMs / 1000.0;
        this->measurementUpdateRate = 1000.0 / measurementUpdateIntervalMs;
        LOGI("Measurement update interval set to %u ms (%.2f Hz)", measurementUpdateIntervalMs, this->measurementUpdateRate);
        return *this;
    }

    AstraConfig &AstraConfig::withHITL(bool hitlEnabled)
    {
        this->hitlMode = hitlEnabled;

        return *this;
    }

    AstraConfig &AstraConfig::withAccel(Accel *accel)
    {
        this->accel = accel;
        LOGI("Accelerometer configured.");
        return *this;
    }

    AstraConfig &AstraConfig::withGyro(Gyro *gyro)
    {
        this->gyro = gyro;
        LOGI("Gyroscope configured.");
        return *this;
    }

    AstraConfig &AstraConfig::withMag(Mag *mag)
    {
        this->mag = mag;
        LOGI("Magnetometer configured.");
        return *this;
    }

    AstraConfig &AstraConfig::withBaro(Barometer *baro)
    {
        this->baro = baro;
        LOGI("Barometer configured.");
        return *this;
    }

    AstraConfig &AstraConfig::withGPS(GPS *gps)
    {
        this->gps = gps;
        LOGI("GPS configured.");
        return *this;
    }

    AstraConfig &AstraConfig::withMiscSensor(Sensor *sensor)
    {
        if (!sensor)
        {
            LOGE("Cannot add null misc sensor.");
            return *this;
        }
        if (numMiscSensors >= MAX_MISC_SENSORS)
        {
            LOGE("Cannot add misc sensor '%s': Maximum of %d misc sensors reached.",
                 sensor->getName(), MAX_MISC_SENSORS);
            return *this;
        }
        miscSensors[numMiscSensors++] = sensor;
        LOGI("Misc sensor '%s' added (index %d).", sensor->getName(), numMiscSensors - 1);
        return *this;
    }

    AstraConfig &AstraConfig::with6DoFIMU(IMU6DoF *imu)
    {
        if (!imu)
        {
            LOGE("Cannot configure null 6-DOF IMU.");
            return *this;
        }

        // Extract accelerometer and gyroscope from IMU
        accel = imu->getAccelSensor();
        gyro = imu->getGyroSensor();

        LOGI("6-DOF IMU '%s' configured - extracted accel and gyro sensors.", imu->getName());

        // Add the IMU itself as a misc sensor for data logging
        // (The IMU logs its own data, the extracted sensors don't duplicate)
        return withMiscSensor(imu);
    }

    AstraConfig &AstraConfig::with9DoFIMU(IMU9DoF *imu)
    {
        if (!imu)
        {
            LOGE("Cannot configure null 9-DOF IMU.");
            return *this;
        }

        // Extract accelerometer, gyroscope, and magnetometer from IMU
        accel = imu->getAccelSensor();
        gyro = imu->getGyroSensor();
        mag = imu->getMagSensor();

        LOGI("9-DOF IMU '%s' configured - extracted accel, gyro, and mag sensors.", imu->getName());

        // Add the IMU itself as a misc sensor for data logging
        return withMiscSensor(imu);
    }

    void AstraConfig::populateSensorManager()
    {
        // Populate SensorManager from individual sensor pointers
        if (accel)
            sensorManager.setAccelSource(accel);
        if (gyro)
            sensorManager.setGyroSource(gyro);
        if (mag)
            sensorManager.setMagSource(mag);
        if (baro)
            sensorManager.setBaroSource(baro);
        if (gps)
            sensorManager.setGPSSource(gps);

        // Add misc sensors
        for (uint8_t i = 0; i < numMiscSensors; i++)
        {
            if (miscSensors[i])
                sensorManager.addMiscSensor(miscSensors[i]);
        }

        LOGI("SensorManager populated with %d primary sensors and %d misc sensors.",
             (accel ? 1 : 0) + (gyro ? 1 : 0) + (mag ? 1 : 0) + (baro ? 1 : 0) + (gps ? 1 : 0),
             numMiscSensors);
    }

}