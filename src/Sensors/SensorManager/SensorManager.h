#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "Sensors/Accel/Accel.h"
#include "Sensors/Gyro/Gyro.h"
#include "Sensors/Mag/Mag.h"
#include "Sensors/Baro/Barometer.h"
#include "Sensors/GPS/GPS.h"
#include "Sensors/Sensor.h"

namespace astra
{
    class SensorManager
    {
    private:
        // Primary sensors for state estimation
        Accel *primaryAccel = nullptr;
        Gyro *primaryGyro = nullptr;
        Mag *primaryMag = nullptr;
        Barometer *primaryBaro = nullptr;
        GPS *primaryGPS = nullptr;

        // Misc sensors that need updating but aren't used in state
        static constexpr uint8_t MAX_MISC_SENSORS = 16;
        Sensor *miscSensors[MAX_MISC_SENSORS] = {nullptr};
        uint8_t numMisc = 0;

    public:
        // Configuration
        void setPrimaryAccel(Accel *a) { primaryAccel = a; }
        void setPrimaryGyro(Gyro *g) { primaryGyro = g; }
        void setPrimaryMag(Mag *m) { primaryMag = m; }
        void setPrimaryBaro(Barometer *b) { primaryBaro = b; }
        void setPrimaryGPS(GPS *g) { primaryGPS = g; }

        Accel *getPrimaryAccel() const { return primaryAccel; }
        Gyro *getPrimaryGyro() const { return primaryGyro; }
        Mag *getPrimaryMag() const { return primaryMag; }
        Barometer *getPrimaryBaro() const { return primaryBaro; }
        GPS *getPrimaryGPS() const { return primaryGPS; }

        bool addMiscSensor(Sensor *s)
        {
            if (!s)
            {
                LOGE("Cannot add null sensor to miscSensors array.");
                return false;
            }
            if (numMisc >= MAX_MISC_SENSORS)
            {
                LOGE("Cannot add sensor '%s': miscSensors array is full (%d/%d).",
                     s->getName(), numMisc, MAX_MISC_SENSORS);
                return false;
            }
            miscSensors[numMisc++] = s;
            LOGI("Added misc sensor '%s' at index %d.", s->getName(), numMisc - 1);
            return true;
        }

        // Operations
        bool begin()
        {
            bool success = true;

            // --- Primary Sensors (Critical) ---

            // Helper lambda to reduce boilerplate for primary sensors
            auto initPrimary = [&](Sensor *sensor, const char *label)
            {
                if (sensor)
                {
                    if (sensor->begin())
                        LOGI("%s (%s) initialized successfully.", label, sensor->getName());
                    else
                    {
                        LOGE("%s (%s) FAILED to initialize!", label, sensor->getName());
                        success = false;
                    }
                }
                else
                    LOGW("No %s sensor defined.", label);
            };

            initPrimary(primaryAccel, "Primary Accel");
            initPrimary(primaryGyro, "Primary Gyro");
            initPrimary(primaryMag, "Primary Mag");
            initPrimary(primaryBaro, "Primary Baro");
            initPrimary(primaryGPS, "Primary GPS");

            // --- Miscellaneous Sensors (Iterative) ---

            LOGI("Initializing %hhd miscellaneous sensors...", numMisc);
            for (uint8_t i = 0; i < numMisc; i++)
            {
                if (miscSensors[i])
                {
                    if (miscSensors[i]->begin())
                        LOGI("Misc sensor [%d] (%s) initialized.", i, miscSensors[i]->getName());
                    else
                    {
                        LOGE("Misc sensor [%d] (%s) FAILED!", i, miscSensors[i]->getName());
                        success = false;
                    }
                }
            }
            if (success)
                LOGI("All sensors initialized successfully.");
            else
                LOGE("Sensor initialization completed with ERRORS.");
            return success;
        }

        void update()
        {

            for (uint8_t i = 0; i < numMisc; i++)
            {
                if (miscSensors[i])
                    miscSensors[i]->update();
                else
                    LOGW("Misc sensor at index %d is null during update.", i);
            }
            if (primaryAccel)
                primaryAccel->update();
            if (primaryGyro)
                primaryGyro->update();
            if (primaryMag)
                primaryMag->update();
            if (primaryBaro)
                primaryBaro->update();
            if (primaryGPS)
                primaryGPS->update();
        }

        /**
         * Check if the SensorManager is ready for use
         * Returns true if at least the primary accelerometer is configured and initialized
         *
         * A SensorManager is considered "OK" if it has a primary accelerometer,
         * as this is the minimum required sensor for most flight control applications.
         * Other sensors are optional but logged if missing.
         */
        bool isOK() const
        {
            // Must have at least a primary accelerometer
            if (!primaryAccel)
            {
                LOGE("SensorManager is not OK: No primary accelerometer configured.");
                return false;
            }

            if (!primaryAccel->isInitialized())
            {
                LOGE("SensorManager is not OK: Primary accelerometer not initialized.");
                return false;
            }

            // Check other critical sensors and warn if missing/not initialized
            if (primaryGyro && !primaryGyro->isInitialized())
            {
                LOGW("Primary gyroscope is configured but not initialized.");
            }

            if (primaryBaro && !primaryBaro->isInitialized())
            {
                LOGW("Primary barometer is configured but not initialized.");
            }

            if (primaryMag && !primaryMag->isInitialized())
            {
                LOGW("Primary magnetometer is configured but not initialized.");
            }

            if (primaryGPS && !primaryGPS->isInitialized())
            {
                LOGW("Primary GPS is configured but not initialized.");
            }

            return true;
        }

        // Accessors for state estimation
        Vector<3> getAccel() const
        {
            if (!primaryAccel)
            {
                LOGW("Attempted to get accel data but primaryAccel is null.");
                return Vector<3>();
            }
            return primaryAccel->getAccel();
        }

        Vector<3> getGyro() const
        {
            if (!primaryGyro)
            {
                LOGW("Attempted to get gyro data but primaryGyro is null.");
                return Vector<3>();
            }
            return primaryGyro->getAngVel();
        }

        Vector<3> getMag() const
        {
            if (!primaryMag)
            {
                LOGW("Attempted to get mag data but primaryMag is null.");
                return Vector<3>();
            }
            return primaryMag->getMag();
        }

        double getPressure() const
        {
            if (!primaryBaro)
            {
                LOGW("Attempted to get pressure data but primaryBaro is null.");
                return 0.0;
            }
            return primaryBaro->getPressure();
        }

        double getTemp() const
        {
            if (!primaryBaro)
            {
                LOGW("Attempted to get temperature data but primaryBaro is null.");
                return 0.0;
            }
            return primaryBaro->getTemp();
        }

        Vector<3> getGPSPos() const
        {
            if (!primaryGPS)
            {
                LOGW("Attempted to get GPS position but primaryGPS is null.");
                return Vector<3>();
            }
            return primaryGPS->getPos();
        }

        double getHeading() const
        {
            if (!primaryGPS)
            {
                LOGW("Attempted to get heading but primaryGPS is null.");
                return 0.0;
            }
            return primaryGPS->getHeading();
        }

        const char *getTimeOfDay() const
        {
            if (!primaryGPS)
            {
                LOGW("Attempted to get time of day but primaryGPS is null.");
                return "";
            }
            return primaryGPS->getTimeOfDay();
        }
    };
}
#endif // SENSOR_MANAGER_H