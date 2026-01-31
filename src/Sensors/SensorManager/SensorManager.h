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
        Accel *accel = nullptr;
        Gyro *gyro = nullptr;
        Mag *mag = nullptr;
        Barometer *baro = nullptr;
        GPS *gps = nullptr;

        // Misc sensors that need updating but aren't used in state
        static constexpr uint8_t MAX_MISC_SENSORS = 16;
        Sensor *miscSensors[MAX_MISC_SENSORS] = {nullptr};
        uint8_t numMisc = 0;
        bool ok = false;


    public:
        // Configuration
        void setAccelSource(Accel *a) { accel = a; }
        void setGyroSource(Gyro *g) { gyro = g; }
        void setMagSource(Mag *m) { mag = m; }
        void setBaroSource(Barometer *b) { baro = b; }
        void setGPSSource(GPS *g) { gps = g; }

        Accel *getAccelSource() const { return accel; }
        Gyro *getGyroSource() const { return gyro; }
        Mag *getMagSource() const { return mag; }
        Barometer *getBaroSource() const { return baro; }
        GPS *getGPSSource() const { return gps; }

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

            initPrimary(accel, "Accelerometer");
            initPrimary(gyro, "Gyroscope");
            initPrimary(mag, "Magnetometer");
            initPrimary(baro, "Barometer");
            initPrimary(gps, "GPS");

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
            return ok = success;
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
            if (accel)
                accel->update();
            if (gyro)
                gyro->update();
            if (mag)
                mag->update();
            if (baro)
                baro->update();
            if (gps)
                gps->update();
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
            return ok;
        }
    };
}
#endif // SENSOR_MANAGER_H