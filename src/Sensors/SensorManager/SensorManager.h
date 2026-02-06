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

        // Update flags for primary sensors (set when new data is available)
        bool accelUpdated = false;
        bool gyroUpdated = false;
        bool magUpdated = false;
        bool baroUpdated = false;
        bool gpsUpdated = false;

        // Init error flags for status feedback
        bool accelInitFailed = false;
        bool gyroInitFailed = false;
        bool magInitFailed = false;
        bool baroInitFailed = false;
        bool gpsInitFailed = false;
        bool miscInitFailed = false;

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

        // Check if sensor has new data since last check
        bool hasAccelUpdate() const { return accelUpdated; }
        bool hasGyroUpdate() const { return gyroUpdated; }
        bool hasMagUpdate() const { return magUpdated; }
        bool hasBaroUpdate() const { return baroUpdated; }
        bool hasGPSUpdate() const { return gpsUpdated; }

        // Clear update flags after consuming the data
        void clearAccelUpdate() { accelUpdated = false; }
        void clearGyroUpdate() { gyroUpdated = false; }
        void clearMagUpdate() { magUpdated = false; }
        void clearBaroUpdate() { baroUpdated = false; }
        void clearGPSUpdate() { gpsUpdated = false; }

        // Check init status for status indicators
        bool didAccelInitFail() const { return accelInitFailed; }
        bool didGyroInitFail() const { return gyroInitFailed; }
        bool didMagInitFail() const { return magInitFailed; }
        bool didBaroInitFail() const { return baroInitFailed; }
        bool didGPSInitFail() const { return gpsInitFailed; }
        bool didMiscInitFail() const { return miscInitFailed; }

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
        // Returns 0 on success, non-zero error count on failure
        int begin()
        {
            int errorCount = 0;
            // Reset error flags
            accelInitFailed = false;
            gyroInitFailed = false;
            magInitFailed = false;
            baroInitFailed = false;
            gpsInitFailed = false;
            miscInitFailed = false;

            // --- Primary Sensors (Critical) ---
            // Helper lambda to reduce boilerplate for primary sensors
            auto initPrimary = [&](Sensor *sensor, const char *label, bool &failFlag)
            {
                if (sensor)
                {
                    int err = sensor->begin();
                    if (err == 0)
                        LOGI("%s (%s) initialized successfully.", label, sensor->getName());
                    else
                    {
                        LOGE("%s (%s) FAILED to initialize with error code: %d", label, sensor->getName(), err);
                        errorCount++;
                        failFlag = true;
                    }
                }
                else
                    LOGW("No %s sensor defined.", label);
            };

            initPrimary(accel, "Accelerometer", accelInitFailed);
            initPrimary(gyro, "Gyroscope", gyroInitFailed);
            initPrimary(mag, "Magnetometer", magInitFailed);
            initPrimary(baro, "Barometer", baroInitFailed);
            initPrimary(gps, "GPS", gpsInitFailed);

            // --- Miscellaneous Sensors (Iterative) ---

            LOGI("Initializing %hhd miscellaneous sensors...", numMisc);
            for (uint8_t i = 0; i < numMisc; i++)
            {
                if (miscSensors[i])
                {
                    int err = miscSensors[i]->begin();
                    if (err == 0)
                        LOGI("Misc sensor [%d] (%s) initialized.", i, miscSensors[i]->getName());
                    else
                    {
                        LOGE("Misc sensor [%d] (%s) FAILED with error code: %d", i, miscSensors[i]->getName(), err);
                        errorCount++;
                        miscInitFailed = true;
                    }
                }
            }
            if (errorCount == 0)
                LOGI("All sensors initialized successfully.");
            else
                LOGE("Sensor initialization completed with %d ERROR(S).", errorCount);

            ok = (errorCount == 0);
            return errorCount;
        }

        void update(double currentTime)
        {
            // Update primary sensors only if their update interval has elapsed
            if (accel && accel->shouldUpdate(currentTime))
            {
                accel->update();
                accelUpdated = true;
            }

            if (gyro && gyro->shouldUpdate(currentTime))
            {
                gyro->update();
                gyroUpdated = true;
            }

            if (mag && mag->shouldUpdate(currentTime))
            {
                mag->update();
                magUpdated = true;
            }

            if (baro && baro->shouldUpdate(currentTime))
            {
                baro->update();
                baroUpdated = true;
            }

            if (gps && gps->shouldUpdate(currentTime))
            {
                gps->update();
                gpsUpdated = true;
            }

            // Update misc sensors
            for (uint8_t i = 0; i < numMisc; i++)
            {
                if (miscSensors[i] && miscSensors[i]->shouldUpdate(currentTime))
                    miscSensors[i]->update();
            }
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

        // ========================= Convenience Data Access Methods =========================
        // These methods provide clean access to sensor data without verbose chaining
        // Returns safe defaults (zero vectors) if sensor is not present or initialized

        /**
         * Get current acceleration vector in m/s^2
         * Returns zero vector if accelerometer not present or not initialized
         */
        Vector<3> getAcceleration() const
        {
            if (accel && accel->isInitialized())
                return accel->getAccel();
            return Vector<3>(0, 0, 0);
        }

        /**
         * Get current angular velocity vector in rad/s
         * Returns zero vector if gyroscope not present or not initialized
         */
        Vector<3> getAngularVelocity() const
        {
            if (gyro && gyro->isInitialized())
                return gyro->getAngVel();
            return Vector<3>(0, 0, 0);
        }

        /**
         * Get current magnetic field vector in uT (microtesla)
         * Returns zero vector if magnetometer not present or not initialized
         */
        Vector<3> getMagneticField() const
        {
            if (mag && mag->isInitialized())
                return mag->getMag();
            return Vector<3>(0, 0, 0);
        }

        /**
         * Get current barometric altitude in meters ASL (Above Sea Level)
         * Returns 0.0 if barometer not present or not initialized
         */
        double getBarometricAltitude() const
        {
            if (baro && baro->isInitialized())
                return baro->getASLAltM();
            return 0.0;
        }

        /**
         * Get current GPS position as Vector<3> (latitude, longitude, altitude)
         * Returns zero vector if GPS not present, not initialized, or no fix
         */
        Vector<3> getGPSPosition() const
        {
            if (gps && gps->isInitialized() && gps->getHasFix())
                return gps->getPos();
            return Vector<3>(0, 0, 0);
        }

        /**
         * Get current GPS velocity in m/s (NED frame: North, East, Down)
         * Returns zero vector if GPS not present, not initialized, or no fix
         */
        Vector<3> getGPSVelocity() const
        {
            if (gps && gps->isInitialized() && gps->getHasFix())
                return gps->getVel();
            return Vector<3>(0, 0, 0);
        }
    };
}
#endif // SENSOR_MANAGER_H