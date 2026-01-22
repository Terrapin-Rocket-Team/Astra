#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "ISensorManager.h"
#include "../Sensor.h"

namespace astra
{
    // Forward declarations
    class IMU6DoF;
    class IMU9DoF;

    /**
     * SensorManager - Default implementation of ISensorManager
     *
     * Provides a working out-of-the-box sensor management solution:
     * - Takes first available sensor of each type
     * - Handles fallback from IMU9 → IMU6 → standalone sensors
     * - Applies mounting transforms to convert to body frame
     * - Monitors sensor health and auto-switches on failure
     *
     * Downstream implementations (e.g., RocketSensorManager) can either:
     * - Extend this class and override specific methods
     * - Implement ISensorManager directly for full control
     *
     * Usage:
     *   SensorManager sensorMgr;
     *   sensorMgr.withSensors(sensors, numSensors);
     *   sensorMgr.setAccelMount(MountingOrientation::FLIP_YZ);
     *   sensorMgr.begin();
     *
     *   // In update loop:
     *   sensorMgr.update();
     *   BodyFrameData data = sensorMgr.getBodyFrameData();
     */
    class SensorManager : public ISensorManager
    {
    public:
        SensorManager();
        virtual ~SensorManager();

        // ========================= Configuration =========================

        /**
         * Register sensors with the manager
         * Call this before begin()
         */
        virtual void withSensors(Sensor** sensors, int numSensors);

        /**
         * Set maximum consecutive failures before marking sensor as failed
         * Default: 10
         */
        void setMaxFailures(uint32_t maxFailures) { this->maxFailures = maxFailures; }

        /**
         * Set the timeout for considering a sensor stale (ms)
         * Default: 1000ms
         */
        void setStaleTimeout(uint32_t timeoutMs) { staleTimeoutMs = timeoutMs; }

        // ========================= ISensorManager Implementation =========================

        bool begin() override;
        bool update() override;

        const BodyFrameData& getBodyFrameData() const override { return bodyData; }
        Vector<3> getAccel() const override { return bodyData.accel; }
        Vector<3> getGyro() const override { return bodyData.gyro; }
        Vector<3> getMag() const override { return bodyData.mag; }
        double getPressure() const override { return bodyData.pressure; }
        double getBaroAltitude() const override { return bodyData.baroAltASL; }

        bool isReady() const override { return initialized && (bodyData.hasAccel || bodyData.hasGyro); }
        SensorHealth getAccelHealth() const override { return accelHealth.status; }
        SensorHealth getGyroHealth() const override { return gyroHealth.status; }
        SensorHealth getMagHealth() const override { return magHealth.status; }
        SensorHealth getBaroHealth() const override { return baroHealth.status; }
        const SensorHealthInfo& getHealthInfo(const char* type) const override;

        void setAccelMount(const MountingTransform& mount) override { accelMount = mount; }
        void setGyroMount(const MountingTransform& mount) override { gyroMount = mount; }
        void setMagMount(const MountingTransform& mount) override { magMount = mount; }

        Accel* getActiveAccel() const override { return activeAccel; }
        Gyro* getActiveGyro() const override { return activeGyro; }
        Mag* getActiveMag() const override { return activeMag; }
        Barometer* getActiveBaro() const override { return activeBaro; }

    protected:
        // ========================= Sensor Discovery =========================

        /**
         * Find and select the best available accelerometer
         * Override to customize sensor selection logic
         */
        virtual Accel* selectAccel();

        /**
         * Find and select the best available gyroscope
         */
        virtual Gyro* selectGyro();

        /**
         * Find and select the best available magnetometer
         */
        virtual Mag* selectMag();

        /**
         * Find and select the best available barometer
         */
        virtual Barometer* selectBaro();

        /**
         * Find a sensor by type hash
         * @param type Type hash (e.g., "Accelerometer"_i)
         * @param sensorNum Which sensor of this type (1-indexed)
         */
        Sensor* findSensor(uint32_t type, int sensorNum = 1) const;

        // ========================= Health Monitoring =========================

        /**
         * Check and update health status for a sensor
         * @param sensor The sensor to check
         * @param health Health info to update
         * @param currentTimeMs Current time in milliseconds
         * @return true if sensor is healthy
         */
        bool checkSensorHealth(Sensor* sensor, SensorHealthInfo& health, uint32_t currentTimeMs);

        /**
         * Handle sensor failure - attempt fallback
         * @param sensorType "accel", "gyro", "mag", or "baro"
         */
        virtual void handleSensorFailure(const char* sensorType);

        // ========================= Data Members =========================

        // Sensor storage
        Sensor** sensors = nullptr;
        int numSensors = 0;

        // Active sensors (may change on failure/fallback)
        Accel* activeAccel = nullptr;
        Gyro* activeGyro = nullptr;
        Mag* activeMag = nullptr;
        Barometer* activeBaro = nullptr;

        // Backup sensors for fallback
        Accel* backupAccel = nullptr;
        Gyro* backupGyro = nullptr;
        Mag* backupMag = nullptr;
        Barometer* backupBaro = nullptr;

        // Mounting transforms (sensor frame → body frame)
        MountingTransform accelMount;
        MountingTransform gyroMount;
        MountingTransform magMount;

        // Health tracking
        SensorHealthInfo accelHealth;
        SensorHealthInfo gyroHealth;
        SensorHealthInfo magHealth;
        SensorHealthInfo baroHealth;

        // Configuration
        uint32_t maxFailures = 10;
        uint32_t staleTimeoutMs = 1000;

        // Output data
        BodyFrameData bodyData;

        // State
        bool initialized = false;

    private:
        // Dummy health info for unknown sensor types
        static SensorHealthInfo unknownHealth;
    };

} // namespace astra

#endif // SENSOR_MANAGER_H
