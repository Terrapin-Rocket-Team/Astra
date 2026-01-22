#ifndef I_SENSOR_MANAGER_H
#define I_SENSOR_MANAGER_H

#include "BodyFrameData.h"
#include "MountingTransform.h"

namespace astra
{
    // Forward declarations
    class Sensor;
    class Accel;
    class Gyro;
    class Mag;
    class Barometer;

    /**
     * ISensorManager - Interface for sensor management
     *
     * Defines the contract for sensor management in Astra. The SensorManager
     * is responsible for:
     *
     * 1. Sensor Frame → Body Frame Transformation (Static)
     *    - Applies mounting rotation to convert sensor data to body frame
     *    - This is a fixed relationship based on physical mounting
     *
     * 2. Sensor Selection and Fallback
     *    - Selects which sensors to use (IMU9 → IMU6 → standalone)
     *    - Handles degraded operation when sensors fail
     *
     * 3. Health Monitoring
     *    - Tracks sensor health and validity
     *    - Auto-switches to backup sensors on failure
     *
     * The SensorManager is "stateless" regarding world frame - it knows nothing
     * about orientation, position, or velocity. That's State's job.
     *
     * Output: BodyFrameData struct with all vectors aligned to rocket body frame.
     */
    class ISensorManager
    {
    public:
        virtual ~ISensorManager() = default;

        // ========================= Lifecycle =========================

        /**
         * Initialize the sensor manager after sensors have been registered
         * @return true if initialization succeeded
         */
        virtual bool begin() = 0;

        /**
         * Update sensor readings and produce body frame data
         * Call this at the sensor update rate (e.g., 100Hz)
         * @return true if update produced valid data
         */
        virtual bool update() = 0;

        // ========================= Data Access =========================

        /**
         * Get the latest body-frame sensor data
         * All vectors are transformed to body frame coordinates
         */
        virtual const BodyFrameData& getBodyFrameData() const = 0;

        /**
         * Get body-frame acceleration (convenience method)
         */
        virtual Vector<3> getAccel() const = 0;

        /**
         * Get body-frame angular velocity (convenience method)
         */
        virtual Vector<3> getGyro() const = 0;

        /**
         * Get body-frame magnetic field (convenience method)
         */
        virtual Vector<3> getMag() const = 0;

        /**
         * Get barometric pressure in hPa
         */
        virtual double getPressure() const = 0;

        /**
         * Get barometric altitude ASL in meters
         */
        virtual double getBaroAltitude() const = 0;

        // ========================= Health & Status =========================

        /**
         * Check if the sensor manager has valid data available
         */
        virtual bool isReady() const = 0;

        /**
         * Get health status for the accelerometer
         */
        virtual SensorHealth getAccelHealth() const = 0;

        /**
         * Get health status for the gyroscope
         */
        virtual SensorHealth getGyroHealth() const = 0;

        /**
         * Get health status for the magnetometer
         */
        virtual SensorHealth getMagHealth() const = 0;

        /**
         * Get health status for the barometer
         */
        virtual SensorHealth getBaroHealth() const = 0;

        /**
         * Get detailed health info for a sensor type
         * @param type "accel", "gyro", "mag", or "baro"
         */
        virtual const SensorHealthInfo& getHealthInfo(const char* type) const = 0;

        // ========================= Configuration =========================

        /**
         * Set the mounting transform for the accelerometer
         */
        virtual void setAccelMount(const MountingTransform& mount) = 0;

        /**
         * Set the mounting transform for the gyroscope
         */
        virtual void setGyroMount(const MountingTransform& mount) = 0;

        /**
         * Set the mounting transform for the magnetometer
         */
        virtual void setMagMount(const MountingTransform& mount) = 0;

        // ========================= Sensor Access (Advanced) =========================

        /**
         * Get the currently active accelerometer sensor
         * May be from IMU or standalone sensor depending on fallback state
         */
        virtual Accel* getActiveAccel() const = 0;

        /**
         * Get the currently active gyroscope sensor
         */
        virtual Gyro* getActiveGyro() const = 0;

        /**
         * Get the currently active magnetometer sensor
         */
        virtual Mag* getActiveMag() const = 0;

        /**
         * Get the currently active barometer sensor
         */
        virtual Barometer* getActiveBaro() const = 0;
    };

} // namespace astra

#endif // I_SENSOR_MANAGER_H
