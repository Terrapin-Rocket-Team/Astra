#ifndef STATE_H
#define STATE_H

#include "../Filters/LinearKalmanFilter.h"
#include "../Filters/Mahony.h"
#include "../RecordData/DataReporter/DataReporter.h"
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"

namespace astra
{
    class SensorManager;

    /**
     * State - Inertial state estimation
     *
     * Responsible for:
     * 1. Body Frame → Inertial Frame transformation (dynamic, via Mahony AHRS)
     * 2. Position/velocity estimation (via Kalman Filter)
     * 3. GPS/coordinate tracking
     *
     * State receives body-frame data from SensorManager (which handles sensor
     * selection, fallback, and sensor-to-body frame transforms). State then
     * transforms to inertial frame and runs the estimation filters.
     *
     * Frame convention:
     *   - Body frame: X forward, Y right, Z down (NED-like, attached to vehicle)
     *   - Inertial frame: NED (North, East, Down) relative to launch point
     */
    class State : public DataReporter
    {
    public:
        State(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter);
        virtual ~State();

        /**
         * Configure sensor manager for state estimation
         * Call this before begin() to enable auto-calibration
         */
        void withSensorManager(SensorManager *sensorManager);

        /**
         * Initialize state estimation
         * If a sensor manager is configured, will auto-calibrate orientation filter
         */
        virtual bool begin();


        virtual void update(double currentTime = -1);

        // ========================= Split Update Methods =========================
        // These are called at different rates by Astra

        virtual void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt);

        /**
         * Run Kalman filter prediction step
         * Uses inertial-frame acceleration from orientation filter
         */
        virtual void predictState(double currentTime = -1);
        // ========================= State Getters =========================

        virtual Vector<3> getPosition() const { return position; }         // in m away from point of launch (inertial frame)
        virtual Vector<3> getVelocity() const { return velocity; }         // m/s (inertial frame)
        virtual Vector<3> getAcceleration() const { return acceleration; } // m/s/s (inertial frame)
        virtual Quaternion getOrientation() const { return orientation; }
        virtual Vector<2> getCoordinates() const { return coordinates; }   // lat lon in decimal degrees
        virtual double getHeading() const { return heading; }              // degrees

        // ========================= Filter Control =========================

        MahonyAHRS *getOrientationFilter() const { return orientationFilter; }
        void setOrientationFilterMode(MahonyMode mode)
        {
            if (orientationFilter)
                orientationFilter->setMode(mode);
        }

    protected:
        double currentTime; // in s since uC turned on
        double lastTime;

        // State variables (all in inertial frame)
        Vector<3> position;     // in m from launch position
        Vector<3> velocity;     // in m/s
        Vector<3> acceleration; // in m/s^2
        Quaternion orientation; // body-to-earth rotation
        Vector<2> coordinates;  // in lat, lon
        double heading;         // in degrees
        Vector<3> origin;       // in lat, lon, alt

        // Kalman Filter for position/velocity/acceleration estimation
        LinearKalmanFilter *filter;

        // Orientation filter (Mahony AHRS) - body to inertial transformation
        MahonyAHRS *orientationFilter;

        // Sensor manager reference (optional, for calibration)
        SensorManager *sensorManager = nullptr;

        bool initialized = false;
    };
}
#endif // STATE_H