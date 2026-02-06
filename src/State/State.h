#ifndef STATE_H
#define STATE_H

#include "../Filters/LinearKalmanFilter.h"
#include "../Filters/Mahony.h"
#include "../RecordData/DataReporter/DataReporter.h"
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"

namespace astra
{
    /**
     * State - Inertial state estimation
     *
     * Pure math component - receives sensor data as vectors, no hardware knowledge.
     *
     * Responsible for:
     * 1. Body Frame → Inertial Frame transformation (via Mahony AHRS)
     * 2. Position/velocity estimation (via Kalman Filter)
     * 3. GPS/coordinate tracking
     *
     * Frame convention:
     *   - Body frame: X forward, Y right, Z down (airframe convention)
     *   - Inertial frame: ENU (East, North, Up) relative to launch point
     *
     * Architecture:
     *   - State is pure math - Astra orchestrates data flow
     *   - Base class (State) provides common functionality and virtual methods
     *   - DefaultState provides ready-to-use implementation with DefaultKalmanFilter
     *   - Custom derived classes can implement application-specific dynamics
     *   - Supports asynchronous sensor updates (sensors update at their own rates)
     */
    class State : public DataReporter
    {
    public:
        State(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter);
        virtual ~State();

        /**
         * Initialize state estimation
         * Returns 0 on success, error code on failure
         */
        virtual int begin();

        // ========================= Pure Vector-Based Interface =========================
        // State receives data as vectors, no hardware knowledge

        /**
         * Update orientation estimate from gyro and accel data
         * Should be called at high rate (100+ Hz)
         * @param gyro Angular velocity in rad/s (body frame)
         * @param accel Acceleration in m/s^2 (body frame)
         * @param dt Time step in seconds
         */
        virtual void updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt);

        /**
         * Run Kalman filter prediction step
         * Uses inertial-frame acceleration from orientation filter
         * @param dt Time step in seconds since last prediction
         */
        virtual void predict(double dt);

        /**
         * Update state with a GPS measurement
         * @param gpsPos GPS position (lat, lon, alt)
         * @param gpsVel GPS velocity (NED frame)
         */
        virtual void updateGPSMeasurement(const Vector<3> &gpsPos, const Vector<3> &gpsVel);

        /**
         * Update state with a barometer measurement
         * @param baroAlt Barometric altitude ASL in meters
         */
        virtual void updateBaroMeasurement(double baroAlt);

        /**
         * Set GPS origin (call once when GPS gets first fix)
         * @param lat Latitude in degrees
         * @param lon Longitude in degrees
         * @param alt Altitude ASL in meters
         */
        virtual void setGPSOrigin(double lat, double lon, double alt);

        /**
         * Set barometric origin (call once at startup)
         * @param altASL Altitude ASL in meters from barometer
         */
        virtual void setBaroOrigin(double altASL);

        // ========================= DataReporter Hook =========================
        // Does not drive estimation. Keeps logging time in sync.
        virtual int update(double currentTime = -1) override;

        // ========================= State Getters =========================

        virtual Vector<3> getPosition() const { return position; }         // in m away from point of launch (inertial frame)
        virtual Vector<3> getVelocity() const { return velocity; }         // m/s (inertial frame)
        virtual Vector<3> getAcceleration() const { return acceleration; } // m/s/s (inertial frame)
        virtual Quaternion getOrientation() const { return orientation; }
        virtual Vector<2> getCoordinates() const { return coordinates; }   // lat lon in decimal degrees
        virtual double getHeading() const { return heading; }              // degrees

        // ========================= Filter Control =========================

        MahonyAHRS *getOrientationFilter() const { return orientationFilter; }

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

        bool initialized = false;
        bool gpsOriginSet = false;
        bool baroOriginSet = false;
    };
}
#endif // STATE_H
