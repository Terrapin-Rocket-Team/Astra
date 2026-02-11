#include "State.h"
#include <Arduino.h>
#include "../Filters/LinearKalmanFilter.h"
#include "Sensors/SensorManager/SensorManager.h"
#include "Sensors/GPS/GPS.h"

#pragma region Constructor and Destructor

namespace astra
{
    State::State(LinearKalmanFilter *filter, MahonyAHRS *orientationFilter) : DataReporter("State")
    {
        if (!filter)
        {
            LOGE("State requires a Kalman Filter! Cannot create State without filter.");
        }
        if (!orientationFilter)
        {
            LOGE("State requires an AHRS orientation filter! Cannot create State without orientation filter.");
        }

        lastTime = 0;
        currentTime = 0;
        this->filter = filter;
        this->orientationFilter = orientationFilter;

        addColumn("%0.3f", &currentTime, "Time (s)");
        addColumn("%0.3f", &position.x(), "PX (m)");
        addColumn("%0.3f", &position.y(), "PY (m)");
        addColumn("%0.3f", &position.z(), "PZ (m)");
        addColumn("%0.3f", &velocity.x(), "VX (m/s)");
        addColumn("%0.3f", &velocity.y(), "VY (m/s)");
        addColumn("%0.3f", &velocity.z(), "VZ (m/s)");
        addColumn("%0.3f", &acceleration.x(), "AX (m/s/s)");
        addColumn("%0.3f", &acceleration.y(), "AY (m/s/s)");
        addColumn("%0.3f", &acceleration.z(), "AZ (m/s/s)");
        autoUpdate = false; // disable DataLogger from updating this class
    }

    State::~State()
    {
    }

#pragma endregion

    int State::begin()
    {
        if (!filter)
        {
            LOGE("No Kalman filter available for State. Required!");
            return -1;
        }

        if (!orientationFilter)
        {
            LOGE("No orientation filter available for State. Required!");
            return -2;
        }

        // Initialize Kalman filter
        filter->initialize();

        LOGI("State initialized. Ready for data.");
        initialized = true;
        return 0;
    }

    void State::setGPSOrigin(double lat, double lon, double alt)
    {
        if (!gpsOriginSet)
        {
            origin.x() = lat;
            origin.y() = lon;
            origin.z() = alt;  // Will be overwritten by baro if available
            gpsOriginSet = true;
            LOGI("GPS origin set to lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt);
        }
    }

    void State::setBaroOrigin(double altASL)
    {
        if (!baroOriginSet)
        {
            origin.z() = altASL;
            baroOriginSet = true;
            LOGI("Baro origin set to %.2f m ASL", altASL);
        }
    }

    void State::predict(double dt)
    {
        if (!filter || !orientationFilter)
        {
            LOGE("Cannot predict - filter not available");
            return;
        }

        // Get earth-frame acceleration from orientation filter
        Vector<3> earthAccel(0, 0, 0);
        if (orientationFilter && orientationFilter->isReady())
        {
            // Note: getEarthAcceleration expects body-frame accel,
            // but we've already transformed it in updateOrientation
            // For now, just use the stored acceleration
            earthAccel = acceleration;
        }

        // Run KF prediction step with acceleration as control input
        double controlData[3] = {earthAccel.x(), earthAccel.y(), earthAccel.z()};
        Matrix controlInput(3, 1, controlData);
        filter->predict(dt, controlInput);

        // Extract predicted state: [px, py, pz, vx, vy, vz]
        Matrix state = filter->getState();

        // Update state variables from prediction
        position.x() = state(0, 0);
        position.y() = state(1, 0);
        position.z() = state(2, 0);
        velocity.x() = state(3, 0);
        velocity.y() = state(4, 0);
        velocity.z() = state(5, 0);
    }

    void State::updateGPSMeasurement(const Vector<3> &gpsPos, const Vector<3> &gpsVel)
    {
        if (!filter)
        {
            LOGE("Cannot update measurements - filter not available");
            return;
        }
        (void)gpsVel;

        // Set GPS origin on first valid GPS reading
        if (!gpsOriginSet)
        {
            setGPSOrigin(gpsPos.x(), gpsPos.y(), gpsPos.z());
        }

        // Convert GPS lat/lon to local ENU coordinates (meters from origin)
        double px = 0, py = 0;
        // Calculate displacement from origin in meters
        double latDiff = gpsPos.x() - origin.x();
        double lonDiff = gpsPos.y() - origin.y();

        // Approximate conversion (works for small distances)
        const double EARTH_RADIUS = 6371000.0; // meters
        const double DEG2RAD = 3.14159265358979323846 / 180.0;
        double north = latDiff * DEG2RAD * EARTH_RADIUS;
        double east = lonDiff * DEG2RAD * EARTH_RADIUS * cos(origin.x() * DEG2RAD);

        // ENU: X=East, Y=North
        px = east;
        py = north;

        // GPS update - horizontal position
        filter->updateGPS(px, py);

        // Extract updated state: [px, py, pz, vx, vy, vz]
        Matrix state = filter->getState();

        // Update state variables from measurement update
        position.x() = state(0, 0);
        position.y() = state(1, 0);
        position.z() = state(2, 0);
        velocity.x() = state(3, 0);
        velocity.y() = state(4, 0);
        velocity.z() = state(5, 0);
    }

    void State::updateBaroMeasurement(double baroAlt)
    {
        if (!filter)
        {
            LOGE("Cannot update measurements - filter not available");
            return;
        }

        // Convert baro altitude to position relative to origin
        double pz = 0;
        if (baroOriginSet)
        {
            pz = baroAlt - origin.z();
        }

        // Baro update - vertical position
        filter->updateBaro(pz);

        // Extract updated state: [px, py, pz, vx, vy, vz]
        Matrix state = filter->getState();

        // Update state variables from measurement update
        position.x() = state(0, 0);
        position.y() = state(1, 0);
        position.z() = state(2, 0);
        velocity.x() = state(3, 0);
        velocity.y() = state(4, 0);
        velocity.z() = state(5, 0);
    }

#pragma region Update Functions

    int State::update(double newTime)
    {
        if (newTime == -1)
            newTime = millis() / 1000.0;

        currentTime = newTime;
        return 0;
    }

    void State::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt)
    {
        if (!orientationFilter)
            return;

        // High-G switching logic:
        // Check if accelerometer is measuring close to 1g (stationary or coasting)
        // If so, trust it for orientation correction. Otherwise, use gyro-only.
        double accelMag = accel.magnitude();
        double accelError = abs(accelMag - 9.81);

        if (accelError < 1.0)
        {
            // Low acceleration - trust accelerometer for tilt correction
            orientationFilter->update(accel, gyro, dt);
        }
        else
        {
            // High-G or freefall - gyro-only mode
            orientationFilter->update(gyro, dt);
        }

        // Update orientation quaternion from filter
        orientation = orientationFilter->getQuaternion();

        // Transform body-frame acceleration to earth-frame
        Vector<3> earthAccel = orientationFilter->getEarthAcceleration(accel);
        acceleration.x() = earthAccel.x();
        acceleration.y() = earthAccel.y();
        acceleration.z() = earthAccel.z();
    }

    void State::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, const Vector<3> &mag, double dt)
    {
        if (!orientationFilter)
            return;

        // High-G switching logic:
        // Check if accelerometer is measuring close to 1g (stationary or coasting)
        // If so, trust it for orientation correction. Otherwise, use gyro-only.
        double accelMag = accel.magnitude();
        double accelError = abs(accelMag - 9.81);

        if (accelError < 1.0)
        {
            // Low acceleration - trust accelerometer (and magnetometer) for correction
            orientationFilter->update(accel, gyro, mag, dt);
        }
        else
        {
            // High-G or freefall - gyro-only mode
            orientationFilter->update(gyro, dt);
        }

        // Update orientation quaternion from filter
        orientation = orientationFilter->getQuaternion();

        // Transform body-frame acceleration to earth-frame
        Vector<3> earthAccel = orientationFilter->getEarthAcceleration(accel);
        acceleration.x() = earthAccel.x();
        acceleration.y() = earthAccel.y();
        acceleration.z() = earthAccel.z();
    }

#pragma endregion Update Functions

} // namespace astra
