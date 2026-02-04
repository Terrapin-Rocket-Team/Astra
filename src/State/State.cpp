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

    bool State::begin()
    {
        if (!filter)
        {
            LOGE("No Kalman filter available for State. Required!");
            return false;
        }

        if (!orientationFilter)
        {
            LOGE("No orientation filter available for State. Required!");
            return false;
        }

        // Initialize Kalman filter
        filter->initialize();

        LOGI("State initialized. Ready for data.");
        initialized = true;
        return true;
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

    void State::updateMeasurements(const Vector<3> &gpsPos, const Vector<3> &gpsVel,
                                   double baroAlt, bool hasGPS, bool hasBaro)
    {
        if (!filter)
        {
            LOGE("Cannot update measurements - filter not available");
            return;
        }

        // Set GPS origin on first valid GPS reading
        if (hasGPS && !gpsOriginSet)
        {
            setGPSOrigin(gpsPos.x(), gpsPos.y(), gpsPos.z());
        }

        // Prepare measurements: [px, py, pz]
        double measurementData[3];

        if (hasGPS)
        {
            // Calculate displacement from origin in meters
            // This is a simplified calculation - proper implementation would use haversine
            double latDiff = gpsPos.x() - origin.x();
            double lonDiff = gpsPos.y() - origin.y();

            // Approximate conversion (works for small distances)
            const double EARTH_RADIUS = 6371000.0; // meters
            const double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
            measurementData[0] = latDiff * DEG_TO_RAD * EARTH_RADIUS; // North
            measurementData[1] = lonDiff * DEG_TO_RAD * EARTH_RADIUS * cos(origin.x() * DEG_TO_RAD); // East
        }
        else
        {
            measurementData[0] = 0;
            measurementData[1] = 0;
        }

        if (hasBaro && baroOriginSet)
        {
            measurementData[2] = baroAlt - origin.z();
        }
        else
        {
            measurementData[2] = 0;
        }

        // Create measurement matrix and run KF update
        Matrix measurements(3, 1, measurementData);
        filter->update(measurements);

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

    bool State::update(double newTime)
    {
        // DEPRECATED: Use the new vector-based API instead:
        // - updateOrientation(gyro, accel, dt)
        // - predict(dt)
        // - updateMeasurements(gpsPos, gpsVel, baroAlt, hasGPS, hasBaro)
        LOGE("State::update() is deprecated. Use the new vector-based API.");
        return false;
    }

    void State::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt)
    {
        if (!orientationFilter)
            return;

        MahonyMode currentMode = orientationFilter->getMode();

        // High-G Logic:
        // Only perform automatic switching if we are NOT in CALIBRATING mode.
        // If we are CALIBRATING (on pad), we trust the specific pad logic (snapping to gravity)
        // and ignore the magnitude checks.
        if (orientationFilter->isReady() && currentMode != MahonyMode::CALIBRATING)
        {
            double accelMag = accel.magnitude();
            double accelError = abs(accelMag - 9.81);

            // Threshold: if accel error < 1 m/s^2, trust the accelerometer
            // This switches between normal flight (CORRECTING) and motor burn (GYRO_ONLY)
            if (accelError < 1.0)
            {
                if (currentMode != MahonyMode::CORRECTING)
                {
                    orientationFilter->setMode(MahonyMode::CORRECTING);
                }
            }
            else
            {
                if (currentMode != MahonyMode::GYRO_ONLY)
                {
                    orientationFilter->setMode(MahonyMode::GYRO_ONLY);
                }
            }
        }

        // Update the orientation filter
        orientationFilter->update(accel, gyro, dt);

        // Update orientation quaternion from filter
        if (orientationFilter->isReady())
        {
            orientation = orientationFilter->getQuaternion();

            // Transform body-frame acceleration to earth-frame
            Vector<3> earthAccel = orientationFilter->getEarthAcceleration(accel);
            acceleration.x() = earthAccel.x();
            acceleration.y() = earthAccel.y();
            acceleration.z() = earthAccel.z();
        }
    }

    void State::predictState(double newTime)
    {
        // DEPRECATED: Use predict(dt) instead
        LOGE("State::predictState() is deprecated. Use predict(dt) instead.");
    }

#pragma endregion Update Functions

} // namespace astra