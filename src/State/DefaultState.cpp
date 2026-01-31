#include "DefaultState.h"
#include "../Sensors/SensorManager/SensorManager.h"
#include "../Sensors/GPS/GPS.h"
#include "../Sensors/Baro/Barometer.h"
#include <Arduino.h>

namespace astra
{
    DefaultState::DefaultState(double processNoise,
                               double gpsNoise,
                               double baroNoise,
                               double mahonyKp,
                               double mahonyKi)
        : State(nullptr, nullptr) // We'll set these after creating them
    {
        // Create owned filters
        ownedKalmanFilter = new DefaultKalmanFilter(processNoise, gpsNoise, baroNoise);
        ownedOrientationFilter = new MahonyAHRS(mahonyKp, mahonyKi);

        // Set them in the base class
        this->filter = ownedKalmanFilter;
        this->orientationFilter = ownedOrientationFilter;

        LOGI("DefaultState created with built-in filters");
    }

    DefaultState::~DefaultState()
    {
        // Clean up owned resources
        delete ownedKalmanFilter;
        delete ownedOrientationFilter;
    }

    bool DefaultState::update(double newTime)
    {
        if (!initialized)
        {
            LOGE("State is not initialized! Call begin() first.");
            return false;
        }
        if (!sensorManager)
        {
            LOGE("No Sensor Manager configured. Add one to Astra or directly to state.");
            return false;
        }
        if (!sensorManager->isOK())
        {
            LOGE("Sensor Manager reports an error. Cannot update State.");
            return false;
        }
        if (!orientationFilter)
        {
            LOGE("Orientation Filter not available or not initialized. Cannot update State.");
            return false;
        }
        if (!filter)
        {
            LOGE("Kalman Filter not available. Cannot update State.");
            return false;
        }

        // Asynchronous sensor updates - only update KF when sensor has new data

        // Check for GPS update (horizontal position)
        if (sensorManager->hasGPSUpdate())
        {
            GPS *gps = sensorManager->getGPSSource();
            if (gps && gps->getHasFix())
            {
                // Set GPS origin on first fix (when origin x,y are still 0)
                if (origin.x() == 0 && origin.y() == 0)
                {
                    Vector<3> gpsPos = gps->getPos();
                    origin.x() = gpsPos.x(); // latitude
                    origin.y() = gpsPos.y(); // longitude
                    LOGI("GPS origin set to lat=%.6f, lon=%.6f", origin.x(), origin.y());
                }

                // Get displacement from origin in meters
                Vector<3> displacement = gps->getDisplacement(origin);
                ownedKalmanFilter->updateGPS(displacement.x(), displacement.y());
            }
            sensorManager->clearGPSUpdate();
        }

        // Check for barometer update (vertical position)
        if (sensorManager->hasBaroUpdate())
        {
            Barometer *baro = sensorManager->getBaroSource();
            if (baro && baro->isInitialized())
            {
                double altitudeASL = baro->getASLAltM();
                ownedKalmanFilter->updateBaro(altitudeASL - origin.z());
            }
            sensorManager->clearBaroUpdate();
        }

        // Extract updated state from KF
        Matrix state = filter->getState();

        // Update state variables from measurement update
        position.x() = state(0, 0);
        position.y() = state(1, 0);
        position.z() = state(2, 0);
        velocity.x() = state(3, 0);
        velocity.y() = state(4, 0);
        velocity.z() = state(5, 0);

        // Acceleration is computed from orientation filter (not from KF state)
        if (orientationFilter && orientationFilter->isReady() && sensorManager->getAccelSource())
        {
            Vector<3> earthAccel = orientationFilter->getEarthAcceleration(sensorManager->getAccelSource()->getAccel());
            acceleration.x() = earthAccel.x();
            acceleration.y() = earthAccel.y();
            acceleration.z() = earthAccel.z();
        }

        return true;
    }

} // namespace astra
