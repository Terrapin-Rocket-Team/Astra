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
        // DEPRECATED: DefaultState now uses the same vector-based API as State.
        // Use updateOrientation(), predict(), and updateMeasurements() instead.
        LOGE("DefaultState::update() is deprecated. Use the new vector-based API.");
        return false;
    }

} // namespace astra
