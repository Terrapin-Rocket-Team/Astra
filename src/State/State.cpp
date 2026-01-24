#include "State.h"
#include <Arduino.h>
#include "../Filters/LinearKalmanFilter.h"
#include "Sensors/SensorManager/SensorManager.h"
#include "Sensors/GPS/GPS.h"

#pragma region Constructor and Destructor

namespace astra
{
    State::State(Filter *filter, MahonyAHRS *orientationFilter) : DataReporter("State")
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
        delete[] stateVars;
    }

#pragma endregion

    void State::withSensorManager(SensorManager *sensorManager)
    {
        this->sensorManager = sensorManager;
    }

    bool State::begin()
    {
        if (!sensorManager)
        {
            LOGE("State does not have a reference to a sensor manager!");
            return false;
        }

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
        stateVars = new double[filter->getStateSize()];

        // Auto-calibrate orientation filter if sensor manager is available
        if (sensorManager && sensorManager->isOK())
        {
            LOGI("Auto-calibrating orientation filter...");

            // Set to CALIBRATING to accumulate bias and track vertical
            orientationFilter->setMode(MahonyMode::CALIBRATING);

#ifndef NATIVE
            // Collect calibration samples
            // The new Mahony update() accumulates sums internally in CALIBRATING mode
            const int calibSamples = 200;
            for (int i = 0; i < calibSamples; i++)
            {
                sensorManager->update();
                // Assume ~100Hz for the delay(10) loop
                orientationFilter->update(sensorManager->getAccel(), sensorManager->getGyro(), 0.01);
                delay(10);
            }
#endif
            // Compute the bias from the samples we just took
            orientationFilter->finalizeCalibration();

            // IMPORTANT: We REMAIN in CALIBRATING mode.
            // This allows the filter to continue snapping to the "nearest vertical"
            // while the rocket is on the rail, compensating for rail tilt or loading.
            //
            // The Launch Detection logic (in StateMachine) MUST call:
            // orientationFilter->lockFrame();
            // orientationFilter->setMode(MahonyMode::CORRECTING);
            // at the moment of liftoff.

            LOGI("Orientation calibration complete. Filter remaining in CALIBRATING mode until liftoff.");
        }
        else
        {
            LOGW("No SensorManager available for orientation filter calibration.");
            return false;
        }

        // Set origin altitude from barometer (sensors have settled during calibration)
        if (sensorManager->getPrimaryBaro() && sensorManager->getPrimaryBaro()->isInitialized())
        {
            origin.z() = sensorManager->getPrimaryBaro()->getASLAltM();
            LOGI("Origin altitude set to %.2f m ASL from barometer.", origin.z());
        }
        else
        {
            LOGW("No barometer available to set origin altitude.");
        }

        // Origin x,y (GPS position) will be set during first update when GPS gets fix
        origin.x() = 0;
        origin.y() = 0;

        initialized = true;
        LOGI("State Initialized.");
        return true;
    }

#pragma region Update Functions

    void State::update(double newTime)
    {
        if (!initialized)
        {
            LOGE("State is not initialized! Call begin() first.");
            return;
        }
        if (!sensorManager)
        {
            LOGE("No Sensor Manager configured. Add one to Astra or directly to state.");
            return;
        }
        if (!sensorManager->isOK())
        {
            LOGE("Sensor Manager reports an error. Cannot update State.");
            return;
        }
        if (!orientationFilter)
        {
            LOGE("Orientation Filter not available or not initialized. Cannot update State.");
            return;
        }
        if (!filter)
        {
            LOGE("Kalman Filter not available. Cannot update State.");
            return;
        }

        // Update time
        double dt = 0.0;
        if (newTime != -1)
        {
            dt = newTime - currentTime;
            currentTime = newTime;
        }
        else
        {
            double newT = millis() / 1000.0;
            dt = newT - currentTime;
            currentTime = newT;
        }

        if (dt <= 0)
        {
            return;
        }

        // Update orientation (happens every update)
        updateOrientation(sensorManager->getGyro(), sensorManager->getAccel(), dt);

        // Prepare measurements from SensorManager
        double *measurements = new double[filter->getMeasurementSize()];

        // Check if we have GPS data
        bool hasGPS = false;
        if (sensorManager->getPrimaryGPS() && sensorManager->getPrimaryGPS()->getHasFix())
        {
            // Set GPS origin on first fix (when origin x,y are still 0)
            if (origin.x() == 0 && origin.y() == 0)
            {
                Vector<3> gpsPos = sensorManager->getPrimaryGPS()->getPos();
                origin.x() = gpsPos.x(); // latitude
                origin.y() = gpsPos.y(); // longitude
                LOGI("GPS origin set to lat=%.6f, lon=%.6f", origin.x(), origin.y());
            }

            hasGPS = true;
            // Get displacement from origin in meters
            Vector<3> displacement = sensorManager->getPrimaryGPS()->getDisplacement(origin);
            measurements[0] = displacement.x(); // displacement in meters (north)
            measurements[1] = displacement.y(); // displacement in meters (east)
        }
        else
        {
            measurements[0] = 0;
            measurements[1] = 0;
        }

        // Check if we have barometer data
        bool hasBaro = false;
        if (sensorManager->getPrimaryBaro() && sensorManager->getPrimaryBaro()->isInitialized())
        {
            hasBaro = true;
            measurements[2] = sensorManager->getPrimaryBaro()->getASLAltM() - origin.z();
        }
        else
        {
            measurements[2] = 0;
        }

        // Run KF measurement update
        LinearKalmanFilter *kf = static_cast<LinearKalmanFilter *>(filter);
        kf->update(measurements);
        kf->getState(stateVars);

        // Update state variables from measurement update
        position.x() = stateVars[0];
        position.y() = stateVars[1];
        position.z() = stateVars[2];
        velocity.x() = stateVars[3];
        velocity.y() = stateVars[4];
        velocity.z() = stateVars[5];

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

        // Update orientation and earth-frame acceleration from filter
        if (orientationFilter->isReady())
        {
            orientation = orientationFilter->getQuaternion();
            // Get earth-frame acceleration (with gravity subtracted)
            acceleration = orientationFilter->getEarthAcceleration(accel);

            // Debugging output could go here
            fprintf(stderr, "DEBUG State::updateOrientation: ...\n");
        }
    }

    void State::predictState(double newTime)
    {
        if (!filter)
        {
            LOGE("No Kalman filter available for prediction.");
            return;
        }

        if (!orientationFilter)
        {
            LOGE("Orientation filter not available or not initialized for prediction.");
            return;
        }

        if (!sensorManager || !sensorManager->isOK())
        {
            LOGE("Sensor manager not available or not OK for prediction.");
            return;
        }

        // Update time
        lastTime = currentTime;
        if (newTime != -1)
            currentTime = newTime;
        else
            currentTime = millis() / 1000.0;

        double dt = currentTime - lastTime;

        if (dt <= 0)
        {
            return;
        }

        // Update orientation first (happens every predict step)
        updateOrientation(sensorManager->getGyro(), sensorManager->getAccel(), dt);

        // Prepare control inputs (earth-frame acceleration from orientation filter)
        double *inputs = new double[filter->getInputSize()];
        inputs[0] = acceleration.x();
        inputs[1] = acceleration.y();
        inputs[2] = acceleration.z();

        // Set current state into filter before predicting
        stateVars[0] = position.x();
        stateVars[1] = position.y();
        stateVars[2] = position.z();
        stateVars[3] = velocity.x();
        stateVars[4] = velocity.y();
        stateVars[5] = velocity.z();

        LinearKalmanFilter *kf = static_cast<LinearKalmanFilter *>(filter);
        kf->setState(stateVars);

        kf->predict(dt, inputs);
        kf->getState(stateVars);

        // Update state variables from prediction
        position.x() = stateVars[0];
        position.y() = stateVars[1];
        position.z() = stateVars[2];
        velocity.x() = stateVars[3];
        velocity.y() = stateVars[4];
        velocity.z() = stateVars[5];

    }

#pragma endregion Update Functions

} // namespace astra