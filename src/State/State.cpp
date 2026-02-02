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
                double t = millis() / 1000.0;
                sensorManager->update(t);
                // Assume ~100Hz for the delay(10) loop
                orientationFilter->update(sensorManager->getAccelSource()->getAccel(), sensorManager->getGyroSource()->getAngVel(), 0.01);
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
        if (sensorManager->getBaroSource() && sensorManager->getBaroSource()->isInitialized())
        {
            origin.z() = sensorManager->getBaroSource()->getASLAltM();
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

    bool State::update(double newTime)
    {
        if (!initialized)
        {
            fprintf(stderr, "ERROR State: not initialized!\n");
            LOGE("State is not initialized! Call begin() first.");
            return false;
        }
        if (!sensorManager)
        {
            fprintf(stderr, "ERROR State: no sensor manager!\n");
            LOGE("No Sensor Manager configured. Add one to Astra or directly to state.");
            return false;
        }
        if (!sensorManager->isOK())
        {
            fprintf(stderr, "ERROR State: sensor manager not OK!\n");
            LOGE("Sensor Manager reports an error. Cannot update State.");
            return false;
        }
        if (!orientationFilter)
        {
            fprintf(stderr, "ERROR State: no orientation filter or not initialized! initialized=%d\n",
                    orientationFilter ? (int)orientationFilter->isReady() : -1);
            LOGE("Orientation Filter not available or not initialized. Cannot update State.");
            return false;
        }
        if (!filter)
        {
            LOGE("Kalman Filter not available. Cannot update State.");
            return false;
        }

        // NOTE: In split predict/update mode, time management is handled by predictState()
        // This method just performs the measurement update without time propagation
        // Orientation is also already updated by predictState(), so we skip it here

        // Prepare measurements: [px, py, pz]
        // Acceleration is now a control input, not a measurement
        double measurementData[3];

        // Check if we have GPS data
        bool hasGPS = false;
        if (sensorManager->getGPSSource() && sensorManager->getGPSSource()->getHasFix())
        {
            // Set GPS origin on first fix (when origin x,y are still 0)
            if (origin.x() == 0 && origin.y() == 0)
            {
                Vector<3> gpsPos = sensorManager->getGPSSource()->getPos();
                origin.x() = gpsPos.x(); // latitude
                origin.y() = gpsPos.y(); // longitude
                LOGI("GPS origin set to lat=%.6f, lon=%.6f", origin.x(), origin.y());
            }

            hasGPS = true;
            // Get displacement from origin in meters
            Vector<3> displacement = sensorManager->getGPSSource()->getDisplacement(origin);
            measurementData[0] = displacement.x(); // displacement in meters (north)
            measurementData[1] = displacement.y(); // displacement in meters (east)
        }
        else
        {
            measurementData[0] = 0;
            measurementData[1] = 0;
        }

        // Check if we have barometer data
        bool hasBaro = false;
        if (sensorManager->getBaroSource() && sensorManager->getBaroSource()->isInitialized())
        {
            hasBaro = true;
            measurementData[2] = sensorManager->getBaroSource()->getASLAltM() - origin.z();
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

        // Acceleration is computed from orientation filter (not from KF state)
        if (orientationFilter && orientationFilter->isReady())
        {
            Vector<3> earthAccel = orientationFilter->getEarthAcceleration(sensorManager->getAccelSource()->getAccel());
            acceleration.x() = earthAccel.x();
            acceleration.y() = earthAccel.y();
            acceleration.z() = earthAccel.z();
        }

        return true;
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

            // NOTE: We DO NOT set acceleration here!
            // Acceleration comes from the Kalman Filter state, not directly from sensors.
            // The orientation filter's job is only to provide the quaternion for rotating
            // body-frame measurements to earth-frame.
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
        // This computes earth-frame acceleration from body-frame measurements
        updateOrientation(sensorManager->getGyroSource()->getAngVel(), sensorManager->getAccelSource()->getAccel(), dt);

        // Get earth-frame acceleration to use as control input
        Vector<3> earthAccel(0, 0, 0);
        if (orientationFilter && orientationFilter->isReady())
        {
            earthAccel = orientationFilter->getEarthAcceleration(sensorManager->getAccelSource()->getAccel());
            // Update acceleration state variable
            acceleration.x() = earthAccel.x();
            acceleration.y() = earthAccel.y();
            acceleration.z() = earthAccel.z();
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

#pragma endregion Update Functions

} // namespace astra