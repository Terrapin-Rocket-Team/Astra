#include "State.h"
#include <Arduino.h>
#include "../Filters/LinearKalmanFilter.h"
#include "../Sensors/SensorManager/ISensorManager.h"

#pragma region Constructor and Destructor

namespace astra
{
    State::State(Filter *filter, MahonyAHRS *orientationFilter) : DataReporter("State")
    {
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
    }

    State::~State()
    {
        delete[] stateVars;
    }

#pragma endregion

    void State::withSensorManager(ISensorManager *sensorManager)
    {
        this->sensorManager = sensorManager;
    }

    bool State::begin()
    {
        // Initialize Kalman filter
        if (filter)
        {
            filter->initialize();
            stateVars = new double[filter->getStateSize()];
        }

        // Auto-calibrate orientation filter if sensor manager is available
        if (orientationFilter && sensorManager && sensorManager->isReady())
        {
            LOGI("Auto-calibrating orientation filter...");
            orientationFilter->setMode(MahonyMode::CALIBRATING);

            // Collect calibration samples using body frame data
            const int calibSamples = 200;
            for (int i = 0; i < calibSamples; i++)
            {
                sensorManager->update();
                const BodyFrameData &data = sensorManager->getBodyFrameData();

                if (data.hasAccel && data.hasGyro)
                {
                    orientationFilter->update(data.accel, data.gyro, 0.01); // Assume ~100Hz
                }
                delay(10);
            }

            // Initialize and switch to normal mode
            orientationFilter->initialize();
            orientationFilter->setMode(MahonyMode::GYRO_ONLY);
            LOGI("Orientation filter calibrated and ready.");
        }
        else if (orientationFilter)
        {
            LOGW("No SensorManager available for orientation filter calibration.");
        }

        initialized = true;
        LOGI("State Initialized.");
        return true;
    }

#pragma region Update Functions

    void State::update(double newTime)
    {
        // This method is deprecated - Astra now calls split update methods
        LOGW("State::update() is deprecated. Use split update methods via Astra.");
    }

    void State::updateOrientation(const BodyFrameData &bodyData, double dt)
    {
        if (!orientationFilter)
            return;

        if (!bodyData.hasAccel || !bodyData.hasGyro)
            return;

        // Delegate to the vector-based implementation
        updateOrientation(bodyData.gyro, bodyData.accel, dt);
    }

    void State::updateOrientation(const Vector<3> &gyro, const Vector<3> &accel, double dt)
    {
        if (!orientationFilter)
            return;

        // Automatic mode switching based on accelerometer magnitude
        // If |accel| is close to 9.81 m/s^2, enable accel correction
        // Otherwise (high-g or freefall), use gyro-only mode
        if (orientationFilter->isInitialized())
        {
            double accelMag = accel.magnitude();
            double accelError = abs(accelMag - 9.81);

            // Threshold: if accel error < 1 m/s^2, trust the accelerometer
            if (accelError < 1.0)
            {
                if (orientationFilter->getMode() != MahonyMode::CORRECTING)
                {
                    orientationFilter->setMode(MahonyMode::CORRECTING);
                }
            }
            else
            {
                if (orientationFilter->getMode() != MahonyMode::GYRO_ONLY)
                {
                    orientationFilter->setMode(MahonyMode::GYRO_ONLY);
                }
            }
        }

        orientationFilter->update(accel, gyro, dt);

        // Update orientation and earth-frame acceleration from filter
        if (orientationFilter->isInitialized())
        {
            orientation = orientationFilter->getQuaternion();
            // Get earth-frame acceleration (with gravity subtracted)
            acceleration = orientationFilter->getEarthAcceleration(accel);
        }
    }

    void State::predictState(double newTime)
    {
        if (!filter)
            return;

        lastTime = currentTime;
        if (newTime != -1)
            currentTime = newTime;
        else
            currentTime = millis() / 1000.0;

        double dt = currentTime - lastTime;
        if (dt <= 0)
            return;

        // Prepare control inputs (earth-frame acceleration)
        double *inputs = new double[filter->getInputSize()];
        if (orientationFilter && orientationFilter->isInitialized())
        {
            inputs[0] = acceleration.x();
            inputs[1] = acceleration.y();
            inputs[2] = acceleration.z();
        }
        else
        {
            inputs[0] = 0.0;
            inputs[1] = 0.0;
            inputs[2] = 0.0;
        }

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

    void State::updateMeasurements(const Vector<3> &gpsPos, double baroAlt, bool hasGPS, bool hasBaro, double newTime)
    {
        if (!filter)
            return;

        if (newTime != -1)
            currentTime = newTime;
        else
            currentTime = millis() / 1000.0;

        // Prepare measurements
        double *measurements = new double[filter->getMeasurementSize()];

        // gps x y barometer z
        measurements[0] = hasGPS ? (gpsPos.x() - origin.x()) : 0;
        measurements[1] = hasGPS ? (gpsPos.y() - origin.y()) : 0;
        measurements[2] = hasBaro ? baroAlt : 0;

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

    void State::updatePositionVelocity(double lat, double lon, double heading, bool hasFix)
    {
        if (hasFix)
        {
            coordinates = Vector<2>(lat, lon);
            this->heading = heading;
        }
        else
        {
            coordinates = Vector<2>(0, 0);
            this->heading = 0;
        }
    }

#pragma endregion Update Functions

} // namespace astra
