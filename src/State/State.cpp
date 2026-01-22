#include "State.h"
#include <Arduino.h>
#include "../Filters/LinearKalmanFilter.h"
#include "../Sensors/Sensor.h"
#include "../Sensors/Accel/Accel.h"
#include "../Sensors/Gyro/Gyro.h"
#include "../Sensors/IMU/IMU6DoF.h"
#include "../Sensors/IMU/IMU9DoF.h"
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

    void State::withSensors(Sensor **sensors, int numSensors)
    {
        this->sensors = sensors;
        this->numSensors = numSensors;
    }

    Sensor *State::findSensor(uint32_t type, int sensorNum) const
    {
        for (int i = 0; i < numSensors; i++)
        {
            if (sensors[i] && type == sensors[i]->getType() && --sensorNum == 0)
                return sensors[i];
        }
        return nullptr;
    }

    Accel *State::findAccel(int sensorNum) const
    {
        // Try standalone accelerometer first
        Sensor *sensor = findSensor("Accelerometer"_i, sensorNum);
        if (sensor)
            return static_cast<Accel *>(sensor);

        // Fall back to IMU6DoF - get contained accel component
        sensor = findSensor("IMU6DoF"_i, sensorNum);
        if (sensor)
            return static_cast<IMU6DoF *>(sensor)->getAccelSensor();

        // Fall back to IMU9DoF - get contained accel component
        sensor = findSensor("IMU9DoF"_i, sensorNum);
        if (sensor)
            return static_cast<IMU9DoF *>(sensor)->getAccelSensor();

        return nullptr;
    }

    Gyro *State::findGyro(int sensorNum) const
    {
        // Try standalone gyroscope first
        Sensor *sensor = findSensor("Gyroscope"_i, sensorNum);
        if (sensor)
            return static_cast<Gyro *>(sensor);

        // Fall back to IMU6DoF - get contained gyro component
        sensor = findSensor("IMU6DoF"_i, sensorNum);
        if (sensor)
            return static_cast<IMU6DoF *>(sensor)->getGyroSensor();

        // Fall back to IMU9DoF - get contained gyro component
        sensor = findSensor("IMU9DoF"_i, sensorNum);
        if (sensor)
            return static_cast<IMU9DoF *>(sensor)->getGyroSensor();

        return nullptr;
    }

    bool State::begin()
    {
        // Sensors are initialized by Astra
        if (filter)
        {
            filter->initialize();
            stateVars = new double[filter->getStateSize()];
        }

        if (orientationFilter && sensors && numSensors > 0)
        {
            // Auto-calibrate orientation filter during initialization
            Accel *accelSensor = findAccel();
            Gyro *gyroSensor = findGyro();
            bool hasAccelGyro = accelSensor && accelSensor->isInitialized() &&
                                gyroSensor && gyroSensor->isInitialized();

            if (hasAccelGyro)
            {
                LOGI("Auto-calibrating orientation filter...");
                orientationFilter->setMode(MahonyMode::CALIBRATING);

                // Collect calibration samples
                const int calibSamples = 200;
                for (int i = 0; i < calibSamples; i++)
                {
                    // Update sensors directly
                    for (int s = 0; s < numSensors; s++)
                    {
                        if (sensors[s] && sensors[s]->isInitialized())
                            sensors[s]->update();
                    }
                    orientationFilter->update(accelSensor->getAccel(), gyroSensor->getAngVel(), 0.01); // Assume ~100Hz
                    delay(10);
                }

                // Initialize and switch to normal mode
                orientationFilter->initialize();
                orientationFilter->setMode(MahonyMode::GYRO_ONLY);
                LOGI("Orientation filter calibrated and ready.");
            }
            else
            {
                LOGW("No Accel/Gyro sensors available for orientation filter calibration.");
            }
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

        delete[] inputs;
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

        delete[] measurements;
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
