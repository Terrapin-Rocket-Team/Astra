#include "State.h"
#include <Arduino.h>
#pragma region Constructor and Destructor

namespace astra
{
    State::State(Sensor **sensors, int numSensors, Filter *filter, MahonyAHRS *orientationFilter) : DataReporter("State")
    {
        lastTime = 0;
        currentTime = 0;
        this->maxNumSensors = numSensors;
        this->sensors = sensors;
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

    bool State::begin()
    {
        int good = 0, tryNumSensors = 0;
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensors[i])
            {
                tryNumSensors++;
                if (sensors[i]->begin())
                {
                    good++;
                    LOGI("%s [%s] initialized.", sensors[i]->getTypeString(), sensors[i]->getName());
                }
                else
                {
                    LOGE("%s [%s] failed to initialize.", sensors[i]->getTypeString(), sensors[i]->getName());
                }
            }
            else
            {
                LOGE("sensor index %d in the array was null!", i);
            }
        }
        if (filter)
        {
            filter->initialize();
            stateVars = new double[filter->getStateSize()];
        }

        if (orientationFilter)
        {
            // Auto-calibrate orientation filter during initialization
            // Try IMU first, fallback to separate Accel/Gyro sensors
            // IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
            Accel *accel_sensor = reinterpret_cast<Accel *>(getSensor("Accelerometer"_i));
            Gyro *gyro_sensor = reinterpret_cast<Gyro *>(getSensor("Gyroscope"_i));

            bool hasIMU = false; // sensorOK(imu);
            bool hasAccelGyro = sensorOK(accel_sensor) && sensorOK(gyro_sensor);

            if (hasIMU || hasAccelGyro)
            {
                LOGI("Auto-calibrating orientation filter...");
                orientationFilter->setMode(MahonyMode::CALIBRATING);

                // Collect calibration samples
                const int calibSamples = 200;
                for (int i = 0; i < calibSamples; i++)
                {
                    Vector<3> accel, gyro;

                    if (hasIMU)
                    {
                        // imu->update();
                        // accel = imu->getAcceleration();
                        // gyro = imu->getAngularVelocity();
                    }
                    else
                    {
                        accel_sensor->update();
                        gyro_sensor->update();
                        accel = accel_sensor->getAccel();
                        gyro = gyro_sensor->getAngVel();
                    }

                    orientationFilter->update(accel, gyro, 0.01); // Assume ~100Hz
                    delay(10);
                }

                // Initialize and switch to normal mode
                orientationFilter->initialize();
                orientationFilter->setMode(MahonyMode::CORRECTING);
                LOGI("Orientation filter calibrated and ready.");
            }
            else
            {
                LOGW("No IMU or Accel/Gyro sensors available for orientation filter calibration.");
            }
        }

        numSensors = good;

        initialized = true;
        if (good == tryNumSensors)
            LOGI("State Initialized. All sensors OK.");
        else
            LOGW("State Initialized. %d of %d sensors OK.", good, tryNumSensors);
        return good == tryNumSensors;
    }

#pragma region Update Functions

    void State::update(double newTime)
    {
        lastTime = currentTime;
        if (newTime != -1)
            currentTime = newTime;
        else
            currentTime = millis() / 1000.0;
        updateSensors();
        updateVariables();
    }

    void State::updateSensors()
    {
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            { // not nullptr and initialized
                sensors[i]->update();
            }
        }
    }

    void State::updateVariables()
    {
        GPS *gps = reinterpret_cast<GPS *>(getSensor("GPS"_i));
        // IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
        Accel *accel_sensor = reinterpret_cast<Accel *>(getSensor("Accelerometer"_i));
        Gyro *gyro_sensor = reinterpret_cast<Gyro *>(getSensor("Gyroscope"_i));
        Barometer *baro = reinterpret_cast<Barometer *>(getSensor("Barometer"_i));

        // Determine which sensors are available for orientation
        bool hasIMU = false; // sensorOK(imu);
        bool hasAccelGyro = sensorOK(accel_sensor) && sensorOK(gyro_sensor);

        // Update orientation filter if available
        if (orientationFilter && (hasIMU || hasAccelGyro))
        {
            double dt = currentTime - lastTime;
            Vector<3> accel, gyro;

            // Get accel and gyro data from IMU or separate sensors
            if (hasIMU)
            {
                // accel = imu->getAcceleration();
                // gyro = imu->getAngularVelocity();
            }
            else
            {
                accel = accel_sensor->getAccel();
                gyro = gyro_sensor->getAngVel();
            }

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

        if (filter)
        {
            double *measurements = new double[filter->getMeasurementSize()];
            double *inputs = new double[filter->getInputSize()];

            // gps x y barometer z
            measurements[0] = sensorOK(gps) ? coordinates.x() - origin.x() : 0;
            measurements[1] = sensorOK(gps) ? coordinates.y() - origin.y() : 0;
            measurements[2] = baro->getASLAltM();

            // Earth-frame acceleration inputs (gravity already subtracted by orientation filter)
            // If orientation filter is available, use its earth-frame acceleration
            // Otherwise, leave inputs as zero
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

            stateVars[0] = position.x();
            stateVars[1] = position.y();
            stateVars[2] = position.z();
            stateVars[3] = velocity.x();
            stateVars[4] = velocity.y();
            stateVars[5] = velocity.z();

            filter->iterate(currentTime - lastTime, stateVars, measurements, inputs);
            // pos x, y, z, vel x, y, z
            position.x() = stateVars[0];
            position.y() = stateVars[1];
            position.z() = stateVars[2];
            velocity.x() = stateVars[3];
            velocity.y() = stateVars[4];
            velocity.z() = stateVars[5];
        }

        if (sensorOK(gps))
        {
            coordinates = gps->getHasFix() ? Vector<2>(gps->getPos().x(), gps->getPos().y()) : Vector<2>(0, 0);
            heading = gps->getHeading();
        }
        else
        {
            coordinates = Vector<2>(0, 0);
            heading = 0;
        }
    }

#pragma endregion Update Functions

#pragma region Helper Functions

    bool State::sensorOK(const Sensor *sensor) const
    {
        if (sensor && *sensor) // not nullptr and initialized
            return true;
        return false;
    }

    Sensor *State::getSensor(SensorType type, int sensorNum) const
    {
        for (int i = 0; i < maxNumSensors; i++)
            if (sensors[i] && type == sensors[i]->getType() && --sensorNum == 0)
                return sensors[i];
        return nullptr;
    }
#pragma endregion

} // namespace astra