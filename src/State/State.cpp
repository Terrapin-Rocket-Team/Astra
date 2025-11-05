#include "State.h"
#include <Arduino.h>
#pragma region Constructor and Destructor

namespace astra
{
    State::State(Sensor **sensors, int numSensors, Filter *filter) : DataReporter("State")
    {
        lastTime = 0;
        currentTime = 0;
        this->maxNumSensors = numSensors;
        this->sensors = sensors;
        this->filter = filter;

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
        IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
        Barometer *baro = reinterpret_cast<Barometer *>(getSensor("Barometer"_i));

        if (filter)
        {
            double *measurements = new double[filter->getMeasurementSize()];
            double *inputs = new double[filter->getInputSize()];

            // gps x y barometer z
            measurements[0] = sensorOK(gps) ? coordinates.x() - origin.x() : 0;
            measurements[1] = sensorOK(gps) ? coordinates.y() - origin.y() : 0;
            measurements[2] = baro->getASLAltM();

            // imu x y z
            // inputs[0] = acceleration.x() = imu->getAccelerationGlobal().x();
            // inputs[1] = acceleration.y() = imu->getAccelerationGlobal().y();
            // inputs[2] = acceleration.z() = imu->getAccelerationGlobal().z();

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