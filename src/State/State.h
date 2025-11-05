#ifndef STATE_H
#define STATE_H

#include "../Filters/Filter.h"

// Include all the sensor classes
#include "../Sensors/Baro/Barometer.h"
#include "../Sensors/GPS/GPS.h"
#include "../Sensors/IMU/IMU.h"
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"

namespace astra
{
    class State : public DataReporter
    {
    public:
        State(Sensor **sensors, int numSensors, Filter *filter);
        virtual ~State();

        // Returns false if any sensor failed to init. Check data log for failed sensor. Disables sensor if failed.
        // useBiasCorrection: whether or not to use bias correction. If true, the override class must call sensor.useBiasCorrection(false) upon liftoff to disable bias correction.
        virtual bool begin();

        // Updates the state with the most recent sensor data. CurrentTime is the time in seconds since the uC was turned on. If not provided, the state will use the current time.
        virtual void update(double currentTime = -1);

        // sensor functions
        virtual Sensor *getSensor(SensorType type, int sensorNum = 1) const; // get a sensor of a certain type. 1 indexed. i.e. getSensor(GPS, 1) gets the first GPS sensor.

        // State Getters
        virtual Vector<3> getPosition() const { return position; } // in m away from point of launch
        virtual Vector<3> getVelocity() const { return velocity; }
        virtual Vector<3> getAcceleration() const { return acceleration; }
        virtual Quaternion getOrientation() const { return orientation; }
        virtual Vector<2> getCoordinates() const { return coordinates; } // lat long in decimal degrees
        virtual double getHeading() const { return heading; }
        virtual int getNumMaxSensors() const { return maxNumSensors; } // how many sensors were passed in the constructor
        virtual Sensor **getSensors() const { return sensors; }
        bool sensorOK(const Sensor *sensor) const;

    protected:
        double currentTime; // in s since uC turned on
        double lastTime;

        int maxNumSensors; // how many sensors were passed in the constructor
        Sensor **sensors;
        int numSensors; // how many sensors are actually enabled

        // ----

        virtual void updateSensors();
        virtual void updateVariables();

        // ----

        // State variables
        Vector<3> position;     // in m from launch position
        Vector<3> velocity;     // in m/s
        Vector<3> acceleration; // in m/s^2
        Quaternion orientation; // in quaternion
        Vector<2> coordinates;  // in lat, lon
        double heading;         // in degrees
        Vector<3> origin;      // in lat, lon, alt

        // Kalman Filter settings
        Filter *filter;
        double *stateVars = nullptr;

        bool initialized = false;
    };
}
#endif // STATE_H