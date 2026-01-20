#ifndef STATE_H
#define STATE_H

#include "../Filters/Filter.h"
#include "../Filters/Mahony.h"

// TODO: Figoure out filter situation

// Include all the sensor classes
#include "../Sensors/Baro/Barometer.h"
#include "../Sensors/GPS/GPS.h"
#include "../Sensors/IMU/IMU.h"
#include "../Sensors/Accel/Accel.h"
#include "../Sensors/Gyro/Gyro.h"
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"

namespace astra
{
    class SensorManager;

    class State : public DataReporter
    {
    public:
        State(Filter *filter, MahonyAHRS *orientationFilter = nullptr);
        virtual ~State();

        // Returns false if any sensor failed to init. Check data log for failed sensor. Disables sensor if failed.
        virtual bool begin(SensorManager *sensorManager = nullptr);

        // Updates the state with the most recent sensor data. CurrentTime is the time in seconds since the uC was turned on. If not provided, the state will use the current time.
        virtual void update(double currentTime = -1);

        // Split update methods for different update rates - now receive primitive data
        virtual void updateOrientation(double *gyro, double *accel, double dt);
        virtual void predictState(double currentTime = -1);
        virtual void updateMeasurements(double gpsLat, double gpsLon, double gpsAlt, double baroAlt, bool hasGPS, bool hasBaro, double currentTime = -1);
        virtual void updatePositionVelocity(double lat, double lon, double heading, bool hasFix);

        // State Getters
        virtual Vector<3> getPosition() const { return position; }         // in m away from point of launch (inertial frame)
        virtual Vector<3> getVelocity() const { return velocity; }         // m/s (inertial frame)
        virtual Vector<3> getAcceleration() const { return acceleration; } // m/s/s (inertial frame)
        virtual Quaternion getOrientation() const { return orientation; }
        virtual Vector<2> getCoordinates() const { return coordinates; } // lat lon in decimal degrees
        virtual double getHeading() const { return heading; }            // degrees

        // Orientation filter control
        MahonyAHRS *getOrientationFilter() const { return orientationFilter; }
        void setOrientationFilterMode(MahonyMode mode)
        {
            if (orientationFilter)
                orientationFilter->setMode(mode);
        }

    protected:
        double currentTime; // in s since uC turned on
        double lastTime;

        // State variables
        Vector<3> position;     // in m from launch position
        Vector<3> velocity;     // in m/s
        Vector<3> acceleration; // in m/s^2
        Quaternion orientation; // in quaternion
        Vector<2> coordinates;  // in lat, lon
        double heading;         // in degrees
        Vector<3> origin;       // in lat, lon, alt

        // Kalman Filter settings
        Filter *filter;
        double *stateVars = nullptr;

        // Orientation filter (Mahony AHRS)
        MahonyAHRS *orientationFilter;

        bool initialized = false;
    };
}
#endif // STATE_H