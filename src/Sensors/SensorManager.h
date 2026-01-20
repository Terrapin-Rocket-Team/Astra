#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "Sensor.h"
#include "../Math/Vector.h"

namespace astra
{
    // Maximum number of sensors that can be managed
    #define MAX_SENSORS 20

    class SensorManager
    {
    public:
        SensorManager();

        // Sensor lifecycle management
        void addSensor(Sensor *sensor);
        void setSensors(Sensor **sensors, int count);
        bool initAll();
        void updateAll();

        // Sensor access
        Sensor *getSensor(uint32_t type, int sensorNum = 1) const;
        Sensor **getSensors() { return sensors; }
        int getCount() const { return numSensors; }

        // Typed data extraction for State
        // Returns true if data was successfully retrieved
        // These methods automatically check for combo IMU sensors (IMU6DoF, IMU9DoF)
        bool getIMUData(double *gyro, double *accel, double *mag = nullptr);
        bool getAccelData(double *accel);
        bool getGyroData(double *gyro);
        bool getMagData(double *mag);
        bool getGPSData(double *lat, double *lon, double *alt);
        bool getGPSVelocity(double *vn, double *ve, double *vd);
        bool getGPSHeading(double *heading);
        bool getGPSHasFix(bool *hasFix);
        bool getBaroData(double *pressure, double *temp = nullptr);
        bool getBaroAltitude(double *altM);

    private:
        Sensor *sensors[MAX_SENSORS];
        int numSensors = 0;
    };

} // namespace astra

#endif // SENSOR_MANAGER_H
