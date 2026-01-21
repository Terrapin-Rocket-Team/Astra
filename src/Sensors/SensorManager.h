#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "Sensor.h"
#include "../Math/Vector.h"

namespace astra
{
    // Forward declarations for sensor types
    class Accel;
    class Gyro;
    class Mag;
    class GPS;
    class Barometer;
    class VoltageSensor;

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

        // Generic sensor access
        Sensor *getSensor(uint32_t type, int sensorNum = 1) const;
        Sensor **getSensors() { return sensors; }
        int getCount() const { return numSensors; }

        // Typed sensor getters - return nullptr if not found
        // Use sensorNum to get the Nth sensor of that type (1-indexed)
        // Note: getAccel/getGyro/getMag will fall back to IMU types if standalone not found
        Accel *getAccel(int sensorNum = 1) const;
        Gyro *getGyro(int sensorNum = 1) const;
        Mag *getMag(int sensorNum = 1) const;
        GPS *getGPS(int sensorNum = 1) const;
        Barometer *getBaro(int sensorNum = 1) const;
        VoltageSensor *getVoltageSensor(int sensorNum = 1) const;

    private:
        Sensor *sensors[MAX_SENSORS];
        int numSensors = 0;
    };

} // namespace astra

#endif // SENSOR_MANAGER_H
