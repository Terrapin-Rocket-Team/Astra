#ifndef IMU6DOF_H
#define IMU6DOF_H

#include "../Sensor.h"
#include "../Accel/Accel.h"
#include "../Gyro/Gyro.h"

namespace astra
{
    class IMU6DoF;

    /**
     * IMU6DoFAccel - Internal accelerometer component for IMU6DoF
     *
     * Provides an Accel interface that references the parent IMU's acceleration data.
     * This allows the IMU's accelerometer to be used anywhere an Accel* is expected.
     */
    class IMU6DoFAccel : public Accel
    {
        friend class IMU6DoF;

    public:
        IMU6DoFAccel(Vector<3> *accPtr, const char *name)
            : Accel(name), accRef(accPtr)
        {
            // Clear columns added by Accel constructor - parent IMU handles data reporting
            clearColumns();
        }
        virtual ~IMU6DoFAccel() {}

        Vector<3> getAccel() const override { return *accRef; }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }

    private:
        Vector<3> *accRef;
    };

    /**
     * IMU6DoFGyro - Internal gyroscope component for IMU6DoF
     *
     * Provides a Gyro interface that references the parent IMU's angular velocity data.
     */
    class IMU6DoFGyro : public Gyro
    {
        friend class IMU6DoF;

    public:
        IMU6DoFGyro(Vector<3> *angVelPtr, const char *name)
            : Gyro(name), angVelRef(angVelPtr)
        {
            // Clear columns added by Gyro constructor - parent IMU handles data reporting
            clearColumns();
        }
        virtual ~IMU6DoFGyro() {}

        Vector<3> getAngVel() const override { return *angVelRef; }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }

    private:
        Vector<3> *angVelRef;
    };

    /**
     * IMU6DoF - 6 Degrees of Freedom IMU (Accelerometer + Gyroscope)
     *
     * This class provides accelerometer and gyroscope data from a single physical sensor.
     * Uses composition - contains Accel and Gyro component objects that reference the
     * IMU's internal data.
     *
     * Usage:
     *   IMU6DoF* imu = ...;
     *   Accel* accel = imu->getAccelSensor();  // Get the contained Accel object
     *   Gyro* gyro = imu->getGyroSensor();     // Get the contained Gyro object
     *   Vector<3> accelData = imu->getAccel(); // Direct data access
     */
    class IMU6DoF : public Sensor
    {
    public:
        virtual ~IMU6DoF() {}

        // Get the contained accelerometer sensor object
        Accel *getAccelSensor() { return &accelComponent; }

        // Get the contained gyroscope sensor object
        Gyro *getGyroSensor() { return &gyroComponent; }

        // Direct data access
        virtual Vector<3> getAccel() const { return acc; }
        virtual Vector<3> getAngVel() const { return angVel; }

        // Override begin to also mark components as initialized
        bool begin() override
        {
            bool result = Sensor::begin();
            if (result)
            {
                accelComponent.initialized = true;
                gyroComponent.initialized = true;
            }
            return result;
        }

    protected:
        IMU6DoF(const char *name = "IMU6DoF")
            : Sensor("IMU6DoF", name),
            accelComponent(&acc, name),
              gyroComponent(&angVel, name)
        {
            // Add IMU's own data columns
            addColumn("%0.3f", &acc.x(), "Acc X (m/s^2)");
            addColumn("%0.3f", &acc.y(), "Acc Y (m/s^2)");
            addColumn("%0.3f", &acc.z(), "Acc Z (m/s^2)");
            addColumn("%0.3f", &angVel.x(), "Gyro X (rad/s)");
            addColumn("%0.3f", &angVel.y(), "Gyro Y (rad/s)");
            addColumn("%0.3f", &angVel.z(), "Gyro Z (rad/s)");
        }

        // Derived classes implement init() and read() to update acc and angVel
        virtual bool init() = 0;
        virtual bool read() = 0;

        Vector<3> acc = Vector<3>(0, 0, 0);
        Vector<3> angVel = Vector<3>(0, 0, 0);

    private:
        IMU6DoFAccel accelComponent;
        IMU6DoFGyro gyroComponent;
    };
}

#endif // IMU6DOF_H
