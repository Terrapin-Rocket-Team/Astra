#ifndef IMU9DOF_H
#define IMU9DOF_H

#include "../Accel/Accel.h"
#include "../Gyro/Gyro.h"
#include "../Mag/Mag.h"

namespace astra
{
    class IMU9DoF;

    /**
     * IMU9DoFAccel - Internal accelerometer component for IMU9DoF
     */
    class IMU9DoFAccel : public Accel
    {
        friend class IMU9DoF;

    public:
        IMU9DoFAccel(Vector<3> *accPtr, const char *name)
            : Accel(name), accRef(accPtr)
        {
            clearColumns();
        }
        virtual ~IMU9DoFAccel() {}

        Vector<3> getAccel() const override { return orient.transform(*accRef); }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }

    private:
        Vector<3> *accRef;
    };

    /**
     * IMU9DoFGyro - Internal gyroscope component for IMU9DoF
     */
    class IMU9DoFGyro : public Gyro
    {
        friend class IMU9DoF;

    public:
        IMU9DoFGyro(Vector<3> *angVelPtr, const char *name)
            : Gyro(name), angVelRef(angVelPtr)
        {
            clearColumns();
        }
        virtual ~IMU9DoFGyro() {}

        Vector<3> getAngVel() const override { return orient.transform(*angVelRef); }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }

    private:
        Vector<3> *angVelRef;
    };

    /**
     * IMU9DoFMag - Internal magnetometer component for IMU9DoF
     */
    class IMU9DoFMag : public Mag
    {
        friend class IMU9DoF;

    public:
        IMU9DoFMag(Vector<3> *magPtr, const char *name)
            : Mag(name), magRef(magPtr)
        {
            clearColumns();
        }
        virtual ~IMU9DoFMag()
        {
            setUpdateRate(100);
        }

        Vector<3> getMag() const override { return orient.transform(*magRef); }

    protected:
        bool init() override { return true; }
        bool read() override { return true; }

    private:
        Vector<3> *magRef;
    };

    /**
     * IMU9DoF - 9 Degrees of Freedom IMU (Accelerometer + Gyroscope + Magnetometer)
     *
     * This class provides accelerometer, gyroscope, and magnetometer data from a single physical sensor.
     * Uses composition - contains Accel, Gyro, and Mag component objects that reference the
     * IMU's internal data.
     *
     * Usage:
     *   IMU9DoF* imu = ...;
     *   Accel* accel = imu->getAccelSensor();  // Get the contained Accel object
     *   Gyro* gyro = imu->getGyroSensor();     // Get the contained Gyro object
     *   Mag* mag = imu->getMagSensor();        // Get the contained Mag object
     *   Vector<3> accelData = imu->getAccel(); // Direct data access
     */
    class IMU9DoF : public RotatableSensor
    {
    public:
        virtual ~IMU9DoF() {}

        // Get the contained accelerometer sensor object
        Accel *getAccelSensor() { return &accelComponent; }

        // Get the contained gyroscope sensor object
        Gyro *getGyroSensor() { return &gyroComponent; }

        // Get the contained magnetometer sensor object
        Mag *getMagSensor() { return &magComponent; }

        // Direct data access
        virtual Vector<3> getAccel() const { return orient.transform(acc); }
        virtual Vector<3> getAngVel() const { return orient.transform(angVel); }
        virtual Vector<3> getMag() const { return orient.transform(mag); }


        virtual void setMountingOrientation(MountingOrientation orientation) override
        {
            RotatableSensor::setMountingOrientation(orientation);
            accelComponent.setMountingOrientation(orientation);
            gyroComponent.setMountingOrientation(orientation);
            magComponent.setMountingOrientation(orientation);
        }

        // Override begin to also mark components as initialized
        bool begin() override
        {
            bool result = Sensor::begin();
            if (result)
            {
                accelComponent.initialized = true;
                gyroComponent.initialized = true;
                magComponent.initialized = true;
            }
            return result;
        }

    protected:
        IMU9DoF(const char *name = "IMU9DoF")
            : RotatableSensor(name),
            accelComponent(&acc, name),
              gyroComponent(&angVel, name),
              magComponent(&mag, name)
        {
            // Add IMU's own data columns
            addColumn("%0.3f", &acc.x(), "Acc X (m/s^2)");
            addColumn("%0.3f", &acc.y(), "Acc Y (m/s^2)");
            addColumn("%0.3f", &acc.z(), "Acc Z (m/s^2)");
            addColumn("%0.3f", &angVel.x(), "Gyro X (rad/s)");
            addColumn("%0.3f", &angVel.y(), "Gyro Y (rad/s)");
            addColumn("%0.3f", &angVel.z(), "Gyro Z (rad/s)");
            addColumn("%0.3f", &mag.x(), "Mag X (uT)");
            addColumn("%0.3f", &mag.y(), "Mag Y (uT)");
            addColumn("%0.3f", &mag.z(), "Mag Z (uT)");
        }

        // Derived classes implement init() and read() to update acc, angVel, and mag
        virtual bool init() = 0;
        virtual bool read() = 0;

        Vector<3> acc = Vector<3>(0, 0, 0);
        Vector<3> angVel = Vector<3>(0, 0, 0);
        Vector<3> mag = Vector<3>(0, 0, 0);

    private:
        IMU9DoFAccel accelComponent;
        IMU9DoFGyro gyroComponent;
        IMU9DoFMag magComponent;
    };
}

#endif // IMU9DOF_H
