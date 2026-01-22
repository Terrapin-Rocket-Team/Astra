#ifndef UNIT_TEST_SENSORS_H
#define UNIT_TEST_SENSORS_H

#include "../../src/Sensors/Accel/Accel.h"
#include "../../src/Sensors/Gyro/Gyro.h"
#include "../../src/Sensors/Mag/Mag.h"
#include "../../src/Sensors/Baro/Barometer.h"
#include "../../src/Sensors/GPS/GPS.h"
#include "../../src/Sensors/IMU/IMU6DoF.h"
#include "../../src/Sensors/IMU/IMU9DoF.h"

using namespace astra;

class FakeAccel : public Accel
{
public:
    FakeAccel() : Accel("FakeAccel")
    {
        initialized = true;
    }
    ~FakeAccel() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void setAccel(double x, double y, double z)
    {
        acc = Vector<3>(x, y, z);
    }
};

class FakeGyro : public Gyro
{
public:
    FakeGyro() : Gyro("FakeGyro")
    {
        initialized = true;
    }
    ~FakeGyro() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void setAngVel(double x, double y, double z)
    {
        angVel = Vector<3>(x, y, z);
    }
};

class FakeMag : public Mag
{
public:
    FakeMag() : Mag("FakeMag")
    {
        initialized = true;
    }
    ~FakeMag() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void setMag(double x, double y, double z)
    {
        mag = Vector<3>(x, y, z);
    }
};

class FakeBarometer : public Barometer
{
public:
    FakeBarometer() : Barometer()
    {
        initialized = true;
        setName("FakeBarometer");
    }
    ~FakeBarometer() {}

    bool read() override
    {
        pressure = fakeP;
        temp = fakeT;
        return true;
    }
    void set(double p, double t)
    {
        pressure = fakeP = p;
        temp = fakeT = t;
    }

    bool init() override
    {
        return initialized;
    }
    double fakeP = 0;
    double fakeT = 0;
};

class FakeGPS : public GPS
{
public:
    FakeGPS() : GPS()
    {
        initialized = true;
        setName("FakeGPS");
    }
    ~FakeGPS() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void set(double lat, double lon, double alt)
    {
        position.x() = lat;
        position.y() = lon;
        position.z() = alt;
    }
    void setHeading(double h)
    {
        heading = h;
    }
    void setDateTime(int y, int m, int d, int h, int mm, int s)
    {
        year = y;
        month = m;
        day = d;
        hr = h;
        min = mm;
        sec = s;
        snprintf(tod, 12, "%02d:%02d:%02d", hr, min, sec);
    }
    void setHasFirstFix(bool fix)
    {
        hasFix = fix;
        if (fix)
            fixQual = 4;
        else
            fixQual = 0;
    }
    void setFixQual(int qual)
    {
        fixQual = qual;
    }
};

class FakeIMU6DoF : public IMU6DoF
{
public:
    FakeIMU6DoF() : IMU6DoF("FakeIMU6DoF")
    {
        initialized = true;
    }
    ~FakeIMU6DoF() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void setAccel(double x, double y, double z)
    {
        acc = Vector<3>(x, y, z);
    }
    void setAngVel(double x, double y, double z)
    {
        angVel = Vector<3>(x, y, z);
    }
};

class FakeIMU9DoF : public IMU9DoF
{
public:
    FakeIMU9DoF() : IMU9DoF("FakeIMU9DoF")
    {
        initialized = true;
    }
    ~FakeIMU9DoF() {}

    bool read() override { return true; }
    bool init() override { return initialized; }

    void setAccel(double x, double y, double z)
    {
        acc = Vector<3>(x, y, z);
    }
    void setAngVel(double x, double y, double z)
    {
        angVel = Vector<3>(x, y, z);
    }
    void setMag(double x, double y, double z)
    {
        mag = Vector<3>(x, y, z);
    }
};

#endif // UNIT_TEST_SENSORS_H
