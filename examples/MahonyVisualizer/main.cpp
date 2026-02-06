#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include "Filters/Mahony.h"
#include "Sensors/HW/IMU/BMI088.h"
#include "Sensors/HW/Mag/LIS3MDL.h"
#include "Sensors/MountingTransform.h"

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr float GRAVITY_MSS = 9.81f;
static constexpr float ACCEL_TRUST_BAND = 1.0f; // m/s^2 around 1g
static constexpr uint32_t OUTPUT_INTERVAL_US = 50000; // 20 Hz
static constexpr uint32_t RAW_INTERVAL_US = 200000; // 5 Hz
static constexpr uint32_t DEBUG_INTERVAL_US = 500000; // 2 Hz
static constexpr uint16_t MAG_CAL_SAMPLES = 400;
static astra::BMI088 imu;
static astra::LIS3MDL mag;
static astra::MahonyAHRS ahrs(0.5, 0.0);

static uint32_t lastMicros = 0;
static uint32_t lastOutput = 0;
static uint32_t lastRaw = 0;
static uint32_t lastDebug = 0;
static uint16_t magSamples = 0;
static bool magCalibrated = false;
void setup()
{
    Serial.begin(SERIAL_BAUD);
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000)
    {
        delay(10);
    }

    Wire.begin();
    delay(10);

    imu.setUpdateRate(200);
    mag.setUpdateRate(100);
    // IMU Z axis is upside down; rotate 180 degrees about X so Z (and Y) flip.
    // If X is the axis that's inverted instead, use FLIP_XZ.
    imu.setMountingOrientation(astra::MountingOrientation::FLIP_XZ);
    // LIS3MDL is mounted 180 degrees around Z on this board.
    mag.setMountingOrientation(astra::MountingOrientation::FLIP_XY);

    int imuErr = imu.begin();
    int magErr = mag.begin();
    if (imuErr != 0)
    {
        Serial.println("ERR,IMU_INIT");
    }
    if (magErr != 0)
    {
        Serial.println("ERR,MAG_INIT");
    }

    Serial.println("Mahony Visualizer Ready");
    Serial.println("Format: Q,w,x,y,z");
    Serial.println("Rotate board in 3D to calibrate mag...");
}

void loop()
{
    const uint32_t now = micros();
    if (lastMicros == 0)
    {
        lastMicros = now;
        return;
    }

    const float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;

    if (dt <= 0.0f || dt > 0.2f)
    {
        return;
    }

    if (imu.update() != 0)
    {
        return;
    }
    if (mag.update() != 0)
    {
        return;
    }
    const astra::Vector<3> accel = imu.getAccel();
    const astra::Vector<3> gyro = imu.getAngVel();
    const astra::Vector<3> magField = mag.getMag();

    if (!magCalibrated)
    {
        ahrs.collectMagCalibrationSample(magField);
        magSamples++;
        if (magSamples >= MAG_CAL_SAMPLES)
        {
            ahrs.finalizeMagCalibration();
            magCalibrated = ahrs.isMagCalibrated();
            Serial.println(magCalibrated ? "CAL,OK" : "CAL,SKIP");
        }
    }

    const float accelMag = accel.magnitude();
    const bool accelOk = fabs(accelMag - GRAVITY_MSS) < ACCEL_TRUST_BAND;

    if (accelOk)
    {
        ahrs.update(accel, gyro, magField, dt);
    }
    else
    {
        ahrs.update(gyro, dt);
    }

    if (now - lastOutput >= OUTPUT_INTERVAL_US)
    {
        lastOutput = now;
        const astra::Quaternion q = ahrs.getQuaternion();

        Serial.print("Q,");
        Serial.print(q.w(), 6);
        Serial.print(",");
        Serial.print(q.x(), 6);
        Serial.print(",");
        Serial.print(q.y(), 6);
        Serial.print(",");
        Serial.print(q.z(), 6);
        Serial.println();
    }

    if (now - lastRaw >= RAW_INTERVAL_US)
    {
        lastRaw = now;
        Serial.print("RAW,");
        Serial.print(accel.x(), 3);
        Serial.print(",");
        Serial.print(accel.y(), 3);
        Serial.print(",");
        Serial.print(accel.z(), 3);
        Serial.print(",");
        Serial.print(gyro.x(), 3);
        Serial.print(",");
        Serial.print(gyro.y(), 3);
        Serial.print(",");
        Serial.print(gyro.z(), 3);
        Serial.print(",");
        Serial.print(magField.x(), 3);
        Serial.print(",");
        Serial.print(magField.y(), 3);
        Serial.print(",");
        Serial.print(magField.z(), 3);
        Serial.println();
    }

    if (now - lastDebug >= DEBUG_INTERVAL_US)
    {
        lastDebug = now;
        const astra::Quaternion q = ahrs.getQuaternion();
        astra::Vector<3> a = accel;
        a.normalize();
        astra::Quaternion qConj = q.conjugate();
        astra::Vector<3> vAcc = qConj.rotateVector(astra::Vector<3>(0.0, 0.0, 1.0));
        astra::Vector<3> eAcc = a.cross(vAcc);

        Serial.print("DBG,");
        Serial.print(accelMag, 3);
        Serial.print(",");
        Serial.print(accelOk ? 1 : 0);
        Serial.print(",");
        Serial.print(a.x(), 3);
        Serial.print(",");
        Serial.print(a.y(), 3);
        Serial.print(",");
        Serial.print(a.z(), 3);
        Serial.print(",");
        Serial.print(vAcc.x(), 3);
        Serial.print(",");
        Serial.print(vAcc.y(), 3);
        Serial.print(",");
        Serial.print(vAcc.z(), 3);
        Serial.print(",");
        Serial.print(eAcc.x(), 3);
        Serial.print(",");
        Serial.print(eAcc.y(), 3);
        Serial.print(",");
        Serial.print(eAcc.z(), 3);
        Serial.println();
    }
}
