#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include "../Math/Quaternion.h"
#include "Math/Vector.h"

namespace astra
{
    enum class MahonyMode
    {
        CALIBRATING, // Pad behavior: continuously snaps orientation to gravity, accumulates bias
        CORRECTING,  // Flight behavior: locks board alignment, applies gyro+accel fusion
        GYRO_ONLY    // Coast behavior: locks board alignment, gyro integration only
    };

    class MahonyAHRS
    {
    public:
        MahonyAHRS(double Kp = 0.1, double Ki = 0.0005)
            : _Kp(Kp), _Ki(Ki), _biasX(0.0), _biasY(0.0), _biasZ(0.0),
              _q(1.0, 0.0, 0.0, 0.0), _q0(1.0, 0.0, 0.0, 0.0),
              _sumAccel(), _sumGyro(), _calibSamples(0),
              _orientationValid(false), _frameLocked(false),
              _mode(MahonyMode::CALIBRATING), _boardToBody(1.0, 0.0, 0.0, 0.0)
        {
        }

        /**
         * Computes the gyro bias from accumulated samples.
         * Call this just before liftoff or when transitioning out of CALIBRATING.
         * Note: This does NOT lock the frame or change the mode.
         */
        void finalizeCalibration()
        {
            if (_calibSamples > 0)
            {
                // Compute average gyro to get bias
                Vector<3> avgGyro = _sumGyro * (1.0 / _calibSamples);

                // We rotate the average gyro into the body frame using the *current*
                // board alignment. This assumes the rocket is stationary during this call.
                Vector<3> bodyGyro = _boardToBody.rotateVector(avgGyro);

                _biasX = bodyGyro.x();
                _biasY = bodyGyro.y();
                _biasZ = bodyGyro.z();
            }
        }

        /**
         * Lock the reference frame (tare) at current orientation.
         * Call this at liftoff to establish "zero" orientation.
         */
        void lockFrame()
        {
            if (!_orientationValid)
                return;
            _q0 = _q;
            _frameLocked = true;
        }

        /**
         * Main filter update.
         */
        void update(const Vector<3> &accel,
                    const Vector<3> &gyro,
                    double dt)
        {
            // --- MODE: CALIBRATING (ON PAD) ---
            // Continuously determine "Up" and calculate tilt.
            if (_mode == MahonyMode::CALIBRATING)
            {
                // --- MODIFICATION START: 1G Guard ---
                // Only snap to gravity if the acceleration vector is close to 1G.
                // If the rocket is being handled, bumped, or shaken, the magnitude will differ.
                double accMag = accel.magnitude();

                // Tolerance: +/- 2.0 m/s^2 (approx 0.2G)
                // If measuring < 7.8 or > 11.8, ignore this update.
                if (fabs(accMag - 9.81) > 2.0)
                {
                    return;
                }
                // --- MODIFICATION END ---

                // 1. Accumulate calibration samples
                _sumAccel += accel;
                _sumGyro += gyro;
                _calibSamples++;

                // 2. Dynamic Alignment: Continuously snap board->body to nearest 90°
                _boardToBody = computeBoardToBodyRotation(accel);

                // 3. Transform accel to aligned body frame
                Vector<3> bodyAccel = _boardToBody.rotateVector(accel);

                // 4. Compute absolute tilt (residual rotation)
                // In ENU Body frame, gravity should be -Z (0, 0, -1).
                // We calculate rotation from Earth Down (-Z) to Measured Down.
                Vector<3> bodyDown = bodyAccel;
                bodyDown.normalize();
                Vector<3> earthDown(0.0, 0.0, -1.0);

                Vector<3> axis = earthDown.cross(bodyDown);
                double axisMag = axis.magnitude();

                if (axisMag < 1e-6)
                {
                    // Already aligned or perfectly opposite
                    if (bodyDown.dot(earthDown) > 0)
                    {
                        _q = Quaternion(1.0, 0.0, 0.0, 0.0);
                    }
                    else
                    {
                        _q = Quaternion(0.0, 1.0, 0.0, 0.0); // 180 flip
                    }
                }
                else
                {
                    axis = axis * (1.0 / axisMag); // Normalize manually
                    double dot = constrain(bodyDown.dot(earthDown), -1.0, 1.0);
                    double angle = acos(dot);
                    _q.fromAxisAngle(axis, angle);
                    _q.normalize();
                }

                // Mark filter as ready immediately
                _orientationValid = true;
                return;
            }

            // --- MODE: CORRECTING / GYRO_ONLY (FLIGHT) ---
            if (!_orientationValid)
                return;

            // Apply the LAST calculated board->body rotation (Locked)
            Vector<3> bodyAccel = _boardToBody.rotateVector(accel);
            Vector<3> bodyGyro = _boardToBody.rotateVector(gyro);

            Vector<3> gCorr;

            if (_mode == MahonyMode::CORRECTING)
            {
                // Standard Mahony Fusion
                Vector<3> a = bodyAccel;
                a.normalize();

                // Estimated gravity direction in body frame
                Vector<3> vAcc = _q.rotateVector(Vector<3>(0.0, 0.0, -1.0));

                // Error
                Vector<3> e = vAcc.cross(a);

                // Integral feedback
                _biasX += _Ki * e.x() * dt;
                _biasY += _Ki * e.y() * dt;
                _biasZ += _Ki * e.z() * dt;

                // Proportional feedback
                gCorr = Vector<3>(
                    -bodyGyro.x() + _biasX + _Kp * e.x(),
                    -bodyGyro.y() + _biasY + _Kp * e.y(),
                    -bodyGyro.z() + _biasZ + _Kp * e.z());
            }
            else // GYRO_ONLY
            {
                gCorr = Vector<3>(
                    -bodyGyro.x() + _biasX,
                    -bodyGyro.y() + _biasY,
                    -bodyGyro.z() + _biasZ);
            }

            // Integrate Quaternion
            Quaternion omega(0.0, gCorr.x(), gCorr.y(), gCorr.z());
            Quaternion qDot = omega * _q;
            qDot = qDot * 0.5;

            _q = _q + (qDot * dt);
            _q.normalize();
        }

        /**
         * Returns true if the filter has processed at least one sample
         * and can provide a valid quaternion.
         */
        bool isReady() const { return _orientationValid; }

        bool isFrameLocked() const { return _frameLocked; }

        MahonyMode getMode() const { return _mode; }

        void setMode(MahonyMode mode) { _mode = mode; }

        /**
         * Returns orientation quaternion.
         * If frame is locked (flight), returns orientation relative to launch.
         * If frame unlocked (pad), returns absolute tilt relative to vertical.
         */
        Quaternion getQuaternion() const
        {
            if (!_orientationValid)
                return Quaternion(1.0, 0.0, 0.0, 0.0);

            if (_mode == MahonyMode::CALIBRATING)
            {
                return _q;
            }
            else
            {
                Quaternion qInv = _q0.conjugate();
                return qInv * _q;
            }
        }

        Vector<3> getEarthAcceleration(const Vector<3> &accel) const
        {
            Vector<3> bodyAccel = _boardToBody.rotateVector(accel);
            Quaternion qInv = _q.conjugate();
            Vector<3> earthAcc = qInv.rotateVector(bodyAccel);
            earthAcc.z() += 9.81; // Remove gravity
            return earthAcc;
        }

        void reset()
        {
            _sumAccel = Vector<3>();
            _sumGyro = Vector<3>();
            _calibSamples = 0;
            _biasX = _biasY = _biasZ = 0.0;
            _q = Quaternion(1.0, 0.0, 0.0, 0.0);
            _q0 = Quaternion(1.0, 0.0, 0.0, 0.0);
            _boardToBody = Quaternion(1.0, 0.0, 0.0, 0.0);
            _orientationValid = false;
            _frameLocked = false;
            _mode = MahonyMode::CALIBRATING;
        }

    private:
        // Logic to snap board frame to body frame (ENU) based on gravity vector
        Quaternion computeBoardToBodyRotation(const Vector<3> &accel) const
        {
            double absX = fabs(accel.x());
            double absY = fabs(accel.y());
            double absZ = fabs(accel.z());

            if (absZ >= absX && absZ >= absY)
            {
                if (accel.z() < 0)
                    return Quaternion(1.0, 0.0, 0.0, 0.0);
                else
                    return Quaternion(0.0, 1.0, 0.0, 0.0);
            }
            else if (absX >= absY && absX >= absZ)
            {
                if (accel.x() < 0)
                    return Quaternion(0.707107, 0.0, 0.707107, 0.0);
                else
                    return Quaternion(0.707107, 0.0, -0.707107, 0.0);
            }
            else
            {
                if (accel.y() < 0)
                    return Quaternion(0.707107, -0.707107, 0.0, 0.0);
                else
                    return Quaternion(0.707107, 0.707107, 0.0, 0.0);
            }
        }

        double _Kp, _Ki;
        double _biasX, _biasY, _biasZ;

        Quaternion _q;           // Current Orientation (Earth->Body)
        Quaternion _q0;          // Reference Orientation (Tare at launch)
        Quaternion _boardToBody; // Rotation from PCB frame to Rocket Body frame

        Vector<3> _sumAccel, _sumGyro;
        int _calibSamples;

        bool _orientationValid; // True if we have computed at least one orientation estimate
        bool _frameLocked;      // True if lockFrame() has been called
        MahonyMode _mode;
    };
}

#endif // MAHONY_H