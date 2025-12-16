#include <Arduino.h>
#include "../Math/Quaternion.h"
#include "Math/Vector.h"

namespace astra
{
    enum class MahonyMode
    {
        CALIBRATING,  // Continuous zeroing - accumulates calibration samples
        CORRECTING,   // Normal operation - gyro + accel correction
        GYRO_ONLY     // Locked mode - gyro integration only, no accel correction
    };

    class MahonyAHRS
    {
    public:
        MahonyAHRS(double Kp = 0.1, double Ki = 0.0005)
            : _Kp(Kp), _Ki(Ki), _biasX(0.0), _biasY(0.0), _biasZ(0.0), _q0(1.0, 0.0, 0.0, 0.0),
              _sumAccel(), _sumGyro(), _calibSamples(0), _initialized(false),
              _mode(MahonyMode::CALIBRATING)
        {
            _q = Quaternion(1.0, 0.0, 0.0, 0.0);
        }

        // Collect static samples before launch
        void calibrate(const Vector<3> &accel, const Vector<3> &gyro)
        {
            _sumAccel += accel;
            _sumGyro += gyro;
            _calibSamples++;
        }

        // Finalize calibration and align "down"
        void initialize()
        {
            if (_calibSamples == 0)
                return;
            Vector<3> avgA = _sumAccel * (1.0 / _calibSamples);
            Vector<3> avgG = _sumGyro * (1.0 / _calibSamples);

            // Set gyro bias
            _biasX = avgG.x();
            _biasY = avgG.y();
            _biasZ = avgG.z();

            // Align initial orientation so body Z matches gravity
            Vector<3> bodyDown = avgA;
            bodyDown.normalize();
            Vector<3> earthDown(0.0, 0.0, 1.0);
            Vector<3> axis = earthDown.cross(bodyDown);
            if (axis.magnitude() < 1e-6)
                axis = Vector<3>(1.0, 0.0, 0.0);
            else
                axis.normalize();
            double dot = bodyDown.dot(earthDown);
            dot = constrain(dot, -1.0, 1.0);
            double angle = acos(dot);
            _q = Quaternion();
            _q.fromAxisAngle(axis, angle);
            _q.normalize();

            // Store initial orientation for relative output
            _q0 = _q;
            _initialized = true;
        }

        /**
         * Update orientation using only accelerometer (for tilt) and gyro.
         * Behavior depends on current mode:
         * - CALIBRATING: accumulates calibration samples
         * - CORRECTING: normal gyro + accel fusion
         * - GYRO_ONLY: pure gyro integration (no accel correction)
         *
         * @param accel : Vector<3> accelerometer readings (m/s^2)
         * @param gyro  : Vector<3> gyroscope readings (rad/s)
         * @param dt    : time step (s)
         */
        void update(const Vector<3> &accel,
                    const Vector<3> &gyro,
                    double dt)
        {
            // Handle calibration mode
            if (_mode == MahonyMode::CALIBRATING)
            {
                calibrate(accel, gyro);
                return;
            }

            if (!_initialized)
                return;

            Vector<3> gCorr;

            if (_mode == MahonyMode::CORRECTING)
            {
                // 1. Normalize accelerometer measurement
                Vector<3> a = accel;
                a.normalize();

                // 2. Estimated gravity direction in body frame
                Vector<3> vAcc = _q.rotateVector(Vector<3>(0.0, 0.0, 1.0));

                // 3. Error between measured and predicted gravity
                Vector<3> e = a.cross(vAcc);

                // 4. Update gyroscope bias
                _biasX += _Ki * e.x() * dt;
                _biasY += _Ki * e.y() * dt;
                _biasZ += _Ki * e.z() * dt;

                // 5. Corrected angular velocity with accel feedback
                gCorr = Vector<3>(
                    -gyro.x() - _biasX + _Kp * e.x(),
                    -gyro.y() - _biasY + _Kp * e.y(),
                    -gyro.z() - _biasZ + _Kp * e.z());
            }
            else // GYRO_ONLY mode
            {
                // Pure gyro integration, no accel correction
                gCorr = Vector<3>(
                    -gyro.x() - _biasX,
                    -gyro.y() - _biasY,
                    -gyro.z() - _biasZ);
            }

            // 6. Quaternion derivative: omega ⊗ q
            Quaternion omega(0.0, gCorr.x(), gCorr.y(), gCorr.z());
            Quaternion qDot = omega * _q;
            qDot = qDot * 0.5;

            // 7. Integrate and normalize quaternion
            _q = _q + (qDot * dt);
            _q.normalize();
        }

        /**
         * Returns the orientation quaternion relative to initial alignment.
         * Identity quaternion at initialization.
         */
        Quaternion getQuaternion() const
        {
            Quaternion qInv = _q0.conjugate();
            return qInv * _q;
        }

        /**
         * Computes the acceleration in Earth frame, compensating gravity.
         * @param accel : raw accelerometer reading (m/s^2)
         * @return       : Vector<3> acceleration in Earth frame (X-east, Y-north, Z-up)
         */
        Vector<3> getEarthAcceleration(const Vector<3> &accel) const
        {
            // Rotate body accel into Earth frame: a_e = q_rel ⊗ [0, a] ⊗ q_rel*
            Quaternion qRel = getQuaternion();
            Vector<3> a = accel;
            // pure quaternion
            Quaternion qa(0.0, a.x(), a.y(), a.z());
            Quaternion rotated = qRel.conjugate() * qa * qRel;
            Vector<3> earthAcc(rotated.x(), rotated.y(), rotated.z());
            // subtract gravity
            earthAcc.z() -= 9.81;
            return earthAcc;
        }

        bool isInitialized() const { return _initialized; }

        /**
         * Set the filter operating mode.
         * - CALIBRATING: Continuous zeroing, accumulates samples
         * - CORRECTING: Normal gyro + accel fusion
         * - GYRO_ONLY: Gyro integration only, no accel correction
         */
        void setMode(MahonyMode mode)
        {
            _mode = mode;
        }

        MahonyMode getMode() const { return _mode; }

        /**
         * Reset calibration and reinitialize filter.
         * Useful for re-zeroing the orientation reference.
         */
        void reset()
        {
            _sumAccel = Vector<3>();
            _sumGyro = Vector<3>();
            _calibSamples = 0;
            _biasX = _biasY = _biasZ = 0.0;
            _q = Quaternion(1.0, 0.0, 0.0, 0.0);
            _q0 = Quaternion(1.0, 0.0, 0.0, 0.0);
            _initialized = false;
            _mode = MahonyMode::CALIBRATING;
        }

    private:
        double _Kp, _Ki;
        double _biasX, _biasY, _biasZ;
        Quaternion _q, _q0;
        Vector<3> _sumAccel, _sumGyro;
        int _calibSamples;
        bool _initialized;
        MahonyMode _mode;
    };
}
// Usage example:
// MahonyAHRS ahrs;
//
// On pad (pre-launch):
// ahrs.setMode(MahonyMode::CALIBRATING);  // default mode
// for(int i=0; i<200; ++i) { ahrs.update(readAccel(), readGyro(), dt); }
// ahrs.initialize();
// ahrs.setMode(MahonyMode::CORRECTING);
//
// During flight:
// ahrs.update(readAccel(), readGyro(), dt);
//
// At motor burnout (high-g complete):
// ahrs.setMode(MahonyMode::GYRO_ONLY);  // disable accel correction during coast/apogee
//
// Get orientation:
// Quaternion q = ahrs.getQuaternion();
// Vector<3> aEarth = ahrs.getEarthAcceleration(readAccel());
