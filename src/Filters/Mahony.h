#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include <ArduinoEigen.h> // Required for Mag Calibration
#include <vector>
#include "../Math/Quaternion.h"
#include "Math/Vector.h"

namespace astra
{
    enum class MahonyMode
    {
        CALIBRATING, // Pad behavior: continuously snaps orientation to gravity, accumulates bias
        CORRECTING,  // Flight behavior: locks board alignment, applies gyro+accel(+mag) fusion
        GYRO_ONLY    // Coast behavior: locks board alignment, gyro integration only
    };

    class MahonyAHRS
    {
    public:
        MahonyAHRS(double Kp = 0.1, double Ki = 0.0005)
            : _Kp(Kp), _Ki(Ki), 
              _biasX(0.0), _biasY(0.0), _biasZ(0.0),
              _q(1.0, 0.0, 0.0, 0.0), _q0(1.0, 0.0, 0.0, 0.0),
              _sumAccel(), _sumGyro(), _calibSamples(0),
              _orientationValid(false), _frameLocked(false),
              _mode(MahonyMode::CALIBRATING), 
              _boardToBody(1.0, 0.0, 0.0, 0.0),
              _magCalibrated(false), _hardIron(), _softIron{ {1,0,0}, {0,1,0}, {0,0,1} }
        {
        }

        /**
         * Add a magnetometer sample to the calibration buffer.
         * Call this repeatedly while rotating the board in 3D space (The "Mag Dance").
         * Do not call this while the rocket is sitting stationary on the pad.
         */
        void collectMagCalibrationSample(const Vector<3> &mag)
        {
            _magData.push_back(mag);
        }

        /**
         * Computes gyro bias from samples and solves Magnetometer Ellipsoid fit.
         * Call this just before liftoff or when transitioning out of CALIBRATING.
         */
        void finalizeCalibration()
        {
            // 1. Gyro Bias
            if (_calibSamples > 0)
            {
                Vector<3> avgGyro = _sumGyro * (1.0 / _calibSamples);
                Vector<3> bodyGyro = _boardToBody.rotateVector(avgGyro);

                _biasX = bodyGyro.x();
                _biasY = bodyGyro.y();
                _biasZ = bodyGyro.z();
            }

            // 2. Magnetometer Calibration (Ellipsoid Fit)
            if (_magData.size() > 100)
            {
                computeMagCalibration();
                _magData.clear();
                _magData.shrink_to_fit();
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

        // --------------------------------------------------------------------------
        // 9-DOF Update (Accel + Gyro + Mag)
        // --------------------------------------------------------------------------
        void update(const Vector<3> &accel,
                    const Vector<3> &gyro,
                    const Vector<3> &mag,
                    double dt)
        {
            // If mag isn't calibrated, fall back to 6-DOF
            if (!_magCalibrated)
            {
                update(accel, gyro, dt);
                return;
            }

            // --- MODE: CALIBRATING (ON PAD) ---
            if (_mode == MahonyMode::CALIBRATING)
            {
                // In calibrating mode, we primarily trust gravity for tilt.
                // We use the 6-DOF update for pad alignment logic.
                update(accel, gyro, dt); 
                return;
            }

            // --- MODE: FLIGHT ---
            if (!_orientationValid) return;

            // 1. Rotate sensors to body frame
            Vector<3> bodyAccel = _boardToBody.rotateVector(accel);
            Vector<3> bodyGyro  = _boardToBody.rotateVector(gyro);
            
            // 2. Calibrate and rotate Magnetometer
            // Note: We calibrate the raw sensor data first (remove soft/hard iron), 
            // THEN rotate it into the rocket body frame.
            Vector<3> calMag = calibrateMag(mag);
            Vector<3> bodyMag = _boardToBody.rotateVector(calMag);

            Vector<3> gCorr;

            if (_mode == MahonyMode::CORRECTING)
            {
                // Normalize measurements
                Vector<3> a = bodyAccel;
                a.normalize();
                
                Vector<3> m = bodyMag;
                m.normalize();

                // Estimated gravity (vAcc) and magnetic field (vMag) direction in body frame
                // q conjugate rotates Earth->Body
                Quaternion qConjugate = _q.conjugate();
                Vector<3> vAcc = qConjugate.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Gravity is +Z in ENU
                Vector<3> vMag = qConjugate.rotateVector(Vector<3>(0.0, 1.0, 0.0)); // Estimate North is +Y in ENU

                // -- Accel Error (Tilt) --
                Vector<3> eAcc = a.cross(vAcc);

                // -- Mag Error (Yaw) --
                // Project measured mag onto horizontal plane to remove dip inclination issues
                // m_horiz = m - (m . vAcc) * vAcc
                Vector<3> m_horiz = m - (vAcc * m.dot(vAcc));
                m_horiz.normalize();

                // Project estimated north onto horizontal plane
                Vector<3> vMag_horiz = vMag - (vAcc * vMag.dot(vAcc));
                vMag_horiz.normalize();

                Vector<3> eMag = m_horiz.cross(vMag_horiz);

                // Combine Errors (Tune magWeight as needed, usually low, e.g., 0.1 - 0.5)
                double magWeight = 0.2;
                Vector<3> e = eAcc + (eMag * magWeight);

                // Integral feedback
                _biasX += _Ki * e.x() * dt;
                _biasY += _Ki * e.y() * dt;
                _biasZ += _Ki * e.z() * dt;

                // Proportional feedback
                gCorr = Vector<3>(
                    -bodyGyro.x() - _biasX + _Kp * e.x(),
                    -bodyGyro.y() - _biasY + _Kp * e.y(),
                    -bodyGyro.z() - _biasZ + _Kp * e.z());
            }
            else // GYRO_ONLY
            {
                gCorr = Vector<3>(
                    -bodyGyro.x() - _biasX,
                    -bodyGyro.y() - _biasY,
                    -bodyGyro.z() - _biasZ);
            }

            // Integrate Quaternion
            integrateQuaternion(gCorr, dt);
        }

        // --------------------------------------------------------------------------
        // 6-DOF Update (Accel + Gyro) - No Mag or Mag Uncalibrated
        // --------------------------------------------------------------------------
        void update(const Vector<3> &accel,
                    const Vector<3> &gyro,
                    double dt)
        {
            // --- MODE: CALIBRATING (ON PAD) ---
            if (_mode == MahonyMode::CALIBRATING)
            {
                // 1G Guard: Only snap to gravity if stationary
                double accMag = accel.magnitude();
                if (fabs(accMag - 9.81) > 2.0) return; 

                _sumAccel += accel;
                _sumGyro += gyro;
                _calibSamples++;

                // Dynamic Alignment
                _boardToBody = computeBoardToBodyRotation(accel);
                Vector<3> bodyAccel = _boardToBody.rotateVector(accel);

                // Compute absolute tilt
                Vector<3> bodyDown = bodyAccel * (-1.0); // Gravity direction
                bodyDown.normalize();
                Vector<3> earthDown(0.0, 0.0, -1.0); // ENU Gravity vector

                Vector<3> axis = earthDown.cross(bodyDown);
                double axisMag = axis.magnitude();

                if (axisMag < 1e-6) {
                    if (bodyDown.dot(earthDown) > 0) _q = Quaternion(1.0, 0.0, 0.0, 0.0);
                    else _q = Quaternion(0.0, 1.0, 0.0, 0.0);
                } else {
                    axis = axis * (1.0 / axisMag);
                    double angle = acos(constrain(bodyDown.dot(earthDown), -1.0, 1.0));
                    _q.fromAxisAngle(axis, angle);
                    _q.normalize();
                }
                
                _orientationValid = true;
                return;
            }

            // --- MODE: FLIGHT ---
            if (!_orientationValid) return;

            Vector<3> bodyAccel = _boardToBody.rotateVector(accel);
            Vector<3> bodyGyro  = _boardToBody.rotateVector(gyro);
            Vector<3> gCorr;

            if (_mode == MahonyMode::CORRECTING)
            {
                Vector<3> a = bodyAccel;
                a.normalize();
                
                // Gravity is [0, 0, -1] in standard ENU for 6DOF pure tilt (Accel points UP, Gravity points DOWN)
                // Note: In 9DOF above we used [0,0,1] logic, keep consistent:
                // If Accel measures specific force (+1g Z), then gravity vector is -Z.
                Vector<3> vAcc = _q.rotateVector(Vector<3>(0.0, 0.0, -1.0)); 
                Vector<3> e = vAcc.cross(a);

                _biasX += _Ki * e.x() * dt;
                _biasY += _Ki * e.y() * dt;
                _biasZ += _Ki * e.z() * dt;

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

            integrateQuaternion(gCorr, dt);
        }

        virtual bool isReady() const { return _orientationValid; }
        bool isFrameLocked() const { return _frameLocked; }
        bool isMagCalibrated() const { return _magCalibrated; }
        MahonyMode getMode() const { return _mode; }
        void setMode(MahonyMode mode) { _mode = mode; }

        /**
         * Returns orientation quaternion.
         * If frame is locked (flight), returns relative to launch.
         */
        virtual Quaternion getQuaternion() const
        {
            if (!_orientationValid) return Quaternion(1.0, 0.0, 0.0, 0.0);
            if (_mode == MahonyMode::CALIBRATING) return _q;
            
            Quaternion qInv = _q0.conjugate();
            return qInv * _q;
        }
        
        // Returns the raw absolute orientation (Earth -> Body)
        Quaternion getAbsoluteQuaternion() const { return _q; }

        virtual Vector<3> getEarthAcceleration(const Vector<3> &accel) const
        {
            Vector<3> bodyAccel = _boardToBody.rotateVector(accel);
            Quaternion qInv = _q.conjugate();
            Vector<3> earthAcc = qInv.rotateVector(bodyAccel);
            earthAcc.z() -= 9.81; 
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
            _magData.clear();
            // Note: We generally don't reset mag calibration matrices on soft reset
        }

    private:
        // Helper to integrate qDot
        void integrateQuaternion(const Vector<3>& gCorr, double dt)
        {
            Quaternion omega(0.0, gCorr.x(), gCorr.y(), gCorr.z());
            Quaternion qDot = omega * _q;
            qDot = qDot * 0.5;
            _q = _q + (qDot * dt);
            _q.normalize();
        }

        // Logic to snap board frame to body frame (ENU) based on gravity vector
        Quaternion computeBoardToBodyRotation(const Vector<3> &accel) const
        {
            double absX = fabs(accel.x());
            double absY = fabs(accel.y());
            double absZ = fabs(accel.z());

            if (absZ >= absX && absZ >= absY) {
                return (accel.z() > 0) ? Quaternion(1.0, 0, 0, 0) : Quaternion(0.0, 1.0, 0, 0);
            }
            else if (absX >= absY && absX >= absZ) {
                return (accel.x() > 0) ? Quaternion(0.707107, 0.0, -0.707107, 0.0) : Quaternion(0.707107, 0.0, 0.707107, 0.0);
            }
            else {
                return (accel.y() > 0) ? Quaternion(0.707107, 0.707107, 0.0, 0.0) : Quaternion(0.707107, -0.707107, 0.0, 0.0);
            }
        }

        // Apply Hard/Soft iron calibration
        Vector<3> calibrateMag(const Vector<3> &raw) const
        {
            if (!_magCalibrated) return raw;

            // 1. Hard iron (Offset)
            Vector<3> temp;
            temp.x() = raw.x() - _hardIron.x();
            temp.y() = raw.y() - _hardIron.y();
            temp.z() = raw.z() - _hardIron.z();

            // 2. Soft iron (Transformation)
            Vector<3> corrected;
            corrected.x() = _softIron[0][0] * temp.x() + _softIron[0][1] * temp.y() + _softIron[0][2] * temp.z();
            corrected.y() = _softIron[1][0] * temp.x() + _softIron[1][1] * temp.y() + _softIron[1][2] * temp.z();
            corrected.z() = _softIron[2][0] * temp.x() + _softIron[2][1] * temp.y() + _softIron[2][2] * temp.z();

            return corrected;
        }

        // Uses Eigen to fit ellipsoid
        void computeMagCalibration()
        {
            // Convert vector buffer to Eigen matrix
            int n = _magData.size();
            Eigen::MatrixXd data(n, 3);
            for (int i = 0; i < n; i++) {
                data(i, 0) = _magData[i].x();
                data(i, 1) = _magData[i].y();
                data(i, 2) = _magData[i].z();
            }

            Eigen::MatrixXd D(n, 9);
            D.col(0) = data.col(0).array().square(); // x^2
            D.col(1) = data.col(1).array().square(); // y^2
            D.col(2) = data.col(2).array().square(); // z^2
            D.col(3) = 2 * data.col(0).array() * data.col(1).array(); // 2xy
            D.col(4) = 2 * data.col(0).array() * data.col(2).array(); // 2xz
            D.col(5) = 2 * data.col(1).array() * data.col(2).array(); // 2yz
            D.col(6) = 2 * data.col(0).array(); // 2x
            D.col(7) = 2 * data.col(1).array(); // 2y
            D.col(8) = 2 * data.col(2).array(); // 2z

            Eigen::MatrixXd A_v = D.transpose() * D;
            Eigen::MatrixXd b_v = (D.transpose()) * Eigen::MatrixXd::Ones(n, 1);
            Eigen::VectorXd x_v = A_v.ldlt().solve(b_v);

            Eigen::Matrix4d A_mat;
            A_mat << x_v(0), x_v(3), x_v(4), x_v(6),
                     x_v(3), x_v(1), x_v(5), x_v(7),
                     x_v(4), x_v(5), x_v(2), x_v(8),
                     x_v(6), x_v(7), x_v(8), -1.0;

            Eigen::Matrix3d A_center = -A_mat.block<3, 3>(0, 0);
            Eigen::Vector3d b_center;
            b_center << x_v(6), x_v(7), x_v(8);
            Eigen::Vector3d center = A_center.ldlt().solve(b_center);

            _hardIron = Vector<3>(center(0), center(1), center(2));

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<1, 3>(3, 0) = center.transpose();
            Eigen::Matrix4d R = T * A_mat * T.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(R.block<3, 3>(0, 0) / -R(3, 3));
            Eigen::Matrix3d evecs = eig.eigenvectors();
            Eigen::Vector3d evals = eig.eigenvalues();

            // Radii
            Eigen::Vector3d radii = (1.0 / evals.array()).sqrt();
            
            // Soft iron correction matrix
            Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
            scale(0,0) = radii(0);
            scale(1,1) = radii(1);
            scale(2,2) = radii(2);
            scale = scale.inverse() * radii.minCoeff();

            Eigen::Matrix3d softCorr = evecs * scale * evecs.transpose();

            for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                    _softIron[i][j] = softCorr(i,j);
                }
            }

            _magCalibrated = true;
        }

        double _Kp, _Ki;
        double _biasX, _biasY, _biasZ;

        Quaternion _q;           // Current Orientation
        Quaternion _q0;          // Tare Orientation
        Quaternion _boardToBody; // Rotation from PCB to Rocket Body

        Vector<3> _sumAccel, _sumGyro;
        int _calibSamples;

        bool _orientationValid;
        bool _frameLocked;
        MahonyMode _mode;

        // Magnetometer specific
        bool _magCalibrated;
        std::vector<Vector<3>> _magData;
        Vector<3> _hardIron;
        double _softIron[3][3];
    };
}

#endif // MAHONY_H