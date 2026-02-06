#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include <ArduinoEigen.h> // Required for Mag Calibration
#include <vector>
#include "../Math/Quaternion.h"
#include "Math/Vector.h"

namespace astra
{
    /**
     * MahonyAHRS - Pure math attitude estimation filter
     *
     * Philosophy: This is an application-agnostic math library.
     * - No rocket-specific logic (snap-to-vertical, launch detection, etc.)
     * - No mode switching (application handles when to trust accel)
     * - No board mounting logic (application provides pre-rotated vectors)
     *
     * The filter provides:
     * - Quaternion integration with gyro bias correction
     * - Accel-based tilt correction (complementary filtering)
     * - Optional magnetometer fusion for yaw stabilization
     * - Frame transformations (body ↔ inertial)
     */
    class MahonyAHRS
    {
    public:
        MahonyAHRS(double Kp = 0.1, double Ki = 0.0005)
            : _Kp(Kp), _Ki(Ki),
              _biasX(0.0), _biasY(0.0), _biasZ(0.0),
              _q(1.0, 0.0, 0.0, 0.0),
              _magCalibrated(false), _hardIron(), _softIron{ {1,0,0}, {0,1,0}, {0,0,1} }
        {
        }

        // ========================= Magnetometer Calibration =========================

        /**
         * Add a magnetometer sample to the calibration buffer.
         * Call this repeatedly while rotating the board in 3D space (The "Mag Dance").
         */
        void collectMagCalibrationSample(const Vector<3> &mag)
        {
            _magData.push_back(mag);
        }

        /**
         * Compute magnetometer calibration from collected samples.
         * Requires at least 100 samples for reliable ellipsoid fit.
         */
        void finalizeMagCalibration()
        {
            if (_magData.size() > 100)
            {
                computeMagCalibration();
                _magData.clear();
                _magData.shrink_to_fit();
            }
        }

        bool isMagCalibrated() const { return _magCalibrated; }

        // ========================= Update Methods =========================

        /**
         * Update with gyro + accel + mag (9-DOF)
         * @param accel Acceleration vector in body frame (m/s^2)
         * @param gyro Angular velocity in body frame (rad/s)
         * @param mag Magnetic field vector in body frame (any units, will be normalized)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &accel, const Vector<3> &gyro, const Vector<3> &mag, double dt)
        {
            // If mag isn't calibrated, fall back to 6-DOF
            if (!_magCalibrated)
            {
                update(accel, gyro, dt);
                return;
            }

            // Normalize measurements
            Vector<3> a = accel;
            a.normalize();

            Vector<3> calMag = calibrateMag(mag);
            Vector<3> m = calMag;
            m.normalize();

            // Estimated specific force (opposite of gravity) direction in body frame
            // q conjugate rotates Inertial → Body
            // In ENU: gravity is -Z (down), so specific force (what accel measures) is +Z (up)
            Quaternion qConj = _q.conjugate();
            Vector<3> vAcc = qConj.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Specific force is +Z (up)
            Vector<3> vMag = qConj.rotateVector(Vector<3>(0.0, 1.0, 0.0));  // North is +Y

            // Accel error (tilt correction)
            Vector<3> eAcc = a.cross(vAcc);

            // Mag error (yaw correction)
            // Project onto horizontal plane to handle magnetic dip
            Vector<3> m_horiz = m - (vAcc * m.dot(vAcc));
            m_horiz.normalize();

            Vector<3> vMag_horiz = vMag - (vAcc * vMag.dot(vAcc));
            vMag_horiz.normalize();

            Vector<3> eMag = m_horiz.cross(vMag_horiz);

            // Combine errors (mag gets lower weight)
            double magWeight = 0.2;
            Vector<3> e = eAcc + (eMag * magWeight);

            // Apply PI feedback
            _biasX += _Ki * e.x() * dt;
            _biasY += _Ki * e.y() * dt;
            _biasZ += _Ki * e.z() * dt;

            Vector<3> gCorr(
                gyro.x() - _biasX + _Kp * e.x(),
                gyro.y() - _biasY + _Kp * e.y(),
                gyro.z() - _biasZ + _Kp * e.z()
            );

            integrateQuaternion(gCorr, dt);
        }

        /**
         * Update with gyro + accel (6-DOF)
         * @param accel Acceleration vector in body frame (m/s^2)
         * @param gyro Angular velocity in body frame (rad/s)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &accel, const Vector<3> &gyro, double dt)
        {
            // Normalize acceleration
            Vector<3> a = accel;
            a.normalize();

            // Estimated specific force (opposite of gravity) direction in body frame
            // q conjugate rotates Inertial → Body
            // In ENU: gravity is -Z (down), so specific force (what accel measures) is +Z (up)
            Quaternion qConj = _q.conjugate();
            Vector<3> vAcc = qConj.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Specific force is +Z (up)

            // Compute error (cross product)
            Vector<3> e = a.cross(vAcc);

            // Apply PI feedback
            _biasX += _Ki * e.x() * dt;
            _biasY += _Ki * e.y() * dt;
            _biasZ += _Ki * e.z() * dt;

            Vector<3> gCorr(
                gyro.x() - _biasX + _Kp * e.x(),
                gyro.y() - _biasY + _Kp * e.y(),
                gyro.z() - _biasZ + _Kp * e.z()
            );

            integrateQuaternion(gCorr, dt);
        }

        /**
         * Update with gyro only (dead reckoning)
         * Use when accelerometer is unreliable (high-G, freefall)
         * @param gyro Angular velocity in body frame (rad/s)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &gyro, double dt)
        {
            Vector<3> gCorr(
                gyro.x() - _biasX,
                gyro.y() - _biasY,
                gyro.z() - _biasZ
            );

            integrateQuaternion(gCorr, dt);
        }

        // ========================= Getters =========================

        /**
         * Get current orientation quaternion (Body → Inertial)
         * Returns identity quaternion before first update
         */
        Quaternion getQuaternion() const { return _q; }

        /**
         * Transform body-frame acceleration to inertial frame and remove gravity
         * @param accel Acceleration in body frame (specific force, m/s^2)
         * @return Linear acceleration in inertial frame (m/s^2)
         */
        Vector<3> getEarthAcceleration(const Vector<3> &accel) const
        {
            // Rotate specific force to inertial frame
            Vector<3> inertialAcc = _q.rotateVector(accel);

            // Subtract gravity to get linear acceleration
            // In ENU: gravity = (0, 0, -9.81), so we subtract it
            // Linear accel = Specific force - Gravity = SpecificForce - (0,0,-9.81)
            inertialAcc.z() -= 9.81;

            return inertialAcc;
        }

        /**
         * Check if filter is ready (always true for math-only version)
         */
        bool isReady() const { return true; }

        // ========================= State Control =========================

        /**
         * Set orientation quaternion directly (for external initialization)
         * Use this for:
         * - Injecting initial alignment from external source
         * - Snap-to-vertical logic (rocket applications)
         * - Resetting orientation to identity at liftoff
         * @param q Quaternion representing Body → Inertial rotation
         */
        void setQuaternion(const Quaternion &q)
        {
            _q = q;
            _q.normalize();
        }

        /**
         * Reset filter to initial state
         * Preserves magnetometer calibration
         */
        void reset()
        {
            _biasX = _biasY = _biasZ = 0.0;
            _q = Quaternion(1.0, 0.0, 0.0, 0.0);
            // Note: Mag calibration is preserved across resets
        }

    private:
        // ========================= Internal Helpers =========================

        /**
         * Integrate quaternion using angular velocity
         * @param gCorr Corrected gyro (rad/s) = gyro - bias + PI feedback
         * @param dt Time step (seconds)
         */
        void integrateQuaternion(const Vector<3>& gCorr, double dt)
        {
            // Quaternion derivative: q_dot = 0.5 * omega * q
            Quaternion omega(0.0, gCorr.x(), gCorr.y(), gCorr.z());
            Quaternion qDot = omega * _q;
            qDot = qDot * 0.5;

            // Euler integration
            _q = _q + (qDot * dt);
            _q.normalize();
        }

        /**
         * Apply hard/soft iron calibration to magnetometer reading
         * @param raw Raw magnetometer reading
         * @return Calibrated magnetometer reading
         */
        Vector<3> calibrateMag(const Vector<3> &raw) const
        {
            if (!_magCalibrated) return raw;

            // 1. Hard iron correction (offset)
            Vector<3> temp;
            temp.x() = raw.x() - _hardIron.x();
            temp.y() = raw.y() - _hardIron.y();
            temp.z() = raw.z() - _hardIron.z();

            // 2. Soft iron correction (scale/rotation)
            Vector<3> corrected;
            corrected.x() = _softIron[0][0] * temp.x() + _softIron[0][1] * temp.y() + _softIron[0][2] * temp.z();
            corrected.y() = _softIron[1][0] * temp.x() + _softIron[1][1] * temp.y() + _softIron[1][2] * temp.z();
            corrected.z() = _softIron[2][0] * temp.x() + _softIron[2][1] * temp.y() + _softIron[2][2] * temp.z();

            return corrected;
        }

        /**
         * Compute magnetometer calibration using ellipsoid fit (Eigen-based)
         * Solves for hard iron (offset) and soft iron (scale/rotation) corrections
         */
        void computeMagCalibration()
        {
            int n = _magData.size();
            Eigen::MatrixXd data(n, 3);
            for (int i = 0; i < n; i++) {
                data(i, 0) = _magData[i].x();
                data(i, 1) = _magData[i].y();
                data(i, 2) = _magData[i].z();
            }

            // Fit ellipsoid: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1
            Eigen::MatrixXd D(n, 9);
            D.col(0) = data.col(0).array().square();
            D.col(1) = data.col(1).array().square();
            D.col(2) = data.col(2).array().square();
            D.col(3) = 2 * data.col(0).array() * data.col(1).array();
            D.col(4) = 2 * data.col(0).array() * data.col(2).array();
            D.col(5) = 2 * data.col(1).array() * data.col(2).array();
            D.col(6) = 2 * data.col(0).array();
            D.col(7) = 2 * data.col(1).array();
            D.col(8) = 2 * data.col(2).array();

            Eigen::MatrixXd A_v = D.transpose() * D;
            Eigen::MatrixXd b_v = (D.transpose()) * Eigen::MatrixXd::Ones(n, 1);
            Eigen::VectorXd x_v = A_v.ldlt().solve(b_v);

            // Extract ellipsoid center (hard iron offset)
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

            // Extract ellipsoid shape (soft iron correction)
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<1, 3>(3, 0) = center.transpose();
            Eigen::Matrix4d R = T * A_mat * T.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(R.block<3, 3>(0, 0) / -R(3, 3));
            Eigen::Matrix3d evecs = eig.eigenvectors();
            Eigen::Vector3d evals = eig.eigenvalues();

            Eigen::Vector3d radii = (1.0 / evals.array()).sqrt();

            Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
            scale(0, 0) = radii(0);
            scale(1, 1) = radii(1);
            scale(2, 2) = radii(2);
            scale = scale.inverse() * radii.minCoeff();

            Eigen::Matrix3d softCorr = evecs * scale * evecs.transpose();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _softIron[i][j] = softCorr(i, j);
                }
            }

            _magCalibrated = true;
        }

        // ========================= Member Variables =========================

        // Filter gains
        double _Kp, _Ki;

        // Gyro bias estimate
        double _biasX, _biasY, _biasZ;

        // Current orientation (Body → Inertial)
        Quaternion _q;

        // Magnetometer calibration
        bool _magCalibrated;
        std::vector<Vector<3>> _magData;
        Vector<3> _hardIron;
        double _softIron[3][3];
    };
}

#endif // MAHONY_H