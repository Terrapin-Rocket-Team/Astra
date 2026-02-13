#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include <ArduinoEigen.h> // Required for Mag Calibration
#include <vector>
#include <math.h>
#include <math.h>
#include "../Math/Quaternion.h"
#include "Math/Vector.h"
#include "../Sensors/MountingTransform.h"

namespace astra
{
    /**
     * MahonyAHRS - Pure math attitude estimation filter
     *
     * Philosophy: This is an application-agnostic math library.
     * - No rocket-specific logic (snap-to-vertical, launch detection, etc.)
     * - No mode switching (application handles when to trust accel)
     * - Optional static board->body mounting transform support
     *
     * The filter provides:
     * - Quaternion integration with gyro bias correction
     * - Accel-based tilt correction (complementary filtering)
     * - Optional magnetometer fusion for yaw stabilization
     * - Frame transformations (body <-> inertial)
     */
    class MahonyAHRS
    {
    public:
        MahonyAHRS(double Kp = 0.1, double Ki = 0.0005)
            : _Kp(Kp), _Ki(Ki),
              _biasX(0.0), _biasY(0.0), _biasZ(0.0),
              _q(1.0, 0.0, 0.0, 0.0),
              _boardToBody(MountingOrientation::IDENTITY),
              _qBoardToBody(1.0, 0.0, 0.0, 0.0),
              _magCalibrated(false), _hardIron(), _softIron{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
        {
        }

        // ========================= Magnetometer Calibration =========================

        /**
         * Add a magnetometer sample to the calibration buffer.
         * Input is expected in board frame.
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

        // ========================= Board -> Body Mounting =========================

        /**
         * Set board->body orientation using a preset mounting orientation.
         * @param orientation Board->Body orientation preset
         * @param preserveEarth If true, remap internal attitude to avoid jumps
         */
        void setBoardToBodyOrientation(MountingOrientation orientation, bool preserveEarth = true)
        {
            MountingTransform transform(orientation);
            Quaternion qBoardToBody = quaternionFromTransform(transform);
            setBoardToBodyInternal(transform, qBoardToBody, preserveEarth);
        }

        /**
         * Set board->body orientation from a 3x3 rotation matrix (row-major).
         * @param rotationMatrix [r00,r01,r02,r10,r11,r12,r20,r21,r22]
         * @param preserveEarth If true, remap internal attitude to avoid jumps
         */
        void setBoardToBodyMatrix(const double *rotationMatrix, bool preserveEarth = true)
        {
            MountingTransform transform(rotationMatrix);
            Quaternion qBoardToBody = quaternionFromTransform(transform);
            setBoardToBodyInternal(transform, qBoardToBody, preserveEarth);
        }

        /**
         * Set board->body orientation from a quaternion.
         * @param qBoardToBody Quaternion rotating board frame into body frame
         * @param preserveEarth If true, remap internal attitude to avoid jumps
         */
        void setBoardToBodyQuaternion(const Quaternion &qBoardToBody, bool preserveEarth = true)
        {
            Quaternion q = qBoardToBody;
            q.normalize();

            Vector<3> bx = q.rotateVector(Vector<3>(1.0, 0.0, 0.0));
            Vector<3> by = q.rotateVector(Vector<3>(0.0, 1.0, 0.0));
            Vector<3> bz = q.rotateVector(Vector<3>(0.0, 0.0, 1.0));
            const double matrix[9] = {
                bx.x(), by.x(), bz.x(),
                bx.y(), by.y(), bz.y(),
                bx.z(), by.z(), bz.z()};

            MountingTransform transform(matrix);
            setBoardToBodyInternal(transform, q, preserveEarth);
        }

        /**
         * Set board->body orientation from a MountingTransform object.
         * @param transform Board->Body transform
         * @param preserveEarth If true, remap internal attitude to avoid jumps
         */
        void setBoardToBodyTransform(const MountingTransform &transform, bool preserveEarth = true)
        {
            Quaternion qBoardToBody = quaternionFromTransform(transform);
            setBoardToBodyInternal(transform, qBoardToBody, preserveEarth);
        }

        MountingTransform getBoardToBodyTransform() const { return _boardToBody; }
        Quaternion getBoardToBodyQuaternion() const { return _qBoardToBody; }
        Quaternion getBodyToEarthQuaternion() const { return _q; }
        Quaternion getBoardToEarthQuaternion() const { return _q * _qBoardToBody; }

        // ========================= Update Methods =========================

        /**
         * Update with gyro + accel + mag (9-DOF)
         * @param accel Acceleration vector in board frame (m/s^2)
         * @param gyro Angular velocity in board frame (rad/s)
         * @param mag Magnetic field vector in board frame (any units, will be normalized)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &accel, const Vector<3> &gyro, const Vector<3> &mag, double dt)
        {
            // Convert board-frame vectors to body frame using configured mount.
            Vector<3> aBody = _boardToBody.transform(accel);
            Vector<3> gBody = _boardToBody.transform(gyro);

            // If mag isn't calibrated, fall back to 6-DOF
            if (!_magCalibrated)
            {
                update(accel, gyro, dt);
                return;
            }

            // Normalize measurements
            Vector<3> a = aBody;
            a.normalize();

            // Calibrate in board frame, then rotate calibrated vector to body frame.
            Vector<3> calMagBoard = calibrateMag(mag);
            if (!isFiniteVec(calMagBoard))
            {
                update(accel, gyro, dt);
                return;
            }
            Vector<3> m = _boardToBody.transform(calMagBoard);
            double mMag = m.magnitude();
            if (!isfinite(mMag) || mMag <= 1e-9)
            {
                update(accel, gyro, dt);
                return;
            }
            m.normalize();

            // Estimated specific force direction in body frame.
            Quaternion qConj = _q.conjugate();
            Vector<3> vAcc = qConj.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Specific force is +Z (up)
            Vector<3> vMag = qConj.rotateVector(Vector<3>(0.0, 1.0, 0.0)); // North is +Y

            // Accel error (tilt correction)
            Vector<3> eAcc = a.cross(vAcc);

            // Mag error (yaw correction)
            Vector<3> mHoriz = m - (vAcc * m.dot(vAcc));
            double mHorizMag = mHoriz.magnitude();
            if (!isfinite(mHorizMag) || mHorizMag <= 1e-9)
            {
                update(accel, gyro, dt);
                return;
            }
            mHoriz.normalize();

            Vector<3> vMagHoriz = vMag - (vAcc * vMag.dot(vAcc));
            double vMagHorizMag = vMagHoriz.magnitude();
            if (!isfinite(vMagHorizMag) || vMagHorizMag <= 1e-9)
            {
                update(accel, gyro, dt);
                return;
            }
            vMagHoriz.normalize();

            Vector<3> eMag = mHoriz.cross(vMagHoriz);

            // Combine errors (mag gets lower weight)
            double magWeight = 0.4; // Tunable parameter: how much to trust magnetometer vs accelerometer
            Vector<3> e = eAcc + (eMag * magWeight);

            // Apply PI feedback
            _biasX += _Ki * e.x() * dt;
            _biasY += _Ki * e.y() * dt;
            _biasZ += _Ki * e.z() * dt;

            Vector<3> gCorr(
                gBody.x() - _biasX + _Kp * e.x(),
                gBody.y() - _biasY + _Kp * e.y(),
                gBody.z() - _biasZ + _Kp * e.z()
            );

            integrateQuaternion(gCorr, dt);
        }

        /**
         * Update with gyro + accel (6-DOF)
         * @param accel Acceleration vector in board frame (m/s^2)
         * @param gyro Angular velocity in board frame (rad/s)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &accel, const Vector<3> &gyro, double dt)
        {
            // Convert board-frame vectors to body frame using configured mount.
            Vector<3> aBody = _boardToBody.transform(accel);
            Vector<3> gBody = _boardToBody.transform(gyro);

            // Normalize acceleration
            Vector<3> a = aBody;
            a.normalize();

            // Estimated specific force direction in body frame.
            Quaternion qConj = _q.conjugate();
            Vector<3> vAcc = qConj.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Specific force is +Z (up)

            // Compute error (cross product)
            Vector<3> e = a.cross(vAcc);

            // Apply PI feedback
            _biasX += _Ki * e.x() * dt;
            _biasY += _Ki * e.y() * dt;
            _biasZ += _Ki * e.z() * dt;

            Vector<3> gCorr(
                gBody.x() - _biasX + _Kp * e.x(),
                gBody.y() - _biasY + _Kp * e.y(),
                gBody.z() - _biasZ + _Kp * e.z()
            );

            integrateQuaternion(gCorr, dt);
        }

        /**
         * Update with gyro only (dead reckoning)
         * Use when accelerometer is unreliable (high-G, freefall)
         * @param gyro Angular velocity in board frame (rad/s)
         * @param dt Time step (seconds)
         */
        void update(const Vector<3> &gyro, double dt)
        {
            // Convert board-frame vectors to body frame using configured mount.
            Vector<3> gBody = _boardToBody.transform(gyro);

            Vector<3> gCorr(
                gBody.x() - _biasX,
                gBody.y() - _biasY,
                gBody.z() - _biasZ
            );

            integrateQuaternion(gCorr, dt);
        }

        // ========================= Getters =========================

        /**
         * Get current orientation quaternion (Body -> Inertial)
         * Returns identity quaternion before first update.
         */
        virtual Quaternion getQuaternion() const { return _q; }

        /**
         * Transform board-frame acceleration to inertial frame and remove gravity.
         * @param accel Acceleration in board frame (specific force, m/s^2)
         * @return Linear acceleration in inertial frame (m/s^2)
         */
        virtual Vector<3> getEarthAcceleration(const Vector<3> &accel) const
        {
            Vector<3> accelBody = _boardToBody.transform(accel);

            // Rotate specific force to inertial frame
            Vector<3> inertialAcc = _q.rotateVector(accelBody);

            // Subtract gravity to get linear acceleration
            inertialAcc.z() -= 9.81;

            return inertialAcc;
        }

        /**
         * Check if filter is ready (always true for math-only version).
         */
        virtual bool isReady() const { return true; }

        // ========================= State Control =========================

        /**
         * Set orientation quaternion directly (body -> inertial).
         */
        void setQuaternion(const Quaternion &q)
        {
            _q = q;
            _q.normalize();
        }

        /**
         * Reset filter to initial state.
         * Preserves magnetometer calibration and board->body mounting transform.
         */
        void reset()
        {
            _biasX = _biasY = _biasZ = 0.0;
            _q = Quaternion(1.0, 0.0, 0.0, 0.0);
        }

    private:
        void setBoardToBodyInternal(const MountingTransform &transform, const Quaternion &qBoardToBody, bool preserveEarth)
        {
            Quaternion qBoardToEarth = _q * _qBoardToBody;

            _boardToBody = transform;
            _qBoardToBody = qBoardToBody;
            _qBoardToBody.normalize();

            if (preserveEarth)
            {
                _q = qBoardToEarth * _qBoardToBody.conjugate();
                _q.normalize();
            }
        }

        static Quaternion quaternionFromTransform(const MountingTransform &transform)
        {
            const double *m = transform.getMatrix();
            Matrix matrix(3, 3);
            matrix(0, 0) = m[0];
            matrix(0, 1) = m[1];
            matrix(0, 2) = m[2];
            matrix(1, 0) = m[3];
            matrix(1, 1) = m[4];
            matrix(1, 2) = m[5];
            matrix(2, 0) = m[6];
            matrix(2, 1) = m[7];
            matrix(2, 2) = m[8];

            Quaternion q;
            q.fromMatrix(matrix);
            q.normalize();
            return q;
        }

        // ========================= Internal Helpers =========================

        /**
         * Integrate quaternion using angular velocity.
         * @param gCorr Corrected gyro (rad/s) = gyro - bias + PI feedback
         * @param dt Time step (seconds)
         */
        void integrateQuaternion(const Vector<3> &gCorr, double dt)
        {
            // Quaternion derivative for Body -> Inertial: q_dot = 0.5 * q * omega
            Quaternion omega(0.0, gCorr.x(), gCorr.y(), gCorr.z());
            Quaternion qDot = _q * omega;
            qDot = qDot * 0.5;

            // Euler integration
            _q = _q + (qDot * dt);
            _q.normalize();
        }

        /**
         * Apply hard/soft iron calibration to magnetometer reading.
         * Input and output are in board frame.
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

            return isFiniteVec(corrected) ? corrected : raw;
        }

        /**
         * Compute magnetometer calibration using ellipsoid fit (Eigen-based).
         * Solves for hard iron (offset) and soft iron (scale/rotation) corrections.
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
            for (int i = 0; i < x_v.size(); i++)
            {
                if (!isfinite(x_v(i)))
                    return;
            }

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
            if (!isfinite(center(0)) || !isfinite(center(1)) || !isfinite(center(2)))
                return;

            _hardIron = Vector<3>(center(0), center(1), center(2));

            // Extract ellipsoid shape (soft iron correction)
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<1, 3>(3, 0) = center.transpose();
            Eigen::Matrix4d R = T * A_mat * T.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(R.block<3, 3>(0, 0) / -R(3, 3));
            Eigen::Matrix3d evecs = eig.eigenvectors();
            Eigen::Vector3d evals = eig.eigenvalues();

            for (int i = 0; i < 3; i++)
            {
                if (!isfinite(evals(i)) || evals(i) <= 0.0)
                    return;
            }

            Eigen::Vector3d radii = (1.0 / evals.array()).sqrt();
            for (int i = 0; i < 3; i++)
            {
                if (!isfinite(radii(i)) || radii(i) <= 0.0)
                    return;
            }

            Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
            scale(0, 0) = radii(0);
            scale(1, 1) = radii(1);
            scale(2, 2) = radii(2);
            scale = scale.inverse() * radii.minCoeff();

            Eigen::Matrix3d softCorr = evecs * scale * evecs.transpose();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (!isfinite(softCorr(i, j)))
                        return;
                    _softIron[i][j] = softCorr(i, j);
                }
            }

            _magCalibrated = true;
        }

        static bool isFiniteVec(const Vector<3> &v)
        {
            return isfinite(v.x()) && isfinite(v.y()) && isfinite(v.z());
        }

        // ========================= Member Variables =========================

        // Filter gains
        double _Kp, _Ki;

        // Gyro bias estimate
        double _biasX, _biasY, _biasZ;

        // Current orientation (Body -> Inertial)
        Quaternion _q;

        // Static mounting transform (Board -> Body)
        MountingTransform _boardToBody;
        Quaternion _qBoardToBody;

        // Magnetometer calibration (board frame)
        bool _magCalibrated;
        std::vector<Vector<3>> _magData;
        Vector<3> _hardIron;
        double _softIron[3][3];
    };
}

#endif // MAHONY_H
