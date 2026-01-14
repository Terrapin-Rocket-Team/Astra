#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <vector>
#include "Math/Matrix.h"
#include "Math/Quaternion.h"
#include "Math/Vector.h"

// need to add magnetomater to Mahony filter to correct accumulated gyrscope drift. Magnetometer spits 
// out strenght of Earth magnetic field in x, y, z components. Has its own calibration process with 
// soft and hard iron dependencies. Calibration: MOtioncal (program from pjrc)

using namespace astra;

class MahonyAHRS
{
public:

    MahonyAHRS(float Kp = 0.1, float Ki = 0.0005)
        : _Kp(Kp), _Ki(Ki), _biasX(0.0), _biasY(0.0), _biasZ(0.0),
          _sumAccel(), _sumGyro(), _sumMag(), _calibSamples(0), _q(1.0, 0, 0, 0), //change number of samples here
          _q0(1.0, 0.0, 0.0, 0.0), _initialized(false),  _magData(_calibSamples), _magCalibrated(false)
    {

    }
    
    //mag needs its own calibration because it needs dynamic, 3D rotation; static samples aren't enough.
    //using eigen matrices to perform c++ calculation of calibrations, cannot seem to find a way to offload the calibration math.
    //found algorithm to calculate the matrices at: https://github.com/TonyPhh/magnetometer-calibration/blob/master/magnetometer_calibrate.cpp


 // Compute magnetometer calibration matrices from collected samples
// This implements ellipsoid fitting to find hard and soft iron corrections
//stores results in hard_iron and soft_iron members
    void computeMagCalibration() {
    
    // Need sufficient samples for calibration (at least 9 parameters to solve)
    if (_calibSamples < 100) {
        return;
    }
    
    // Convert vector buffer to Eigen matrix for computation
    Eigen::MatrixXd data(_calibSamples, 3); //dynamically sized matrix, could change later on
    for (int i = 0; i < _calibSamples; i++) {
        data(i, 0) = _magData[i].x();
        data(i, 1) = _magData[i].y();
        data(i, 2) = _magData[i].z();
    }
    
    _magData.clear(); // free up vector memory
    _magData.shrink_to_fit();  // frees heap memory

    // Extract individual columns for easier manipulation
    Eigen::MatrixXd data_x = data.col(0);
    Eigen::MatrixXd data_y = data.col(1);
    Eigen::MatrixXd data_z = data.col(2);
    
    // Build design matrix D with quadratic terms for ellipsoid fitting
    // Each row: [x², y², z², 2xy, 2xz, 2yz, 2x, 2y, 2z]
    Eigen::MatrixXd D(_calibSamples, 9);
    D.col(0) = data_x.array().square();              // x²
    D.col(1) = data_y.array().square();              // y²
    D.col(2) = data_z.array().square();              // z²
    D.col(3) = 2 * data_x.array() * data_y.array(); // 2xy
    D.col(4) = 2 * data_x.array() * data_z.array(); // 2xz
    D.col(5) = 2 * data_y.array() * data_z.array(); // 2yz
    D.col(6) = 2 * data_x.array();                   // 2x
    D.col(7) = 2 * data_y.array();                   // 2y
    D.col(8) = 2 * data_z.array();                   // 2z
    
    // Solve least squares: D'*D*v = D'*ones
    // This finds the ellipsoid parameters that best fit the data
    Eigen::MatrixXd A_v = D.transpose() * D;
    Eigen::MatrixXd b_v = (D.transpose() )* Eigen::MatrixXd::Ones(D.rows(), 1);
    Eigen::MatrixXd x_v = A_v.lu().solve(b_v);
    
    // Form the algebraic representation of the ellipsoid as a 4x4 matrix
    // This represents the quadratic form: v'*A*v = 0
    Eigen::MatrixXd A;
    A.resize(4,4);
    A<<x_v(0),x_v(3),x_v(4),x_v(6),
        x_v(3),x_v(1),x_v(5),x_v(7),
        x_v(4),x_v(5),x_v(2),x_v(8),
            x_v(6),x_v(7),x_v(8),-1.0;
    
    // Extract the center of the ellipsoid (hard-iron offset)
    // Solve: -A[0:3,0:3] * center = [x_v(6), x_v(7), x_v(8)]
    Eigen::Matrix3d A_center = -A.block<3, 3>(0, 0);
    Eigen::MatrixXd b_center;
    b_center.resize(3,1);
    b_center<<x_v(6),x_v(7),x_v(8);
    Eigen::Vector3d x_center = A_center.lu().solve(b_center);
    
    // Store hard-iron correction (offset to center the ellipsoid)
    hard_iron[0] = x_center(0);
    hard_iron[1] = x_center(1);
    hard_iron[2] = x_center(2);
    
    // Create translation matrix to move ellipsoid to origin
    Eigen::MatrixXd T=Eigen::MatrixXd::Identity(4,4);
    T.block<1,3>(3,0)=x_center.transpose();
    
    // Transform ellipsoid to centered position
    Eigen::MatrixXd R=T*A*T.transpose();
    
    // Perform eigenvalue decomposition on the centered ellipsoid
    // This gives us the principal axes and radii
    Eigen::EigenSolver<Eigen::MatrixXd> eig(R.block<3,3>(0,0)/(-R(3,3)));
    Eigen::MatrixXd eigen_vectors=eig.pseudoEigenvectors();
    Eigen::MatrixXd eigen_values=eig.pseudoEigenvalueMatrix();
    
    // Ensure all eigenvalues are positive (flip eigenvector if needed)
    if(eigen_values(0,0)<0){
        eigen_values(0,0)=-eigen_values(0,0);
        eigen_vectors.col(0)=-1*eigen_values.col(0);
    }
    if(eigen_values(1,1)<0){
        eigen_values(1,1)=-eigen_values(1,1);
        eigen_vectors.col(1)=-1*eigen_values.col(1);
    }
    if(eigen_values(2,2)<0){
        eigen_values(2,2)=-eigen_values(2,2);
        eigen_vectors.col(2)=-1*eigen_values.col(2);
    }
    
    // Compute the radii of the ellipsoid from eigenvalues
    Eigen::MatrixXd radii=(1/(eigen_values.diagonal().array())).array().sqrt();

    Eigen::MatrixXd map=eigen_vectors.transpose();
    Eigen::MatrixXd inv_map=eigen_vectors;
    Eigen::MatrixXd scale_first=Eigen::MatrixXd::Identity(3,3);

    scale_first(0,0)=radii(0);
    scale_first(1,1)=radii(1);
    scale_first(2,2)=radii(2);

    Eigen::MatrixXd scale=scale_first.inverse()*radii.minCoeff();
    Eigen::MatrixXd soft_corr=inv_map*scale*map;
    Eigen::MatrixXd hard_corr=x_center;
 
    // Store soft-iron correction matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron(i, j) = soft_corr(i, j);
        }
    }

    //store hard-iron correction vector
    hard_iron = Vector<3>(hard_corr(0), hard_corr(1), hard_corr(2));
    
    // Mark calibration as complete
    _magCalibrated = true;

    }



// Apply calibration to a single raw magnetometer reading
// Returns calibrated measurement with hard and soft iron corrections applied
//this function is called every time in order to apply corrections to mag 
Vector<3> calibrateMag(const Vector<3> &raw) {
    // If not calibrated, return raw reading
    if (!_magCalibrated) {
        return raw;
    }
    
    // Step 1: Apply hard-iron offset (subtract center of ellipsoid)
    Vector<3> temp;
    temp.x() = raw.x() - hard_iron[0];
    temp.y() = raw.y() - hard_iron[1];
    temp.z() = raw.z() - hard_iron[2];
    
    // Step 2: Apply soft-iron correction (transform ellipsoid to sphere)
    Vector<3> corrected;
    corrected.x() = soft_iron(0,0) * temp.x() + soft_iron(0,1) * temp.y() + soft_iron(0,2) * temp.z();
    corrected.y() = soft_iron(1,0) * temp.x() + soft_iron(1,1) * temp.y() + soft_iron(1,2) * temp.z();
    corrected.z() = soft_iron(2,0) * temp.x() + soft_iron(2,1) * temp.y() + soft_iron(2,2) * temp.z();
    
    return corrected;
}


    // Collect static samples before launch
    void calibrate(const Vector<3> &accel, const Vector<3> &gyro, const Vector<3> &mag)
    {
        _sumAccel += accel;
        _sumGyro += gyro;
        _sumMag += mag;
        _magData.emplace_back(mag);
        _calibSamples++;
    }

    // Finalize calibration and align "down"
    void initialize()
    {
        if (_calibSamples == 0)
            return;
        Vector<3> avgA = _sumAccel * (1.0 / _calibSamples);
        Vector<3> avgG = _sumGyro * (1.0 / _calibSamples);
        Vector<3> avgM = _sumMag * (1.0 / _calibSamples);
        computeMagCalibration(); //compute mag calibration matrices and store results in soft_iron and hard_iron

        // Set gyro bias
        _biasX = avgG.x();
        _biasY = avgG.y();
        _biasZ = avgG.z();

        //mag calibration for inital orientation
        Vector<3> calibratedMag = calibrateMag(avgM);
        calibratedMag.normalize();

        // Align initial orientation so body Z matches gravity
        Vector<3> bodyDown = avgA;
        bodyDown.normalize();
        Vector<3> earthDown(0.0, 0.0, 1.0);
        Vector<3> axis = earthDown.cross(bodyDown);
        if (axis.magnitude() < 1e-6)
        {
            axis = Vector<3>(1.0, 0.0, 0.0);
        }
        else
        {
            axis.normalize();
        }
        double dot = bodyDown.dot(earthDown);
        dot = constrain(dot, -1.0, 1.0);
        double angle = acos(dot);
        _q = Quaternion();
        _q.fromAxisAngle(axis, angle);
        _q.normalize();

        //adding magnetometer to initial alignment to correct yaw
        Vector<3> magToEarthFrame = _q.conjugate().rotateVector(calibratedMag);

        //in earth frame, magnetic heading is simply the x and y components
        Vector<3> magHorizontal(magToEarthFrame.x(), magToEarthFrame.y(), 0.0);
        magHorizontal.normalize();

        //expected magnetic north in ENU frame (North = +Y)
        Vector<3> earthMagNorth(0.0, 1.0, 0.0);


        //find yaw correction angle between mag heading and earth x axis
        double yaw = magHorizontal.dot(earthMagNorth);
        yaw = constrain(yaw, -1.0, 1.0);
        double yawAngle = acos(yaw);

        //determine sign of yaw angle using cross product
        Vector<3> cross = earthMagNorth.cross(magHorizontal);
        if (cross.z() < 0.0)
        {
            yawAngle = -yawAngle;
        }


        Vector<3> yawAxis(0.0, 0.0, 1.0); //yaw in Earth Frame

        Quaternion yawCorrection;
        yawCorrection.fromAxisAngle(yawAxis, yawAngle);
        _q = yawCorrection * _q; //fix quaternion with yaw correction
        _q.normalize();


        // Store initial orientation for relative output
        _q0 = _q;
        _initialized = true;
    }

    /**
     * Update orientation using only accelerometer (for tilt) and gyro.
     * @param accel : Vector<3> accelerometer readings (m/s^2)
     * @param gyro  : Vector<3> gyroscope readings (rad/s)
     * @param dt    : time step (s)
     */
    void update(const Vector<3> &accel,
                const Vector<3> &gyro, 
                const Vector<3> &mag,
                double dt)
    {
        if (!_initialized)
            return;

        // 1. Normalize accelerometer measurement
        Vector<3> a = accel; // gravity is positive Z
        a.normalize();

        // 2. Estimated gravity direction in body frame
        Vector<3> vAcc = _q.rotateVector(Vector<3>(0.0, 0.0, 1.0)); // Earth frame gravity vector [0,0,1]

        // 3. Error between measured and estimated gravity
        Vector<3> e = a.cross(vAcc);

        // 4. Magnetometer correction
        Vector<3> calibratedMag = calibrateMag(mag);
        calibratedMag.normalize();

        // 5. estimated magnetic field direction in Earth Frame (ENU)
        Vector<3> vMag = _q.rotateVector(Vector<3>(0.0, 1.0, 0.0));

            // 6. Error between measured and estimated mag field
            Vector<3> mag_horizontal = calibratedMag - (vAcc * calibratedMag.dot(vAcc)); //projection of heading onto Earth horizontal plane
            mag_horizontal.normalize();
        
            // Remove vertical component from expected magnetometer
            Vector<3> vMag_horizontal = vMag - (vAcc * vMag.dot(vAcc));
            vMag_horizontal.normalize();
        
            // Error between measured and estimated magnetic field (yaw correction)
            Vector<3> eMag = mag_horizontal.cross(vMag_horizontal);

        // 8. Combine errors
            //tuning will be required for mag, assume 0.2 for now

            float magWeight = 0.2;
            e += eMag * magWeight;


        // 9. Update gyroscope bias (integral action)
        _biasX += _Ki * e.x() * dt;
        _biasY += _Ki * e.y() * dt;
        _biasZ += _Ki * e.z() * dt;

        // 10. Corrected angular velocity (gyroscope measurements compensated for bias and error)
        Vector<3> gCorr(
            -gyro.x() - _biasX + _Kp * e.x(),
            -gyro.y() - _biasY + _Kp * e.y(),
            -gyro.z() - _biasZ + _Kp * e.z());

        Quaternion qDot = Quaternion(0.0, gCorr.x(), gCorr.y(), gCorr.z()) * _q * 0.5;

        // 11. Integrate and normalize
        _q = _q + (qDot * dt);
        _q.normalize();
    }

    // Returns the orientation relative to initial alignment
    Quaternion getQuaternion() const
    {
        // compute q_rel = q0*^-1 * q
        Quaternion qInv = _q0.conjugate();
        return qInv * _q;
    }

    Quaternion getAbsoluteQuaternion() const
    {
        return _q;
    }

    Vector<3> toEarthFrame(const Vector<3> &v) const
    {
        Vector<3> ret = _q.conjugate().rotateVector(v) * -1.0;
        ret.z() *= -1.0;
        return ret;
    }

    bool isInitialized() const { return _initialized; }

    void reset() //for simulating the filter, used to reset everything 
    {
        _biasX = _biasY = _biasZ = 0.0;
        _q = Quaternion(1.0, 0.0, 0.0, 0.0);
        _q0 = _q;
        _initialized = false;

        _sumAccel = Vector<3>();
        _sumGyro  = Vector<3>();
        _sumMag   = Vector<3>();
        _calibSamples = 0;
        _magCalibrated = false;
    }



    double _Kp, _Ki; //tuning parameters for mahony filter
    double _biasX, _biasY, _biasZ; //biases = averages 
    Quaternion _q, _q0; //rotation quaternion 
    Vector<3> _sumAccel, _sumGyro, _sumMag; //vectors which hold sum of accel, gyro readings for calibration
    int _calibSamples; //number of samples
    bool _initialized;
    bool _magCalibrated;
    std::vector<Vector<3>> _magData; //dynamic array to hold mag samples for calibration, it doubles in memory when full!!

    Vector<3> hard_iron; //astra vector to hold hard iron calibration values, used the entire flight 
    Matrix soft_iron{3, 3, new double[9]()}; //astra matrix to hold soft iron calibration values, used the entire flight

};

#endif // MAHONY_H