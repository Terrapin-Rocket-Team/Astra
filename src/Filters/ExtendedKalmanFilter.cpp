//Created by Divyansh Srivastava on 11/11/2026
#include "ExtendedKalmanFilter.h"

#include <cmath>
#include <limits>

namespace astra {

ExtendedKalmanFilter::ExtendedKalmanFilter(int measurementSize, int controlSize, int stateSize)
    : measurementSize(measurementSize), controlSize(controlSize), stateSize(stateSize)
{
}

numx_status_t ExtendedKalmanFilter::EvaluateJacobian(
    JacobianEvaluator evaluator, const Eigen::VectorXd &nominalState,
    int outputSize, int errorSize, Eigen::MatrixXd &jacobian, void *context)
{
    return EvaluateJacobianWithScratch(evaluator, nominalState, outputSize,
                                      errorSize, jacobian, context,
                                      evaluatedJacobian);
}

numx_status_t ExtendedKalmanFilter::EvaluateJacobianWithScratch(
    JacobianEvaluator evaluator, const Eigen::VectorXd &nominalState,
    int outputSize, int errorSize, Eigen::MatrixXd &jacobian, void *context,
    Eigen::MatrixXd &scratch)
{
    if (evaluator == nullptr) {
        return NUMX_ERR_NULL_PTR;
    }
    if (nominalState.size() == 0 || outputSize <= 0 || errorSize <= 0 ||
        !nominalState.allFinite()) {
        return NUMX_ERR_INVALID_ARG;
    }

    scratch.resize(outputSize, errorSize);
    scratch.setConstant(std::numeric_limits<double>::quiet_NaN());
    const numx_status_t status = evaluator(nominalState, scratch, context);
    if (status != NUMX_OK) {
        return status;
    }
    if (scratch.rows() != outputSize || scratch.cols() != errorSize ||
        !scratch.allFinite()) {
        return NUMX_ERR_INVALID_ARG;
    }

    jacobian = scratch;
    return NUMX_OK;
}

numx_status_t ExtendedKalmanFilter::SetDynamicsJacobianEvaluator(
    JacobianEvaluator evaluator, int errorSize, void *context)
{
    if (evaluator == nullptr) {
        return NUMX_ERR_NULL_PTR;
    }
    if (errorSize <= 0 || stateSize <= 0) {
        return NUMX_ERR_INVALID_ARG;
    }

    A.resize(errorSize, errorSize);
    A.setConstant(std::numeric_limits<double>::quiet_NaN());
    dynamicsJacobianCandidate.resize(errorSize, errorSize);
    dynamicsJacobianEvaluator = evaluator;
    dynamicsJacobianContext = context;
    dynamicsErrorSize = errorSize;
    return NUMX_OK;
}

numx_status_t ExtendedKalmanFilter::SetMeasurementJacobianEvaluator(
    JacobianEvaluator evaluator, int errorSize, void *context)
{
    if (evaluator == nullptr) {
        return NUMX_ERR_NULL_PTR;
    }
    if (errorSize <= 0 || stateSize <= 0 || measurementSize <= 0) {
        return NUMX_ERR_INVALID_ARG;
    }

    H.resize(measurementSize, errorSize);
    H.setConstant(std::numeric_limits<double>::quiet_NaN());
    measurementJacobianCandidate.resize(measurementSize, errorSize);
    measurementJacobianEvaluator = evaluator;
    measurementJacobianContext = context;
    measurementErrorSize = errorSize;
    return NUMX_OK;
}

numx_status_t ExtendedKalmanFilter::UpdateDynamicsJacobian(
    const Eigen::VectorXd &nominalState)
{
    if (nominalState.size() != stateSize) {
        return NUMX_ERR_INVALID_ARG;
    }
    return EvaluateJacobianWithScratch(dynamicsJacobianEvaluator, nominalState,
                                      dynamicsErrorSize, dynamicsErrorSize, A,
                                      dynamicsJacobianContext,
                                      dynamicsJacobianCandidate);
}

numx_status_t ExtendedKalmanFilter::UpdateMeasurementJacobian(
    const Eigen::VectorXd &nominalState)
{
    if (nominalState.size() != stateSize) {
        return NUMX_ERR_INVALID_ARG;
    }
    return EvaluateJacobianWithScratch(measurementJacobianEvaluator, nominalState,
                                      measurementSize, measurementErrorSize, H,
                                      measurementJacobianContext,
                                      measurementJacobianCandidate);
}

//Function used to calculate the Jacobian given a jacobianfuncion that needs to be converted,
// An input, an outputsize to check, and a context. The jacobianFunction is a function pointer that 
//is used later as it returns a status enum type from the numx library; used to aclculate each column
// and check if there are any failures that need to be taken care of.  The final result is weapped
//in an arduinoeigen library matrix. This function currently returns a numx_status_t type which 
//I will probably change this to return a boolean. 
numx_status_t ExtendedKalmanFilter::CalculateJacobian(
    JacobianFunction function, const Eigen::VectorXd &input, int outputSize,
    Eigen::MatrixXd &jacobian, void *context)
{
    // Rows are function outputs; columns are input variables (m outputs, n inputs).
    // J (m x n) = [ df0/dx0       ... df0/dx(n-1)       ]
    //             [    ...                 ...         ]
    //             [ df(m-1)/dx0   ... df(m-1)/dx(n-1)   ]
    if (function == nullptr) {
        return NUMX_ERR_NULL_PTR;
    }
    if (input.size() == 0 || outputSize <= 0) {
        return NUMX_ERR_INVALID_ARG;
    }
    const double maxValue = static_cast<double>(std::numeric_limits<numx_real_t>::max());
    for (Eigen::Index j = 0; j < input.size(); ++j) {
        if (!std::isfinite(input(j)) || input(j) < -maxValue || input(j) > maxValue) {
            return NUMX_ERR_INVALID_ARG;
        }
    }

    jacobianInput.resize(input.size());
    jacobianOutput.resize(outputSize);
    jacobianResult.resize(outputSize, input.size());
    for (Eigen::Index j = 0; j < input.size(); ++j) {
        jacobianInput(j) = numx_dual_const(static_cast<numx_real_t>(input(j)));
    }

    const numx_real_t nan = std::numeric_limits<numx_real_t>::quiet_NaN();
    const numx_dual_t unwritten = {nan, nan};
    for (Eigen::Index j = 0; j < input.size(); ++j) {
        // Seed one independent variable; all other input derivatives stay zero.
        jacobianInput(j).du = 1;
        jacobianOutput.setConstant(unwritten);
        const numx_status_t status = function(jacobianInput, jacobianOutput, context);
        jacobianInput(j).du = 0;
        if (status != NUMX_OK) {
            return status;
        }
        if (jacobianOutput.size() != outputSize) {
            return NUMX_ERR_INVALID_ARG;
        }
        for (int i = 0; i < outputSize; ++i) {
            if (!std::isfinite(jacobianOutput(i).re) ||
                !std::isfinite(jacobianOutput(i).du)) {
                return NUMX_ERR_INVALID_ARG;
            }
            jacobianResult(i, j) = jacobianOutput(i).du;
        }
    }

    jacobian = jacobianResult;
    return NUMX_OK;
}

Eigen::Matrix<double, 1, 10> ExtendedKalmanFilter::BarometerJacobian()
{
    // Columns: [dp_E dp_N dp_U dv_E dv_N dv_U dth_x dth_y dth_z dc].
    // H_baro = [ 0    0    1    0    0    0    0     0     0    0 ].
    // One row: relative barometric altitude (Up).
    Eigen::Matrix<double, 1, 10> jacobian = Eigen::Matrix<double, 1, 10>::Zero();
    jacobian(0, 2) = 1;
    return jacobian;
}

Eigen::Matrix<double, 2, 10> ExtendedKalmanFilter::GPSHorizontalJacobian()
{
    // Columns: [dp_E dp_N dp_U dv_E dv_N dv_U dth_x dth_y dth_z dc].
    // H_gps = [ 1    0    0    0    0    0    0     0     0    0 ]  East
    //         [ 0    1    0    0    0    0    0     0     0    0 ]  North.
    Eigen::Matrix<double, 2, 10> jacobian = Eigen::Matrix<double, 2, 10>::Zero();
    jacobian(0, 0) = 1;
    jacobian(1, 1) = 1;
    return jacobian;
}

numx_status_t ExtendedKalmanFilter::CalculateNavigationMeasurementJacobian(
    JacobianFunction measurementModel, const Eigen::VectorXd &nominalState,
    int outputSize, Eigen::MatrixXd &measurementJacobian, void *context)
{
    // Rows are sensor outputs; columns are [dp_ENU(3), dv_ENU(3), dtheta_body(3), dc].
    // H (m x 10) = [ H_p(m x 3)  H_v(m x 3)  H_theta(m x 3)  H_c(m x 1) ].
    // Each block contains dh/d(error component); values depend on the sensor model.
    // H = J_nominal(m x 11) * stateLift(11 x 10).
    if (nominalState.size() != 11) {
        return NUMX_ERR_INVALID_ARG;
    }
    const Eigen::Vector4d q = nominalState.segment<4>(6);
    const double normSquared = q.squaredNorm();
    if (!std::isfinite(normSquared) || std::abs(normSquared - 1.0) > 1e-6) {
        return NUMX_ERR_INVALID_ARG;
    }
    const numx_status_t status = CalculateJacobian(
        measurementModel, nominalState, outputSize, nominalMeasurementJacobian, context);
    if (status != NUMX_OK) {
        return status;
    }

    // Derivative of [p, v, q_est * Exp(dtheta_body), c] at zero error.
    // stateLift columns:       dp(3)  dv(3)  dtheta_body(3)             dc(1)
    // stateLift (11 x 10) = [   I3     0       0                         0  ] p(3)
    //                      [    0    I3       0                         0  ] v(3)
    //                      [    0     0      -0.5*qv^T                  0  ] qw(1)
    //                      [    0     0       0.5*(qw*I3 + [qv]_x)      0  ] qv(3)
    //                      [    0     0       0                         1  ] c(1)
    // qv = [qx, qy, qz]^T and [qv]_x is the cross-product matrix.
    Eigen::Matrix<double, 11, 10> stateLift = Eigen::Matrix<double, 11, 10>::Zero();
    stateLift.block<6, 6>(0, 0).setIdentity();
    stateLift.block<1, 3>(6, 6) = -0.5 * q.tail<3>().transpose();
    Eigen::Matrix3d cross;
    cross << 0, -q(3), q(2), q(3), 0, -q(1), -q(2), q(1), 0;
    stateLift.block<3, 3>(7, 6) = 0.5 * (q(0) * Eigen::Matrix3d::Identity() + cross);
    stateLift(10, 9) = 1;

    errorMeasurementJacobian.noalias() = nominalMeasurementJacobian * stateLift;
    if (!errorMeasurementJacobian.allFinite()) {
        return NUMX_ERR_INVALID_ARG;
    }
    measurementJacobian = errorMeasurementJacobian;
    return NUMX_OK;
}

} // namespace astra
