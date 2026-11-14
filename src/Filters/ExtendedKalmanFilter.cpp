//Created by Divyansh Srivastava on 11/11/2026
#include "ExtendedKalmanFilter.h"

#include <cmath>
#include <limits>

namespace astra {

ExtendedKalmanFilter::ExtendedKalmanFilter(int measurementSize, int controlSize, int stateSize)
    : measurementSize(measurementSize), controlSize(controlSize), stateSize(stateSize)
{
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

} // namespace astra
