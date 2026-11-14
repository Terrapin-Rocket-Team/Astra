#pragma once

#include <cmath>
#include <limits>
#include <unity.h>
#include "Filters/ExtendedKalmanFilter.h"

namespace test_extended_kalman_filter {

using Filter = astra::ExtendedKalmanFilter;
using DualVector = Filter::DualVector;

struct ModelContext {
    numx_real_t coefficient;
    int calls = 0;
};

numx_status_t mixedModel(const DualVector &input, DualVector &output, void *context)
{
    auto &model = *static_cast<ModelContext *>(context);
    ++model.calls;
    output(0) = numx_dual_add(numx_dual_mul(input(0), input(1)),
                              numx_dual_sin(input(0)));
    output(1) = numx_dual_add(numx_dual_mul(numx_dual_const(model.coefficient), input(0)),
                              numx_dual_mul(input(1), input(1)));
    output(2) = numx_dual_const(7);
    return NUMX_OK;
}

Eigen::MatrixXd sentinel()
{
    Eigen::MatrixXd value(2, 2);
    value << 11, 12, 21, 22;
    return value;
}

void assertPreserved(const Eigen::MatrixXd &value)
{
    TEST_ASSERT_EQUAL(2, value.rows());
    TEST_ASSERT_EQUAL(2, value.cols());
    TEST_ASSERT_EQUAL_DOUBLE(11, value(0, 0));
    TEST_ASSERT_EQUAL_DOUBLE(12, value(0, 1));
    TEST_ASSERT_EQUAL_DOUBLE(21, value(1, 0));
    TEST_ASSERT_EQUAL_DOUBLE(22, value(1, 1));
}

void assertMixedJacobian(const Eigen::MatrixXd &jacobian,
                         const Eigen::VectorXd &point, double coefficient)
{
    TEST_ASSERT_EQUAL(3, jacobian.rows());
    TEST_ASSERT_EQUAL(2, jacobian.cols());
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, point(1) + std::cos(point(0)), jacobian(0, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, point(0), jacobian(0, 1));
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, coefficient, jacobian(1, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 2 * point(1), jacobian(1, 1));
    TEST_ASSERT_EQUAL_DOUBLE(0, jacobian(2, 0));
    TEST_ASSERT_EQUAL_DOUBLE(0, jacobian(2, 1));
}

void test_jacobian_rectangular_model_and_repeated_calls()
{
    Filter filter(3, 0, 2);
    ModelContext context{3};
    Eigen::VectorXd point(2);
    point << 0.4, -0.6;
    Eigen::MatrixXd jacobian;
    TEST_ASSERT_EQUAL(NUMX_OK, filter.CalculateJacobian(mixedModel, point, 3, jacobian, &context));
    assertMixedJacobian(jacobian, point, 3);
    TEST_ASSERT_EQUAL(2, context.calls);

    point << -0.8, 2.1;
    context.coefficient = -4;
    TEST_ASSERT_EQUAL(NUMX_OK, filter.CalculateJacobian(mixedModel, point, 3, jacobian, &context));
    assertMixedJacobian(jacobian, point, -4);
    TEST_ASSERT_EQUAL(4, context.calls);
}

void test_jacobian_rejects_invalid_input_without_changing_result()
{
    Filter filter(3, 0, 2);
    ModelContext context{3};
    Eigen::VectorXd point = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd jacobian = sentinel();
    TEST_ASSERT_EQUAL(NUMX_ERR_NULL_PTR, filter.CalculateJacobian(nullptr, point, 3, jacobian));
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                      filter.CalculateJacobian(mixedModel, Eigen::VectorXd(), 3, jacobian, &context));
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                      filter.CalculateJacobian(mixedModel, point, 0, jacobian, &context));
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                      filter.CalculateJacobian(mixedModel, point, -1, jacobian, &context));
    point(0) = std::numeric_limits<double>::quiet_NaN();
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                      filter.CalculateJacobian(mixedModel, point, 3, jacobian, &context));
    point(0) = std::numeric_limits<double>::infinity();
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                      filter.CalculateJacobian(mixedModel, point, 3, jacobian, &context));
    if (std::numeric_limits<numx_real_t>::max() < std::numeric_limits<double>::max()) {
        point(0) = std::numeric_limits<double>::max();
        TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                          filter.CalculateJacobian(mixedModel, point, 3, jacobian, &context));
    }
    TEST_ASSERT_EQUAL(0, context.calls);
    assertPreserved(jacobian);
}

struct FailureContext {
    int calls;
    int failAt;
};

numx_status_t failingModel(const DualVector &input, DualVector &output, void *context)
{
    auto &failure = *static_cast<FailureContext *>(context);
    if (++failure.calls == failure.failAt) {
        return NUMX_ERR_SINGULAR;
    }
    output(0) = numx_dual_mul(input(0), input(1));
    return NUMX_OK;
}

void test_jacobian_propagates_callback_errors_without_partial_results()
{
    Filter filter(1, 0, 2);
    Eigen::VectorXd point(2);
    point << 2, 5;
    Eigen::MatrixXd jacobian = sentinel();
    for (int failAt = 1; failAt <= 2; ++failAt) {
        FailureContext context{0, failAt};
        TEST_ASSERT_EQUAL(NUMX_ERR_SINGULAR,
                          filter.CalculateJacobian(failingModel, point, 1, jacobian, &context));
        TEST_ASSERT_EQUAL(failAt, context.calls);
        assertPreserved(jacobian);
    }
    FailureContext success{0, 3};
    TEST_ASSERT_EQUAL(NUMX_OK,
                      filter.CalculateJacobian(failingModel, point, 1, jacobian, &success));
    TEST_ASSERT_EQUAL(1, jacobian.rows());
    TEST_ASSERT_EQUAL(2, jacobian.cols());
    TEST_ASSERT_EQUAL_DOUBLE(5, jacobian(0, 0));
    TEST_ASSERT_EQUAL_DOUBLE(2, jacobian(0, 1));
}

enum class MalformedOutput { WrongSize, Unwritten, NaNDerivative, InfiniteValue };

numx_status_t malformedModel(const DualVector &, DualVector &output, void *context)
{
    const auto mode = *static_cast<MalformedOutput *>(context);
    if (mode == MalformedOutput::WrongSize) {
        output.resize(output.size() + 1);
    } else {
        output(0) = numx_dual_const(1);
        if (mode != MalformedOutput::Unwritten) {
            output(1) = numx_dual_const(2);
        }
        if (mode == MalformedOutput::NaNDerivative) {
            output(0).du = std::numeric_limits<numx_real_t>::quiet_NaN();
        } else if (mode == MalformedOutput::InfiniteValue) {
            output(0).re = std::numeric_limits<numx_real_t>::infinity();
        }
    }
    return NUMX_OK;
}

void test_jacobian_rejects_malformed_output_without_changing_result()
{
    Filter filter(2, 0, 2);
    Eigen::VectorXd point = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd jacobian = sentinel();
    for (auto mode : {MalformedOutput::WrongSize, MalformedOutput::Unwritten,
                      MalformedOutput::NaNDerivative, MalformedOutput::InfiniteValue}) {
        TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
                          filter.CalculateJacobian(malformedModel, point, 2, jacobian, &mode));
        assertPreserved(jacobian);
    }
}

void run_test_extended_kalman_filter_tests()
{
    RUN_TEST(test_jacobian_rectangular_model_and_repeated_calls);
    RUN_TEST(test_jacobian_rejects_invalid_input_without_changing_result);
    RUN_TEST(test_jacobian_propagates_callback_errors_without_partial_results);
    RUN_TEST(test_jacobian_rejects_malformed_output_without_changing_result);
}

} // namespace test_extended_kalman_filter
