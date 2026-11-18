#pragma once

#include <cmath>
#include <limits>
#include <unity.h>
#include "Filters/ExtendedKalmanFilter.h"
#include <ArduinoEigen/Eigen/Geometry>

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

void test_sensor_jacobians_predict_physical_measurement_changes()
{
    const Eigen::Vector3d position(125, -32, 240);
    Eigen::Matrix<double, 10, 1> error;
    error << 0.35, -0.7, 1.2, -2.3, 3.4, -4.5, 0.01, -0.02, 0.03, 0.04;
    const Eigen::Vector3d perturbedPosition = position + error.head<3>();

    // Baro reports altitude relative to the launch origin, not pressure or NED Down.
    const double padAltitude = 123.5;
    const double altitudeBefore = (padAltitude + position.z()) - padAltitude;
    const double altitudeAfter = (padAltitude + perturbedPosition.z()) - padAltitude;
    const Eigen::Matrix<double, 1, 1> baroChange = Filter::BarometerJacobian() * error;
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, altitudeAfter - altitudeBefore, baroChange(0));

    // GPS has already been converted into local East/North metres by State.
    const Eigen::Vector2d gpsBefore(position.x(), position.y());
    const Eigen::Vector2d gpsAfter(perturbedPosition.x(), perturbedPosition.y());
    const Eigen::Vector2d gpsChange = Filter::GPSHorizontalJacobian() * error;
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, gpsAfter.x() - gpsBefore.x(), gpsChange.x());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, gpsAfter.y() - gpsBefore.y(), gpsChange.y());
}

Eigen::VectorXd navigationPoint()
{
    Eigen::VectorXd point(11);
    point << 125, -32, 240, 0.3, -0.7, 1.5, 0.3, -0.4, 0.5, 0.7, 1.2;
    point.segment<4>(6).normalize();
    return point;
}

numx_status_t bodyVectorMeasurement(const DualVector &input, DualVector &output, void *context)
{
    const auto &field = *static_cast<const Eigen::Vector3d *>(context);
    const auto add = numx_dual_add, sub = numx_dual_sub, mul = numx_dual_mul;
    const auto one = numx_dual_const(1), two = numx_dual_const(2);
    const auto w = input(6), x = input(7), y = input(8), z = input(9);
    const numx_dual_t rotation[3][3] = {
        {sub(one, mul(two, add(mul(y, y), mul(z, z)))),
         mul(two, sub(mul(x, y), mul(w, z))), mul(two, add(mul(x, z), mul(w, y)))},
        {mul(two, add(mul(x, y), mul(w, z))),
         sub(one, mul(two, add(mul(x, x), mul(z, z)))), mul(two, sub(mul(y, z), mul(w, x)))},
        {mul(two, sub(mul(x, z), mul(w, y))), mul(two, add(mul(y, z), mul(w, x))),
         sub(one, mul(two, add(mul(x, x), mul(y, y))))}
    };
    for (int i = 0; i < 3; ++i) {
        output(i) = numx_dual_const(0);
        for (int j = 0; j < 3; ++j) {
            output(i) = add(output(i), mul(rotation[j][i], numx_dual_const(field(j))));
        }
    }
    return NUMX_OK;
}

Eigen::VectorXd injectNavigationError(const Eigen::VectorXd &point,
                                     const Eigen::Matrix<double, 10, 1> &error)
{
    Eigen::VectorXd perturbed = point;
    perturbed.head<6>() += error.head<6>();
    perturbed(10) += error(9);
    const Eigen::Quaterniond q(point(6), point(7), point(8), point(9));
    const Eigen::Vector3d angle = error.segment<3>(6);
    const Eigen::Quaterniond delta = angle.norm() == 0 ? Eigen::Quaterniond::Identity()
        : Eigen::Quaterniond(Eigen::AngleAxisd(angle.norm(), angle.normalized()));
    const Eigen::Quaterniond rotated = q * delta;
    perturbed.segment<4>(6) << rotated.w(), rotated.x(), rotated.y(), rotated.z();
    return perturbed;
}

void test_navigation_measurement_jacobian_matches_right_local_rotation()
{
    Filter filter(3, 0, 11);
    const Eigen::VectorXd point = navigationPoint();
    Eigen::Vector3d field(0.23, -0.11, 0.41);
    Eigen::MatrixXd jacobian;
    TEST_ASSERT_EQUAL(NUMX_OK,
        filter.CalculateNavigationMeasurementJacobian(bodyVectorMeasurement, point, 3, jacobian, &field));
    TEST_ASSERT_EQUAL(3, jacobian.rows());
    TEST_ASSERT_EQUAL(10, jacobian.cols());
    const double step = 1e-5;
    for (int j = 0; j < 10; ++j) {
        Eigen::Matrix<double, 10, 1> error = Eigen::Matrix<double, 10, 1>::Zero();
        error(j) = step;
        const Eigen::VectorXd plus = injectNavigationError(point, error);
        error(j) = -step;
        const Eigen::VectorXd minus = injectNavigationError(point, error);
        const Eigen::Quaterniond qPlus(plus(6), plus(7), plus(8), plus(9));
        const Eigen::Quaterniond qMinus(minus(6), minus(7), minus(8), minus(9));
        const Eigen::Vector3d expected =
            (qPlus.conjugate() * field - qMinus.conjugate() * field) / (2 * step);
        for (int i = 0; i < 3; ++i) {
            TEST_ASSERT_DOUBLE_WITHIN(2e-6, expected(i), jacobian(i, j));
        }
    }
}

numx_status_t coupledNavigationMeasurement(const DualVector &input, DualVector &output, void *)
{
    output(0) = numx_dual_mul(numx_dual_const(7), input(10));
    for (int i = 0; i < 6; ++i) {
        output(0) = numx_dual_add(output(0), numx_dual_mul(numx_dual_const(i + 1), input(i)));
    }
    output(1) = numx_dual_add(numx_dual_mul(input(10), input(3)), numx_dual_mul(input(2), input(2)));
    return NUMX_OK;
}

void test_navigation_measurement_jacobian_maps_translation_and_drag_coordinates()
{
    Filter filter(2, 0, 11);
    const Eigen::VectorXd point = navigationPoint();
    Eigen::MatrixXd jacobian;
    TEST_ASSERT_EQUAL(NUMX_OK,
        filter.CalculateNavigationMeasurementJacobian(coupledNavigationMeasurement, point, 2, jacobian));
    Eigen::Matrix<double, 2, 10> expected = Eigen::Matrix<double, 2, 10>::Zero();
    expected.block<1, 6>(0, 0) << 1, 2, 3, 4, 5, 6;
    expected(0, 9) = 7;
    expected(1, 2) = 2 * point(2);
    expected(1, 3) = point(10);
    expected(1, 9) = point(3);
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 10; ++j) {
            TEST_ASSERT_DOUBLE_WITHIN(1e-5, expected(i, j), jacobian(i, j));
        }
    }
}

void test_navigation_measurement_jacobian_preserves_result_on_failure()
{
    Filter filter(1, 0, 11);
    const Eigen::VectorXd point = navigationPoint();
    Eigen::MatrixXd jacobian = sentinel();
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
        filter.CalculateNavigationMeasurementJacobian(failingModel, point.head(10), 1, jacobian));
    assertPreserved(jacobian);
    Eigen::VectorXd invalid = point;
    invalid.segment<4>(6) *= 2;
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
        filter.CalculateNavigationMeasurementJacobian(failingModel, invalid, 1, jacobian));
    assertPreserved(jacobian);
    invalid = point;
    invalid(6) = std::numeric_limits<double>::quiet_NaN();
    TEST_ASSERT_EQUAL(NUMX_ERR_INVALID_ARG,
        filter.CalculateNavigationMeasurementJacobian(failingModel, invalid, 1, jacobian));
    assertPreserved(jacobian);
    FailureContext failure{0, 2};
    TEST_ASSERT_EQUAL(NUMX_ERR_SINGULAR,
        filter.CalculateNavigationMeasurementJacobian(failingModel, point, 1, jacobian, &failure));
    TEST_ASSERT_EQUAL(2, failure.calls);
    assertPreserved(jacobian);
}

void run_test_extended_kalman_filter_tests()
{
    RUN_TEST(test_jacobian_rectangular_model_and_repeated_calls);
    RUN_TEST(test_jacobian_rejects_invalid_input_without_changing_result);
    RUN_TEST(test_jacobian_propagates_callback_errors_without_partial_results);
    RUN_TEST(test_jacobian_rejects_malformed_output_without_changing_result);
    RUN_TEST(test_sensor_jacobians_predict_physical_measurement_changes);
    RUN_TEST(test_navigation_measurement_jacobian_matches_right_local_rotation);
    RUN_TEST(test_navigation_measurement_jacobian_maps_translation_and_drag_coordinates);
    RUN_TEST(test_navigation_measurement_jacobian_preserves_result_on_failure);
}

} // namespace test_extended_kalman_filter
