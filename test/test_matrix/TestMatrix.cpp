#include <unity.h>
#include "../src/Math/Matrix.h"
#include "NativeTestHelper.h"

using astra::Matrix;

// Test matrices - using array constructors
double m1_data[4] = {1, 2, 3, 4};
double m2_data[4] = {5, 6, 7, 8};
double m3_data[6] = {1, 2, 3, 4, 5, 6};
double m5_data[16] = {0,0,-1,2,0,1,0,0,9,0,0,0,0,0,0,1};

Matrix m1(2, 2, m1_data);
Matrix m2(2, 2, m2_data);
Matrix m3(2, 3, m3_data);
Matrix m5(4, 4, m5_data);



void test_copy_constructor()
{
    Matrix m4(m1);
    m1.disp();
    TEST_ASSERT_NOT_EQUAL(&m1, &m4);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(1, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(2, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(3, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[3]);
}

void test_assignment_operator()
{
    Matrix m4 = m1;
    TEST_ASSERT_NOT_EQUAL(&m1, &m4);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(1, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(2, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(3, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[3]);
}

void test_multiply_operator_matrix()
{
    Matrix m4 = m1 * m2;
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(19, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(22, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(43, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(50, m4.getArr()[3]);
}

void test_multiply_operator_scalar()
{
    Matrix m4 = m1 * 2;
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(2, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(6, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(8, m4.getArr()[3]);
}

void test_add_operator_matrix()
{
    Matrix m4 = m1 + m2;
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(6, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(8, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(10, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(12, m4.getArr()[3]);
}
void test_subtract_operator_matrix()
{
    Matrix m4 = m1 - m2;
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(-4, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[3]);
}
void test_transpose()
{
    Matrix m4 = m3.T();
    TEST_ASSERT_EQUAL(3, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(1, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(2, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(5, m4.getArr()[3]);
    TEST_ASSERT_EQUAL(3, m4.getArr()[4]);
    TEST_ASSERT_EQUAL(6, m4.getArr()[5]);
}
void test_inverse_small()
{
    Matrix m4 = m1.inv();
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -2.0, m4.getArr()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m4.getArr()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.5, m4.getArr()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -0.5, m4.getArr()[3]);
}

void test_inverse_large()
{
    Matrix m4 = m5.inv();
    TEST_ASSERT_EQUAL(4, m4.getRows());
    TEST_ASSERT_EQUAL(4, m4.getCols());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0/9, m4.getArr()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[4]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m4.getArr()[5]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[6]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[7]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -1.0, m4.getArr()[8]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[9]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[10]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 2.0, m4.getArr()[11]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[12]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[13]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0, m4.getArr()[14]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m4.getArr()[15]);
}

void test_trace()
{
    // Case 1: m1 (2x2 Matrix)
    double trace_m1 = m1.trace();
    double expected_trace_m1 = 1.0 + 4.0; // a00 + a11
    TEST_ASSERT_EQUAL_FLOAT(expected_trace_m1, trace_m1);

    // Case 2: m2 (2x2 Matrix)
    double trace_m2 = m2.trace();
    double expected_trace_m2 = 5.0 + 8.0; // a00 + a11
    TEST_ASSERT_EQUAL_FLOAT(expected_trace_m2, trace_m2);

    // Case 3: m5 (4x4 Matrix)
    double trace_m5 = m5.trace();
    double expected_trace_m5 = 0.0 + 1.0 + 0.0 + 1.0; // a00 + a11 + a22 + a33
    TEST_ASSERT_EQUAL_FLOAT(expected_trace_m5, trace_m5);
}

// --- New Comprehensive Tests ---

void test_default_constructor()
{
    Matrix m_empty;
    TEST_ASSERT_EQUAL(0, m_empty.getRows());
    TEST_ASSERT_EQUAL(0, m_empty.getCols());
    TEST_ASSERT_NULL(m_empty.getArr());
}

void test_allocating_constructor()
{
    Matrix m(3, 3);
    TEST_ASSERT_EQUAL(3, m.getRows());
    TEST_ASSERT_EQUAL(3, m.getCols());
    TEST_ASSERT_NOT_NULL(m.getArr());
    // Check that all elements are zero-initialized
    for (int i = 0; i < 9; i++) {
        TEST_ASSERT_EQUAL_FLOAT(0.0, m.getArr()[i]);
    }
}

void test_array_constructor()
{
    double data[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    Matrix m(2, 3, data);
    TEST_ASSERT_EQUAL(2, m.getRows());
    TEST_ASSERT_EQUAL(3, m.getCols());
    // Verify deep copy occurred
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL_FLOAT(data[i], m.getArr()[i]);
    }
}

void test_move_constructor()
{
    double data[4] = {1, 2, 3, 4};
    Matrix m_temp(2, 2, data);
    double* original_ptr = m_temp.getArr();

    Matrix m_moved(std::move(m_temp));

    // m_moved should have the original data
    TEST_ASSERT_EQUAL(2, m_moved.getRows());
    TEST_ASSERT_EQUAL(2, m_moved.getCols());
    TEST_ASSERT_EQUAL(original_ptr, m_moved.getArr());

    // m_temp should be nullified
    TEST_ASSERT_EQUAL(0, m_temp.getRows());
    TEST_ASSERT_EQUAL(0, m_temp.getCols());
    TEST_ASSERT_NULL(m_temp.getArr());
}

void test_move_assignment()
{
    double data1[4] = {1, 2, 3, 4};
    double data2[4] = {5, 6, 7, 8};

    Matrix m1(2, 2, data1);
    Matrix m2(2, 2, data2);
    double* m2_ptr = m2.getArr();

    m1 = std::move(m2);

    // m1 should have m2's data
    TEST_ASSERT_EQUAL(2, m1.getRows());
    TEST_ASSERT_EQUAL(2, m1.getCols());
    TEST_ASSERT_EQUAL(m2_ptr, m1.getArr());

    // m2 should be nullified
    TEST_ASSERT_EQUAL(0, m2.getRows());
    TEST_ASSERT_EQUAL(0, m2.getCols());
    TEST_ASSERT_NULL(m2.getArr());
}

void test_get_method()
{
    double data[6] = {1, 2, 3, 4, 5, 6};
    Matrix m(2, 3, data);

    TEST_ASSERT_EQUAL_FLOAT(1.0, m.get(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(2.0, m.get(0, 1));
    TEST_ASSERT_EQUAL_FLOAT(3.0, m.get(0, 2));
    TEST_ASSERT_EQUAL_FLOAT(4.0, m.get(1, 0));
    TEST_ASSERT_EQUAL_FLOAT(5.0, m.get(1, 1));
    TEST_ASSERT_EQUAL_FLOAT(6.0, m.get(1, 2));
}

void test_parenthesis_operator_mutable()
{
    Matrix m(2, 2);
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(1, 0) = 3.0;
    m(1, 1) = 4.0;

    TEST_ASSERT_EQUAL_FLOAT(1.0, m.get(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(2.0, m.get(0, 1));
    TEST_ASSERT_EQUAL_FLOAT(3.0, m.get(1, 0));
    TEST_ASSERT_EQUAL_FLOAT(4.0, m.get(1, 1));
}

void test_parenthesis_operator_const()
{
    double data[4] = {1, 2, 3, 4};
    const Matrix m(2, 2, data);

    TEST_ASSERT_EQUAL_FLOAT(1.0, m(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(2.0, m(0, 1));
    TEST_ASSERT_EQUAL_FLOAT(3.0, m(1, 0));
    TEST_ASSERT_EQUAL_FLOAT(4.0, m(1, 1));
}

void test_multiply_method()
{
    Matrix m4 = m1.multiply(m2);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(19, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(22, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(43, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(50, m4.getArr()[3]);
}

void test_multiply_scalar_method()
{
    Matrix m4 = m1.multiply(2.0);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(2, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(6, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(8, m4.getArr()[3]);
}

void test_add_method()
{
    Matrix m4 = m1.add(m2);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(6, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(8, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(10, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(12, m4.getArr()[3]);
}

void test_subtract_method()
{
    Matrix m4 = m1.subtract(m2);
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(-4, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(-4, m4.getArr()[3]);
}

void test_transpose_method()
{
    Matrix m4 = m3.transpose();
    TEST_ASSERT_EQUAL(3, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_EQUAL(1, m4.getArr()[0]);
    TEST_ASSERT_EQUAL(4, m4.getArr()[1]);
    TEST_ASSERT_EQUAL(2, m4.getArr()[2]);
    TEST_ASSERT_EQUAL(5, m4.getArr()[3]);
    TEST_ASSERT_EQUAL(3, m4.getArr()[4]);
    TEST_ASSERT_EQUAL(6, m4.getArr()[5]);
}

void test_inverse_method()
{
    Matrix m4 = m1.inverse();
    TEST_ASSERT_EQUAL(2, m4.getRows());
    TEST_ASSERT_EQUAL(2, m4.getCols());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -2.0, m4.getArr()[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0, m4.getArr()[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.5, m4.getArr()[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, -0.5, m4.getArr()[3]);
}

void test_identity_matrix()
{
    Matrix I = Matrix::ident(3);
    TEST_ASSERT_EQUAL(3, I.getRows());
    TEST_ASSERT_EQUAL(3, I.getCols());

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                TEST_ASSERT_EQUAL_FLOAT(1.0, I.get(i, j));
            } else {
                TEST_ASSERT_EQUAL_FLOAT(0.0, I.get(i, j));
            }
        }
    }
}

void test_matrix_chain_operations()
{
    // Test chaining: (A + B) * 2
    Matrix result = (m1 + m2) * 2.0;
    TEST_ASSERT_EQUAL(2, result.getRows());
    TEST_ASSERT_EQUAL(2, result.getCols());
    TEST_ASSERT_EQUAL_FLOAT(12.0, result.get(0, 0));
    TEST_ASSERT_EQUAL_FLOAT(16.0, result.get(0, 1));
    TEST_ASSERT_EQUAL_FLOAT(20.0, result.get(1, 0));
    TEST_ASSERT_EQUAL_FLOAT(24.0, result.get(1, 1));
}

void test_matrix_dimension_mismatch_multiply()
{
    double data1[6] = {1, 2, 3, 4, 5, 6};
    double data2[6] = {1, 2, 3, 4, 5, 6};
    Matrix m_2x3(2, 3, data1);
    Matrix m_2x3_other(2, 3, data2);

    // Should return copy of m_2x3 since dimensions don't match
    Matrix result = m_2x3 * m_2x3_other;
    TEST_ASSERT_EQUAL(2, result.getRows());
    TEST_ASSERT_EQUAL(3, result.getCols());
}

void test_matrix_dimension_mismatch_add()
{
    double data1[4] = {1, 2, 3, 4};
    double data2[6] = {1, 2, 3, 4, 5, 6};
    Matrix m_2x2(2, 2, data1);
    Matrix m_2x3(2, 3, data2);

    // Should return copy of m_2x2 since dimensions don't match
    Matrix result = m_2x2 + m_2x3;
    TEST_ASSERT_EQUAL(2, result.getRows());
    TEST_ASSERT_EQUAL(2, result.getCols());
}

void test_matrix_dimension_mismatch_subtract()
{
    double data1[4] = {1, 2, 3, 4};
    double data2[6] = {1, 2, 3, 4, 5, 6};
    Matrix m_2x2(2, 2, data1);
    Matrix m_2x3(2, 3, data2);

    // Should return copy of m_2x2 since dimensions don't match
    Matrix result = m_2x2 - m_2x3;
    TEST_ASSERT_EQUAL(2, result.getRows());
    TEST_ASSERT_EQUAL(2, result.getCols());
}

void test_inverse_non_square()
{
    // Should return copy of m3 since it's not square
    Matrix result = m3.inv();
    TEST_ASSERT_EQUAL(2, result.getRows());
    TEST_ASSERT_EQUAL(3, result.getCols());
}

void test_trace_non_square()
{
    // Should return 0 for non-square matrix
    double trace = m3.trace();
    TEST_ASSERT_EQUAL_FLOAT(0.0, trace);
}

void test_self_assignment()
{
    Matrix m(2, 2, m1_data);
    m = m;  // Self-assignment

    TEST_ASSERT_EQUAL(2, m.getRows());
    TEST_ASSERT_EQUAL(2, m.getCols());
    TEST_ASSERT_EQUAL_FLOAT(1.0, m.get(0, 0));
}

void test_empty_matrix_operations()
{
    Matrix empty1;
    Matrix empty2;

    Matrix result_add = empty1 + empty2;
    TEST_ASSERT_EQUAL(0, result_add.getRows());
    TEST_ASSERT_EQUAL(0, result_add.getCols());

    Matrix result_mult = empty1 * 2.0;
    TEST_ASSERT_EQUAL(0, result_mult.getRows());
    TEST_ASSERT_EQUAL(0, result_mult.getCols());
}

void setUp(void)
{
    //runs before each test
    //must be included for unity to work, even if empty
}

void tearDown(void)
{
    //runs after each test
    //must be included for unity to work, even if empty
}


int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Constructor tests
    RUN_TEST(test_default_constructor);
    RUN_TEST(test_allocating_constructor);
    RUN_TEST(test_array_constructor);
    RUN_TEST(test_copy_constructor);
    RUN_TEST(test_assignment_operator);
    RUN_TEST(test_move_constructor);
    RUN_TEST(test_move_assignment);
    RUN_TEST(test_self_assignment);

    // Accessor tests
    RUN_TEST(test_get_method);
    RUN_TEST(test_parenthesis_operator_mutable);
    RUN_TEST(test_parenthesis_operator_const);

    // Operator tests
    RUN_TEST(test_multiply_operator_matrix);
    RUN_TEST(test_multiply_operator_scalar);
    RUN_TEST(test_add_operator_matrix);
    RUN_TEST(test_subtract_operator_matrix);

    // Method tests
    RUN_TEST(test_multiply_method);
    RUN_TEST(test_multiply_scalar_method);
    RUN_TEST(test_add_method);
    RUN_TEST(test_subtract_method);
    RUN_TEST(test_transpose);
    RUN_TEST(test_transpose_method);
    RUN_TEST(test_inverse_small);
    RUN_TEST(test_inverse_large);
    RUN_TEST(test_inverse_method);
    RUN_TEST(test_trace);

    // Static method tests
    RUN_TEST(test_identity_matrix);

    // Advanced tests
    RUN_TEST(test_matrix_chain_operations);

    // Error handling tests
    RUN_TEST(test_matrix_dimension_mismatch_multiply);
    RUN_TEST(test_matrix_dimension_mismatch_add);
    RUN_TEST(test_matrix_dimension_mismatch_subtract);
    RUN_TEST(test_inverse_non_square);
    RUN_TEST(test_trace_non_square);
    RUN_TEST(test_empty_matrix_operations);

    UNITY_END();
}

