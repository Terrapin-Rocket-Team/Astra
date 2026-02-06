#include <unity.h>

#include "test_matrix.h"
#include "test_vector.h"
#include "test_quaternion.h"

void setUp(void)
{
    // Called before each test
}

void tearDown(void)
{
    // Called after each test
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    test_matrix::run_test_matrix_tests();
    test_vector::run_test_vector_tests();
    test_quaternion::run_test_quaternion_tests();

    UNITY_END();
    return 0;
}
