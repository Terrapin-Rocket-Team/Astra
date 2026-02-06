#include <unity.h>

#include "test_mahony.h"
#include "test_kalman_filter.h"
#include "test_mounting_transform.h"

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

    test_mahony::run_test_mahony_tests();
    test_kalman_filter::run_test_kalman_filter_tests();
    test_mounting_transform::run_test_mounting_transform_tests();

    UNITY_END();
    return 0;
}
