#include <unity.h>

#include "test_astra.h"

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

    test_astra::run_test_astra_tests();

    UNITY_END();
    return 0;
}
