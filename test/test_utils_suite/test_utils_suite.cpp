#include <unity.h>

#include "test_circular_buffer.h"
#include "test_blinkbuzz.h"

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

    test_circular_buffer::run_test_circular_buffer_tests();
    test_blinkbuzz::run_test_blinkbuzz_tests();

    UNITY_END();
    return 0;
}
