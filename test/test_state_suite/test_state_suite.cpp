#include <unity.h>

#include "test_state.h"
#include "test_default_state.h"
#include "test_astra_config.h"

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

    test_state::run_test_state_tests();
    test_default_state::run_test_default_state_tests();
    test_astra_config::run_test_astra_config_tests();

    UNITY_END();
    return 0;
}
