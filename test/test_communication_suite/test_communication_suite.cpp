#include <unity.h>

#include "test_serial_router.h"
#include "test_cmd_handler.h"
#include "test_hitl_parser.h"
#include "test_sitl.h"

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

    test_serial_router::run_test_serial_router_tests();
    test_cmd_handler::run_test_cmd_handler_tests();
    test_hitl_parser::run_test_hitl_parser_tests();
    test_sitl::run_test_sitl_tests();

    UNITY_END();
    return 0;
}
