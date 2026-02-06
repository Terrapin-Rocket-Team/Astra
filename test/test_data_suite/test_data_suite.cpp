#include <unity.h>

#include "test_data_reporter.h"
#include "test_simple_data_reporter.h"
#include "test_logger.h"
#include "test_storage.h"
#include "test_hash.h"

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

    test_data_reporter::run_test_data_reporter_tests();
    test_simple_data_reporter::run_test_simple_data_reporter_tests();
    test_logger::run_test_logger_tests();
    test_storage::run_test_storage_tests();
    test_hash::run_test_hash_tests();

    UNITY_END();
    return 0;
}
