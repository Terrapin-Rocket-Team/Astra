#pragma once

#include <unity.h>
#include <string>
#include <cstring>
#include "RecordData/DataReporter/SimpleDataReporter.h"
#include "RecordData/DataReporter/DataReporter.h"

using namespace astra;

namespace test_simple_data_reporter {

// Global variables to track callback calls
static bool init_called = false;
static bool update_called = false;
static int init_call_count = 0;
static int update_call_count = 0;

// Test callback functions for float
static bool floatInitCallback() {
    init_called = true;
    init_call_count++;
    return true;
}

static bool floatInitFailCallback() {
    init_called = true;
    init_call_count++;
    return false;
}

static float floatUpdateCallback() {
    update_called = true;
    update_call_count++;
    return 42.5f;
}

// Test callback functions for int
static bool intInitCallback() {
    init_called = true;
    init_call_count++;
    return true;
}

static int intUpdateCallback() {
    update_called = true;
    update_call_count++;
    return 100;
}

// Test callback functions for double
static bool doubleInitCallback() {
    init_called = true;
    init_call_count++;
    return true;
}

static double doubleUpdateCallback() {
    update_called = true;
    update_call_count++;
    return 3.14159;
}

void local_setUp(void)
{
    // Reset static counter and tracking variables
    DataReporter::numReporters = 0;
    init_called = false;
    update_called = false;
    init_call_count = 0;
    update_call_count = 0;
}

void local_tearDown(void)
{
}

void test_simple_constructor_with_defaults(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter;
    
    TEST_ASSERT_EQUAL_STRING("Simple Data Reporter", reporter.getName());
    TEST_ASSERT_EQUAL(1, reporter.getNumColumns());
    local_tearDown();
}

void test_simple_constructor_with_custom_name(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter("Temperature");
    
    TEST_ASSERT_EQUAL_STRING("Temperature", reporter.getName());
    local_tearDown();
}

void test_simple_constructor_with_all_parameters(void)
{
    local_setUp();
    auto reporter = SimpleDataReporter<float>(
        "CustomReporter",
        "%.3f",
        "temp_c",
        floatInitCallback,
        floatUpdateCallback,
        25.0f
    );
    
    TEST_ASSERT_EQUAL_STRING("CustomReporter", reporter.getName());
    TEST_ASSERT_EQUAL(1, reporter.getNumColumns());
    
    // Check column setup
    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("temp_c", dp->label);
    TEST_ASSERT_EQUAL_STRING("%.3f", dp->fmt);
    TEST_ASSERT_NULL(dp->next);
    local_tearDown();
}

void test_simple_begin_calls_init_callback(void)
{
  local_setUp();
  SimpleDataReporter<float> reporter(
      "Test",
      "%.2f",
      "value",
      floatInitCallback,
      floatUpdateCallback,
      0.0f);

  TEST_ASSERT_FALSE(init_called);

  int result = reporter.begin();

  TEST_ASSERT_TRUE(init_called);
  TEST_ASSERT_EQUAL(1, init_call_count);
  TEST_ASSERT_EQUAL(0, result);
  TEST_ASSERT_TRUE(reporter.isInitialized());
  local_tearDown();
}

void test_simple_begin_with_null_callback(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        nullptr,
        floatUpdateCallback,
        0.0f
    );
    
    int result = reporter.begin();
    
    // Should handle nullptr gracefully
    TEST_ASSERT_FALSE(init_called);
    TEST_ASSERT_NOT_EQUAL(0, result);  // Should return error
    TEST_ASSERT_FALSE(reporter.isInitialized());
    local_tearDown();
}

void test_simple_begin_with_failing_callback(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitFailCallback,
        floatUpdateCallback,
        0.0f
    );
    
    int result = reporter.begin();
    
    TEST_ASSERT_TRUE(init_called);
    TEST_ASSERT_NOT_EQUAL(0, result);  // Should return error
    TEST_ASSERT_FALSE(reporter.isInitialized());
    local_tearDown();
}

void test_simple_update_calls_callback(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    reporter.begin();
    
    TEST_ASSERT_FALSE(update_called);
    
    int result = reporter.update();
    
    TEST_ASSERT_TRUE(update_called);
    TEST_ASSERT_EQUAL(1, update_call_count);
    TEST_ASSERT_EQUAL(0, result);
    local_tearDown();
}

void test_simple_update_with_time_parameter(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    reporter.begin();
    
    int result = reporter.update(123.456);
    
    TEST_ASSERT_TRUE(update_called);
    TEST_ASSERT_EQUAL(0, result);
    local_tearDown();
}

void test_simple_update_with_null_callback(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        nullptr,
        0.0f
    );
    
    reporter.begin();
    
    int result = reporter.update();
    
    TEST_ASSERT_FALSE(update_called);
    TEST_ASSERT_NOT_EQUAL(0, result);  // Should return error
    local_tearDown();
}

void test_simple_update_modifies_logged_variable(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    reporter.begin();
    reporter.update();
    
    // Get the data point and check its value
    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    
    // The value should be updated to 42.5f (from floatUpdateCallback)
    const float *value = static_cast<const float *>(dp->data);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.5f, *value);
    local_tearDown();
}

void test_simple_multiple_updates(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    reporter.begin();
    
    reporter.update();
    TEST_ASSERT_EQUAL(1, update_call_count);
    
    reporter.update();
    TEST_ASSERT_EQUAL(2, update_call_count);
    
    reporter.update();
    TEST_ASSERT_EQUAL(3, update_call_count);
    local_tearDown();
}

void test_simple_int_type(void)
{
    local_setUp();
    SimpleDataReporter<int> reporter(
        "IntReporter",
        "%d",
        "count",
        intInitCallback,
        intUpdateCallback,
        0
    );
    
    TEST_ASSERT_EQUAL_STRING("IntReporter", reporter.getName());
    
    reporter.begin();
    TEST_ASSERT_TRUE(init_called);
    
    reporter.update();
    TEST_ASSERT_TRUE(update_called);
    
    // Check the value
    DataPoint *dp = reporter.getDataPoints();
    const int *value = static_cast<const int *>(dp->data);
    TEST_ASSERT_EQUAL(100, *value);
    local_tearDown();
}

void test_simple_double_type(void)
{
    local_setUp();
    SimpleDataReporter<double> reporter(
        "DoubleReporter",
        "%.5f",
        "pi",
        doubleInitCallback,
        doubleUpdateCallback,
        0.0
    );
    
    TEST_ASSERT_EQUAL_STRING("DoubleReporter", reporter.getName());
    
    reporter.begin();
    TEST_ASSERT_TRUE(init_called);
    
    reporter.update();
    TEST_ASSERT_TRUE(update_called);
    
    // Check the value
    DataPoint *dp = reporter.getDataPoints();
    const double *value = static_cast<const double *>(dp->data);
    TEST_ASSERT_DOUBLE_WITHIN(0.00001, 3.14159, *value);
    local_tearDown();
}

void test_simple_default_value_initialization(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        nullptr,
        nullptr,
        99.9f
    );
    
    // Check that default value is set
    DataPoint *dp = reporter.getDataPoints();
    const float *value = static_cast<const float *>(dp->data);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 99.9f, *value);
    local_tearDown();
}

void test_simple_custom_format_string(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.4f",
        "precision",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("%.4f", dp->fmt);
    local_tearDown();
}

void test_simple_custom_label(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "custom_label_123",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("custom_label_123", dp->label);
    local_tearDown();
}

void test_simple_bool_operator(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter(
        "Test",
        "%.2f",
        "value",
        floatInitCallback,
        floatUpdateCallback,
        0.0f
    );
    
    TEST_ASSERT_FALSE((bool)reporter);
    
    reporter.begin();
    
    TEST_ASSERT_TRUE((bool)reporter);
    local_tearDown();
}

void test_simple_auto_update_default(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter;
    
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
    local_tearDown();
}

void test_simple_set_auto_update(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter;
    
    reporter.setAutoUpdate(false);
    TEST_ASSERT_FALSE(reporter.getAutoUpdate());
    
    reporter.setAutoUpdate(true);
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
    local_tearDown();
}

void test_simple_destructor_cleanup(void)
{
    local_setUp();
    // Just test that this doesn't crash
    {
        SimpleDataReporter<float> reporter(
            "Temp",
            "%.2f",
            "value",
            floatInitCallback,
            floatUpdateCallback,
            0.0f
        );
        reporter.begin();
        reporter.update();
    }
    // Reporter destroyed, memory should be freed
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void test_simple_multiple_reporters_independent(void)
{
    local_setUp();
    SimpleDataReporter<float> r1("reporter1", "%.2f", "val1", floatInitCallback, floatUpdateCallback, 0.0f);
    SimpleDataReporter<int> r2("reporter2", "%d", "val2", intInitCallback, intUpdateCallback, 0);
    
    TEST_ASSERT_EQUAL_STRING("reporter1", r1.getName());
    TEST_ASSERT_EQUAL_STRING("reporter2", r2.getName());
    
    TEST_ASSERT_EQUAL(1, r1.getNumColumns());
    TEST_ASSERT_EQUAL(1, r2.getNumColumns());
    
    // Check that columns are independent
    DataPoint *dp1 = r1.getDataPoints();
    DataPoint *dp2 = r2.getDataPoints();
    
    TEST_ASSERT_EQUAL_STRING("val1", dp1->label);
    TEST_ASSERT_EQUAL_STRING("val2", dp2->label);
    local_tearDown();
}

void test_simple_setName_after_construction(void)
{
    local_setUp();
    SimpleDataReporter<float> reporter("OldName");
    
    TEST_ASSERT_EQUAL_STRING("OldName", reporter.getName());
    
    reporter.setName("NewName");
    
    TEST_ASSERT_EQUAL_STRING("NewName", reporter.getName());
    local_tearDown();
}

void run_test_simple_data_reporter_tests()
{
    RUN_TEST(test_simple_constructor_with_defaults);
    RUN_TEST(test_simple_constructor_with_custom_name);
    RUN_TEST(test_simple_constructor_with_all_parameters);
    RUN_TEST(test_simple_begin_calls_init_callback);
    RUN_TEST(test_simple_begin_with_null_callback);
    RUN_TEST(test_simple_begin_with_failing_callback);
    RUN_TEST(test_simple_update_calls_callback);
    RUN_TEST(test_simple_update_with_time_parameter);
    RUN_TEST(test_simple_update_with_null_callback);
    RUN_TEST(test_simple_update_modifies_logged_variable);
    RUN_TEST(test_simple_multiple_updates);
    RUN_TEST(test_simple_int_type);
    RUN_TEST(test_simple_double_type);
    RUN_TEST(test_simple_default_value_initialization);
    RUN_TEST(test_simple_custom_format_string);
    RUN_TEST(test_simple_custom_label);
    RUN_TEST(test_simple_bool_operator);
    RUN_TEST(test_simple_auto_update_default);
    RUN_TEST(test_simple_set_auto_update);
    RUN_TEST(test_simple_destructor_cleanup);
    RUN_TEST(test_simple_multiple_reporters_independent);
    RUN_TEST(test_simple_setName_after_construction);
}

} // namespace test_simple_data_reporter
