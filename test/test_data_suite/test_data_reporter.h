#pragma once

#include <unity.h>
#include <string>
#include <cstring>
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

using namespace astra;

namespace test_data_reporter {

class MockPrint : public Print
{
public:
    std::string buf;
    size_t write(uint8_t b) override
    {
        buf.push_back((char)b);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override
    {
        buf.append((const char *)buffer, size);
        return size;
    }
    void clear() { buf.clear(); }
};
class TestReporter : public DataReporter
{
public:
    float value1 = 0.0f;
    int value2 = 0;
    double value3 = 0.0;
    bool beginCalled = false;
    bool updateCalled = false;
    int beginResult = 0;
    int updateResult = 0;
    explicit TestReporter(const char *name = nullptr) : DataReporter(name)
    {
    }
    void setupColumns()
    {
        addColumn<float>("%.2f", &value1, "value1");
        addColumn<int>("%d", &value2, "value2");
        addColumn<double>("%.3f", &value3, "value3");
    }
    int begin() override
    {
        beginCalled = true;
        initialized = (beginResult == 0);
        return beginResult;
    }
    int update(double currentTime = -1) override
    {
        updateCalled = true;
        return updateResult;
    }

    // Public wrappers for testing protected methods
    template <typename T>
    void testAddColumn(const char *fmt, T *variable, const char *label)
    {
        addColumn<T>(fmt, variable, label);
    }

    template <typename T>
    void testInsertColumn(int place, const char *fmt, T *variable, const char *label)
    {
        insertColumn<T>(place, fmt, variable, label);
    }

    void testRemoveColumn(const char *label)
    {
        removeColumn(label);
    }

    void testClearColumns()
    {
        clearColumns();
    }
};

void local_setUp(void)
{
    // Reset static counter if needed
    DataReporter::numReporters = 0;
}

void local_tearDown(void)
{
}

void test_constructor_with_name(void)
{
    local_setUp();
    TestReporter reporter("TestName");
    TEST_ASSERT_EQUAL_STRING("TestName", reporter.getName());
    local_tearDown();
}

void test_constructor_without_name(void)
{
    local_setUp();
    TestReporter reporter;
    const char *name = reporter.getName();

    // Should be "Reporter #1" (or similar)
    TEST_ASSERT_NOT_NULL(name);
    TEST_ASSERT_TRUE(strstr(name, "Reporter #") != nullptr);
    local_tearDown();
}

void test_constructor_increments_counter(void)
{
    local_setUp();
    int initial = DataReporter::numReporters;
    TestReporter reporter1;
    TEST_ASSERT_EQUAL(initial + 1, DataReporter::numReporters);

    TestReporter reporter2;
    TEST_ASSERT_EQUAL(initial + 2, DataReporter::numReporters);
    local_tearDown();
}

void test_getName(void)
{
    local_setUp();
    TestReporter reporter("MyReporter");
    TEST_ASSERT_EQUAL_STRING("MyReporter", reporter.getName());
    local_tearDown();
}

void test_setName(void)
{
    local_setUp();
    TestReporter reporter("OldName");
    reporter.setName("NewName");
    TEST_ASSERT_EQUAL_STRING("NewName", reporter.getName());
    local_tearDown();
}

void test_setName_multiple_times(void)
{
    local_setUp();
    TestReporter reporter("Name1");
    reporter.setName("Name2");
    TEST_ASSERT_EQUAL_STRING("Name2", reporter.getName());

    reporter.setName("Name3");
    TEST_ASSERT_EQUAL_STRING("Name3", reporter.getName());
    local_tearDown();
}

void test_addColumn_single(void)
{
    local_setUp();
    TestReporter reporter;
    float val = 1.5f;
    reporter.testAddColumn<float>("%.2f", &val, "testFloat");

    TEST_ASSERT_EQUAL(1, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("testFloat", dp->label);
    TEST_ASSERT_EQUAL_STRING("%.2f", dp->fmt);
    TEST_ASSERT_EQUAL(&val, dp->data);
    TEST_ASSERT_NULL(dp->next);
    local_tearDown();
}

void test_addColumn_multiple(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    TEST_ASSERT_EQUAL(3, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("value1", dp->label);

    dp = dp->next;
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("value2", dp->label);

    dp = dp->next;
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("value3", dp->label);

    TEST_ASSERT_NULL(dp->next);
    local_tearDown();
}

void test_getLastPoint(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_NOT_NULL(last);
    TEST_ASSERT_EQUAL_STRING("value3", last->label);
    TEST_ASSERT_NULL(last->next);
    local_tearDown();
}

void test_removeColumn_first(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    reporter.testRemoveColumn("value1");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("value2", dp->label);
    local_tearDown();
}

void test_removeColumn_middle(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    reporter.testRemoveColumn("value2");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("value1", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("value3", dp->label);
    local_tearDown();
}

void test_removeColumn_last(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    reporter.testRemoveColumn("value3");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_EQUAL_STRING("value2", last->label);
    local_tearDown();
}

void test_removeColumn_nonexistent(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    int before = reporter.getNumColumns();
    reporter.testRemoveColumn("nonexistent");

    TEST_ASSERT_EQUAL(before, reporter.getNumColumns());
    local_tearDown();
}

void test_removeColumn_empty_list(void)
{
    local_setUp();
    TestReporter reporter;

    reporter.testRemoveColumn("anything");

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
    local_tearDown();
}

void test_clearColumns(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    TEST_ASSERT_EQUAL(3, reporter.getNumColumns());

    reporter.testClearColumns();

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
    TEST_ASSERT_NULL(reporter.getDataPoints());
    TEST_ASSERT_NULL(reporter.getLastPoint());
    local_tearDown();
}

void test_insertColumn_at_beginning(void)
{
    local_setUp();
    TestReporter reporter;
    float val = 1.0f;
    reporter.testAddColumn<float>("%.2f", &val, "first");

    int newVal = 999;
    reporter.testInsertColumn<int>(0, "%d", &newVal, "inserted");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("inserted", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("first", dp->label);
    local_tearDown();
}

void test_insertColumn_at_end(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    double newVal = 99.9;
    reporter.testInsertColumn<double>(999, "%.1f", &newVal, "appended");

    TEST_ASSERT_EQUAL(4, reporter.getNumColumns());

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_EQUAL_STRING("appended", last->label);
    local_tearDown();
}

void test_insertColumn_in_middle(void)
{
    local_setUp();
    TestReporter reporter;
    float v1 = 1.0f, v2 = 2.0f;
    reporter.testAddColumn<float>("%.1f", &v1, "first");
    reporter.testAddColumn<float>("%.1f", &v2, "third");

    float v_mid = 1.5f;
    reporter.testInsertColumn<float>(1, "%.1f", &v_mid, "second");

    TEST_ASSERT_EQUAL(3, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("first", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("second", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("third", dp->label);
    local_tearDown();
}

void test_emit_float(void)
{
    local_setUp();
    MockPrint mock;
    float val = 1.234f;
    DataPoint *dp = make_dp<float>("%.2f", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("1.23", mock.buf.c_str());

    delete dp;
    local_tearDown();
}

void test_emit_int(void)
{
    local_setUp();
    MockPrint mock;
    int val = 42;
    DataPoint *dp = make_dp<int>("%d", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("42", mock.buf.c_str());

    delete dp;
    local_tearDown();
}

void test_emit_double(void)
{
    local_setUp();
    MockPrint mock;
    double val = 3.14159;
    DataPoint *dp = make_dp<double>("%.3f", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("3.142", mock.buf.c_str());

    delete dp;
    local_tearDown();
}

void test_emit_with_different_formats(void)
{
    local_setUp();
    MockPrint mock;
    int val = 255;

    // Hexadecimal format
    DataPoint *dp1 = make_dp<int>("%x", &val, "hex");
    dp1->emit(&mock, dp1);
    TEST_ASSERT_EQUAL_STRING("ff", mock.buf.c_str());

    mock.clear();

    // Octal format
    DataPoint *dp2 = make_dp<int>("%o", &val, "oct");
    dp2->emit(&mock, dp2);
    TEST_ASSERT_EQUAL_STRING("377", mock.buf.c_str());

    delete dp1;
    delete dp2;
    local_tearDown();
}

void test_emit_negative_values(void)
{
    local_setUp();
    MockPrint mock;

    float neg_float = -1.5f;
    DataPoint *dp1 = make_dp<float>("%.1f", &neg_float, "neg_f");
    dp1->emit(&mock, dp1);
    TEST_ASSERT_EQUAL_STRING("-1.5", mock.buf.c_str());

    mock.clear();

    int neg_int = -42;
    DataPoint *dp2 = make_dp<int>("%d", &neg_int, "neg_i");
    dp2->emit(&mock, dp2);
    TEST_ASSERT_EQUAL_STRING("-42", mock.buf.c_str());

    delete dp1;
    delete dp2;
    local_tearDown();
}

void test_isInitialized_default(void)
{
    local_setUp();
    TestReporter reporter;
    TEST_ASSERT_FALSE(reporter.isInitialized());
    local_tearDown();
}

void test_begin_sets_initialized(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.beginResult = 0;  // Success

    reporter.begin();

    TEST_ASSERT_TRUE(reporter.beginCalled);
    TEST_ASSERT_TRUE(reporter.isInitialized());
    local_tearDown();
}

void test_begin_error_does_not_set_initialized(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.beginResult = -1;  // Failure

    reporter.begin();

    TEST_ASSERT_TRUE(reporter.beginCalled);
    TEST_ASSERT_FALSE(reporter.isInitialized());
    local_tearDown();
}

void test_bool_operator(void)
{
    local_setUp();
    TestReporter reporter;
    TEST_ASSERT_FALSE((bool)reporter);

    reporter.beginResult = 0;
    reporter.begin();

    TEST_ASSERT_TRUE((bool)reporter);
    local_tearDown();
}

void test_update_called(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.update();

    TEST_ASSERT_TRUE(reporter.updateCalled);
    local_tearDown();
}

void test_update_with_time_parameter(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.update(123.456);

    TEST_ASSERT_TRUE(reporter.updateCalled);
    local_tearDown();
}

void test_autoUpdate_default(void)
{
    local_setUp();
    TestReporter reporter;
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
    local_tearDown();
}

void test_setAutoUpdate(void)
{
    local_setUp();
    TestReporter reporter;

    reporter.setAutoUpdate(false);
    TEST_ASSERT_FALSE(reporter.getAutoUpdate());

    reporter.setAutoUpdate(true);
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
    local_tearDown();
}

void test_empty_reporter(void)
{
    local_setUp();
    TestReporter reporter("empty");

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
    TEST_ASSERT_NULL(reporter.getDataPoints());
    TEST_ASSERT_NULL(reporter.getLastPoint());
    local_tearDown();
}

void test_multiple_reporters_independent(void)
{
    local_setUp();
    TestReporter r1("reporter1");
    TestReporter r2("reporter2");

    r1.setupColumns();

    TEST_ASSERT_EQUAL(3, r1.getNumColumns());
    TEST_ASSERT_EQUAL(0, r2.getNumColumns());
    local_tearDown();
}

void test_column_chain_integrity_after_operations(void)
{
    local_setUp();
    TestReporter reporter;
    reporter.setupColumns();

    // Remove middle, add new, remove first
    reporter.testRemoveColumn("value2");

    int newVal = 100;
    reporter.testAddColumn<int>("%d", &newVal, "value4");

    reporter.testRemoveColumn("value1");

    // Should have value3, value4
    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("value3", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("value4", dp->label);

    TEST_ASSERT_NULL(dp->next);
    local_tearDown();
}

void test_large_number_of_columns(void)
{
    local_setUp();
    TestReporter reporter;

    const int numCols = 100;
    float values[numCols];

    for (int i = 0; i < numCols; i++) {
        values[i] = (float)i;
        char label[32];
        snprintf(label, sizeof(label), "col%d", i);
        reporter.testAddColumn<float>("%.0f", &values[i], label);
    }

    TEST_ASSERT_EQUAL(numCols, reporter.getNumColumns());

    // Verify chain integrity
    DataPoint *dp = reporter.getDataPoints();
    int count = 0;
    while (dp != nullptr) {
        count++;
        dp = dp->next;
    }

    TEST_ASSERT_EQUAL(numCols, count);
    local_tearDown();
}

void test_destructor_cleanup(void)
{
    local_setUp();
    // Just test that this doesn't crash
    {
        TestReporter reporter("temp");
        reporter.setupColumns();
    }
    // Reporter destroyed, memory should be freed
    TEST_ASSERT_TRUE(true);
    local_tearDown();
}

void run_test_data_reporter_tests()
{
    RUN_TEST(test_constructor_with_name);
    RUN_TEST(test_constructor_without_name);
    RUN_TEST(test_constructor_increments_counter);
    RUN_TEST(test_getName);
    RUN_TEST(test_setName);
    RUN_TEST(test_setName_multiple_times);
    RUN_TEST(test_addColumn_single);
    RUN_TEST(test_addColumn_multiple);
    RUN_TEST(test_getLastPoint);
    RUN_TEST(test_removeColumn_first);
    RUN_TEST(test_removeColumn_middle);
    RUN_TEST(test_removeColumn_last);
    RUN_TEST(test_removeColumn_nonexistent);
    RUN_TEST(test_removeColumn_empty_list);
    RUN_TEST(test_clearColumns);
    RUN_TEST(test_insertColumn_at_beginning);
    RUN_TEST(test_insertColumn_at_end);
    RUN_TEST(test_insertColumn_in_middle);
    RUN_TEST(test_emit_float);
    RUN_TEST(test_emit_int);
    RUN_TEST(test_emit_double);
    RUN_TEST(test_emit_with_different_formats);
    RUN_TEST(test_emit_negative_values);
    RUN_TEST(test_isInitialized_default);
    RUN_TEST(test_begin_sets_initialized);
    RUN_TEST(test_begin_error_does_not_set_initialized);
    RUN_TEST(test_bool_operator);
    RUN_TEST(test_update_called);
    RUN_TEST(test_update_with_time_parameter);
    RUN_TEST(test_autoUpdate_default);
    RUN_TEST(test_setAutoUpdate);
    RUN_TEST(test_empty_reporter);
    RUN_TEST(test_multiple_reporters_independent);
    RUN_TEST(test_column_chain_integrity_after_operations);
    RUN_TEST(test_large_number_of_columns);
    RUN_TEST(test_destructor_cleanup);
}

} // namespace test_data_reporter
