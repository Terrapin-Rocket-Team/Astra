#include <unity.h>
#include <string>
#include <cstring>
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"

using namespace astra;

// Mock Print for testing DataPoint emission
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

// Concrete test reporter
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
};

// setUp and tearDown
void test_data_reporter_setUp(void)
{
    // Reset static counter if needed
    DataReporter::numReporters = 0;
}

void test_data_reporter_tearDown(void)
{
}

// --- Constructor Tests ---

void test_data_reporter_test_constructor_with_name(void)
{
    TestReporter reporter("TestName");
    TEST_ASSERT_EQUAL_STRING("TestName", reporter.getName());
}

void test_data_reporter_test_constructor_without_name(void)
{
    TestReporter reporter;
    const char *name = reporter.getName();

    // Should be "Reporter #1" (or similar)
    TEST_ASSERT_NOT_NULL(name);
    TEST_ASSERT_TRUE(strstr(name, "Reporter #") != nullptr);
}

void test_data_reporter_test_constructor_increments_counter(void)
{
    int initial = DataReporter::numReporters;
    TestReporter reporter1;
    TEST_ASSERT_EQUAL(initial + 1, DataReporter::numReporters);

    TestReporter reporter2;
    TEST_ASSERT_EQUAL(initial + 2, DataReporter::numReporters);
}

// --- Name Tests ---

void test_data_reporter_test_getName(void)
{
    TestReporter reporter("MyReporter");
    TEST_ASSERT_EQUAL_STRING("MyReporter", reporter.getName());
}

void test_data_reporter_test_setName(void)
{
    TestReporter reporter("OldName");
    reporter.setName("NewName");
    TEST_ASSERT_EQUAL_STRING("NewName", reporter.getName());
}

void test_data_reporter_test_setName_multiple_times(void)
{
    TestReporter reporter("Name1");
    reporter.setName("Name2");
    TEST_ASSERT_EQUAL_STRING("Name2", reporter.getName());

    reporter.setName("Name3");
    TEST_ASSERT_EQUAL_STRING("Name3", reporter.getName());
}

// --- Column Tests ---

void test_data_reporter_test_addColumn_single(void)
{
    TestReporter reporter;
    float val = 1.5f;
    reporter.addColumn<float>("%.2f", &val, "testFloat");

    TEST_ASSERT_EQUAL(1, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("testFloat", dp->label);
    TEST_ASSERT_EQUAL_STRING("%.2f", dp->fmt);
    TEST_ASSERT_EQUAL(&val, dp->data);
    TEST_ASSERT_NULL(dp->next);
}

void test_data_reporter_test_addColumn_multiple(void)
{
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
}

void test_data_reporter_test_getLastPoint(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_NOT_NULL(last);
    TEST_ASSERT_EQUAL_STRING("value3", last->label);
    TEST_ASSERT_NULL(last->next);
}

void test_data_reporter_test_removeColumn_first(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    reporter.removeColumn("value1");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_NOT_NULL(dp);
    TEST_ASSERT_EQUAL_STRING("value2", dp->label);
}

void test_data_reporter_test_removeColumn_middle(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    reporter.removeColumn("value2");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("value1", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("value3", dp->label);
}

void test_data_reporter_test_removeColumn_last(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    reporter.removeColumn("value3");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_EQUAL_STRING("value2", last->label);
}

void test_data_reporter_test_removeColumn_nonexistent(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    int before = reporter.getNumColumns();
    reporter.removeColumn("nonexistent");

    TEST_ASSERT_EQUAL(before, reporter.getNumColumns());
}

void test_data_reporter_test_removeColumn_empty_list(void)
{
    TestReporter reporter;

    reporter.removeColumn("anything");

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
}

void test_data_reporter_test_clearColumns(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    TEST_ASSERT_EQUAL(3, reporter.getNumColumns());

    reporter.clearColumns();

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
    TEST_ASSERT_NULL(reporter.getDataPoints());
    TEST_ASSERT_NULL(reporter.getLastPoint());
}

void test_data_reporter_test_insertColumn_at_beginning(void)
{
    TestReporter reporter;
    float val = 1.0f;
    reporter.addColumn<float>("%.2f", &val, "first");

    int newVal = 999;
    reporter.insertColumn<int>(0, "%d", &newVal, "inserted");

    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("inserted", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("first", dp->label);
}

void test_data_reporter_test_insertColumn_at_end(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    double newVal = 99.9;
    reporter.insertColumn<double>(999, "%.1f", &newVal, "appended");

    TEST_ASSERT_EQUAL(4, reporter.getNumColumns());

    DataPoint *last = reporter.getLastPoint();
    TEST_ASSERT_EQUAL_STRING("appended", last->label);
}

void test_data_reporter_test_insertColumn_in_middle(void)
{
    TestReporter reporter;
    float v1 = 1.0f, v2 = 2.0f;
    reporter.addColumn<float>("%.1f", &v1, "first");
    reporter.addColumn<float>("%.1f", &v2, "third");

    float v_mid = 1.5f;
    reporter.insertColumn<float>(1, "%.1f", &v_mid, "second");

    TEST_ASSERT_EQUAL(3, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("first", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("second", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("third", dp->label);
}

// --- DataPoint Emission Tests ---

void test_data_reporter_test_emit_float(void)
{
    MockPrint mock;
    float val = 1.234f;
    DataPoint *dp = make_dp<float>("%.2f", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("1.23", mock.buf.c_str());

    delete dp;
}

void test_data_reporter_test_emit_int(void)
{
    MockPrint mock;
    int val = 42;
    DataPoint *dp = make_dp<int>("%d", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("42", mock.buf.c_str());

    delete dp;
}

void test_data_reporter_test_emit_double(void)
{
    MockPrint mock;
    double val = 3.14159;
    DataPoint *dp = make_dp<double>("%.3f", &val, "test");

    dp->emit(&mock, dp);

    TEST_ASSERT_EQUAL_STRING("3.142", mock.buf.c_str());

    delete dp;
}

void test_data_reporter_test_emit_with_different_formats(void)
{
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
}

void test_data_reporter_test_emit_negative_values(void)
{
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
}

// --- Initialization Tests ---

void test_data_reporter_test_isInitialized_default(void)
{
    TestReporter reporter;
    TEST_ASSERT_FALSE(reporter.isInitialized());
}

void test_data_reporter_test_begin_sets_initialized(void)
{
    TestReporter reporter;
    reporter.beginResult = 0;  // Success

    reporter.begin();

    TEST_ASSERT_TRUE(reporter.beginCalled);
    TEST_ASSERT_TRUE(reporter.isInitialized());
}

void test_data_reporter_test_begin_error_does_not_set_initialized(void)
{
    TestReporter reporter;
    reporter.beginResult = -1;  // Failure

    reporter.begin();

    TEST_ASSERT_TRUE(reporter.beginCalled);
    TEST_ASSERT_FALSE(reporter.isInitialized());
}

void test_data_reporter_test_bool_operator(void)
{
    TestReporter reporter;
    TEST_ASSERT_FALSE((bool)reporter);

    reporter.beginResult = 0;
    reporter.begin();

    TEST_ASSERT_TRUE((bool)reporter);
}

// --- Update Tests ---

void test_data_reporter_test_update_called(void)
{
    TestReporter reporter;
    reporter.update();

    TEST_ASSERT_TRUE(reporter.updateCalled);
}

void test_data_reporter_test_update_with_time_parameter(void)
{
    TestReporter reporter;
    reporter.update(123.456);

    TEST_ASSERT_TRUE(reporter.updateCalled);
}

// --- AutoUpdate Tests ---

void test_data_reporter_test_autoUpdate_default(void)
{
    TestReporter reporter;
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
}

void test_data_reporter_test_setAutoUpdate(void)
{
    TestReporter reporter;

    reporter.setAutoUpdate(false);
    TEST_ASSERT_FALSE(reporter.getAutoUpdate());

    reporter.setAutoUpdate(true);
    TEST_ASSERT_TRUE(reporter.getAutoUpdate());
}

// --- Edge Cases ---

void test_data_reporter_test_empty_reporter(void)
{
    TestReporter reporter("empty");

    TEST_ASSERT_EQUAL(0, reporter.getNumColumns());
    TEST_ASSERT_NULL(reporter.getDataPoints());
    TEST_ASSERT_NULL(reporter.getLastPoint());
}

void test_data_reporter_test_multiple_reporters_independent(void)
{
    TestReporter r1("reporter1");
    TestReporter r2("reporter2");

    r1.setupColumns();

    TEST_ASSERT_EQUAL(3, r1.getNumColumns());
    TEST_ASSERT_EQUAL(0, r2.getNumColumns());
}

void test_data_reporter_test_column_chain_integrity_after_operations(void)
{
    TestReporter reporter;
    reporter.setupColumns();

    // Remove middle, add new, remove first
    reporter.removeColumn("value2");

    int newVal = 100;
    reporter.addColumn<int>("%d", &newVal, "value4");

    reporter.removeColumn("value1");

    // Should have value3, value4
    TEST_ASSERT_EQUAL(2, reporter.getNumColumns());

    DataPoint *dp = reporter.getDataPoints();
    TEST_ASSERT_EQUAL_STRING("value3", dp->label);

    dp = dp->next;
    TEST_ASSERT_EQUAL_STRING("value4", dp->label);

    TEST_ASSERT_NULL(dp->next);
}

void test_data_reporter_test_large_number_of_columns(void)
{
    TestReporter reporter;

    const int numCols = 100;
    float values[numCols];

    for (int i = 0; i < numCols; i++) {
        values[i] = (float)i;
        char label[32];
        snprintf(label, sizeof(label), "col%d", i);
        reporter.addColumn<float>("%.0f", &values[i], label);
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
}

void test_data_reporter_test_destructor_cleanup(void)
{
    // Just test that this doesn't crash
    {
        TestReporter reporter("temp");
        reporter.setupColumns();
    }
    // Reporter destroyed, memory should be freed
    TEST_ASSERT_TRUE(true);
}

// Main function
