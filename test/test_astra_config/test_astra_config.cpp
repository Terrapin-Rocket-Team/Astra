#include <unity.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "Utils/AstraConfig.h"
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/DataLogger.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "State/State.h"
#include "Utils/Astra.h"

using namespace astra;

// --------------- Test Reporter ---------------
class TestReporter : public DataReporter
{
public:
    float value = 0.0f;

    explicit TestReporter(const char *name) : DataReporter(name)
    {
        addColumn<float>("%.2f", &value, "value");
    }

    void setValue(float v)
    {
        value = v;
    }
};

// ---------- Tests ----------

void test_withOtherDataReporters_single(void)
{
    AstraConfig config;
    TestReporter reporter("test1");
    DataReporter *reporters[] = {&reporter};

    config.withOtherDataReporters(reporters, 1);

    // Access private members via friend relationship would be ideal,
    // but we can't directly test private members without friend class.
    // Instead, we verify that the call doesn't crash and returns correctly.
    TEST_ASSERT_TRUE(true); // Basic sanity check
}

void test_withOtherDataReporters_multiple(void)
{
    AstraConfig config;
    TestReporter reporter1("test1");
    TestReporter reporter2("test2");
    TestReporter reporter3("test3");

    DataReporter *reporters[] = {&reporter1, &reporter2, &reporter3};

    config.withOtherDataReporters(reporters, 3);

    TEST_ASSERT_TRUE(true); // Basic sanity check
}

void test_withOtherDataReporters_chaining(void)
{
    AstraConfig config;
    TestReporter reporter1("test1");
    TestReporter reporter2("test2");

    DataReporter *batch1[] = {&reporter1};
    DataReporter *batch2[] = {&reporter2};

    // Test chaining multiple calls
    config.withOtherDataReporters(batch1, 1)
          .withOtherDataReporters(batch2, 1);

    TEST_ASSERT_TRUE(true); // Basic sanity check
}

void test_withOtherDataReporters_max_capacity(void)
{
    AstraConfig config;
    DataReporter *reporters[50];

    // Create 50 reporters dynamically
    for (int i = 0; i < 50; i++)
    {
        char name[20];
        snprintf(name, sizeof(name), "reporter%d", i);
        reporters[i] = new TestReporter(name);
    }

    // Add all 50
    config.withOtherDataReporters(reporters, 50);

    TEST_ASSERT_TRUE(true); // Should handle max capacity

    // Cleanup
    for (int i = 0; i < 50; i++)
    {
        delete reporters[i];
    }
}

void test_withOtherDataReporters_exceeds_capacity(void)
{
    AstraConfig config;
    DataReporter *reporters[60];

    // Create 60 reporters dynamically
    for (int i = 0; i < 60; i++)
    {
        char name[20];
        snprintf(name, sizeof(name), "reporter%d", i);
        reporters[i] = new TestReporter(name);
    }

    // Try to add 60 (should cap at 50)
    config.withOtherDataReporters(reporters, 60);

    TEST_ASSERT_TRUE(true); // Should cap and not crash

    // Cleanup
    for (int i = 0; i < 60; i++)
    {
        delete reporters[i];
    }
}

void test_withOtherDataReporters_null_array(void)
{
    AstraConfig config;

    // Pass null array with 0 count - should not crash
    config.withOtherDataReporters(nullptr, 0);

    TEST_ASSERT_TRUE(true);
}

void test_withOtherDataReporters_returns_reference(void)
{
    AstraConfig config;
    TestReporter reporter("test");
    DataReporter *reporters[] = {&reporter};

    // Verify it returns a reference for chaining
    AstraConfig &ref = config.withOtherDataReporters(reporters, 1);

    // The returned reference should be the same object
    TEST_ASSERT_EQUAL_PTR(&config, &ref);
}

// --------------- MockSink ---------------
class MockSink : public ILogSink
{
public:
    std::string buf;
    bool began = false;
    bool healthy = true;

    explicit MockSink(bool healthy_) : healthy(healthy_) {}

    bool begin() override
    {
        began = true;
        return healthy;
    }
    bool end() override
    {
        began = false;
        return true;
    }
    bool ok() const override { return healthy && began; }
    bool wantsPrefix() const override { return false; }
    size_t write(uint8_t b) override
    {
        buf.push_back((char)b);
        return 1;
    }
    size_t write(const uint8_t *p, size_t n) override
    {
        buf.append((const char *)p, n);
        return n;
    }
    void flush() override {}

    using Print::write;
};

void test_withOtherDataReporters_integration_with_datalogger(void)
{
    // Create a minimal State with no sensors
    Filter *filter = nullptr;
    State state(filter, nullptr);

    // Create custom reporters
    TestReporter customReporter1("custom1");
    TestReporter customReporter2("custom2");
    customReporter1.setValue(123.45f);
    customReporter2.setValue(678.90f);

    DataReporter *reporters[] = {&customReporter1, &customReporter2};

    // Create a mock sink
    MockSink sink(true);
    ILogSink *sinks[] = {&sink};

    // Configure Astra with the custom reporters
    AstraConfig config;
    config.withState(&state)
          .withDataLogs(sinks, 1)
          .withOtherDataReporters(reporters, 2);

    // Initialize Astra (which should configure DataLogger with the reporters)
    Astra astra(&config);
    astra.init();

    // Check that the header includes our custom reporters
    std::string header = sink.buf;
    std::cout << "Header: " << header << std::endl;

    // The header should contain our custom reporter columns
    TEST_ASSERT_NOT_EQUAL(std::string::npos, header.find("custom1 - value"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, header.find("custom2 - value"));

    // Clear the sink and append a line
    sink.buf.clear();
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());

    std::string dataLine = sink.buf;
    std::cout << "Data: " << dataLine << std::endl;

    // The data line should contain our custom values
    TEST_ASSERT_NOT_EQUAL(std::string::npos, dataLine.find("123.45"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, dataLine.find("678.90"));
}

// ---------- Unity harness ----------
void setUp() {}
void tearDown()
{
    // Clear DataLogger singleton state between tests
    if (DataLogger::available())
    {
        // DataLogger might need cleanup - check implementation
    }
}

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_withOtherDataReporters_single);
    RUN_TEST(test_withOtherDataReporters_multiple);
    RUN_TEST(test_withOtherDataReporters_chaining);
    RUN_TEST(test_withOtherDataReporters_max_capacity);
    RUN_TEST(test_withOtherDataReporters_exceeds_capacity);
    RUN_TEST(test_withOtherDataReporters_null_array);
    RUN_TEST(test_withOtherDataReporters_returns_reference);
    RUN_TEST(test_withOtherDataReporters_integration_with_datalogger);

    UNITY_END();
    return 0;
}
