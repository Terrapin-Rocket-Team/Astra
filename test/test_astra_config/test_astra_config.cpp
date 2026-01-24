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

void test_datareporter_auto_registration_single(void)
{
    // DataReporter should auto-register on construction
    TestReporter reporter("test1");

    // Verify reporter is registered with DataLogger (even if not initialized)
    TEST_ASSERT_EQUAL(1, DataLogger::instance().getNumReporters());
}

void test_datareporter_auto_registration_multiple(void)
{
    TestReporter reporter1("test1");
    TestReporter reporter2("test2");
    TestReporter reporter3("test3");

    // All should be auto-registered
    TEST_ASSERT_EQUAL(3, DataLogger::instance().getNumReporters());
}

void test_datareporter_unregistration(void)
{
    {
        TestReporter reporter1("test1");
        TestReporter reporter2("test2");

        TEST_ASSERT_EQUAL(2, DataLogger::instance().getNumReporters());
    } // reporters go out of scope and should unregister

    // After destruction, reporters should be unregistered
    TEST_ASSERT_EQUAL(0, DataLogger::instance().getNumReporters());
}

void test_datareporter_max_capacity(void)
{
    DataReporter *reporters[32]; // MAX_REPORTERS is 32

    // Create 32 reporters (max capacity)
    for (int i = 0; i < 32; i++)
    {
        char name[20];
        snprintf(name, sizeof(name), "reporter%d", i);
        reporters[i] = new TestReporter(name);
    }

    TEST_ASSERT_EQUAL(32, DataLogger::instance().getNumReporters());

    // Cleanup
    for (int i = 0; i < 32; i++)
    {
        delete reporters[i];
    }

    TEST_ASSERT_EQUAL(0, DataLogger::instance().getNumReporters());
}

void test_datareporter_exceeds_capacity(void)
{
    DataReporter *reporters[35];

    // Create 35 reporters (exceeds max capacity of 32)
    for (int i = 0; i < 35; i++)
    {
        char name[20];
        snprintf(name, sizeof(name), "reporter%d", i);
        reporters[i] = new TestReporter(name);
    }

    // Should cap at 32
    TEST_ASSERT_EQUAL(32, DataLogger::instance().getNumReporters());

    // Cleanup
    for (int i = 0; i < 35; i++)
    {
        delete reporters[i];
    }
}

void test_astra_config_basic(void)
{
    AstraConfig config;

    // Test basic configuration methods still work
    config.withSensorUpdateRate(100.0)
          .withLoggingRate(50.0)
          .withPredictRate(60.0);

    TEST_ASSERT_TRUE(true); // Verify chaining works
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

void test_datareporter_integration_with_datalogger(void)
{
    // Create a minimal State with no sensors
    Filter *filter = nullptr;
    State state(filter, nullptr);

    // Create custom reporters (they auto-register)
    TestReporter customReporter1("custom1");
    TestReporter customReporter2("custom2");
    customReporter1.setValue(123.45f);
    customReporter2.setValue(678.90f);

    // Create a mock sink
    MockSink sink(true);
    ILogSink *sinks[] = {&sink};

    // Configure Astra
    AstraConfig config;
    config.withState(&state)
          .withDataLogs(sinks, 1);

    // Initialize Astra
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
    // Cleanup any remaining reporters
    // Note: In a real scenario, we'd need to manually unregister or delete all reporters
    // Since we're using scoped objects in tests, they should clean up automatically
}

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_datareporter_auto_registration_single);
    RUN_TEST(test_datareporter_auto_registration_multiple);
    RUN_TEST(test_datareporter_unregistration);
    RUN_TEST(test_datareporter_max_capacity);
    RUN_TEST(test_datareporter_exceeds_capacity);
    RUN_TEST(test_astra_config_basic);
    RUN_TEST(test_datareporter_integration_with_datalogger);

    UNITY_END();
    return 0;
}
