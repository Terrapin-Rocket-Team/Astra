#include <unity.h>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>

// Include the necessary headers
#include "Communication/SerialMessageRouter.h"
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/DataLogger.h"

using namespace astra;

// --------------- MockSink ---------------
class MockSink : public ILogSink
{
public:
    std::string buf;
    bool began = false;
    bool healthy = true;
    bool prefix = false;

    explicit MockSink(bool healthy_, bool prefx = false) : healthy(healthy_), prefix(prefx) {}

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
    bool wantsPrefix() const override { return prefix; }

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

// --------------- FakeReporter ---------------
class FakeReporter : public DataReporter
{
public:
    float ax = 0, ay = 0;
    int count = 0;

    explicit FakeReporter(const char *name) : DataReporter(name)
    {
        addColumn<float>("%.2f", &ax, "ax");
        addColumn<float>("%.3f", &ay, "ay");
        addColumn<int>("%d", &count, "count");
    }

    void set(float ax_, float ay_, int c)
    {
        ax = ax_;
        ay = ay_;
        count = c;
    }
};

// --------------- Test Streams ---------------
Stream testStream;
Stream stream1, stream2;
std::string lastCommandMessage;
Stream* lastCommandSource = nullptr;

void handleCommandMessage(const char* message, const char* prefix, Stream* source)
{
    if (!message || !source)
        return;

    lastCommandMessage = message;
    lastCommandSource = source;

    // Check if command is "HEADER"
    if (strcmp(message, "HEADER") == 0)
    {
        // Create a temporary PrintLog wrapper to send header to the requesting stream
        PrintLog tempLog(*source, true);  // true = wants prefix
        if (tempLog.begin())
        {
            if (DataLogger::available())
            {
                DataLogger::instance().printHeaderTo(&tempLog);
            }
            tempLog.end();
        }
    }
}

// --------------- Helper Functions ---------------
std::vector<std::string> splitLines(const std::string &s)
{
    std::vector<std::string> out;
    std::string cur;
    for (char c : s)
    {
        if (c == '\n')
        {
            out.push_back(cur);
            cur.clear();
        }
        else
            cur.push_back(c);
    }
    if (!cur.empty())
        out.push_back(cur);
    return out;
}

// --------------- Tests ---------------

void setUp()
{
    testStream.clearBuffer();
    stream1.clearBuffer();
    stream2.clearBuffer();
    lastCommandMessage.clear();
    lastCommandSource = nullptr;
}

void tearDown() {}

void test_cmd_header_basic()
{
    // Setup DataLogger with a reporter
    MockSink sink(true, true);
    FakeReporter rp("test");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);

    // Setup SerialMessageRouter with CMD/ listener
    SerialMessageRouter router;
    router.withInterface(&testStream)
          .withListener("CMD/", handleCommandMessage);

    // Send CMD/HEADER message
    testStream.simulateInput("CMD/HEADER\n");
    router.update();

    // Verify callback was invoked
    TEST_ASSERT_EQUAL_STRING("HEADER", lastCommandMessage.c_str());
    TEST_ASSERT_EQUAL(&testStream, lastCommandSource);

    // Verify header was written to the stream
    auto lines = splitLines(testStream.fakeBuffer);
    TEST_ASSERT_FALSE_MESSAGE(lines.empty(), "Stream should have received header");

    if (!lines.empty())
    {
        auto &hdr = lines[0];
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("TELEM/"));
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("test - ax"));
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("test - ay"));
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("test - count"));
    }
}

void test_cmd_header_multiple_requests()
{
    // Setup DataLogger
    MockSink sink(true, true);
    FakeReporter rp("sensor");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);

    // Setup router
    SerialMessageRouter router;
    router.withInterface(&testStream)
          .withListener("CMD/", handleCommandMessage);

    // Send first CMD/HEADER
    testStream.simulateInput("CMD/HEADER\n");
    router.update();

    auto lines1 = splitLines(testStream.fakeBuffer);
    TEST_ASSERT_EQUAL(1, lines1.size());

    // Clear and send second CMD/HEADER
    testStream.clearBuffer();
    testStream.simulateInput("CMD/HEADER\n");
    router.update();

    auto lines2 = splitLines(testStream.fakeBuffer);
    TEST_ASSERT_EQUAL(1, lines2.size());

    // Both should be identical headers
    TEST_ASSERT_EQUAL_STRING(lines1[0].c_str(), lines2[0].c_str());
}

void test_cmd_unknown_command()
{
    // Setup DataLogger
    MockSink sink(true, true);
    FakeReporter rp("test");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);

    // Setup router
    SerialMessageRouter router;
    router.withInterface(&testStream)
          .withListener("CMD/", handleCommandMessage);

    // Send unknown command
    testStream.simulateInput("CMD/UNKNOWN\n");
    router.update();

    // Callback should be invoked
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", lastCommandMessage.c_str());

    // But no output should be sent (unknown command)
    TEST_ASSERT_EQUAL_MESSAGE('\0', testStream.fakeBuffer[0], "Unknown command should not produce output");
}

void test_cmd_header_with_multiple_reporters()
{
    // Setup DataLogger with multiple reporters
    MockSink sink(true, true);
    FakeReporter rp1("accel");
    FakeReporter rp2("gyro");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);

    // Setup router
    SerialMessageRouter router;
    router.withInterface(&testStream)
          .withListener("CMD/", handleCommandMessage);

    // Send CMD/HEADER
    testStream.simulateInput("CMD/HEADER\n");
    router.update();

    // Verify both reporters are in the header
    auto lines = splitLines(testStream.fakeBuffer);
    TEST_ASSERT_FALSE(lines.empty());

    if (!lines.empty())
    {
        auto &hdr = lines[0];
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - ax"));
        TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("gyro - ax"));
    }
}

void test_cmd_header_only_to_requesting_stream()
{
    // Setup DataLogger
    MockSink sink(true, true);
    FakeReporter rp("test");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);

    // Setup router with both streams
    SerialMessageRouter router;
    router.withInterface(&stream1)
          .withInterface(&stream2)
          .withListener("CMD/", handleCommandMessage);

    // Send CMD/HEADER only on stream1
    stream1.simulateInput("CMD/HEADER\n");
    router.update();

    // Verify only stream1 received the header (non-empty buffer)
    TEST_ASSERT_NOT_EQUAL_MESSAGE('\0', stream1.fakeBuffer[0], "Stream1 should have received header");
    TEST_ASSERT_EQUAL_MESSAGE('\0', stream2.fakeBuffer[0], "Stream2 should NOT have received header");

    auto lines = splitLines(stream1.fakeBuffer);
    TEST_ASSERT_FALSE(lines.empty());
    if (!lines.empty())
    {
        TEST_ASSERT_NOT_EQUAL(std::string::npos, lines[0].find("TELEM/"));
    }
}

// --------------- Main Test Runner ---------------
int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_cmd_header_basic);
    RUN_TEST(test_cmd_header_multiple_requests);
    RUN_TEST(test_cmd_unknown_command);
    RUN_TEST(test_cmd_header_with_multiple_reporters);
    RUN_TEST(test_cmd_header_only_to_requesting_stream);

    UNITY_END();
    return 0;
}
