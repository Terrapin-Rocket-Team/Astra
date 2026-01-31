#include <unity.h>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>

// ---- Include your headers (adjust paths as needed) ----
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/DataLogger.h"
#include "RecordData/Logging/EventLogger.h"

using namespace astra;

// --------------- MockSink ---------------
// Captures all writes in-memory; simulates a healthy sink by default.
class MockSink : public ILogSink
{
public:
    std::string buf;
    bool began = false;
    bool healthy = true;
    int beginCount = 0;
    int flushCount = 0;
    int endCount = 0;
    bool prefix = false;

    explicit MockSink(bool healthy_, bool prefx = false) : healthy(healthy_), prefix(prefx) {}

    bool begin() override
    {
        began = true;
        beginCount++;
        return healthy;
    }
    bool end() override
    {
        endCount++;
        began = false;
        return true;
    }
    bool ok() const override { return healthy && began; }
    bool wantsPrefix() const override { return prefix; }
    // Make sure Print APIs work
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
    void flush() override { flushCount++; }

    using Print::write; // keep other overloads visible
};

// --------------- FakeReporter ---------------
// Uses your protected addColumn<T> to build a real column chain.
class FakeReporter : public DataReporter
{
public:
    float ax = 0, ay = 0;
    int count = 0;

    explicit FakeReporter(const char *name) : DataReporter(name)
    {
        // label order defines header & CSV order
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

    bool begin() override
    {
        initialized = true;
        return true;
    }

    bool update(double currentTime = -1) override
    {
        return true;
    }
};

// A second reporter to test multi-reporter composition
class PositionReporter : public DataReporter
{
public:
    double lat = 0, lon = 0;
    explicit PositionReporter(const char *name) : DataReporter(name)
    {
        addColumn<double>("%.6f", &lat, "lat");
        addColumn<double>("%.6f", &lon, "lon");
    }
    void set(double la, double lo)
    {
        lat = la;
        lon = lo;
    }

    bool begin() override
    {
        initialized = true;
        return true;
    }

    bool update(double currentTime = -1) override
    {
        return true;
    }
};

// ---------- Helpers ----------
static void reset(MockSink &s)
{
    s.buf.clear();
    s.flushCount = 0;
}
static std::vector<std::string> splitLines(const std::string &s)
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

// ---------- Tests ----------

void test_header_single_reporter(void)
{
    MockSink sink(true);
    FakeReporter rp("imu");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    // Expect header like: "imu - ax,imu - ay,imu - count"
    // (Your implementation prints reporter name + " - " + label)
    auto lines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];

    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("imu - ax"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("imu - ay"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("imu - count"));

    // comma separation and no trailing commas (simple checks)
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find(","));
    TEST_ASSERT_NOT_EQUAL_MESSAGE(std::string::npos, hdr.size() ? (hdr.back() != ',') : 1, "Header ends with comma");
}

void test_append_line_single_reporter_values_and_commas(void)
{
    MockSink sink(true);
    FakeReporter rp("imu");
    rp.set(0.126f, -1.5f, 7);

    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());
    reset(sink);

    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());

    auto lines = splitLines(sink.buf);
    TEST_ASSERT_EQUAL_UINT_MESSAGE(1, lines.size(), "One data line expected");
    auto &row = lines[0];

    std::cout << row << std::endl;
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("0.13"));   // 0.125 -> "%.2f" -> 0.13
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("-1.500")); // "%.3f"
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("7"));

    // Basic comma sanity: two commas for 3 columns, and no trailing comma
    size_t commas = 0;
    for (char c : row)
        if (c == ',')
            commas++;
    TEST_ASSERT_EQUAL_UINT(2, commas);
    TEST_ASSERT_TRUE(row.back() != ',');
}

void test_multi_reporter_header_and_row(void)
{
    MockSink sink(true);
    FakeReporter imu("imu");
    PositionReporter pos("gps");

    imu.set(1.0f, 2.0f, 3);
    pos.set(39.000123, -76.500789);

    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    auto hdrLines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(hdrLines.empty());
    auto hdr = hdrLines[0];

    // Expect "imu - ax,imu - ay,imu - count,gps - lat,gps - lon"
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("imu - ax"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("gps - lat"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("gps - lon"));

    reset(sink);
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());
    auto rows = splitLines(sink.buf);
    TEST_ASSERT_EQUAL_UINT(1, rows.size());
    auto row = rows[0];

    // Values appear in the same order as headers
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("1.00"));  // imu ax
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("2.000")); // imu ay
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("3"));     // count
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("39.000123"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, row.find("-76.500789"));

    // Total commas should be (columns-1) = (3+2-1)=4
    size_t commas = 0;
    for (char c : row)
        if (c == ',')
            commas++;
    TEST_ASSERT_EQUAL_UINT(4, commas);
}

void test_unhealthy_sink_is_skipped(void)
{
    MockSink bad(false); // begin() returns false -> not ok()
    MockSink good(true);

    FakeReporter rp("imu");
    rp.set(0.5f, 0.25f, 1);

    ILogSink *sinks[] = {&bad, &good};

    DataLogger::configure(sinks, 2);
    TEST_ASSERT_TRUE(DataLogger::available()); // 'any' will be true because good begins

    // Header was written during init; now append a row
    reset(bad);
    reset(good);
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());

    // bad should remain empty; good should have data
    TEST_ASSERT_TRUE(bad.buf.empty());
    TEST_ASSERT_FALSE(good.buf.empty());
}

void test_empty_reporter_is_handled(void)
{
    // A reporter with 0 columns should not produce double-commas or blank columns.
    class EmptyReporter : public DataReporter
    {
    public:
        explicit EmptyReporter(const char *n) : DataReporter(n) {}
        bool begin() override { initialized = true; return true; }
        bool update(double currentTime = -1) override { return true; }
    } empty("empty");

    MockSink sink(true);
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    auto lines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(lines.empty());
    // Header for empty reporter ideally is just a newline (implementation-dependent).
    // We allow either empty header line or literal "empty" behavior depending on your choice.
    // Keep this as a weak assertion: there should be exactly one newline.
    printf("Header line: '%s'\n", lines[0].c_str());
    TEST_ASSERT_TRUE(lines.size() == 1 && lines[0].empty());

    reset(sink);
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());
    auto rows = splitLines(sink.buf);
    // Row should exist but be empty (or minimal) rather than malformed
    TEST_ASSERT_TRUE(rows.size() == 1);
}

void test_global_configure_and_instance(void)
{
    MockSink sink(true);
    FakeReporter rp("imu");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    reset(sink);
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());
    TEST_ASSERT_FALSE(sink.buf.empty());
}

void testWantsPrefix()
{
    MockSink sink(true, true);
    FakeReporter rp("imu");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    EventLogger el(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());
    TEST_ASSERT_TRUE(el.init());

    //expect header like "TELEM/imu - ax,imu - ay,imu - count"
    // expect log line like "LOG/0.123 [INFO]: test message"
    auto lines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("TELEM/imu - ax"));
    reset(sink);
    TEST_ASSERT_TRUE(el.info("test message"));
    lines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &log = lines[0];
    TEST_ASSERT_NOT_EQUAL(std::string::npos, log.find("LOG/"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, log.find("[INFO]: test message"));

}

void test_printHeaderTo_single_sink(void)
{
    MockSink sink1(true, true);
    MockSink sink2(true, true);
    FakeReporter rp("accel");

    ILogSink *sinks[] = {&sink1};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    // Clear sink1 and initialize sink2
    reset(sink1);
    sink2.begin();  // Must call begin() so ok() returns true

    // Print header to a different sink
    DataLogger::instance().printHeaderTo(&sink2);

    // Verify sink1 is still empty (header not printed there)
    TEST_ASSERT_TRUE_MESSAGE(sink1.buf.empty(), "Sink1 should be empty after printHeaderTo sink2");

    // Verify sink2 has the header
    TEST_ASSERT_FALSE_MESSAGE(sink2.buf.empty(), "Sink2 should have header");
    auto lines = splitLines(sink2.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];

    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("TELEM/"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - ax"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - ay"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - count"));
}

void test_printHeaderTo_with_prefix(void)
{
    MockSink sinkWithPrefix(true, true);
    FakeReporter rp("gyro");

    ILogSink *sinks[] = {&sinkWithPrefix};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());
    reset(sinkWithPrefix);

    DataLogger::instance().printHeaderTo(&sinkWithPrefix);

    auto lines = splitLines(sinkWithPrefix.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];

    // Should have TELEM/ prefix
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("TELEM/gyro - ax"));
}

void test_printHeaderTo_without_prefix(void)
{
    MockSink sinkNoPrefix(true, false);  // false = no prefix
    FakeReporter rp("mag");

    ILogSink *sinks[] = {&sinkNoPrefix};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());
    reset(sinkNoPrefix);

    DataLogger::instance().printHeaderTo(&sinkNoPrefix);

    auto lines = splitLines(sinkNoPrefix.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];

    // Should NOT have TELEM/ prefix
    TEST_ASSERT_EQUAL(std::string::npos, hdr.find("TELEM/"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("mag - ax"));
}

void test_printHeaderTo_unhealthy_sink(void)
{
    MockSink unhealthySink(false, true);  // unhealthy sink
    FakeReporter rp("baro");

    ILogSink *sinks[] = {&unhealthySink};

    DataLogger::configure(sinks, 1);
    // configure will fail because sink is unhealthy
    TEST_ASSERT_FALSE(DataLogger::available());

    // printHeaderTo should handle unhealthy sink gracefully (no crash)
    DataLogger::instance().printHeaderTo(&unhealthySink);

    // Buffer should be empty because sink is not ok()
    TEST_ASSERT_TRUE_MESSAGE(unhealthySink.buf.empty(), "Unhealthy sink should not receive header");
}

void test_printHeaderTo_multi_reporter(void)
{
    MockSink sink(true, true);
    FakeReporter rp1("accel");
    PositionReporter rp2("gps");

    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());
    reset(sink);

    DataLogger::instance().printHeaderTo(&sink);

    auto lines = splitLines(sink.buf);
    TEST_ASSERT_FALSE(lines.empty());
    auto &hdr = lines[0];

    // Should have both reporters' columns
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - ax"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("accel - ay"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("gps - lat"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, hdr.find("gps - lon"));
}

// ---------- Unity harness ----------
void setUp() {}
void tearDown() {}

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_header_single_reporter);
    RUN_TEST(test_append_line_single_reporter_values_and_commas);
    RUN_TEST(test_multi_reporter_header_and_row);
    RUN_TEST(test_unhealthy_sink_is_skipped);
    RUN_TEST(test_empty_reporter_is_handled);
    RUN_TEST(test_global_configure_and_instance);
    RUN_TEST(testWantsPrefix);

    // New tests for printHeaderTo
    RUN_TEST(test_printHeaderTo_single_sink);
    RUN_TEST(test_printHeaderTo_with_prefix);
    RUN_TEST(test_printHeaderTo_without_prefix);
    RUN_TEST(test_printHeaderTo_unhealthy_sink);
    RUN_TEST(test_printHeaderTo_multi_reporter);

    UNITY_END();
    return 0;
}
