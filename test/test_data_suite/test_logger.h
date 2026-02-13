#pragma once

#include <unity.h>
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include "RecordData/Logging/LoggingBackend/ILogSink.h"
#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/Logging/DataLogger.h"
#include "RecordData/Logging/EventLogger.h"

using namespace astra;

namespace test_logger {

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
};

class MemoryFile final : public IFile
{
public:
    MemoryFile(std::string *storage, bool isOpen = true) : storage_(storage), open_(isOpen) {}

    size_t write(uint8_t b) override
    {
        if (!open_ || !storage_)
            return 0;
        storage_->push_back(static_cast<char>(b));
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        if (!open_ || !storage_ || !buffer)
            return 0;
        storage_->append(reinterpret_cast<const char *>(buffer), size);
        return size;
    }

    bool flush() override { return open_; }

    int read() override
    {
        if (!open_ || !storage_ || readPos_ >= storage_->size())
            return -1;
        return static_cast<unsigned char>((*storage_)[readPos_++]);
    }

    int readBytes(uint8_t *buffer, size_t length) override
    {
        if (!open_ || !storage_ || !buffer)
            return 0;
        size_t bytesRead = 0;
        while (bytesRead < length && readPos_ < storage_->size())
            buffer[bytesRead++] = static_cast<uint8_t>((*storage_)[readPos_++]);
        return static_cast<int>(bytesRead);
    }

    int available() override
    {
        if (!open_ || !storage_)
            return 0;
        return static_cast<int>(storage_->size() - readPos_);
    }

    bool seek(uint32_t pos) override
    {
        if (!open_ || !storage_ || pos > storage_->size())
            return false;
        readPos_ = pos;
        return true;
    }

    uint32_t position() override { return static_cast<uint32_t>(readPos_); }
    uint32_t size() override { return storage_ ? static_cast<uint32_t>(storage_->size()) : 0U; }

    bool close() override
    {
        open_ = false;
        return true;
    }

    bool isOpen() const override { return open_; }

private:
    std::string *storage_ = nullptr;
    size_t readPos_ = 0;
    bool open_ = false;
};

class MemoryStorage final : public IStorage
{
public:
    bool begin() override
    {
        if (failBegin_)
            return false;
        begun_ = true;
        return true;
    }

    bool end() override
    {
        begun_ = false;
        return true;
    }

    bool ok() const override { return begun_; }

    IFile *openRead(const char *filename) override
    {
        if (!begun_ || !filename)
            return nullptr;
        auto it = files_.find(filename);
        if (it == files_.end())
            return nullptr;
        return new MemoryFile(&it->second, true);
    }

    IFile *openWrite(const char *filename, bool append = true) override
    {
        if (!begun_ || !filename)
            return nullptr;

        std::string &content = files_[filename];
        if (!append)
            content.clear();
        return new MemoryFile(&content, true);
    }

    bool exists(const char *filename) override
    {
        if (!begun_ || !filename)
            return false;
        return files_.find(filename) != files_.end();
    }

    bool remove(const char *filename) override
    {
        if (!begun_ || !filename)
            return false;
        return files_.erase(filename) > 0;
    }

    bool mkdir(const char *path) override
    {
        (void)path;
        return begun_;
    }

    bool rmdir(const char *path) override
    {
        (void)path;
        return begun_;
    }

    void setFailBegin(bool fail) { failBegin_ = fail; }

    bool hasFile(const std::string &filename) const
    {
        return files_.find(filename) != files_.end();
    }

    std::string readAll(const std::string &filename) const
    {
        auto it = files_.find(filename);
        return it == files_.end() ? std::string() : it->second;
    }

private:
    std::map<std::string, std::string> files_;
    bool begun_ = false;
    bool failBegin_ = false;
};

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
    int begin() override
    {
        initialized = true;
        return 0;
    }
    int update(double currentTime = -1) override
    {
        return 0;
    }
};
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
    int begin() override
    {
        initialized = true;
        return 0;
    }
    int update(double currentTime = -1) override
    {
        return 0;
    }
};
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

void local_setUp(void)
{
}

void local_tearDown(void)
{
}

void test_header_single_reporter(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_append_line_single_reporter_values_and_commas(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_multi_reporter_header_and_row(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_unhealthy_sink_is_skipped(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_empty_reporter_is_handled(void)
{
    local_setUp();
    // A reporter with 0 columns should not produce double-commas or blank columns.
    class EmptyReporter : public DataReporter
    {
    public:
        explicit EmptyReporter(const char *n) : DataReporter(n) {}
        int begin() override { initialized = true; return 0; }
        int update(double currentTime = -1) override { return 0; }
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
    local_tearDown();
}

void test_global_configure_and_instance(void)
{
    local_setUp();
    MockSink sink(true);
    FakeReporter rp("imu");
    ILogSink *sinks[] = {&sink};

    DataLogger::configure(sinks, 1);
    TEST_ASSERT_TRUE(DataLogger::available());

    reset(sink);
    TEST_ASSERT_TRUE(DataLogger::instance().appendLine());
    TEST_ASSERT_FALSE(sink.buf.empty());
    local_tearDown();
}

void test_printHeaderTo_single_sink(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_printHeaderTo_with_prefix(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_printHeaderTo_without_prefix(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_printHeaderTo_unhealthy_sink(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_printHeaderTo_multi_reporter(void)
{
    local_setUp();
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
    local_tearDown();
}

void test_event_logger_unavailable_without_sinks(void)
{
    local_setUp();
    EventLogger logger(nullptr, 0);
    TEST_ASSERT_FALSE(logger.init());
    TEST_ASSERT_FALSE(logger.info("hello"));
    TEST_ASSERT_FALSE(logger.warn("warn"));
    TEST_ASSERT_FALSE(logger.err("err"));
    local_tearDown();
}

void test_event_logger_writes_prefix_and_level(void)
{
    local_setUp();
    MockSink sink(true, true);
    ILogSink *sinks[] = {&sink};

    EventLogger logger(sinks, 1);
    TEST_ASSERT_TRUE(logger.init());
    TEST_ASSERT_TRUE(logger.info("value=%d", 42));

    TEST_ASSERT_NOT_EQUAL(std::string::npos, sink.buf.find("LOG/"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, sink.buf.find("[INFO]"));
    TEST_ASSERT_NOT_EQUAL(std::string::npos, sink.buf.find("value=42"));
    TEST_ASSERT_GREATER_THAN(0, sink.flushCount);
    local_tearDown();
}

void test_event_logger_skips_sink_that_becomes_unhealthy(void)
{
    local_setUp();
    MockSink sink(true, false);
    ILogSink *sinks[] = {&sink};

    EventLogger logger(sinks, 1);
    TEST_ASSERT_TRUE(logger.init());

    sink.healthy = false;
    sink.buf.clear();
    TEST_ASSERT_FALSE(logger.warn("dropped"));
    TEST_ASSERT_TRUE(sink.buf.empty());
    local_tearDown();
}

void test_event_logger_empty_message_returns_false(void)
{
    local_setUp();
    MockSink sink(true, false);
    ILogSink *sinks[] = {&sink};

    EventLogger logger(sinks, 1);
    TEST_ASSERT_TRUE(logger.init());
    sink.buf.clear();

    TEST_ASSERT_FALSE(logger.err(""));
    TEST_ASSERT_TRUE(sink.buf.empty());
    local_tearDown();
}

void test_file_log_sink_begin_fails_without_backend(void)
{
    local_setUp();
    FileLogSink sink("flight.csv", static_cast<IStorage *>(nullptr));
    TEST_ASSERT_FALSE(sink.begin());
    TEST_ASSERT_FALSE(sink.ok());
    local_tearDown();
}

void test_file_log_sink_begin_fails_when_backend_begin_fails(void)
{
    local_setUp();
    MemoryStorage backend;
    backend.setFailBegin(true);

    FileLogSink sink("flight.csv", &backend);
    TEST_ASSERT_FALSE(sink.begin());
    TEST_ASSERT_FALSE(sink.ok());
    local_tearDown();
}

void test_file_log_sink_allocates_suffix_when_filename_exists(void)
{
    local_setUp();
    MemoryStorage backend;
    TEST_ASSERT_TRUE(backend.begin());

    IFile *f0 = backend.openWrite("flight.csv", false);
    IFile *f1 = backend.openWrite("flight_1.csv", false);
    TEST_ASSERT_NOT_NULL(f0);
    TEST_ASSERT_NOT_NULL(f1);
    f0->close();
    f1->close();
    delete f0;
    delete f1;

    FileLogSink sink("flight.csv", &backend, true);
    TEST_ASSERT_TRUE(sink.begin());
    TEST_ASSERT_TRUE(sink.ok());
    TEST_ASSERT_TRUE(sink.wantsPrefix());

    const uint8_t payload[] = {'A', 'B', 'C'};
    TEST_ASSERT_EQUAL(3, sink.write(payload, 3));
    sink.flush();
    sink.end();

    TEST_ASSERT_TRUE(backend.hasFile("flight_2.csv"));
    TEST_ASSERT_EQUAL_STRING_LEN("ABC", backend.readAll("flight_2.csv").c_str(), 3);
    local_tearDown();
}

void test_file_log_sink_no_extension_suffix_path(void)
{
    local_setUp();
    MemoryStorage backend;
    TEST_ASSERT_TRUE(backend.begin());

    IFile *existing = backend.openWrite("session", false);
    TEST_ASSERT_NOT_NULL(existing);
    existing->close();
    delete existing;

    FileLogSink sink("session", &backend);
    TEST_ASSERT_TRUE(sink.begin());
    TEST_ASSERT_TRUE(sink.write(static_cast<uint8_t>('Z')) == 1);
    sink.end();

    TEST_ASSERT_TRUE(backend.hasFile("session_1"));
    TEST_ASSERT_EQUAL_STRING_LEN("Z", backend.readAll("session_1").c_str(), 1);
    local_tearDown();
}

void test_file_log_sink_write_after_end_returns_zero(void)
{
    local_setUp();
    MemoryStorage backend;
    TEST_ASSERT_TRUE(backend.begin());

    FileLogSink sink("final.csv", &backend);
    TEST_ASSERT_TRUE(sink.begin());
    sink.end();

    TEST_ASSERT_EQUAL(0, sink.write(static_cast<uint8_t>('X')));
    uint8_t buffer[2] = {'1', '2'};
    TEST_ASSERT_EQUAL(0, sink.write(buffer, 2));
    local_tearDown();
}

void run_test_logger_tests()
{
    RUN_TEST(test_header_single_reporter);
    RUN_TEST(test_append_line_single_reporter_values_and_commas);
    RUN_TEST(test_multi_reporter_header_and_row);
    RUN_TEST(test_unhealthy_sink_is_skipped);
    RUN_TEST(test_empty_reporter_is_handled);
    RUN_TEST(test_global_configure_and_instance);
    RUN_TEST(test_printHeaderTo_single_sink);
    RUN_TEST(test_printHeaderTo_with_prefix);
    RUN_TEST(test_printHeaderTo_without_prefix);
    RUN_TEST(test_printHeaderTo_unhealthy_sink);
    RUN_TEST(test_printHeaderTo_multi_reporter);
    RUN_TEST(test_event_logger_unavailable_without_sinks);
    RUN_TEST(test_event_logger_writes_prefix_and_level);
    RUN_TEST(test_event_logger_skips_sink_that_becomes_unhealthy);
    RUN_TEST(test_event_logger_empty_message_returns_false);
    RUN_TEST(test_file_log_sink_begin_fails_without_backend);
    RUN_TEST(test_file_log_sink_begin_fails_when_backend_begin_fails);
    RUN_TEST(test_file_log_sink_allocates_suffix_when_filename_exists);
    RUN_TEST(test_file_log_sink_no_extension_suffix_path);
    RUN_TEST(test_file_log_sink_write_after_end_returns_zero);
}

} // namespace test_logger
