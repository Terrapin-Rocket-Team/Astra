#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>

class NativeFileLog : public astra::ILogSink
{
    std::string path_;
    std::ofstream ofs_;
    bool started_ = false;

public:
    explicit NativeFileLog(std::string path) : path_(std::move(path)) {}
    ~NativeFileLog() { end(); }

    NativeFileLog(const NativeFileLog &) = delete;
    NativeFileLog &operator=(const NativeFileLog &) = delete;
    NativeFileLog(NativeFileLog &&other) noexcept
        : path_(std::move(other.path_)), ofs_(std::move(other.ofs_)), started_(other.started_)
    {
        other.started_ = false;
    }
    NativeFileLog &operator=(NativeFileLog &&other) noexcept
    {
        if (this != &other)
        {
            end();
            path_ = std::move(other.path_);
            ofs_ = std::move(other.ofs_);
            started_ = other.started_;
            other.started_ = false;
        }
        return *this;
    }

    bool begin() override
    {
        ofs_.open(path_, std::ios::binary | std::ios::out | std::ios::app);
        // Optional: speed up large writes in native tests
        static std::vector<char> buf(256 * 1024);
        ofs_.rdbuf()->pubsetbuf(buf.data(), buf.size());
        started_ = ofs_.is_open();
        return started_;
    }

    bool end() override
    {
        if (ofs_.is_open())
            ofs_.close();
        started_ = false;
        return true;
    }

    bool ok() const override { return started_ && ofs_.good(); }

    bool wantsPrefix() const override { return false; }

    void flush() override
    {
        if (ofs_.is_open())
            ofs_.flush();
    }

    size_t write(uint8_t b) override
    {
        if (!ofs_.is_open())
            return 0;
        ofs_.write(reinterpret_cast<const char *>(&b), 1);
        return ofs_.good() ? 1 : 0;
    }

    size_t write(const uint8_t *buf, size_t n) override
    {
        if (!ofs_.is_open())
            return 0;
        ofs_.write(reinterpret_cast<const char *>(buf), static_cast<std::streamsize>(n));
        return ofs_.good() ? n : 0;
    }

    using Print::write; // keep other Print overloads visible
};
