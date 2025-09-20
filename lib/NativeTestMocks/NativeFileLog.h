#pragma once
#include <fstream>
#include <string>
#include "../../src/RecordData/Logging/ILogSink.h"

using namespace mmfs;
class NativeFileLog : public ILogSink
{
    std::string path_;
    std::ofstream ofs_;
    bool started_ = false;

public:
    explicit NativeFileLog(std::string path) : path_(std::move(path)) {}

    bool begin() override
    {
        // open in binary append; create if not exists
        ofs_.open(path_, std::ios::binary | std::ios::out | std::ios::app);
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

    bool ok() override
    {
        return started_ && ofs_.good();
    }

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

    using Print::write;
};
