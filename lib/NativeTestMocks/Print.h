#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include <new>

using uint8_t = std::uint8_t;

class Print
{
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t) = 0;

    virtual size_t write(const uint8_t *buf, size_t n)
    {
        size_t w = 0;
        for (size_t i = 0; i < n; ++i)
            w += write(buf[i]);
        return w;
    }

    // Tiny helpers (enough for tests)
    size_t print(const char *s)
    {
        if (!s)
            return 0;
        return write(reinterpret_cast<const uint8_t *>(s), std::strlen(s));
    }

    size_t print(double d, int precision = 2)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", precision, d);
        return print(buf);
    }

    size_t print(int i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d", i);
        return print(buf);
    }

    size_t print(unsigned int i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u", i);
        return print(buf);
    }

    size_t print(long i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%ld", i);
        return print(buf);
    }

    size_t print(unsigned long i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu", i);
        return print(buf);
    }

    size_t println(double d, int precision = 2)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", precision, d);
        return println(buf);
    }

    size_t println(int i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d", i);
        return println(buf);
    }

    size_t println(unsigned int i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u", i);
        return println(buf);
    }

    size_t println(long i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%ld", i);
        return println(buf);
    }

    size_t println(unsigned long i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu", i);
        return println(buf);
    }

    size_t println(size_t i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%zu", i);
        return println(buf);
    }

    // FlashStringHelper support - on native, flash strings are just regular const char*
    size_t print(const void *flashStr) {
        return print((const char*)flashStr);
    }

    size_t println(const void *flashStr) {
        return println((const char*)flashStr);
    }

    size_t println(const char *s)
    {
        size_t n = print(s);
        n += write(reinterpret_cast<const uint8_t *>("\n"), 1);
        return n;
    }

    size_t println()
    {
        return write(reinterpret_cast<const uint8_t *>("\n"), 1);
    }
    // Add these methods inside your Print-compatible class:
    size_t vprintf(const char *fmt, va_list ap)
    {
        if (!fmt)
            return 0;

        // First pass: determine required length (excluding null)
        va_list ap_copy;
        va_copy(ap_copy, ap);
        int needed = vsnprintf(nullptr, 0, fmt, ap_copy);
        va_end(ap_copy);
        if (needed <= 0)
            return 0; // formatting error or empty

        // Fast path: fit into small stack buffer
        if (needed < 128)
        {
            char buf[128];
            int n = vsnprintf(buf, sizeof(buf), fmt, ap);
            if (n <= 0)
                return 0;
            return write(reinterpret_cast<const uint8_t *>(buf), static_cast<size_t>(n));
        }

        // Large output: allocate exact size + NUL, then write once
        char *heap = new (std::nothrow) char[static_cast<size_t>(needed) + 1];
        if (!heap)
            return 0;
        int n = vsnprintf(heap, static_cast<size_t>(needed) + 1, fmt, ap);
        size_t wrote = 0;
        if (n > 0)
            wrote = write(reinterpret_cast<const uint8_t *>(heap), static_cast<size_t>(n));
        delete[] heap;
        return wrote;
    }

    size_t printf(const char *fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        size_t wrote = vprintf(fmt, ap);
        va_end(ap);
        return wrote;
    }

    // Convenience: printf + newline
    size_t printlnf(const char *fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        size_t wrote = vprintf(fmt, ap);
        va_end(ap);
        wrote += write(reinterpret_cast<const uint8_t *>("\n"), 1);
        return wrote;
    }

    virtual void flush() {} // no-op by default (Arduino's default too)
};