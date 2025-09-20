#ifndef DATA_REPORTER_H
#define DATA_REPORTER_H

#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <Arduino.h>                            // for Print
#include "../Logging/LoggingBackend/ILogSink.h" // gives you ILogSink : Print

namespace mmfs
{

    struct DataPoint
    {
        const char *fmt = nullptr;                          // printf format for this value
        const char *label = nullptr;                        // column label
        DataPoint *next = nullptr;                          // next in list
        const void *data = nullptr;                         // pointer to the value
        void (*emit)(Print *, const DataPoint *) = nullptr; // type-erased printer
    };

    // Templated helper to allocate & populate a node
    template <typename T>
    inline DataPoint *make_dp(const char *fmt, const T *p, const char *label)
    {
        auto *d = new DataPoint{};
        d->fmt = fmt;
        d->label = label;
        d->data = p;
        d->emit = [](Print *s, const DataPoint *self)
        {
            const T *val = static_cast<const T *>(self->data);
            // Portable: format into a buffer, then write
            char buf[64];
            int n = snprintf(buf, sizeof(buf), self->fmt, *val);
            if (n > 0)
                s->write(reinterpret_cast<const uint8_t *>(buf), (size_t)n);
        };
        return d;
    }

    class DataReporter
    {
    public:
        static int numReporters;

        DataReporter(const char *name = nullptr);
        virtual ~DataReporter();

        virtual const char *getName() const;
        virtual void setName(const char *n);

        int getNumColumns();
        DataPoint *getDataPoints();
        DataPoint *getLastPoint() { return last; }

    protected:
        uint8_t numColumns = 0;
        DataPoint *first = nullptr, *last = nullptr;

        template <typename T>
        void insertColumn(int place, const char *fmt, T *variable, const char *label);

        template <typename T>
        void addColumn(const char *fmt, T *variable, const char *label);

        void removeColumn(const char *label);

    private:
        char *name = nullptr;
    };

} // namespace mmfs

#include "DataReporter.inl"
#endif // DATA_REPORTER_H
