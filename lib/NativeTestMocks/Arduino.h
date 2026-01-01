#ifndef ARDUINO_H
#define ARDUINO_H
#include <cstdint>
#include <cstdio>
#include <string.h>
#include <chrono>
#include <stdarg.h>
#include "Wire.h"
#include "Print.h"
#ifdef WIN32
#include <windows.h>
#endif
#define SS 10 // random ass numbers lol

#define HIGH 1
#define LOW 0

#define INPUT 1
#define OUTPUT 0

#define LED_BUILTIN 13

uint64_t millis();

void delay(unsigned long ms);

void delay(int ms);

void digitalWrite(int pin, int value);

// Constrain function (Arduino-compatible)
template<typename T>
T constrain(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

class Stream : public Print
{
public:
    void begin(int baud = 9600);
    void end();
    void clearBuffer();
    bool available();
    int read();
    int readBytesUntil(char i, char *buf, size_t s);
    size_t write(uint8_t b) override;
    operator bool() { return true; }

    // For simulating incoming data in tests
    void simulateInput(const char* data);

    char fakeBuffer[1000];
    int cursor = 0;

    // Input buffer for read operations
    char inputBuffer[1000];
    int inputCursor = 0;
    int inputLength = 0;
};

class SerialClass : public Stream
{
};

extern SerialClass Serial;

class CrashReportClass
{
public:
    explicit CrashReportClass() {}
    operator bool() const { return false; }
};
extern CrashReportClass CrashReport;

#endif