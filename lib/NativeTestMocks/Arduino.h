#ifndef ARDUINO_H
#define ARDUINO_H

#ifdef __cplusplus
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#else
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#endif

#include <string.h>
#include <stdarg.h>
#include "Wire.h"
#include "Print.h"
#define SS 10 // random ass numbers lol

#define HIGH 1
#define LOW 0

#define INPUT 1
#define OUTPUT 0

#define LED_BUILTIN 13
#define BUILTIN_SDCARD 254

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino print format constants
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

// SPI bit order
#define LSBFIRST 0
#define MSBFIRST 1
typedef uint8_t BitOrder;

// F() macro for flash strings (just return the string on native)
#define F(string_literal) (string_literal)

// Flash string helper type
class __FlashStringHelper;
typedef const __FlashStringHelper *FlashStringHelper;

// Arduino boolean and byte types
typedef bool boolean;
typedef uint8_t byte;

// Arduino helper functions
template<typename T>
T constrain(T x, T low, T high) {
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

template<typename T, typename U, typename V>
T map(T x, U in_min, U in_max, V out_min, V out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint64_t millis();
uint64_t micros();

void setMillis(uint64_t ms);

void resetMillis();

void delay(unsigned long ms);

void delay(int ms);

void delayMicroseconds(unsigned int us);

void yield();

void digitalWrite(int pin, int value);

int digitalRead(int pin);

void pinMode(int pin, int mode);

class Stream : public Print
{
public:
    void begin(int baud = 9600);
    void end();
    void clearBuffer();
    bool available();
    int read() { return -1; }  // Mock read - returns -1 (no data)
    int readBytesUntil(char i, char *buf, size_t s);
    size_t readBytes(char *buf, size_t len) { return 0; }  // Mock - no data
    size_t readBytes(uint8_t *buf, size_t len) { return 0; }  // Mock - no data
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buf, size_t len) {  // Add buffer write
        size_t written = 0;
        for (size_t i = 0; i < len; i++) {
            written += write(buf[i]);
        }
        return written;
    }
    operator bool() { return true; }

    char fakeBuffer[1000];
    int cursor = 0;
};

// Arduino String class
#include <string>
class String {
private:
    std::string str;
public:
    String() : str("") {}
    String(const char* s) : str(s ? s : "") {}
    String(const std::string& s) : str(s) {}
    String(int n) : str(std::to_string(n)) {}
    String(unsigned int n) : str(std::to_string(n)) {}
    String(long n) : str(std::to_string(n)) {}
    String(unsigned long n) : str(std::to_string(n)) {}
    String(float f) : str(std::to_string(f)) {}
    String(double d) : str(std::to_string(d)) {}

    const char* c_str() const { return str.c_str(); }
    size_t length() const { return str.length(); }
    bool startsWith(const char* prefix) const {
        return str.find(prefix) == 0;
    }
    bool startsWith(const String& prefix) const {
        return str.find(prefix.str) == 0;
    }
    int indexOf(char c) const {
        size_t pos = str.find(c);
        return (pos == std::string::npos) ? -1 : (int)pos;
    }
    String substring(int start) const {
        return String(str.substr(start));
    }
    String substring(int start, int end) const {
        return String(str.substr(start, end - start));
    }
    operator const char*() const { return str.c_str(); }
};

class SerialClass : public Stream
{
public:
    String readStringUntil(char terminator) {
        // Mock implementation - return empty string
        return String("");
    }
};

extern SerialClass Serial;
extern SerialClass Serial1;
extern SerialClass Serial2;
extern SerialClass Serial3;

class CrashReportClass
{
public:
    explicit CrashReportClass() {}
    operator bool() const { return false; }
};
extern CrashReportClass CrashReport;

#endif