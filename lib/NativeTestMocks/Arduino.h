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

class Stream : public Print
{
public:
    void begin(int baud = 9600);
    void end();
    void clearBuffer();
    bool available();
    int readBytesUntil(char i, char *buf, size_t s);
    size_t write(uint8_t b) override;
    operator bool() { return true; }

    char fakeBuffer[1000];
    int cursor = 0;
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