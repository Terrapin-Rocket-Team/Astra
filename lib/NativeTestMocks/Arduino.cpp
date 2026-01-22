#include "Arduino.h"

const uint64_t start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
const uint64_t startMicros = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
uint64_t fakeMillis = 0;
bool useFakeMillis = false;

uint64_t millis()
{
    if (useFakeMillis)
    {
        return fakeMillis;
    }
    return (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start);
}

uint64_t micros()
{
    if (useFakeMillis)
    {
        return fakeMillis * 1000;
    }
    return (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startMicros);
}

void setMillis(uint64_t ms)
{
    fakeMillis = ms;
    useFakeMillis = true;
}

void resetMillis()
{
    fakeMillis = 0;
    useFakeMillis = false;
}

#ifndef WIN32
void Sleep(long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
#endif

void delay(unsigned long ms) { Sleep(ms); }

void delay(int ms) { Sleep(ms); }

void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void yield() {
    std::this_thread::yield();
}

void pinMode(int pin, int mode) {
    // Mock - does nothing
}

void digitalWrite(int pin, int value)
{

    int color;
    switch (pin)
    {
    case 13:
        color = 36;
        break;
    case 33:
        color = 33;
        break;
    case 32:
        color = 95;
        break;
    default:
        color = 0;
        break;
    }
    printf("\x1B[%dm%.3f - %d to \x1B[%dm%s\x1B[0m\n", color, millis() / 1000.0, pin, value == LOW ? 91 : 92, value == LOW ? "LOW" : "HIGH");
}

int digitalRead(int pin) {
    // Mock - always return LOW
    return LOW;
}

void Stream::begin(int baud) {}
void Stream::end() {}

void Stream::clearBuffer()
{
    cursor = 0;
    fakeBuffer[0] = '\0';
}

int Stream::readBytesUntil(char c, char *i, size_t len) { return 0; }

bool Stream::available() { return true; }

size_t Stream::write(uint8_t b)
{
    fakeBuffer[cursor++] = b;
    return 1;
}

SerialClass Serial;
SerialClass Serial1;
SerialClass Serial2;
SerialClass Serial3;
CrashReportClass CrashReport;