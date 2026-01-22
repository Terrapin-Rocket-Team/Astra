#include "Arduino.h"

const uint64_t start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
WireClass Wire;
uint64_t millis()
{
    return (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start);
}

#ifndef WIN32
void Sleep(long ms) {}
#endif

void delay(unsigned long ms) { Sleep(ms); }

void delay(int ms) { Sleep(ms); }

double analogRead(int pin) { return 0; }

void pinMode(int pin, int mode) { }

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

void Stream::begin(int baud) {}
void Stream::end() {}

void Stream::clearBuffer()
{
    cursor = 0;
    fakeBuffer[0] = '\0';
    inputCursor = 0;
    inputLength = 0;
    inputBuffer[0] = '\0';
}

int Stream::readBytesUntil(char c, char *i, size_t len) {return 0;}

bool Stream::available()
{
    return inputCursor < inputLength;
}

int Stream::read()
{
    if (inputCursor >= inputLength) {
        return -1;
    }
    return inputBuffer[inputCursor++];
}

void Stream::simulateInput(const char* data)
{
    if (!data) return;

    inputLength = strlen(data);
    if (inputLength >= sizeof(inputBuffer)) {
        inputLength = sizeof(inputBuffer) - 1;
    }

    memcpy(inputBuffer, data, inputLength);
    inputBuffer[inputLength] = '\0';
    inputCursor = 0;
}

size_t Stream::write(uint8_t b) { fakeBuffer[cursor++] = b; return 1;}

SerialClass Serial;
CrashReportClass CrashReport;