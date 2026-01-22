#ifndef MOCK_WIRE_H
#define MOCK_WIRE_H

#include <cstdint>

// A mock version of the Arduino TwoWire class for native builds.
class TwoWire {
public:
    void begin() {}
    void end() {}
    void setClock(uint32_t frequency) {}  // Mock - does nothing
    void beginTransmission(uint8_t address) {}
    uint8_t endTransmission(bool stop = true) { return 0; }
    size_t write(uint8_t data) { return 1; }
    size_t write(const uint8_t *data, size_t quantity) { return quantity; }
    int available() { return 0; }
    int read() { return -1; }
    uint8_t requestFrom(uint8_t address, size_t quantity, bool stop = true) { return 0; }
};

// Extern the global Wire instance
extern TwoWire Wire;

#endif // MOCK_WIRE_H
