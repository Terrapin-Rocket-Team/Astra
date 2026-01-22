#ifndef MOCK_SPI_H
#define MOCK_SPI_H

#include <cstdint>

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

// Adafruit BusIO compatibility
#define SPI_BITORDER_MSBFIRST MSBFIRST
#define SPI_BITORDER_LSBFIRST LSBFIRST

class SPISettings {
public:
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {}
    SPISettings() {}
};

class SPIClass {
public:
    void begin() {}
    void end() {}
    void beginTransaction(SPISettings settings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t data) { return 0; }
    void transfer(void *buf, size_t count) {}
};

extern SPIClass SPI;

#endif // MOCK_SPI_H
