#include "SdFat.h"
#include <Arduino.h>
// length of an individual line (should change depending on worst case scenario)
#define LINE_LENGTH 512
// make sure to use the custom, optimized SPI driver
#define SPI_DRIVER_SELCT 0
// make sure to use CRC
#define USE_SD_CRC 1
#define SD_FAT_TYPE 3
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif // HAS_SDIO_CLASS


namespace astra
{
    class USBSerialOffload
    {
    private:
        char *currentFileName;
        void formatAndEraseCard();

// define file format being used
// unfortunately means recompilation needs to be done
//          every time a different file format is used
#if SD_FAT_TYPE == 0
        SdFat sd;
        File file;
#elif SD_FAT_TYPE == 1
        SdFat32 sd;
        File32 file;
#elif SD_FAT_TYPE == 2
        SdExFat sd;
        ExFile file;
#elif SD_FAT_TYPE == 3
        SdFs sd;
        FsFile file;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

    public:
        bool init();
        bool deleteFile(char *path);
        void listFiles();
        bool handleChoices();
        bool readFile(char *path);
        bool formatCard(); // TODO: implement later
    };
    USBSerialOffload &getDataRetrieverInstance();
}


