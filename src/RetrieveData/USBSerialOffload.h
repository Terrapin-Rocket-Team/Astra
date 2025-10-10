#include <SdFat.h>
// misc config items
#include <RecordData/Logging/LoggingBackend/SdFatBoilerplate.h>
// length of an individual line (should change depending on worst case scenario)
#define LINE_LENGTH 512
// make sure to use the custom, optimized SPI driver
#define SPI_DRIVER_SELCT 0
// make sure to use CRC
#define USE_SD_CRC 1
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

namespace astra
{
    class USBSerialOffload
    {
    private:
        char *currentFileName;
        char *skipSpace(char *str);
        bool parseLine(char *str);

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
        bool validateFileName(char *name);
        bool deleteFile(char *path);
        void listFiles();
        void handleChoices();
        bool readFile(char *path);
    };
    USBSerialOffload &getDataRetrieverInstance();
}