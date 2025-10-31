#ifndef SDMMC_BACKEND_H
#define SDMMC_BACKEND_H

#define LINE_LENGTH 512

#if defined(ENV_STM)
// funny me including this in the namespace :|
#define TAB_SIZE 1
/*
 *   CAVEATS USING STM32SD
 *   - Will be using 4 bit mode (max speed of about 1 MB/s)
 *   - Not all functions for eMMC will be available with STM32SD
 */
#include <STM32SD.h>
#elif defined(ENV_ESP)
#include <SD_MMC.h>
#endif
// Note that Teensy is not included here because it will never be used with an MMC connection
namespace astra
{
    /*
    all files will be stored in the root "/" directory. 
    There is no folder structure, the name of the files will be dictated by the user
    */
    class SDMMCBackend
    {
    public:
        virtual bool begin();
        virtual bool end();
        virtual bool write(char *data, char *filename);
        virtual bool createFile(char *filename);
        virtual bool deleteFile(char *filename);
        virtual void listFiles();
        virtual bool handleChoices();
        virtual bool readFile(char *filename);
        virtual bool exists(char *filename);

    private:
#if defined(ENV_STM)
        File file;
#elif defined(ENV_ESP)

#endif
    };

}
#endif