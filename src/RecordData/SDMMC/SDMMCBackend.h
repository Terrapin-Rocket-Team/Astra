#ifndef SDMMC_BACKEND_H
#define SDMMC_BACKEND_H

#if defined(ENV_STM)
// funny me including this in the namespace :|

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
    class SDMMCBackend
    {
        public:
        virtual bool begin();
        virtual bool end();
        virtual bool write(char *data, char *filename);
        virtual bool createFile(char *filename);
        private:
        #if defined(ENV_STM)
        File file;
        #elif defined(ENV_ESP)
        
        #endif
    };
   
}
#endif