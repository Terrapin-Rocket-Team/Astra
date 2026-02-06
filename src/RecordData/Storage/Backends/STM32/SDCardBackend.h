#ifndef STM32_SDCARD_BACKEND_H
#define STM32_SDCARD_BACKEND_H

#include "../../IStorage.h"
#include "STM32File.h"
#include <STM32SD.h>
#include <ff.h> // For FA_OPEN_APPEND and other FatFs constants

namespace astra
{

    /**
     * @brief IStorage implementation for SD cards using STM32 SDMMC interface
     *
     * Uses STM32SD library to access SD cards via SDMMC interface.
     * Pins are defined by the board variant.
     */
    class SDCardBackend : public IStorage
    {
    private:
        bool _initialized;

    public:
        SDCardBackend() : _initialized(false) {}

        bool begin() override
        {
            // STM32SD library with SDMMC
            if (!SD.begin(SD_DETECT_NONE))
            {
                _initialized = false;
                return false;
            }
            _initialized = true;
            return true;
        }

        bool end() override
        {
            _initialized = false;
            return true;
        }

        bool ok() const override
        {
            return _initialized;
        }

        IFile *openRead(const char *filename) override
        {
            if (!_initialized)
                return nullptr;

            File file = SD.open(filename, FILE_READ);
            if (!file)
                return nullptr;
            return new STM32File(file);
        }

        IFile *openWrite(const char *filename, bool append = true) override
        {
            if (!_initialized)
                return nullptr;

            uint8_t mode = append ? (FILE_WRITE | FA_OPEN_APPEND) : (FILE_WRITE | FA_OPEN_ALWAYS);
            File file = SD.open(filename, mode);
            if (!file)
                return nullptr;
            return new STM32File(file);
        }

        bool exists(const char *filename) override
        {
            if (!_initialized)
                return false;
            return SD.exists(filename);
        }

        bool remove(const char *filename) override
        {
            if (!_initialized)
                return false;
            return SD.remove(filename);
        }

        bool mkdir(const char *path) override
        {
            if (!_initialized)
                return false;
            return SD.mkdir(path);
        }

        bool rmdir(const char *path) override
        {
            if (!_initialized)
                return false;
            return SD.rmdir(path);
        }
    };

} // namespace astra

#endif // STM32_SDCARD_BACKEND_H
