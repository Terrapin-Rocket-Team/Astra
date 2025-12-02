#ifndef SDCARD_BACKEND_H
#define SDCARD_BACKEND_H

#include "../../IStorage.h"

#if defined(ENV_ESP)
#include "SDCardFile.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "esp_vfs_fat.h"
#include <cstdio>
#include <sys/stat.h>
#include <sys/unistd.h>
#elif defined(ENV_STM)
#include "../EMMC/EMMCFile.h"
#include <STM32SD.h>
#include <ff.h>
#elif defined(ENV_TEENSY)
#include "SDCardFile.h"
#include <SdFat.h>
#endif

namespace astra
{

/**
 * @brief Unified IStorage implementation for SD cards
 *
 * Automatically selects the appropriate SD card interface based on platform:
 * - ESP32: SDMMC interface (4-bit mode) with VFS FAT filesystem
 * - STM32: SDMMC interface via STM32SD library
 * - Teensy: SDIO interface via SdFat library
 *
 * ESP32 Default pin configuration (ESP32-S3):
 * - CLK: GPIO1, CMD: GPIO2
 * - D0: GPIO3, D1: GPIO4, D2: GPIO5, D3: GPIO6
 *
 * STM32/Teensy: Pins are defined by board variant
 */
class SDCardBackend : public IStorage {
private:
#if defined(ENV_ESP)
    static bool _mounted;
    static int _mountRefCount;
    static sdmmc_card_t* _sharedCard;

    const char* _mountPoint;
    int _clkPin;
    int _cmdPin;
    int _d0Pin;
    int _d1Pin;
    int _d2Pin;
    int _d3Pin;
#elif defined(ENV_TEENSY)
    SdFat _sd;
#endif
    bool _initialized;

public:
#if defined(ENV_ESP)
    /**
     * @brief Construct ESP32 SD backend with default pins
     */
    SDCardBackend()
        : _mountPoint("/"),
          _clkPin(1), _cmdPin(2), _d0Pin(3),
          _d1Pin(4), _d2Pin(5), _d3Pin(6),
          _initialized(false) {}

    /**
     * @brief Construct ESP32 SD backend with custom pins
     * @param clkPin Clock pin
     * @param cmdPin Command pin
     * @param d0Pin Data 0 pin
     * @param d1Pin Data 1 pin (set to -1 for 1-bit mode)
     * @param d2Pin Data 2 pin (set to -1 for 1-bit mode)
     * @param d3Pin Data 3 pin (set to -1 for 1-bit mode)
     */
    SDCardBackend(int clkPin, int cmdPin, int d0Pin, int d1Pin = -1, int d2Pin = -1, int d3Pin = -1)
        : _mountPoint("/"),
          _clkPin(clkPin), _cmdPin(cmdPin), _d0Pin(d0Pin),
          _d1Pin(d1Pin), _d2Pin(d2Pin), _d3Pin(d3Pin),
          _initialized(false) {}
#else
    /**
     * @brief Construct SD backend (STM32/Teensy)
     * Pins are defined by the board variant
     */
    SDCardBackend() : _initialized(false) {}
#endif

    bool begin() override {
        if (_initialized) return true;

#if defined(ENV_ESP)
        if (_clkPin < 0 || _cmdPin < 0 || _d0Pin < 0) {
            return false;
        }

        // Mount only once; subsequent instances just increment ref count
        if (!_mounted) {
            sdmmc_host_t host = SDMMC_HOST_DEFAULT();
            sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

            host.max_freq_khz = SDMMC_FREQ_DEFAULT;
            slot_config.clk = (gpio_num_t)_clkPin;
            slot_config.cmd = (gpio_num_t)_cmdPin;
            slot_config.d0 = (gpio_num_t)_d0Pin;
            slot_config.d1 = (_d1Pin >= 0) ? (gpio_num_t)_d1Pin : GPIO_NUM_NC;
            slot_config.d2 = (_d2Pin >= 0) ? (gpio_num_t)_d2Pin : GPIO_NUM_NC;
            slot_config.d3 = (_d3Pin >= 0) ? (gpio_num_t)_d3Pin : GPIO_NUM_NC;

            // Determine bus width
            slot_config.width = (_d1Pin >= 0 && _d2Pin >= 0 && _d3Pin >= 0) ? 4 : 1;

            // Enable internal pull-ups
            slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

            esp_vfs_fat_sdmmc_mount_config_t mount_config = {
                .format_if_mount_failed = false,
                .max_files = 5,
                .allocation_unit_size = 16 * 1024
            };

            if (esp_vfs_fat_sdmmc_mount(_mountPoint, &host, &slot_config, &mount_config, &_sharedCard) != ESP_OK) {
                return false;
            }
            _mounted = true;
            _mountRefCount = 0;
        }
        _mountRefCount++;
        _initialized = true;
        return true;

#elif defined(ENV_STM)
        // STM32: Use STM32SD library with SDMMC
        if (!SD.begin(SD_DETECT_NONE)) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;

#elif defined(ENV_TEENSY)
        // Teensy: Use SdFat with SDIO configuration
        if (!_sd.begin(SdioConfig(FIFO_SDIO))) {
            _initialized = false;
            return false;
        }
        _initialized = true;
        return true;
#endif
    }

    bool end() override {
#if defined(ENV_ESP)
        if (_initialized && _mounted && _mountRefCount > 0) {
            _mountRefCount--;
            if (_mountRefCount == 0) {
                esp_vfs_fat_sdcard_unmount(_mountPoint, _sharedCard);
                _sharedCard = nullptr;
                _mounted = false;
            }
        }
#endif
        _initialized = false;
        return true;
    }

    bool ok() const override {
#if defined(ENV_ESP)
        return _initialized && _mounted;
#else
        return _initialized;
#endif
    }

    IFile* openRead(const char* filename) override {
        if (!_initialized) return nullptr;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, filename);

        FILE* file = fopen(filepath, "r");
        if (!file) return nullptr;

        return new SDCardFile(file);

#elif defined(ENV_STM)
        File file = SD.open(filename, FILE_READ);
        if (!file) return nullptr;
        return new EMMCFile(file);

#elif defined(ENV_TEENSY)
        FsFile file = _sd.open(filename, O_READ);
        if (!file) return nullptr;
        return new SDCardFile(std::move(file));
#endif
    }

    IFile* openWrite(const char* filename, bool append = true) override {
        if (!_initialized) return nullptr;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, filename);

        const char* mode = append ? "a" : "w";
        FILE* file = fopen(filepath, mode);
        if (!file) return nullptr;

        // Disable stdio buffering to reduce data loss on power failure
        setvbuf(file, NULL, _IONBF, 0);

        return new SDCardFile(file);

#elif defined(ENV_STM)
        uint8_t mode = append ? (FILE_WRITE | FA_OPEN_APPEND) : (FILE_WRITE | FA_OPEN_ALWAYS);
        File file = SD.open(filename, mode);
        if (!file) return nullptr;
        return new EMMCFile(file);

#elif defined(ENV_TEENSY)
        uint8_t mode = append ? (O_WRITE | O_CREAT | O_APPEND) : (O_WRITE | O_CREAT | O_TRUNC);
        FsFile file = _sd.open(filename, mode);
        if (!file) return nullptr;
        return new SDCardFile(std::move(file));
#endif
    }

    bool exists(const char* filename) override {
        if (!_initialized) return false;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, filename);

        FILE* file = fopen(filepath, "r");
        if (file) {
            fclose(file);
            return true;
        }
        return false;

#elif defined(ENV_STM)
        return SD.exists(filename);

#elif defined(ENV_TEENSY)
        return _sd.exists(filename);
#endif
    }

    bool remove(const char* filename) override {
        if (!_initialized) return false;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, filename);

        return ::remove(filepath) == 0;

#elif defined(ENV_STM)
        return SD.remove(filename);

#elif defined(ENV_TEENSY)
        return _sd.remove(filename);
#endif
    }

    bool mkdir(const char* path) override {
        if (!_initialized) return false;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, path);

        return ::mkdir(filepath, 0755) == 0;

#elif defined(ENV_STM)
        return SD.mkdir(path);

#elif defined(ENV_TEENSY)
        return _sd.mkdir(path);
#endif
    }

    bool rmdir(const char* path) override {
        if (!_initialized) return false;

#if defined(ENV_ESP)
        char filepath[128];
        snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, path);

        return ::rmdir(filepath) == 0;

#elif defined(ENV_STM)
        return SD.rmdir(path);

#elif defined(ENV_TEENSY)
        return _sd.rmdir(path);
#endif
    }
};

#if defined(ENV_ESP)
// Static member initialization (ESP32 only)
bool SDCardBackend::_mounted = false;
int SDCardBackend::_mountRefCount = 0;
sdmmc_card_t* SDCardBackend::_sharedCard = nullptr;
#endif

} // namespace astra

#endif // SDCARD_BACKEND_H
