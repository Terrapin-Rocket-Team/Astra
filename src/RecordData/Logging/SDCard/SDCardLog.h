#ifndef SD_CARD_LOG_H
#define SD_CARD_LOG_H

#include "../LoggingBackend/ILogSink.h"

#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "esp_vfs_fat.h"
#include "FS.h"
#include "SD_MMC.h"
#include <unistd.h>

namespace astra
{
    class SDCardLog : public ILogSink
    {
    private:
        FILE* _file;
        const char* _mountPoint;

        static bool _mounted;
        static int _mountRefCount;
        static sdmmc_card_t* _sharedCard;
        const char* _filename;
        bool _prefix;
        bool _rdy;
        bool _fileOpen;
        unsigned long _lastFlush;
        static const unsigned long FLUSH_INTERVAL = 1000;

        int _clkPin = -1;
        int _cmdPin = -1;
        int _d0Pin = -1;
        int _d1Pin = -1;
        int _d2Pin = -1;
        int _d3Pin = -1;

    public:
        // Default constructor with ahrdcoded pins
        // CLK=IO1, CMD=IO2, D0=IO3, D1=IO4, D2=IO5, D3=IO6 (4-bit)
        SDCardLog(const char* filename, bool prefix = false) 
            : _file(nullptr), _mountPoint("/sdcard"),
              _filename(filename), _prefix(prefix), _rdy(false), _fileOpen(false), _lastFlush(0),
              _clkPin(1), _cmdPin(2), _d0Pin(3), _d1Pin(4), _d2Pin(5), _d3Pin(6)
        {}

        // Constructor with custom SDIO pins for ESP32-S3
        // Pins: CLK, CMD, D0, D1, D2, D3
        SDCardLog(const char* filename, int clkPin, int cmdPin, int d0Pin, int d1Pin, int d2Pin, int d3Pin, bool prefix = false)
            : _file(nullptr), _mountPoint("/sdcard"),
              _filename(filename), _prefix(prefix), _rdy(false), _fileOpen(false), _lastFlush(0),
              _clkPin(clkPin), _cmdPin(cmdPin), _d0Pin(d0Pin), _d1Pin(d1Pin), _d2Pin(d2Pin), _d3Pin(d3Pin)
        {}
        
        ~SDCardLog() 
        {
            end();
        }

        bool begin() override
        {
            if (_rdy) return true;
            
            if (_clkPin >= 0 && _cmdPin >= 0 && _d0Pin >= 0) {
                _mountPoint = "/sdcard";
                // Mount only once; subsequent instances just open files
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

                    slot_config.width = 4;

                    // Enable internal pull-ups
                    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
                    
                    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
                        .format_if_mount_failed = false,
                        .max_files = 5, //number of open files at one time
                        .allocation_unit_size = 16 * 1024
                    };
                    
                    if (esp_vfs_fat_sdmmc_mount(_mountPoint, &host, &slot_config, &mount_config, &_sharedCard) != ESP_OK) {
                        return false;
                    }
                    _mounted = true;
                    _mountRefCount = 0;
                }
                _mountRefCount++;
                
                char filepath[64];
                snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, _filename);
                _file = fopen(filepath, "a");
                if (!_file) {
                    // Roll back reference increment and unmount if this was the first and only user
                    if (_mounted && _mountRefCount > 0) {
                        _mountRefCount--;
                        if (_mountRefCount == 0) {
                            esp_vfs_fat_sdcard_unmount(_mountPoint, _sharedCard);
                            _sharedCard = nullptr;
                            _mounted = false;
                        }
                    }
                    return false;
                }
                // Disable stdio buffering to reduce data loss on power failure
                setvbuf(_file, NULL, _IONBF, 0);
            } else {
                return false;
            }
            
            _fileOpen = true;
            _rdy = true;
            _lastFlush = millis();
            return true;
        }

        bool end() override
        {
            if (_fileOpen) {
                if (_file) {
                    fflush(_file);
                    fsync(fileno(_file));
                    fclose(_file);
                    _file = nullptr;
                }
                if (_mounted && _mountRefCount > 0) {
                    _mountRefCount--;
                    if (_mountRefCount == 0) {
                        esp_vfs_fat_sdcard_unmount(_mountPoint, _sharedCard);
                        _sharedCard = nullptr;
                        _mounted = false;
                    }
                }
                _fileOpen = false;
            }
            _rdy = false;
            return true;
        }

        bool ok() const override 
        { 
            return _rdy && _fileOpen; 
        }

        bool wantsPrefix() const override 
        { 
            return _prefix; 
        }

        size_t write(uint8_t b) override
        {
            if (!ok()) return 0;
            
            size_t written = fwrite(&b, 1, 1, _file);
            if (millis() - _lastFlush > FLUSH_INTERVAL) {
                fflush(_file);
                fsync(fileno(_file));
                _lastFlush = millis();
            }
            return written;
        }

        //useful functions perhaps

        void flush() override
        {
            if (ok()) {
                fflush(_file);
                fsync(fileno(_file));
                _lastFlush = millis();
            }
        }

        uint32_t getFileSize() const
        {
            if (!ok()) return 0;
            long pos = ftell(const_cast<FILE*>(_file));
            fseek(const_cast<FILE*>(_file), 0, SEEK_END);
            long size = ftell(const_cast<FILE*>(_file));
            fseek(const_cast<FILE*>(_file), pos, SEEK_SET);
            return size;
        }

        bool fileExists() const
        {
            if (!_rdy) return false;
            char filepath[64];
            snprintf(filepath, sizeof(filepath), "%s/%s", _mountPoint, _filename);
            FILE* f = fopen(filepath, "r");
            if (f) {
                fclose(f);
                return true;
            }
            return false;
        }
    };
}

#endif // SD_CARD_LOG_H
