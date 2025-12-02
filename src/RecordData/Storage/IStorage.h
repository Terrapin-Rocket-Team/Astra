#ifndef ISTORAGE_H
#define ISTORAGE_H

#include "IFile.h"

namespace astra
{

/**
 * @brief Platform-specific storage backend types
 *
 * This enum is conditionally compiled based on platform.
 * Only backends supported on the current platform are included.
 */
enum class StorageBackend {
    #if defined(ENV_STM)
    EMMC,           // eMMC via STM32SD MMC interface
    SD_CARD,        // SD card (SDMMC via STM32SD)
    INTERNAL_FLASH, // Onboard flash (future)
    #elif defined(ENV_ESP)
    SD_CARD,        // SD card (SDMMC 4-bit via ESP-IDF)
    INTERNAL_FLASH, // Onboard flash (future)
    #elif defined(ENV_TEENSY)
    SD_CARD,        // SD card (SDIO via SdFat)
    INTERNAL_FLASH, // Onboard flash (future)
    #else
    // Native/testing platform - no actual storage backends
    NONE
    #endif
};

/**
 * @brief Interface for storage backend operations
 *
 * Represents a filesystem/storage medium (e.g., eMMC chip, SD card, flash).
 * Platform-specific implementations handle the actual hardware/library calls.
 */
class IStorage {
public:
    virtual ~IStorage() {}

    // Lifecycle
    virtual bool begin() = 0;
    virtual bool end() = 0;
    virtual bool ok() const = 0;

    // File operations - return IFile handles
    virtual IFile* openRead(const char* filename) = 0;
    virtual IFile* openWrite(const char* filename, bool append = true) = 0;

    // Filesystem operations
    virtual bool exists(const char* filename) = 0;
    virtual bool remove(const char* filename) = 0;
    virtual bool mkdir(const char* path) = 0;
    virtual bool rmdir(const char* path) = 0;
};

} // namespace astra

#endif