#include "StorageFactory.h"

// Include backend implementations based on platform
#if defined(ENV_STM)
#include "Backends/EMMC/EMMCBackend.h"
#include "Backends/SDCard/SDCardSDIOBackend.h"
#include "Backends/SDCard/SDCardSPIBackend.h"
// #include "Backends/Flash/FlashBackend.h"  // Future

#elif defined(ENV_ESP)
#include "Backends/SDCard/SDCardSPIBackend.h"
// #include "Backends/Flash/FlashBackend.h"  // Future

#elif defined(ENV_TEENSY)
#include "Backends/SDCard/SDCardSDIOBackend.h"
#include "Backends/SDCard/SDCardSPIBackend.h"
// #include "Backends/Flash/FlashBackend.h"  // Future
#endif

namespace astra
{

IStorage* StorageFactory::create(StorageBackend type) {
    switch(type) {
        #if defined(ENV_STM)
        case StorageBackend::EMMC:
            return new EMMCBackend();

        case StorageBackend::SD_SDIO:
            return new SDCardSDIOBackend();

        case StorageBackend::SD_SPI:
            return new SDCardSPIBackend();

        // case StorageBackend::INTERNAL_FLASH:
        //     return new FlashBackend();
        #endif

        #if defined(ENV_TEENSY)
        case StorageBackend::SD_SDIO:
            return new SDCardSDIOBackend();

        case StorageBackend::SD_SPI:
            return new SDCardSPIBackend();

        // case StorageBackend::INTERNAL_FLASH:
        //     return new FlashBackend();
        #endif

        #if defined(ENV_ESP)
        case StorageBackend::SD_SPI:
            return new SDCardSPIBackend();

        // case StorageBackend::INTERNAL_FLASH:
        //     return new FlashBackend();
        #endif

        default:
            return nullptr;  // Unsupported backend on this platform
    }
}

} // namespace astra
