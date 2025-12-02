#include "StorageFactory.h"

// Include backend implementations based on platform
#if defined(ENV_STM)
#include "Backends/EMMC/EMMCBackend.h"
#endif

#if defined(ENV_STM) || defined(ENV_ESP) || defined(ENV_TEENSY)
#include "Backends/SDCard/SDCardBackend.h"
#endif
// #include "Backends/Flash/FlashBackend.h"  // Future

namespace astra
{

IStorage* StorageFactory::create(StorageBackend type) {
    switch(type) {
        #if defined(ENV_STM)
        case StorageBackend::EMMC:
            return new EMMCBackend();
        #endif

        #if defined(ENV_STM) || defined(ENV_ESP) || defined(ENV_TEENSY)
        case StorageBackend::SD_CARD:
            return new SDCardBackend();
        #endif

        // case StorageBackend::INTERNAL_FLASH:
        //     return new FlashBackend();

        default:
            return nullptr;  // Unsupported backend on this platform
    }
}

} // namespace astra
