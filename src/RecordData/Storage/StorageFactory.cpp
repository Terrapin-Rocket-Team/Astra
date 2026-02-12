#include "StorageFactory.h"

// Include platform-specific backend implementations
#if defined(ENV_STM) && !defined(USE_SIMPLE_STORAGE)
#include "Backends/STM32/EMMCBackend.h"
#include "Backends/STM32/SDCardBackend.h"
#elif defined(ENV_ESP)
#include "Backends/ESP32/SDCardBackend.h"
#elif defined(ENV_TEENSY)
#include "Backends/Teensy/SDCardBackend.h"
#endif
// #include "Backends/Flash/FlashBackend.h"  // Future

namespace astra
{

    IStorage *StorageFactory::create(StorageBackend type)
    {
        switch (type)
        {
#if defined(ENV_STM) && !defined(USE_SIMPLE_STORAGE)
        case StorageBackend::EMMC:
            return new EMMCBackend();
#endif

#if (defined(ENV_STM) && !defined(USE_SIMPLE_STORAGE)) || defined(ENV_ESP) || defined(ENV_TEENSY)
        case StorageBackend::SD_CARD:
            return new SDCardBackend();
#endif

            // case StorageBackend::INTERNAL_FLASH:
            //     return new FlashBackend();

        default:
            return nullptr; // Unsupported backend on this platform
        }
    }

} // namespace astra
