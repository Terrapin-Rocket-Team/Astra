#ifndef STORAGE_FACTORY_H
#define STORAGE_FACTORY_H

#include "IStorage.h"

namespace astra
{

    /**
     * @brief Factory for creating storage backend instances
     *
     * Creates the appropriate IStorage implementation based on the
     * StorageBackend enum value. Returns nullptr if the backend is
     * not supported on the current platform.
     */
    class StorageFactory
    {
    public:
        /**
         * @brief Create a storage backend instance
         * @param type The type of storage backend to create
         * @return Pointer to IStorage instance, or nullptr if unsupported
         * @note Caller is responsible for deleting the returned instance
         */
        static IStorage *create(StorageBackend type);
    };

} // namespace astra

#endif // STORAGE_FACTORY_H
