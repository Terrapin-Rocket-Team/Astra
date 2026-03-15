#ifndef STM32_SDCARD_BACKEND_H
#define STM32_SDCARD_BACKEND_H

#include "../../IStorage.h"

namespace astra
{

    /**
     * @brief IStorage implementation for SD cards using STM32 SDMMC interface
     *
     * This backend is currently disabled in STM32 eMMC builds because
     * STM32SD and STM32EMMC export colliding filesystem symbols.
     */
    class SDCardBackend : public IStorage
    {
    private:
        bool _initialized;

    public:
        SDCardBackend();

        bool begin() override;
        bool end() override;
        bool ok() const override;

        IFile *openRead(const char *filename) override;
        IFile *openWrite(const char *filename, bool append = true) override;

        bool exists(const char *filename) override;
        bool remove(const char *filename) override;
        bool mkdir(const char *path) override;
        bool rmdir(const char *path) override;
    };

} // namespace astra

#endif // STM32_SDCARD_BACKEND_H
