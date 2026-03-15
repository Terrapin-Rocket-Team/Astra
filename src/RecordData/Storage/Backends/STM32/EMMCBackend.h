#ifndef STM32_EMMC_BACKEND_H
#define STM32_EMMC_BACKEND_H

#include "../../IStorage.h"

namespace astra
{

    /**
     * @brief IStorage implementation for eMMC using STM32 MMC interface
     *
     * Uses STM32EMMC to access eMMC over the STM32 SDMMC interface.
     *
     * Pin configuration (hardcoded):
     * - PC8:  D0
     * - PC9:  D1
     * - PC10: D2
     * - PC11: D3
     * - PC12: CLK
     * - PD2:  CMD
     */
    class EMMCBackend : public IStorage
    {
    private:
        bool _initialized;

    public:
        EMMCBackend();

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

#endif // STM32_EMMC_BACKEND_H
