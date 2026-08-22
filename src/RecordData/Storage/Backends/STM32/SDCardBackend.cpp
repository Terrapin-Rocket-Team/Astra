#include "SDCardBackend.h"

namespace astra
{
    SDCardBackend::SDCardBackend() : _initialized(false) {}

    bool SDCardBackend::begin()
    {
        // STM32SD and STM32EMMC currently export the same global filesystem
        // symbols, so the STM32 SD backend is disabled while eMMC support is
        // linked in.
        _initialized = false;
        return false;
    }

    bool SDCardBackend::end()
    {
        _initialized = false;
        return true;
    }

    bool SDCardBackend::ok() const
    {
        return _initialized;
    }

    IFile *SDCardBackend::openRead(const char * /*filename*/)
    {
        return nullptr;
    }

    IFile *SDCardBackend::openWrite(const char * /*filename*/, bool /*append*/)
    {
        return nullptr;
    }

    bool SDCardBackend::exists(const char * /*filename*/)
    {
        return false;
    }

    bool SDCardBackend::remove(const char * /*filename*/)
    {
        return false;
    }

    bool SDCardBackend::mkdir(const char * /*path*/)
    {
        return false;
    }

    bool SDCardBackend::rmdir(const char * /*path*/)
    {
        return false;
    }

} // namespace astra
