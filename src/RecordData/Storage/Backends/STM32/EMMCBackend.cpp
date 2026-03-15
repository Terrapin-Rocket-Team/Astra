#include "EMMCBackend.h"

#include <STM32EMMC.h>
#include <ff.h>

namespace astra
{
    namespace
    {
        class STM32EMMCFile : public IFile
        {
        public:
            explicit STM32EMMCFile(File handle) : _handle(handle) {}

            size_t write(uint8_t b) override
            {
                return _handle.write(b);
            }

            size_t write(const uint8_t *buffer, size_t size) override
            {
                return _handle.write(buffer, size);
            }

            bool flush() override
            {
                _handle.flush();
                return true;
            }

            int read() override
            {
                return _handle.read();
            }

            int readBytes(uint8_t *buffer, size_t length) override
            {
                return _handle.read(buffer, length);
            }

            int available() override
            {
                return _handle.available();
            }

            bool seek(uint32_t pos) override
            {
                return _handle.seek(pos);
            }

            uint32_t position() override
            {
                return _handle.position();
            }

            uint32_t size() override
            {
                return _handle.size();
            }

            bool close() override
            {
                _handle.close();
                return true;
            }

            bool isOpen() const override
            {
                return const_cast<File &>(_handle) ? true : false;
            }

        private:
            File _handle;
        };
    } // namespace

    EMMCBackend::EMMCBackend() : _initialized(false) {}

    bool EMMCBackend::begin()
    {
#if defined(PC8) && defined(PC9) && defined(PC10) && defined(PC11) && defined(PC12) && defined(PD2)
        EMMC.setDx(PC8, PC9, PC10, PC11);
        EMMC.setCK(PC12);
        EMMC.setCMD(PD2);
#endif

        _initialized = EMMC.begin();
        return _initialized;
    }

    bool EMMCBackend::end()
    {
        if (!_initialized)
        {
            return true;
        }

        if (!EMMC.end())
        {
            return false;
        }

        _initialized = false;
        return true;
    }

    bool EMMCBackend::ok() const
    {
        return _initialized;
    }

    IFile *EMMCBackend::openRead(const char *filename)
    {
        if (!_initialized)
        {
            return nullptr;
        }

        File file = EMMC.open(filename, FILE_READ);
        if (!file)
        {
            return nullptr;
        }

        return new STM32EMMCFile(file);
    }

    IFile *EMMCBackend::openWrite(const char *filename, bool append)
    {
        if (!_initialized)
        {
            return nullptr;
        }

        const uint8_t mode = append ? static_cast<uint8_t>(FILE_WRITE | FA_OPEN_APPEND)
                                    : static_cast<uint8_t>(FILE_WRITE | FA_OPEN_ALWAYS);
        File file = EMMC.open(filename, mode);
        if (!file)
        {
            return nullptr;
        }

        return new STM32EMMCFile(file);
    }

    bool EMMCBackend::exists(const char *filename)
    {
        return _initialized && EMMC.exists(filename);
    }

    bool EMMCBackend::remove(const char *filename)
    {
        return _initialized && EMMC.remove(filename);
    }

    bool EMMCBackend::mkdir(const char *path)
    {
        return _initialized && EMMC.mkdir(path);
    }

    bool EMMCBackend::rmdir(const char *path)
    {
        return _initialized && EMMC.rmdir(path);
    }

} // namespace astra
