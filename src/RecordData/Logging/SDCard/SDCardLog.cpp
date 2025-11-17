#include "SDCardLog.h"

namespace astra
{
    bool SDCardLog::_mounted = false;
    int SDCardLog::_mountRefCount = 0;
    sdmmc_card_t* SDCardLog::_sharedCard = nullptr;
}
