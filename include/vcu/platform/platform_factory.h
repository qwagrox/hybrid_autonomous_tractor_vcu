#ifndef PLATFORM_FACTORY_H
#define PLATFORM_FACTORY_H

#include "platform_interface.h"
#include <memory>

class PlatformFactory {
public:
    static std::unique_ptr<PlatformInterface> create_platform();
};

#endif // PLATFORM_FACTORY_H

