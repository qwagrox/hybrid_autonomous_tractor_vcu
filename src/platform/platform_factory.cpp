#include "vcu/platform/platform_factory.h"

#ifdef PLATFORM_LINUX
#include "linux/linux_platform.h"
#elif defined(PLATFORM_NUTTX)
#include "nuttx/nuttx_platform.h"
#else
// 默认使用Linux平台
#include "linux/linux_platform.h"
#endif

std::unique_ptr<PlatformInterface> PlatformFactory::create_platform() {
#ifdef PLATFORM_LINUX
    return std::make_unique<LinuxPlatform>();
#elif defined(PLATFORM_NUTTX)
    return std::make_unique<NuttxPlatform>();
#else
    // 默认使用Linux平台
    return std::make_unique<LinuxPlatform>();
#endif
}
