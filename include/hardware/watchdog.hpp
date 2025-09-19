// include/hardware/watchdog.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <cstdint>

namespace VCUCore {

/**
 * @class Watchdog
 * @brief 看门狗定时器，用于系统安全监控
 */
class Watchdog {
private:
    uint32_t timeoutMs_;      // 超时时间 (毫秒)
    bool isEnabled_;          // 是否启用
    uint32_t lastKickTime_;   // 上次喂狗时间

public:
    explicit Watchdog(uint32_t timeoutMs = 1000);
    ~Watchdog() = default;

    bool initialize();
    void shutdown();
    
    bool enable();
    bool disable();
    void kick();
    
    bool checkTimeout() const;
    void setTimeout(uint32_t timeoutMs);
    uint32_t getTimeout() const;
    bool isEnabled() const;
    uint32_t getTimeSinceLastKick() const;

private:
    uint32_t getCurrentTimeMs() const;
};

} // namespace VCUCore
