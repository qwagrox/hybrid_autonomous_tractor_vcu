// src/hardware/watchdog.cpp
#include "hardware/watchdog.hpp"
#include <iostream>
#include <chrono>

namespace VCUCore {

Watchdog::Watchdog(uint32_t timeoutMs) 
    : timeoutMs_(timeoutMs), isEnabled_(false), lastKickTime_(0) {
}

bool Watchdog::initialize() {
    std::cout << "Initializing watchdog with timeout: " << timeoutMs_ << " ms" << std::endl;
    
    lastKickTime_ = getCurrentTimeMs();
    isEnabled_ = false;
    
    std::cout << "Watchdog initialized successfully" << std::endl;
    return true;
}

void Watchdog::shutdown() {
    isEnabled_ = false;
    std::cout << "Watchdog shutdown" << std::endl;
}

bool Watchdog::enable() {
    lastKickTime_ = getCurrentTimeMs();
    isEnabled_ = true;
    std::cout << "Watchdog enabled" << std::endl;
    return true;
}

bool Watchdog::disable() {
    isEnabled_ = false;
    std::cout << "Watchdog disabled" << std::endl;
    return true;
}

void Watchdog::kick() {
    if (isEnabled_) {
        lastKickTime_ = getCurrentTimeMs();
        // std::cout << "Watchdog kicked at: " << lastKickTime_ << std::endl;
    }
}

bool Watchdog::checkTimeout() const {
    if (!isEnabled_) {
        return false;
    }
    
    uint32_t currentTime = getCurrentTimeMs();
    uint32_t elapsed = currentTime - lastKickTime_;
    
    if (elapsed > timeoutMs_) {
        std::cerr << "Watchdog timeout detected! Elapsed: " << elapsed 
                  << " ms, Timeout: " << timeoutMs_ << " ms" << std::endl;
        return true;
    }
    
    return false;
}

void Watchdog::setTimeout(uint32_t timeoutMs) {
    timeoutMs_ = timeoutMs;
    std::cout << "Watchdog timeout set to: " << timeoutMs_ << " ms" << std::endl;
}

uint32_t Watchdog::getTimeout() const {
    return timeoutMs_;
}

bool Watchdog::isEnabled() const {
    return isEnabled_;
}

uint32_t Watchdog::getTimeSinceLastKick() const {
    if (!isEnabled_) {
        return 0;
    }
    
    uint32_t currentTime = getCurrentTimeMs();
    return currentTime - lastKickTime_;
}

uint32_t Watchdog::getCurrentTimeMs() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    return static_cast<uint32_t>(millis.count());
}

} // namespace VCUCore
