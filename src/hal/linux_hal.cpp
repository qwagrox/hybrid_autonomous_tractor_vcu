// Copyright 2025 Manus AI

#include "vcu/hal/linux_hal.h"
#include "vcu/can/can_interface.h"
#include <fstream>
#include <filesystem>

namespace vcu {
namespace hal {

LinuxHal::LinuxHal()
    : initialized_(false) {
}

LinuxHal::~LinuxHal() {
    LinuxHal::shutdown();  // 显式调用本类的shutdown方法
}

bool LinuxHal::initialize() {
    if (initialized_) {
        return true;
    }

    // 检查系统要求（在实际部署中可能失败）
    bool system_ok = check_system_requirements();
    if (!system_ok) {
        // 在开发环境中，系统要求检查可能失败，但仍可继续
        // 在生产环境中，这里应该返回false
    }

    // 设置CAN接口（在实际部署中可能失败）
    bool can_ok = setup_can_interfaces();
    if (!can_ok) {
        return false;
    }

    initialized_ = true;
    return true;
}

void LinuxHal::shutdown() {
    if (!initialized_) {
        return;
    }

    // Shutdown all CAN interfaces
    for (auto& pair : can_interfaces_) {
        auto& interface = pair.second;
        if (interface) {
            interface->shutdown();
        }
    }
    can_interfaces_.clear();

    initialized_ = false;
}

std::shared_ptr<can::ICanInterface> LinuxHal::get_can_interface(const std::string& can_bus_name) {
    if (!initialized_) {
        return nullptr;
    }

    auto it = can_interfaces_.find(can_bus_name);
    if (it != can_interfaces_.end()) {
        return it->second;
    }

    // Create new CAN interface if not exists
    auto can_interface = can::create_can_interface();
    if (can_interface) {
        std::shared_ptr<can::ICanInterface> shared_interface = std::move(can_interface);
        can_interfaces_[can_bus_name] = shared_interface;
        return shared_interface;
    }

    return nullptr;
}

bool LinuxHal::check_system_requirements() {
    // Check if running on Linux
    #ifndef __linux__
    return false;
    #endif

    // Check if CAN utilities are available
    if (!std::filesystem::exists("/sys/class/net")) {
        return false;
    }

    // Check kernel modules
    std::ifstream modules("/proc/modules");
    if (!modules.is_open()) {
        return false;
    }

    std::string line;
    bool can_support = false;

    while (std::getline(modules, line)) {
        if (line.find("can") != std::string::npos) {
            can_support = true;
            break;  // 找到CAN支持就可以退出
        }
    }

    return can_support;
}

bool LinuxHal::setup_can_interfaces() {
    // In a real implementation, this would:
    // 1. Scan for available CAN interfaces
    // 2. Configure CAN bitrates
    // 3. Bring up CAN interfaces
    // 4. Set up error handling

    // For simulation, we assume CAN interfaces are available
    return true;
}

} // namespace hal
} // namespace vcu
