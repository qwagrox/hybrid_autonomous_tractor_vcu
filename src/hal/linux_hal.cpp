#include "vcu/hal/linux_hal.h"
#include "vcu/can/socketcan_interface.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/utsname.h>

namespace vcu {
namespace hal {

// 修复：使用正确的成员变量名
LinuxHal::LinuxHal() : initialized_(false) {}

LinuxHal::~LinuxHal() {
    // 修复：显式调用本类的shutdown方法，避免虚函数调用
    LinuxHal::shutdown();
}

bool LinuxHal::initialize() {
    if (initialized_) {
        return true;
    }

    if (!check_system_requirements()) {
        return false;
    }

    // 修复：移除总是true的条件判断，直接调用
    setup_can_interfaces();

    initialized_ = true;
    return true;
}

void LinuxHal::shutdown() {
    if (!initialized_) {
        return;
    }

    can_interfaces_.clear();
    initialized_ = false;
}

std::shared_ptr<can::ICanInterface> LinuxHal::get_can_interface(const std::string& can_bus_name) {
    auto it = can_interfaces_.find(can_bus_name);
    if (it != can_interfaces_.end()) {
        return it->second;
    }

    // Create new SocketCAN interface
    auto can_interface = std::make_shared<can::SocketCanInterface>();
    if (can_interface->initialize(can_bus_name, 500000) == can::CanResult::SUCCESS) {
        can_interfaces_[can_bus_name] = can_interface;
        return can_interface;
    }

    return nullptr;
}

bool LinuxHal::check_system_requirements() {
    // Check if running on Linux
    std::ifstream version("/proc/version");
    if (!version.is_open()) {
        return false;
    }

    // Check for CAN support in kernel modules
    std::ifstream modules("/proc/modules");
    if (!modules.is_open()) {
        return false;
    }

    std::string line;
    bool can_support = false;

    while (std::getline(modules, line)) {
        if (line.find("can") != std::string::npos) {
            can_support = true;
        }
        // SocketCAN raw protocol support enhances functionality
        if (line.find("can_raw") != std::string::npos) {
            can_support = true;
        }
    }

    return can_support;
}

bool LinuxHal::setup_can_interfaces() {
    // In a real implementation, this would:
    // 1. Configure CAN interfaces (can0, can1, etc.)
    // 2. Set bitrates
    // 3. Bring interfaces up
    // For simulation, we just return true
    return true;
}

} // namespace hal
} // namespace vcu
