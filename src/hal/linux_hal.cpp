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
    shutdown();
}

bool LinuxHal::initialize() {
    if (initialized_) {
        return true;
    }

    if (!check_system_requirements()) {
        return false;
    }

    if (!setup_can_interfaces()) {
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
    for (auto& [name, interface] : can_interfaces_) {
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
    bool socketcan_support = false;

    while (std::getline(modules, line)) {
        if (line.find("can") != std::string::npos) {
            can_support = true;
        }
        if (line.find("can_raw") != std::string::npos) {
            socketcan_support = true;
        }
    }

    return can_support; // socketcan_support is optional for simulation
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
