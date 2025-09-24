#include "vcu/hal/linux_hal.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/utsname.h>

namespace vcu {
namespace hal {

LinuxHal::LinuxHal() : is_initialized_(false) {}

LinuxHal::~LinuxHal() {
    shutdown();
}

HalResult LinuxHal::initialize() {
    if (is_initialized_) {
        return HalResult::SUCCESS;
    }

    if (!check_system_requirements()) {
        return HalResult::ERROR_SYSTEM_REQUIREMENTS;
    }

    if (!setup_can_interfaces()) {
        return HalResult::ERROR_CAN_SETUP;
    }

    is_initialized_ = true;
    return HalResult::SUCCESS;
}

HalResult LinuxHal::shutdown() {
    if (!is_initialized_) {
        return HalResult::SUCCESS;
    }

    cleanup_can_interfaces();
    is_initialized_ = false;
    return HalResult::SUCCESS;
}

bool LinuxHal::is_ready() const {
    return is_initialized_;
}

std::string LinuxHal::get_system_info() const {
    struct utsname system_info;
    if (uname(&system_info) == 0) {
        std::ostringstream info;
        info << "System: " << system_info.sysname << " " << system_info.release
             << " (" << system_info.machine << ")";
        return info.str();
    }
    return "Unknown Linux system";
}

HalResult LinuxHal::read_sensor_data(const std::string& sensor_name, float& value) {
    // Simulate sensor reading for testing
    if (sensor_name == "engine_temp") {
        value = 85.5f;
    } else if (sensor_name == "oil_pressure") {
        value = 3.2f;
    } else if (sensor_name == "fuel_level") {
        value = 75.0f;
    } else {
        return HalResult::ERROR_SENSOR_NOT_FOUND;
    }
    
    return HalResult::SUCCESS;
}

HalResult LinuxHal::write_actuator_data(const std::string& actuator_name, float value) {
    // Simulate actuator control for testing
    if (actuator_name == "cvt_ratio" && value >= 0.5f && value <= 3.0f) {
        return HalResult::SUCCESS;
    } else if (actuator_name == "engine_throttle" && value >= 0.0f && value <= 100.0f) {
        return HalResult::SUCCESS;
    } else {
        return HalResult::ERROR_ACTUATOR_INVALID;
    }
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
        // 修复：使用socketcan_support变量来检查SocketCAN支持
        if (line.find("can_raw") != std::string::npos) {
            // SocketCAN raw protocol support found
            // This enhances CAN functionality but is not strictly required
            can_support = true; // 将socketcan支持也视为CAN支持的一部分
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

void LinuxHal::cleanup_can_interfaces() {
    // In a real implementation, this would:
    // 1. Bring CAN interfaces down
    // 2. Clean up any allocated resources
    // For simulation, nothing to do
}

} // namespace hal
} // namespace vcu
