// src/control/implement_drivers/sprayer_controller.cpp
#include "control/implement_drivers/sprayer_controller.hpp"
#include <iostream>
#include <algorithm>

namespace VCUCore {

SprayerController::SprayerController() 
    : sprayRate_(0.0), sprayPressure_(0.0), isSpraying_(false) {
}

std::string SprayerController::getType() const {
    return "Sprayer";
}

bool SprayerController::initialize(const ImplementConfig& config) {
    std::cout << "Initializing sprayer: " << config.name << std::endl;
    config_ = config;
    
    sprayRate_ = 0.0;
    sprayPressure_ = 0.0;
    isSpraying_ = false;
    
    std::cout << "Sprayer controller initialized successfully" << std::endl;
    return true;
}

bool SprayerController::start() {
    isSpraying_ = true;
    std::cout << "Sprayer started - beginning spray operation" << std::endl;
    return true;
}

bool SprayerController::stop() {
    isSpraying_ = false;
    std::cout << "Sprayer stopped - spray operation halted" << std::endl;
    return true;
}

bool SprayerController::setWorkParameter(const std::string& key, double value) {
    if (key == "pressure") {
        if (value < 0 || value > 10.0) {
            std::cerr << "Invalid spray pressure: " << value << " bar" << std::endl;
            return false;
        }
        sprayPressure_ = value;
        std::cout << "Spray pressure set to: " << sprayPressure_ << " bar" << std::endl;
        return true;
    } else if (key == "rate") {
        if (!validateSprayRate(value)) {
            std::cerr << "Invalid spray rate: " << value << " L/ha" << std::endl;
            return false;
        }
        sprayRate_ = value;
        std::cout << "Spray rate set to: " << sprayRate_ << " L/ha" << std::endl;
        return true;
    }
    
    std::cerr << "Unknown parameter: " << key << std::endl;
    return false;
}

ImplementStatus SprayerController::getStatus() const {
    ImplementStatus status;
    status.state = isSpraying_ ? ImplementState::ACTIVE : ImplementState::IDLE;
    status.is_connected = true;
    status.current_depth = 0.0; // 喷雾器没有深度概念
    status.current_rate = sprayRate_;
    // status.error_code = 0;  // 该成员不存在
    // status.timestamp = 0;   // 该成员不存在
    
    return status;
}

DiagnosticReport SprayerController::runDiagnostics() {
    DiagnosticReport report;
    report.passed = true;
    // report.error_count = 0;  // 该成员不存在
    
    // 检查喷雾压力
    if (sprayPressure_ < 1.0 || sprayPressure_ > 8.0) {
        report.findings.push_back("Warning: Spray pressure may be out of optimal range");
    }
    
    // 检查喷雾率
    if (sprayRate_ < 50.0 || sprayRate_ > 500.0) {
        report.findings.push_back("Warning: Spray rate may be out of typical range");
    }
    
    if (report.findings.empty()) {
        report.findings.push_back("Sprayer system operational - all parameters within normal range");
    }
    
    std::cout << "Sprayer diagnostics completed" << std::endl;
    return report;
}

void SprayerController::update(double dt) {
    (void)dt; // 避免未使用参数警告
    
    if (isSpraying_) {
        // 模拟喷雾过程中的状态更新
        // 在实际实现中，这里会更新喷雾进度、检查药箱状态等
    }
}

bool SprayerController::validateSprayRate(double rate) const {
    // 典型喷雾率范围：50-500 L/ha
    return rate >= 0 && rate <= 1000.0;
}

} // namespace VCUCore
