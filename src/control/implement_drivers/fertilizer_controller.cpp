// src/control/implement_drivers/fertilizer_controller.cpp
#include "control/implement_drivers/fertilizer_controller.hpp"
#include <iostream>
#include <algorithm>

namespace VCUCore {

FertilizerController::FertilizerController() 
    : applicationRate_(0.0), currentDepth_(0.0), isApplying_(false) {
}

std::string FertilizerController::getType() const {
    return "Fertilizer";
}

bool FertilizerController::initialize(const ImplementConfig& config) {
    std::cout << "Initializing fertilizer applicator: " << config.name << std::endl;
    config_ = config;
    
    applicationRate_ = 0.0;
    currentDepth_ = 0.0;
    isApplying_ = false;
    
    std::cout << "Fertilizer controller initialized successfully" << std::endl;
    return true;
}

bool FertilizerController::start() {
    isApplying_ = true;
    std::cout << "Fertilizer applicator started - beginning application" << std::endl;
    return true;
}

bool FertilizerController::stop() {
    isApplying_ = false;
    std::cout << "Fertilizer applicator stopped - application halted" << std::endl;
    return true;
}

bool FertilizerController::setWorkParameter(const std::string& key, double value) {
    if (key == "depth") {
        if (value < 0 || value > 0.3) {
            std::cerr << "Invalid application depth: " << value << " m" << std::endl;
            return false;
        }
        currentDepth_ = value;
        std::cout << "Application depth set to: " << currentDepth_ << " m" << std::endl;
        return true;
    } else if (key == "rate") {
        if (!validateApplicationRate(value)) {
            std::cerr << "Invalid application rate: " << value << " kg/ha" << std::endl;
            return false;
        }
        applicationRate_ = value;
        std::cout << "Application rate set to: " << applicationRate_ << " kg/ha" << std::endl;
        return true;
    }
    
    std::cerr << "Unknown parameter: " << key << std::endl;
    return false;
}

ImplementStatus FertilizerController::getStatus() const {
    ImplementStatus status;
    status.state = isApplying_ ? ImplementState::ACTIVE : ImplementState::IDLE;
    status.is_connected = true;
    status.current_depth = currentDepth_;
    status.current_rate = applicationRate_;
    // status.error_code = 0;  // 该成员不存在
    // status.timestamp = 0;   // 该成员不存在
    
    return status;
}

DiagnosticReport FertilizerController::runDiagnostics() {
    DiagnosticReport report;
    report.passed = true;
    // report.error_count = 0;  // 该成员不存在
    
    // 检查施肥深度
    if (currentDepth_ < 0.02 || currentDepth_ > 0.25) {
        report.findings.push_back("Warning: Application depth may be out of optimal range");
    }
    
    // 检查施肥率
    if (applicationRate_ < 50.0 || applicationRate_ > 500.0) {
        report.findings.push_back("Warning: Application rate may be out of typical range");
    }
    
    if (report.findings.empty()) {
        report.findings.push_back("Fertilizer system operational - all parameters within normal range");
    }
    
    std::cout << "Fertilizer diagnostics completed" << std::endl;
    return report;
}

void FertilizerController::update(double dt) {
    (void)dt; // 避免未使用参数警告
    
    if (isApplying_) {
        // 模拟施肥过程中的状态更新
        // 在实际实现中，这里会更新施肥进度、检查肥料箱状态等
    }
}

bool FertilizerController::validateApplicationRate(double rate) const {
    // 典型施肥率范围：50-500 kg/ha
    return rate >= 0 && rate <= 1000.0;
}

} // namespace VCUCore
