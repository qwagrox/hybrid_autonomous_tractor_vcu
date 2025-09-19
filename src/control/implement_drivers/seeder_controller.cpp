// src/control/implement_drivers/seeder_controller.cpp
#include "control/implement_drivers/seeder_controller.hpp"
#include <iostream>
#include <algorithm>

namespace VCUCore {

SeederController::SeederController() 
    : seedingRate_(0.0), currentDepth_(0.0), isSeeding_(false) {
}

std::string SeederController::getType() const {
    return "Seeder";
}

bool SeederController::initialize(const ImplementConfig& config) {
    std::cout << "Initializing seeder: " << config.name << std::endl;
    config_ = config;
    
    seedingRate_ = 0.0;
    currentDepth_ = 0.0;
    isSeeding_ = false;
    
    std::cout << "Seeder controller initialized successfully" << std::endl;
    return true;
}

bool SeederController::start() {
    isSeeding_ = true;
    std::cout << "Seeder started - beginning seeding operation" << std::endl;
    return true;
}

bool SeederController::stop() {
    isSeeding_ = false;
    std::cout << "Seeder stopped - seeding operation halted" << std::endl;
    return true;
}

bool SeederController::setWorkParameter(const std::string& key, double value) {
    if (key == "depth") {
        if (value < 0 || value > 0.5) {
            std::cerr << "Invalid seeding depth: " << value << " m" << std::endl;
            return false;
        }
        currentDepth_ = value;
        std::cout << "Seeding depth set to: " << currentDepth_ << " m" << std::endl;
        return true;
    } else if (key == "rate") {
        if (!validateSeedingRate(value)) {
            std::cerr << "Invalid seeding rate: " << value << " kg/ha" << std::endl;
            return false;
        }
        seedingRate_ = value;
        std::cout << "Seeding rate set to: " << seedingRate_ << " kg/ha" << std::endl;
        return true;
    }
    
    std::cerr << "Unknown parameter: " << key << std::endl;
    return false;
}

ImplementStatus SeederController::getStatus() const {
    ImplementStatus status;
    status.state = isSeeding_ ? ImplementState::ACTIVE : ImplementState::IDLE;
    status.is_connected = true;
    status.current_depth = currentDepth_;
    status.current_rate = seedingRate_;
    // status.error_code = 0;  // 该成员不存在
    // status.timestamp = 0;   // 该成员不存在
    
    return status;
}

DiagnosticReport SeederController::runDiagnostics() {
    DiagnosticReport report;
    report.passed = true;
    // report.error_count = 0;  // 该成员不存在
    
    // 检查播种深度
    if (currentDepth_ < 0.01 || currentDepth_ > 0.4) {
        report.findings.push_back("Warning: Seeding depth may be out of optimal range");
    }
    
    // 检查播种率
    if (seedingRate_ < 10.0 || seedingRate_ > 200.0) {
        report.findings.push_back("Warning: Seeding rate may be out of typical range");
    }
    
    if (report.findings.empty()) {
        report.findings.push_back("Seeder system operational - all parameters within normal range");
    }
    
    std::cout << "Seeder diagnostics completed" << std::endl;
    return report;
}

void SeederController::update(double dt) {
    (void)dt; // 避免未使用参数警告
    
    if (isSeeding_) {
        // 模拟播种过程中的状态更新
        // 在实际实现中，这里会更新播种进度、检查种子箱状态等
    }
}

bool SeederController::validateSeedingRate(double rate) const {
    // 典型播种率范围：10-200 kg/ha
    return rate >= 0 && rate <= 300.0;
}

} // namespace VCUCore
