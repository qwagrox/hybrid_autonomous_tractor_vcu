// src/execution/safety_monitor.cpp
#include "safety_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCUCore {

SafetyMonitor::SafetyMonitor(uint32_t historySize, uint32_t checkInterval)
    : maxViolationHistory_(historySize), checkIntervalMs_(checkInterval),
      safetyViolation_(false), adaptiveSafetyMargin_(1.0f) {
    
    initializeDefaultLimits();
    
    // 初始化模型
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    batteryModel_ = std::make_unique<BatteryModel>();
    
    // 初始化风险模式
    std::fill(riskPattern_.begin(), riskPattern_.end(), 0.5f);
    std::fill(safetyScores_.begin(), safetyScores_.end(), 1.0f);
}

void SafetyMonitor::initializeDefaultLimits() {
    // 默认安全限制
    limits_ = {
        .maxEngineTorque = 600.0f,
        .maxMotorTorque = 450.0f,
        .maxEngineSpeed = 2500.0f,
        .maxMotorSpeed = 6000.0f,
        .maxVehicleSpeed = 40.0f,
        .maxHydraulicPressure = 250.0f,
        .maxSystemTemperature = 105.0f,
        .minBatteryVoltage = 480.0f,
        .maxBatteryCurrent = 300.0f,
        .wheelSlipThreshold = 0.35f,
        .rolloverThreshold = 0.8f,
        .collisionRiskThreshold = 0.7f
    };
}

SafetyCheckResult SafetyMonitor::checkSafety(const ControlCommands& commands,
                                           const VehicleState& vehicleState,
                                           const SystemHealthStatus& healthStatus) {
    
    SafetyCheckResult result;
    result.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    // 执行各项安全检查
    result.torqueCheck = checkTorqueLimits(commands, vehicleState);
    result.speedCheck = checkSpeedLimits(vehicleState);
    result.temperatureCheck = checkTemperatureLimits(healthStatus);
    result.stabilityCheck = checkStabilityLimits(vehicleState);
    
    // 组合检查结果
    result.overallSafe = result.torqueCheck.isSafe &&
                        result.speedCheck.isSafe &&
                        result.temperatureCheck.isSafe &&
                        result.stabilityCheck.isSafe;
    
    result.riskScore = calculateRiskScore(vehicleState, EnvironmentData{});
    
    // 更新安全状态
    if (!result.overallSafe) {
        safetyViolation_ = true;
        
        // 记录严重违规
        for (const auto& violation : result.torqueCheck.violations) {
            if (isCriticalViolation(violation)) {
                logViolation(violation);
            }
        }
    } else {
        safetyViolation_ = false;
    }
    
    return result;
}

bool SafetyMonitor::validateCommands(const ControlCommands& commands, const VehicleState& state) {
    // 快速命令验证
    if (commands.engineTorqueRequest > limits_.maxEngineTorque * 1.1f) {
        return false;
    }
    
    if (commands.motorTorqueRequest > limits_.maxMotorTorque * 1.1f) {
        return false;
    }
    
    if (commands.cvtRatioRequest < 0.4f || commands.cvtRatioRequest > 3.2f) {
        return false;
    }
    
    if (state.velocity.norm() > limits_.maxVehicleSpeed * 1.1f) {
        return false;
    }
    
    return true;
}

ControlCommands SafetyMonitor::applySafetyLimits(const ControlCommands& commands, const VehicleState& state) {
    ControlCommands safeCommands = commands;
    
    // 应用扭矩限制
    safeCommands = limitTorqueCommands(safeCommands, state);
    
    // 应用速度限制
    safeCommands = limitSpeedCommands(safeCommands, state);
    
    // 应用紧急限制
    if (safetyViolation_) {
        safeCommands = limitEmergencyCommands(safeCommands);
    }
    
    return safeCommands;
}

SafetyCheckResult SafetyMonitor::checkTorqueLimits(const ControlCommands& commands, const VehicleState& state) {
    SafetyCheckResult result;
    
    // 检查发动机扭矩
    if (commands.engineTorqueRequest > limits_.maxEngineTorque * adaptiveSafetyMargin_) {
        SafetyViolation violation = {
            .type = ViolationType::TORQUE_OVERLIMIT,
            .component = "Engine",
            .severity = ViolationSeverity::HIGH,
            .currentValue = commands.engineTorqueRequest,
            .limitValue = limits_.maxEngineTorque,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    // 检查电机扭矩
    if (commands.motorTorqueRequest > limits_.maxMotorTorque * adaptiveSafetyMargin_) {
        SafetyViolation violation = {
            .type = ViolationType::TORQUE_OVERLIMIT,
            .component = "Motor",
            .severity = ViolationSeverity::HIGH,
            .currentValue = commands.motorTorqueRequest,
            .limitValue = limits_.maxMotorTorque,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    // 检查扭矩变化率
    float torqueChangeRate = std::abs(commands.engineTorqueRequest - state.actualTorque);
    if (torqueChangeRate > 100.0f) { // 100 Nm/s
        SafetyViolation violation = {
            .type = ViolationType::TORQUE_RATE_OVERLIMIT,
            .component = "Powertrain",
            .severity = ViolationSeverity::MEDIUM,
            .currentValue = torqueChangeRate,
            .limitValue = 100.0f,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkSpeedLimits(const VehicleState& state) {
    SafetyCheckResult result;
    
    // 检查车辆速度
    if (state.velocity.norm() > limits_.maxVehicleSpeed) {
        SafetyViolation violation = {
            .type = ViolationType::SPEED_OVERLIMIT,
            .component = "Vehicle",
            .severity = ViolationSeverity::HIGH,
            .currentValue = state.velocity.norm(),
            .limitValue = limits_.maxVehicleSpeed,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    // 检查发动机转速
    if (state.engineRpm > limits_.maxEngineSpeed) {
        SafetyViolation violation = {
            .type = ViolationType::RPM_OVERLIMIT,
            .component = "Engine",
            .severity = ViolationSeverity::HIGH,
            .currentValue = state.engineRpm,
            .limitValue = limits_.maxEngineSpeed,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkTemperatureLimits(const SystemHealthStatus& healthStatus) {
    SafetyCheckResult result;
    
    // 检查系统温度
    for (const auto& fault : healthStatus.activeFaults) {
        if (fault.component.find("temperature") != std::string::npos &&
            fault.severity >= FaultSeverity::WARNING) {
            SafetyViolation violation = {
                .type = ViolationType::TEMPERATURE_OVERLIMIT,
                .component = fault.component,
                .severity = ViolationSeverity::MEDIUM,
                .currentValue = 0.0f, // 需要实际温度值
                .limitValue = limits_.maxSystemTemperature,
                .timestamp = std::chrono::duration_cast<Timestamp>(
                    std::chrono::system_clock::now().time_since_epoch())
            };
            result.violations.push_back(violation);
        }
    }
    
    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkStabilityLimits(const VehicleState& state) {
    SafetyCheckResult result;
    
    // 检查侧倾风险
    float rollAngle = std::abs(state.roll);
    if (rollAngle > limits_.rolloverThreshold) {
        SafetyViolation violation = {
            .type = ViolationType::STABILITY_RISK,
            .component = "Chassis",
            .severity = ViolationSeverity::CRITICAL,
            .currentValue = rollAngle,
            .limitValue = limits_.rolloverThreshold,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    // 检查轮滑
    if (state.wheelSlipRatio > limits_.wheelSlipThreshold) {
        SafetyViolation violation = {
            .type = ViolationType::WHEEL_SLIP,
            .component = "Drivetrain",
            .severity = ViolationSeverity::MEDIUM,
            .currentValue = state.wheelSlipRatio,
            .limitValue = limits_.wheelSlipThreshold,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch())
        };
        result.violations.push_back(violation);
    }
    
    result.isSafe = result.violations.empty();
    return result;
}

ControlCommands SafetyMonitor::limitTorqueCommands(const ControlCommands& commands, const VehicleState& state) {
    ControlCommands limited = commands;
    
    // 限制发动机扭矩
    limited.engineTorqueRequest = std::min(limited.engineTorqueRequest,
        limits_.maxEngineTorque * adaptiveSafetyMargin_);
    
    // 限制电机扭矩
    limited.motorTorqueRequest = std::min(limited.motorTorqueRequest,
        limits_.maxMotorTorque * adaptiveSafetyMargin_);
    
    // 限制扭矩变化率
    float currentTorque = state.actualTorque;
    float maxChange = 100.0f * (1.0f / 100.0f); // 100 Nm/s 变化率限制
    
    limited.engineTorqueRequest = std::clamp(limited.engineTorqueRequest,
        currentTorque - maxChange, currentTorque + maxChange);
    
    return limited;
}

ControlCommands SafetyMonitor::limitEmergencyCommands(const ControlCommands& commands) {
    ControlCommands emergencyCommands = commands;
    
    // 紧急状态下限制命令
    emergencyCommands.engineTorqueRequest *= 0.5f;
    emergencyCommands.motorTorqueRequest *= 0.5f;
    emergencyCommands.hydraulicPressureRequest = 0.0f;
    emergencyCommands.emergencyStop = true;
    
    return emergencyCommands;
}

float SafetyMonitor::calculateRiskScore(const VehicleState& state, const EnvironmentData& environment) const {
    float score = 0.0f;
    
    // 速度风险
    score += (state.velocity.norm() / limits_.maxVehicleSpeed) * 0.3f;
    
    // 负载风险
    score += (state.drawbarPull / 10000.0f) * 0.2f;
    
    // 地形风险
    score += (std::abs(state.gradeAngle) / 0.3f) * 0.2f; // 30%坡度
    
    // 时间风险（夜间或恶劣天气）
    score += riskPattern_[0] * 0.3f; // 简化实现
    
    return std::min(score, 1.0f);
}

void SafetyMonitor::logViolation(const SafetyViolation& violation) {
    violationHistory_.push_back(violation);
    if (violationHistory_.size() > maxViolationHistory_) {
        violationHistory_.erase(violationHistory_.begin());
    }
    
    // 更新安全评分
    float violationScore = static_cast<float>(violation.severity) / 10.0f;
    updateSafetyScore(1.0f - violationScore);
}

void SafetyMonitor::updateSafetyScore(float newScore) {
    // 移动平均更新安全评分
    for (int i = safetyScores_.size() - 1; i > 0; --i) {
        safetyScores_[i] = safetyScores_[i - 1];
    }
    safetyScores_[0] = newScore;
}

float SafetyMonitor::getOverallSafetyScore() const {
    float sum = 0.0f;
    for (float score : safetyScores_) {
        sum += score;
    }
    return sum / safetyScores_.size();
}

bool SafetyMonitor::isCriticalViolation(const SafetyViolation& violation) const {
    return violation.severity >= ViolationSeverity::HIGH ||
           violation.type == ViolationType::STABILITY_RISK ||
           violation.type == ViolationType::COLLISION_IMMINENT;
}

} // namespace VCUCore