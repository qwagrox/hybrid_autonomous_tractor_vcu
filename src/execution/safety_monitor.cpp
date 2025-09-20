#include "execution/safety_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCUCore {

SafetyMonitor::SafetyMonitor(uint32_t historySize, uint32_t checkInterval)
    : maxViolationHistory_(historySize), checkIntervalMs_(checkInterval),
      safetyViolation_(false), adaptiveSafetyMargin_(1.0f) {

    initializeDefaultLimits();

    // Initialize models
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    batteryModel_ = std::make_unique<BatteryModel>();

    // Initialize risk patterns
    std::fill(riskPattern_.begin(), riskPattern_.end(), 0.5f);
    std::fill(safetyScores_.begin(), safetyScores_.end(), 1.0f);
}

void SafetyMonitor::initializeDefaultLimits() {
    // Default safety limits
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
                                           const TractorVehicleState& vehicleState,
                                           const SystemHealthStatus& healthStatus) {

    SafetyCheckResult result{};
    result.timestamp = getCurrentTimestamp();

    // Perform various safety checks
    auto torqueCheck = checkTorqueLimits(commands, vehicleState);
    auto speedCheck = checkSpeedLimits(vehicleState);
    auto temperatureCheck = checkTemperatureLimits(healthStatus);
    auto stabilityCheck = checkStabilityLimits(vehicleState);

    result.violations.insert(result.violations.end(), torqueCheck.violations.begin(), torqueCheck.violations.end());
    result.violations.insert(result.violations.end(), speedCheck.violations.begin(), speedCheck.violations.end());
    result.violations.insert(result.violations.end(), temperatureCheck.violations.begin(), temperatureCheck.violations.end());
    result.violations.insert(result.violations.end(), stabilityCheck.violations.begin(), stabilityCheck.violations.end());

    // Combine check results
    result.isSafe = torqueCheck.isSafe && speedCheck.isSafe && temperatureCheck.isSafe && stabilityCheck.isSafe;

    result.riskScore = calculateRiskScore(vehicleState, EnvironmentData{});

    // Update safety status
    if (!result.isSafe) {
        safetyViolation_ = true;

        // Log critical violations
        for (const auto& violation : result.violations) {
            if (isCriticalViolation(violation)) {
                logViolation(violation);
            }
        }
    } else {
        safetyViolation_ = false;
    }

    return result;
}

bool SafetyMonitor::validateCommands(const ControlCommands& commands, const TractorVehicleState& state) {
    // Quick command validation
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

ControlCommands SafetyMonitor::applySafetyLimits(const ControlCommands& commands, const TractorVehicleState& state) {
    ControlCommands safeCommands = commands;

    // Apply torque limits
    safeCommands = limitTorqueCommands(safeCommands, state);

    // Apply speed limits
    safeCommands = limitSpeedCommands(safeCommands, state);

    // Apply emergency limits
    if (safetyViolation_) {
        safeCommands = limitEmergencyCommands(safeCommands);
    }

    return safeCommands;
}

SafetyCheckResult SafetyMonitor::checkTorqueLimits(const ControlCommands& commands, const TractorVehicleState& state) {
    SafetyCheckResult result{};
    result.timestamp = getCurrentTimestamp();

    // Check engine torque
    if (commands.engineTorqueRequest > limits_.maxEngineTorque * adaptiveSafetyMargin_) {
        result.violations.push_back({
            .type = ViolationType::TORQUE_LIMIT_EXCEEDED,
            .severity = ViolationSeverity::HIGH,
            .component = "Engine",
            .value = commands.engineTorqueRequest,
            .limit = limits_.maxEngineTorque,
            .currentValue = commands.engineTorqueRequest,
            .timestamp = getCurrentTimestamp()
        });
    }

    // Check motor torque
    if (commands.motorTorqueRequest > limits_.maxMotorTorque * adaptiveSafetyMargin_) {
        result.violations.push_back({
            .type = ViolationType::TORQUE_LIMIT_EXCEEDED,
            .severity = ViolationSeverity::HIGH,
            .component = "Motor",
            .value = commands.motorTorqueRequest,
            .limit = limits_.maxMotorTorque,
            .currentValue = commands.motorTorqueRequest,
            .timestamp = getCurrentTimestamp()
        });
    }

    // Check torque change rate
    float torqueChangeRate = std::abs(static_cast<float>(commands.engineTorqueRequest) - state.actualTorque);
    if (torqueChangeRate > 100.0f) { // 100 Nm/s
        result.violations.push_back({
            .type = ViolationType::TORQUE_RATE_OVERLIMIT,
            .severity = ViolationSeverity::MEDIUM,
            .component = "Powertrain",
            .value = torqueChangeRate,
            .limit = 100.0f,
            .currentValue = torqueChangeRate,
            .timestamp = getCurrentTimestamp()
        });
    }

    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkSpeedLimits(const TractorVehicleState& state) {
    SafetyCheckResult result{};
    result.timestamp = getCurrentTimestamp();

    // Check vehicle speed
    if (state.velocity.norm() > limits_.maxVehicleSpeed) {
        result.violations.push_back({
            .type = ViolationType::SPEED_LIMIT_EXCEEDED,
            .severity = ViolationSeverity::HIGH,
            .component = "Vehicle",
            .value = state.velocity.norm(),
            .limit = limits_.maxVehicleSpeed,
            .currentValue = state.velocity.norm(),
            .timestamp = getCurrentTimestamp()
        });
    }

    // Check engine speed
    if (state.engineRpm > limits_.maxEngineSpeed) {
        result.violations.push_back({
            .type = ViolationType::RPM_OVERLIMIT,
            .severity = ViolationSeverity::HIGH,
            .component = "Engine",
            .value = state.engineRpm,
            .limit = limits_.maxEngineSpeed,
            .currentValue = state.engineRpm,
            .timestamp = getCurrentTimestamp()
        });
    }

    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkTemperatureLimits(const SystemHealthStatus& healthStatus) {
    SafetyCheckResult result{};
    result.timestamp = getCurrentTimestamp();

    // Check system temperature
    for (const auto& fault : healthStatus.activeFaults) {
        if (fault.component.find("temperature") != std::string::npos &&
            fault.severity >= static_cast<double>(FaultSeverity::WARNING)) {
            result.violations.push_back({
                .type = ViolationType::TEMPERATURE_LIMIT_EXCEEDED,
                .severity = ViolationSeverity::MEDIUM,
                .component = fault.component,
                .value = 0.0f, // Actual temperature value needed
                .limit = limits_.maxSystemTemperature,
                .currentValue = 0.0f,
                .timestamp = getCurrentTimestamp()
            });
        }
    }

    result.isSafe = result.violations.empty();
    return result;
}

SafetyCheckResult SafetyMonitor::checkStabilityLimits(const TractorVehicleState& state) {
    SafetyCheckResult result{};
    result.timestamp = getCurrentTimestamp();

    // Check roll risk
    float rollAngle = std::abs(static_cast<float>(state.roll));
    if (rollAngle > limits_.rolloverThreshold) {
        result.violations.push_back({
            .type = ViolationType::STABILITY_RISK,
            .severity = ViolationSeverity::CRITICAL,
            .component = "Chassis",
            .value = rollAngle,
            .limit = limits_.rolloverThreshold,
            .currentValue = rollAngle,
            .timestamp = getCurrentTimestamp()
        });
    }

    // Check wheel slip
    if (state.wheelSlipRatio > limits_.wheelSlipThreshold) {
        result.violations.push_back({
            .type = ViolationType::WHEEL_SLIP,
            .severity = ViolationSeverity::MEDIUM,
            .component = "Drivetrain",
            .value = state.wheelSlipRatio,
            .limit = limits_.wheelSlipThreshold,
            .currentValue = state.wheelSlipRatio,
            .timestamp = getCurrentTimestamp()
        });
    }

    result.isSafe = result.violations.empty();
    return result;
}

ControlCommands SafetyMonitor::limitTorqueCommands(const ControlCommands& commands, const TractorVehicleState& state) {
    ControlCommands limited = commands;

    // Limit engine torque
    limited.engineTorqueRequest = std::min(limited.engineTorqueRequest,
        static_cast<double>(limits_.maxEngineTorque * adaptiveSafetyMargin_));

    // Limit motor torque
    limited.motorTorqueRequest = std::min(limited.motorTorqueRequest,
        static_cast<double>(limits_.maxMotorTorque * adaptiveSafetyMargin_));

    // Limit torque change rate
    double currentTorque = static_cast<double>(state.actualTorque);
    double maxChange = 100.0 * (1.0 / 100.0); // 100 Nm/s change rate limit

    limited.engineTorqueRequest = std::clamp(limited.engineTorqueRequest,
        currentTorque - maxChange, currentTorque + maxChange);

    return limited;
}

ControlCommands SafetyMonitor::limitEmergencyCommands(const ControlCommands& commands) {
    ControlCommands emergencyCommands = commands;

    // Limit commands in emergency state
    emergencyCommands.engineTorqueRequest *= 0.5f;
    emergencyCommands.motorTorqueRequest *= 0.5f;
    emergencyCommands.hydraulicPressureRequest = 0.0f;
    emergencyCommands.emergencyStop = true;

    return emergencyCommands;
}

float SafetyMonitor::calculateRiskScore(const TractorVehicleState& state, const EnvironmentData& environment) const {
    float score = 0.0f;

    // Speed risk
    score += (state.velocity.norm() / limits_.maxVehicleSpeed) * 0.3f;

    // Load risk
    score += (state.drawbarPull / 10000.0f) * 0.2f;

    // Terrain risk
    score += (std::abs(static_cast<float>(state.gradeAngle)) / 0.3f) * 0.2f; // 30% slope

    // Time risk (night or bad weather)
    score += riskPattern_[0] * 0.3f; // Simplified implementation

    return std::min(score, 1.0f);
}

void SafetyMonitor::logViolation(const SafetyViolation& violation) {
    violationHistory_.push_back(violation);
    if (violationHistory_.size() > maxViolationHistory_) {
        violationHistory_.erase(violationHistory_.begin());
    }

    // Update safety score
    float violationScore = static_cast<float>(violation.severity) / 10.0f;
    updateSafetyScore(1.0f - violationScore);
}

void SafetyMonitor::updateSafetyScore(float newScore) {
    // Moving average update of safety score
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

// Dummy implementations for functions that require more context
SafetyCheckResult SafetyMonitor::checkBatteryLimits(const BatteryState& battery) { 
    SafetyCheckResult result{};
    result.isSafe = true;
    return result; 
}

SafetyCheckResult SafetyMonitor::checkCollisionRisk(const TractorVehicleState& state, const PerceptionData& perception) { 
    SafetyCheckResult result{};
    result.isSafe = true;
    return result; 
}

RiskAssessment SafetyMonitor::assessRisk(const TractorVehicleState& state, const PredictionResult& prediction) const {
    return RiskAssessment{};
}

EmergencyResponse SafetyMonitor::handleEmergency(const SafetyViolation& violation) {
    return EmergencyResponse{};
}

bool SafetyMonitor::triggerEmergencyStop(EmergencyLevel level, const std::string& reason) {
    return true;
}

void SafetyMonitor::updateSafetyModel(const TractorVehicleState& state, const SafetyViolation& violation) {}

void SafetyMonitor::adjustSafetyLimitsBasedOnExperience() {}

SafetyStatus SafetyMonitor::getSafetyStatus() const {
    return SafetyStatus{};
}

std::vector<SafetyViolation> SafetyMonitor::getViolationHistory() const {
    return violationHistory_;
}

} // namespace VCUCore

