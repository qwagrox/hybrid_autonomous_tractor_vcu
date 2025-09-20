#pragma once

#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/battery_model.hpp"
#include <memory>
#include <vector>
#include <atomic>
#include <chrono> // Add for std::chrono

namespace VCUCore {

class SafetyMonitor {
private:
    struct SafetyLimits {
        float maxEngineTorque;
        float maxMotorTorque;
        float maxEngineSpeed;
        float maxMotorSpeed;
        float maxVehicleSpeed;
        float maxHydraulicPressure;
        float maxSystemTemperature;
        float minBatteryVoltage;
        float maxBatteryCurrent;
        float wheelSlipThreshold;
        float rolloverThreshold;
        float collisionRiskThreshold;
    };

    SafetyLimits limits_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    std::unique_ptr<BatteryModel> batteryModel_;

    std::atomic<bool> safetyViolation_;
    std::vector<SafetyViolation> violationHistory_;
    std::array<float, 10> safetyScores_;

    uint32_t maxViolationHistory_;
    uint32_t checkIntervalMs_;

    // Learning parameters
    std::array<float, 24> riskPattern_;
    float adaptiveSafetyMargin_;

public:
    SafetyMonitor(uint32_t historySize = 1000, uint32_t checkInterval = 100);

    SafetyCheckResult checkSafety(const ControlCommands& commands,
                                const TractorVehicleState& vehicleState,
                                const SystemHealthStatus& healthStatus);

    bool validateCommands(const ControlCommands& commands, const TractorVehicleState& state);
    ControlCommands applySafetyLimits(const ControlCommands& commands, const TractorVehicleState& state);

    // Risk assessment
    float calculateRiskScore(const TractorVehicleState& state, const EnvironmentData& environment) const;
    RiskAssessment assessRisk(const TractorVehicleState& state, const PredictionResult& prediction) const;

    // Emergency handling
    EmergencyResponse handleEmergency(const SafetyViolation& violation);
    bool triggerEmergencyStop(EmergencyLevel level, const std::string& reason);

    // Learning functions
    void updateSafetyModel(const TractorVehicleState& state, const SafetyViolation& violation);
    void adjustSafetyLimitsBasedOnExperience();

    // Monitoring functions
    SafetyStatus getSafetyStatus() const;
    std::vector<SafetyViolation> getViolationHistory() const;
    float getOverallSafetyScore() const;

private:
    void initializeDefaultLimits();
    SafetyCheckResult checkTorqueLimits(const ControlCommands& commands, const TractorVehicleState& state);
    SafetyCheckResult checkSpeedLimits(const TractorVehicleState& state);
    SafetyCheckResult checkTemperatureLimits(const SystemHealthStatus& healthStatus);
    // Corrected from BatteryData to BatteryState
    SafetyCheckResult checkBatteryLimits(const BatteryState& battery);
    SafetyCheckResult checkStabilityLimits(const TractorVehicleState& state);
    SafetyCheckResult checkCollisionRisk(const TractorVehicleState& state, const PerceptionData& perception);

    bool isCriticalViolation(const SafetyViolation& violation) const;
    void logViolation(const SafetyViolation& violation);
    void updateSafetyScore(float newScore);

    ControlCommands limitTorqueCommands(const ControlCommands& commands, const TractorVehicleState& state);
    ControlCommands limitSpeedCommands(const ControlCommands& commands, const TractorVehicleState& state);
    ControlCommands limitEmergencyCommands(const ControlCommands& commands);
    
    // Helper to get current timestamp
    static uint64_t getCurrentTimestamp() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

} // namespace VCUCore

