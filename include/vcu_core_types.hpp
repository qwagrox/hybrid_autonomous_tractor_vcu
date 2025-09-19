// include/vcu_core_types.hpp
#pragma once
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <map>

namespace VCUCore {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Timestamp = uint64_t;

enum class SystemState : uint8_t {
    OFF, STANDBY, READY, RUNNING, 
    DEGRADED, FAULT, EMERGENCY, CALIBRATION
};

enum class DriveMode : uint8_t {
    ECO, COMFORT, SPORT, 
    PLOWING, SEEDING, TRANSPORT,
    ROAD, FIELD, MANUAL,
    LOADING, UNLOADING, PARKING
};

enum class CVTManufacturer : uint8_t {
    UNKNOWN,
    JOHN_DEERE,
    CASE_IH,
    CLAAS
};

struct CVTState {
    float currentRatio;
    float targetRatio;
    bool isShifting;
    uint32_t timestamp;
};

struct EngineData {
    float actualTorque;
    float percentLoad;
    float speed;
    float fuelRate;
    float boostPressure;
    float temperature;
    float oilPressure;
    uint8_t derateStatus;
    uint16_t errorCodes;
    uint32_t timestamp;
    uint8_t operatingHours;
    float efficiency;
};

struct TractorVehicleState {
    Vector3d position;
    Vector3d velocity;
    Vector3d acceleration;
    float heading;
    float pitch;
    float roll;
    float gradeAngle;
    Vector3d imuAngularRate;
    float drawbarPull;
    float drawbarPower;
    float wheelSlipRatio;
    float tractionEfficiency;
    float estimatedMass;
    float frontAxleLoad;
    float rearAxleLoad;
    float ballastMass;
    float actualTorque;
    float demandedTorque;
    float engineLoad;
    float pto_rpm;
    float pto_torque;
    float powerConsumption;
    float fuelConsumption;
    float energyEfficiency;
    float specificFuelConsumption;
    float workingWidth;
    float workingDepth;
    float workingSpeed;
    float fieldEfficiency;
    float workedArea;
    uint32_t workingHours;
    float stabilityMargin;
    float centerOfGravityHeight;
    float turningRadius;
    bool rolloverRisk;
    float groundClearance;
    float hydraulicPressure;
    float hydraulicFlowRate;
    float hydraulicOilTemperature;
    float hitchHeight;
    float soilCompaction;
    float soilMoisture;
    float wheelLoadDistribution[4];
    Matrix3d estimationCovariance;
    SystemState systemState;
    DriveMode driveMode;
    uint32_t timestamp;
    float batterySOC;
    float engineRpm;
    float motorRpm;
    float engineTemperature;
    float motorTemperature;
    bool isWorking;
    bool isTransporting;
    bool isTurning;
    bool isPTOEngaged;
    bool isHydraulicActive;
};

struct ControlCommands {
    float engineTorqueRequest;
    float motorTorqueRequest;
    int transmissionGearRequest;
    float hydraulicPressureRequest;
    bool implementLiftRequest;
    float cvtRatioRequest;
    bool emergencyStop;
    uint8_t controlMode;
    float maxTorqueLimit;
    float minTorqueLimit;
    float torqueChangeRate;
    float ratioChangeRate;
    uint32_t timestamp;
};

struct PerceptionData {
    TractorVehicleState vehicleState;
    float terrainSlope;
    float rollingResistance;
    float aerodynamicDrag;
};

struct PredictionResult {
    std::vector<float> loadForecast;
};

struct SensorData {
    Vector3d gnssPosition;
    Vector3d imuAcceleration;
    Vector3d imuAngularRate;
    float wheelSpeed[4];
    float steeringAngle;
    float engineRpm;
    float motorRpm;
    float batteryVoltage;
    float batteryCurrent;
    uint32_t timestamp;
};

struct LearningExperience {
    // Placeholder
};

struct LearningModel {
    // Placeholder
};

struct EnvironmentData {
    // Placeholder
};

struct OperatorBehavior {
    // Placeholder
};

struct TaskRequirements {
    // Placeholder
};

enum class MaintenanceSeverity : uint8_t {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

enum class FaultSeverity : uint8_t {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

struct MaintenanceItem {
    std::string component;
    std::string description;
    MaintenanceSeverity severity;
    float estimatedCost;
    int estimatedTime;
    uint32_t dueDate;
};

struct FaultDiagnosis {
    uint32_t faultCode;
    FaultSeverity severity;
    std::string description;
    std::string component;
    uint64_t timestamp;
    uint32_t duration;
    bool isActive;
    bool isRecoverable;
    std::vector<std::string> recoverySteps;
};

struct SystemHealthStatus {
    uint64_t timestamp;
    float overallHealth;
    bool isHealthy;
    uint32_t uptime;
    uint32_t lastMaintenance;
    uint32_t nextMaintenance;
    std::map<std::string, float> componentHealth;
    std::vector<FaultDiagnosis> activeFaults;
};

struct PerformanceStatistics {
    // Placeholder
};

struct HealthForecast {
    // Placeholder
};

struct MaintenanceRecord {
    // Placeholder
};

struct DiagnosticTest {
    // Placeholder
};

struct DiagnosticResult {
    // Placeholder
};

struct HealthReport {
    // Placeholder
};

struct HealthTrend {
    // Placeholder
};

struct EngineState {
    float operatingHours;
    float coolantTemperature;
    float currentTorque;
    float maxAvailableTorque;
};

struct MotorState {
    float operatingHours;
    float windingTemperature;
    float currentCurrent;
};

struct BatteryState {
    // Placeholder
};

} // namespace VCUCore



namespace VCUCore {

struct BatteryHealth {
    float stateOfHealth;
    float stateOfCharge;
    float internalResistance;
};

struct BatteryFault {
    uint32_t faultCode;
    std::string description;
};

struct BatteryStatistics {
    float totalEnergyCharged;
    float totalEnergyDischarged;
    uint32_t cycleCount;
};

} // namespace VCUCore



namespace VCUCore {

const int GPIO_PIN_IMPLEMENT_LIFT = 1;

} // namespace VCUCore



namespace VCUCore {

struct ActuatorCommand {
    uint64_t timestamp;
    float engineTorque;
    float motorTorque;
};

struct ActuatorDiagnostic {
    // Placeholder
};

} // namespace VCUCore

