// include/vcu_core_types.hpp - Patched Version
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <chrono>
#include <atomic>
#include <functional>
#include <array>

namespace VCUCore {

// Using unified data types
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// Timestamp type definition
using Timestamp = uint64_t;

// Enum type definitions
enum class SystemState {
    INITIALIZING,
    IDLE,
    ACTIVE,
    ERROR,
    SHUTDOWN
};

enum class DriveMode {
    MANUAL,
    SEMI_AUTONOMOUS,
    AUTONOMOUS,
    EMERGENCY,
    PLOWING,
    SEEDING,
    TRANSPORT,
    ECO,
    COMFORT,
    SPORT
};

enum class LoadChangeType {
    INCREASE,
    DECREASE,
    STABLE,
    UNKNOWN
};

enum class LoadType {
    LIGHT,
    MEDIUM,
    HEAVY,
    VARIABLE,
    UNKNOWN
};

enum class TrendDirection {
    INCREASING,
    DECREASING,
    STABLE,
    OSCILLATING
};

enum class CVTManufacturer {
    UNKNOWN,
    GENERIC,
    JOHN_DEERE,
    CASE_IH,
    NEW_HOLLAND,
    FENDT,
    CLAAS,
    MASSEY_FERGUSON,
    KUBOTA
};

enum class FaultSeverity {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

// Prediction strategy enum
enum class PredictionStrategy {
    SIMPLE_LINEAR,
    KALMAN_FILTER,
    NEURAL_NETWORK,
    HYBRID_APPROACH,
    NMPC_BASED
};

enum class ChargingMode {
    NONE,
    REGENERATIVE,
    GRID_CHARGING,
    SOLAR_CHARGING,
    HYBRID_CHARGING
};

// Basic structure definitions
struct TractorVehicleState {
    Vector3d position;
    Vector3d velocity;
    Vector3d acceleration;
    double heading;
    double pitch;
    double roll;
    double gradeAngle;
    Vector3d imuAngularRate;
    double drawbarPull;
    double drawbarPower;
    double wheelSlipRatio;
    double tractionEfficiency;
    double estimatedMass;
    double frontAxleLoad;
    double rearAxleLoad;
    double ballastMass;
    double actualTorque;
    double demandedTorque;
    double engineLoad;
    double pto_rpm;
    double pto_torque;
    double powerConsumption;
    double fuelConsumption;
    double energyEfficiency;
    double specificFuelConsumption;
    double workingWidth;
    double workingDepth;
    double workingSpeed;
    double speed;
    double fieldEfficiency;
    double workedArea;
    uint32_t workingHours;
    double stabilityMargin;
    double wheelSpeed;

    // Added missing members
    double centerOfGravityHeight;
    double rolloverRisk;
    double batterySOC;
    double engineRpm;
    double motorRpm;
    double engineTemperature;
    double motorTemperature;
    bool isWorking;
    bool isPTOEngaged;
    bool isTransporting;

    uint64_t timestamp;
};

struct CVTState {
    double currentRatio;
    double targetRatio;
    double ratioChangeRate;
    bool isShifting;
    uint32_t timestamp;
};

struct PerceptionData {
    TractorVehicleState vehicleState;
    double terrainSlope;
    double rollingResistance;
    double aerodynamicDrag;
    uint64_t timestamp;
    double stabilityIndex;
    double tractionEfficiency;
};

struct PredictionResult {
    std::vector<double> loadForecast;
    TractorVehicleState predictedState;
    double confidence;
    uint64_t timestamp;
};

struct SensorData {
    Vector3d gnssPosition;
    Vector3d gnssVelocity;
    Vector3d imuAcceleration;
    Vector3d imuAngularRate;
    double wheelSpeed;
    std::vector<double> wheelSpeeds;

    // Engine related - using names from actual code
    double engineRPM;      // Keep this
    double engineRpm;      // Add this (name used in code)
    double engineTorque;

    double fuelLevel;
    double batteryVoltage;
    double hydraulicPressure;
    double oilTemperature;
    double coolantTemperature;
    uint64_t timestamp;
};

// EngineData struct - based on members used in actual code
struct EngineData {
    double rpm;
    double torque;
    double power;
    double fuelConsumption;
    double temperature;
    double oilPressure;
    double throttlePosition;
    bool isRunning;

    // Added members used in actual code
    double actualTorque;
    double percentLoad;
    double speed;
    double fuelRate;
    double boostPressure;

    uint64_t timestamp;
};

struct EngineState {
    double rpm;
    double torque;
    double power;
    double fuelConsumption;
    double temperature;
    double oilPressure;
    double throttlePosition;
    double efficiency;
    bool isRunning;
    bool isOverheating;
    bool hasLowOilPressure;
    uint64_t timestamp;
};

struct MotorState {
    double rpm;
    double torque;
    double power;
    double current;
    double voltage;
    double temperature;
    double efficiency;
    bool isRunning;
    bool isOverheating;
    bool hasOvercurrent;
    uint64_t timestamp;
};

struct EnergyState {
    double totalPower;
    double enginePower;
    double motorPower;
    double batteryPower;
    double hydraulicPower;
    double auxiliaryPower;
    double efficiency;
    double fuelConsumption;
    double batteryLevel;
    uint64_t timestamp;
};

// Added OptimizationResult struct
struct OptimizationResult {
    double optimalValue;
    std::vector<double> parameters;
    bool converged;
    int iterations;
    double computationTime;
    uint64_t timestamp;
};

// Define PowerFlow first, then EnergyOptimization
struct PowerFlow {
    double enginePower;
    double motorPower;
    double batteryPower;
    double hydraulicPower;
    double auxiliaryPower;
    double totalPower;
    double totalDemand;
    double efficiency;
    double operatingCost;
    uint64_t timestamp;
};

// Energy management related structs - forward declaration issue fixed
struct EnergyOptimization {
    double optimalEngineTorque;
    double optimalMotorTorque;
    double optimalBatteryPower;
    double predictedEfficiency;
    double estimatedFuelSaving;
    double computationTime;
    bool isValid;

    // PowerFlow is now defined, safe to use
    PowerFlow optimalFlow;
    double costSavings;
    double efficiencyGain;
    double batteryLifeImpact;

    uint64_t timestamp;
};

struct ChargingStrategy {
    ChargingMode mode;
    double targetSOC;
    double chargingRate;
    double estimatedTime;
    double estimatedCost;
    bool isOptimal;
    bool shouldCharge;
    uint64_t timestamp;
};

struct EnergyForecast {
    std::vector<double> powerDemandForecast;
    std::vector<double> batterySOCForecast;
    std::vector<double> fuelConsumptionForecast;
    double forecastHorizon;
    double confidence;

    // Added missing members
    double predictedGeneration;
    double predictedSOC;
    double predictedConsumption;

    uint64_t timestamp;
};

// Battery related structs
struct CellModel {
    double voltage;
    double current;
    double temperature;
    double soc;
    double soh;
    double internalResistance;
    double capacity;
    uint32_t cycleCount;
    bool isBalancing;
    uint64_t timestamp;
};

struct BatteryHealth {
    double overallHealth;
    double stateOfHealth;
    double degradationRate;
    uint32_t cycleCount;
    double temperatureHealth;
    double voltageHealth;
    double currentHealth;
    uint64_t lastMaintenanceTime;
    std::vector<std::string> healthWarnings;
};

struct BatteryFault {
    uint32_t faultCode;
    std::string faultDescription;
    double severity;
    uint64_t timestamp;
    bool isActive;
    std::string recommendedAction;
};

struct BatteryStatistics {
    double averageVoltage;
    double averageCurrent;
    double averageTemperature;
    double totalEnergyConsumed;
    double totalEnergyGenerated;
    uint32_t totalCycles;
    double efficiency;
    uint64_t operatingTime;
};

struct BatteryState {
    double voltage;
    double current;
    double temperature;
    double stateOfCharge;
    double stateOfHealth;
    double power;
    double energy;
    double internalResistance;
    double soc;
    std::vector<CellModel> cells;
    BatteryHealth health;
    BatteryFault currentFault;
    BatteryStatistics statistics;
    bool isCharging;
    bool isDischarging;
    uint64_t timestamp;
};

// Load detection related structs
struct LoadSignature {
    double drawbarForce;
    double motorTorque;
    double implementForce;
    double wheelSlip;
    double fuelConsumption;
    double powerConsumption;
    double workingDepth;
    double workingSpeed;
    double soilResistance;
    uint64_t timestamp;
};

struct LoadChangeResult {
    LoadChangeType changeType;
    double magnitude;
    double confidence;
    LoadSignature currentSignature;
    uint64_t timestamp;
};

struct LoadTrend {
    TrendDirection direction;
    double slope;
    double confidence;
    uint32_t duration;
    uint64_t timestamp;
};

// Control command struct
struct ControlCommands {
    double torqueRequest;
    double steeringAngleRequest;
    double brakeRequest;
    double cvtRatioRequest;
    double ptoSpeedRequest;
    double hydraulicPressureRequest;
    double engineTorqueRequest;
    double motorTorqueRequest;
    bool emergencyStop;
    uint64_t timestamp;
};

// Fault diagnosis related structs
struct FaultDiagnosis {
    uint32_t faultCode;
    std::string description;
    double severity;
    std::string component;
    uint64_t timestamp;
    uint32_t duration;
    bool isActive;
    bool isRecoverable;
    std::string recommendedAction;
    std::vector<std::string> recoverySteps;
};

// System health related structs
struct SystemHealthStatus {
    double overallHealth;
    double batteryLevel;
    double engineLoad;
    double fuelLevel;
    double hydraulicPressure;
    bool isHealthy;
    std::vector<std::string> activeWarnings;
    std::vector<FaultDiagnosis> activeFaults;
    uint64_t timestamp;
};

struct SystemStatus {
    VCUCore::SystemState state;
    double overallHealth;
    double batteryLevel;
    double engineLoad;
    double fuelLevel;
    double hydraulicPressure;
    std::vector<std::string> activeWarnings;
    std::vector<std::string> activeFaults;
    uint64_t timestamp;
};

struct SystemParameters {
    double maxTorque;
    double maxSpeed;
    double maxSteeringAngle;
    double wheelbase;
    double trackWidth;
    double maxDrawbarPull;
    double maxPTOPower;
    std::map<std::string, double> calibrationValues;
    uint64_t lastCalibrationTime;
};

// CVT manufacturer parameters struct - unified definition to avoid repetition
struct CVTManufacturerParams {
    double minRatio;
    double maxRatio;
    double defaultRatio;
    std::string name;
};

// Torque distribution result struct
struct TorqueDistribution {
    double engineTorque;
    double motorTorque;
    double totalTorque;
    double efficiency;
    uint64_t timestamp;
};

// Fault diagnosis related structs moved to the front

struct FaultTrend {
    std::string component;
    TrendDirection direction;
    double frequency;
    double severity;
    uint64_t timestamp;
};

struct MaintenanceItem {
    std::string component;
    std::string description;
    uint64_t dueTime;
    double priority;
    bool isOverdue;
};

struct MaintenanceRecord {
    std::string component;
    std::string action;
    uint64_t timestamp;
    std::string technician;
    std::string notes;
};

struct HealthForecast {
    std::string component;
    double predictedHealth;
    uint64_t forecastTime;
    double confidence;
    std::string recommendations;
};

struct DiagnosticTest {
    std::string testName;
    std::string component;
    std::function<bool()> testFunction;
    double expectedDuration;
};

struct DiagnosticResult {
    std::string testName;
    bool passed;
    double actualDuration;
    std::string details;
    uint64_t timestamp;
};

// Dynamics related structs - unified definition to avoid repetition
struct DynamicsStatistics {
    float maxAcceleration;
    float maxDeceleration;
    float maxSpeed;
    float averagePower;
    float energyEfficiency;
    float stabilityMargin;
    uint32_t stabilityEvents;
    uint32_t slipEvents;
};

struct DynamicsFault {
    uint16_t faultCode;
    FaultSeverity severity;
    std::string description;
    std::string component;
    uint32_t timestamp;
    float magnitude;
    bool isRecoverable;
};

struct StabilityAssessment {
    float rollStability;
    float yawStability;
    float pitchStability;
    float overallStability;
    bool isStable;
    std::vector<std::string> stabilityIssues;
};

// Prediction performance struct - unified definition to avoid repetition
struct PredictionPerformance {
    PredictionStrategy strategy;
    double averageError;
    double maxError;
    double computationalTime;
    double robustnessScore;
    uint32_t sampleCount;
    Timestamp lastUpdate;
};

// Learning related structs - for adaptive_learner
struct LearningExperience {
    SensorData sensorData;
    PerceptionData perceptionData;
    ControlCommands commands;
    double efficiency;
    double fuelConsumption;
    double performance;
    double timestamp;
    std::string context;
    std::map<std::string, double> metrics;
};

struct LearningModel {
    std::string modelType;
    std::vector<double> parameters;
    double accuracy;
    double confidence;
    uint32_t trainingIterations;
    uint64_t lastTrainingTime;
    std::map<std::string, double> hyperparameters;
};

struct OperatorBehavior {
    std::string operatorId;
    double aggressiveness;
    double efficiency;
    double experience;
    std::vector<std::string> preferences;
    std::map<std::string, double> behaviorMetrics;
    uint64_t timestamp;
};

struct TaskRequirements {
    std::string taskType;
    double requiredPrecision;
    double requiredSpeed;
    double requiredEfficiency;
    std::vector<std::string> constraints;
    std::map<std::string, double> parameters;
    uint64_t timestamp;
};

struct EnvironmentData {
    double temperature;
    double humidity;
    double windSpeed;
    double windDirection;
    double soilMoisture;
    double soilType;
    double terrainSlope;
    double visibility;
    std::string weatherCondition;
    std::map<std::string, double> environmentalFactors;
    uint64_t timestamp;
};

struct PerformanceStatistics {
    double fuelEfficiency;
    double workEfficiency;
    double energyEfficiency;
    double operationalTime;
    double idleTime;
    double workingTime;
    double averageSpeed;
    double maxSpeed;
    double totalDistance;
    double totalWorkArea;
    std::map<std::string, double> performanceMetrics;
    uint64_t timestamp;
};

struct ActuatorCommand {
    std::string actuatorId;
    std::string commandType;
    double value;
    double targetPosition;
    double targetSpeed;
    double engineTorque;
    double motorTorque;
    bool isEnabled;
    uint32_t priority;
    uint64_t timestamp;
    std::map<std::string, double> parameters;
};

struct ActuatorDiagnostic {
    std::string actuatorId;
    std::string diagnosticType;
    bool isHealthy;
    double responseTime;
    double accuracy;
    std::string errorMessage;
    uint32_t errorCode;
    uint64_t timestamp;
    std::map<std::string, double> diagnosticData;
};

// Fault statistics struct
struct FaultStatistics {
    uint32_t totalFaults;
    uint32_t activeFaults;
    uint32_t resolvedFaults;
    double averageResolutionTime;
    std::map<std::string, uint32_t> faultsByComponent;
    std::map<uint32_t, uint32_t> faultsByCode;
    uint64_t timestamp;
};

// Safety violation related enums and structs
enum class ViolationSeverity {
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
    CRITICAL = 4
};

enum class ViolationType {
    SPEED_LIMIT_EXCEEDED,
    TORQUE_LIMIT_EXCEEDED,
    TEMPERATURE_LIMIT_EXCEEDED,
    PRESSURE_LIMIT_EXCEEDED,
    STABILITY_RISK,
    COLLISION_IMMINENT,
    EMERGENCY_STOP_REQUIRED,
    WHEEL_SLIP, // Added
    RPM_OVERLIMIT, // Added
    TORQUE_RATE_OVERLIMIT // Added
};

struct SafetyViolation {
    ViolationType type;
    ViolationSeverity severity;
    std::string description;
    std::string component;
    double value;
    double limit;
    double currentValue; // Added
    uint64_t timestamp;
    bool isResolved;
};

// Safety check result enums and structs
enum class SafetyCheckStatus {
    SAFE,
    WARNING,
    CRITICAL,
    EMERGENCY
};

struct SafetyCheckResult {
    SafetyCheckStatus status;
    std::string message;
    std::vector<SafetyViolation> violations;
    double riskScore;
    bool requiresAction;
    bool isSafe; // Added
    std::string recommendedAction;
    uint64_t timestamp;
    // For detailed checks
    SafetyCheckResult* torqueCheck;
    SafetyCheckResult* speedCheck;
    SafetyCheckResult* temperatureCheck;
    SafetyCheckResult* stabilityCheck;
    bool overallSafe;
};


// Risk assessment related structs
enum class RiskLevel {
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
    CRITICAL = 4
};

struct RiskAssessment {
    RiskLevel overallRisk;
    double riskScore;
    std::map<std::string, double> componentRisks;
    std::vector<std::string> riskFactors;
    std::string primaryConcern;
    uint64_t timestamp;
};

// Emergency response related enums and structs
enum class EmergencyLevel {
    NONE = 0,
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
    CRITICAL = 4
};

enum class EmergencyAction {
    NONE,
    REDUCE_POWER,
    STOP_VEHICLE,
    ENGAGE_BRAKES,
    SHUTDOWN_SYSTEM,
    ALERT_OPERATOR
};

struct EmergencyResponse {
    EmergencyLevel level;
    EmergencyAction action;
    std::string reason;
    std::vector<std::string> steps;
    bool isActive;
    uint64_t timestamp;
    uint32_t duration;
};

// Safety status struct
struct SafetyStatus {
    SafetyCheckStatus overallStatus;
    double safetyScore;
    uint32_t activeViolations;
    uint32_t totalChecks;
    uint32_t passedChecks;
    std::vector<SafetyViolation> currentViolations;
    EmergencyLevel emergencyLevel;
    bool emergencyStopActive;
    uint64_t lastCheckTime;
    uint64_t timestamp;
};

// GPIO pin constant definitions
constexpr uint32_t GPIO_PIN_IMPLEMENT_LIFT = 12;
constexpr uint32_t GPIO_PIN_IMPLEMENT_LOWER = 13;
constexpr uint32_t GPIO_PIN_PTO_ENGAGE = 14;
constexpr uint32_t GPIO_PIN_PTO_DISENGAGE = 15;
constexpr uint32_t GPIO_PIN_HYDRAULIC_ENABLE = 16;
constexpr uint32_t GPIO_PIN_HYDRAULIC_DISABLE = 17;

} // namespace VCUCore

