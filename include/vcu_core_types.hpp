// include/vcu_core_types.hpp - 基于GitHub最新代码的完整修复版本
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

// 使用统一的数据类型
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// 时间戳类型定义
using Timestamp = uint64_t;

// 枚举类型定义
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
    EMERGENCY
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

// 预测策略枚举
enum class PredictionStrategy {
    SIMPLE_LINEAR,
    KALMAN_FILTER,
    NEURAL_NETWORK,
    HYBRID_APPROACH,
    NMPC_BASED
};

// 基础结构体定义
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
    
    // 添加缺失的成员
    double centerOfGravityHeight;
    double rolloverRisk;
    bool isWorking;
    bool isPTOEngaged;
    bool isTransporting;
    
    uint64_t timestamp;
};

struct CVTState {
    double currentRatio;
    double targetRatio;
    double ratioChangeRate;
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
    
    // 发动机相关 - 使用实际代码中的名称
    double engineRPM;      // 保留这个
    double engineRpm;      // 添加这个（代码中使用的名称）
    double engineTorque;
    
    double fuelLevel;
    double batteryVoltage;
    double hydraulicPressure;
    double oilTemperature;
    double coolantTemperature;
    uint64_t timestamp;
};

// EngineData结构体 - 基于实际代码使用的成员
struct EngineData {
    double rpm;
    double torque;
    double power;
    double fuelConsumption;
    double temperature;
    double oilPressure;
    double throttlePosition;
    bool isRunning;
    
    // 添加实际代码中使用的成员
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

// 添加OptimizationResult结构体
struct OptimizationResult {
    double optimalValue;
    std::vector<double> parameters;
    bool converged;
    int iterations;
    double computationTime;
    uint64_t timestamp;
};

// 能源管理相关结构体 - 添加缺失的类型
struct EnergyOptimization {
    double optimalEngineTorque;
    double optimalMotorTorque;
    double optimalBatteryPower;
    double predictedEfficiency;
    double estimatedFuelSaving;
    double computationTime;
    bool isValid;
    uint64_t timestamp;
};

struct PowerFlow {
    double enginePower;
    double motorPower;
    double batteryPower;
    double hydraulicPower;
    double auxiliaryPower;
    double totalDemand;
    double efficiency;
    double operatingCost;
    uint64_t timestamp;
};

enum class ChargingMode {
    NONE,
    REGENERATIVE,
    GRID_CHARGING,
    SOLAR_CHARGING,
    HYBRID_CHARGING
};

struct ChargingStrategy {
    ChargingMode mode;
    double targetSOC;
    double chargingRate;
    double estimatedTime;
    double estimatedCost;
    bool isOptimal;
    uint64_t timestamp;
};

struct EnergyForecast {
    std::vector<double> powerDemandForecast;
    std::vector<double> batterySOCForecast;
    std::vector<double> fuelConsumptionForecast;
    double forecastHorizon;
    double confidence;
    uint64_t timestamp;
};

// 电池相关结构体
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

// 负载检测相关结构体
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

// 控制命令结构体
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

// 系统健康相关结构体
struct SystemHealthStatus {
    double overallHealth;
    double batteryLevel;
    double engineLoad;
    double fuelLevel;
    double hydraulicPressure;
    bool isHealthy;
    std::vector<std::string> activeWarnings;
    std::vector<std::string> activeFaults;
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

// CVT制造商参数结构体 - 统一定义，避免重复
struct CVTManufacturerParams {
    float minRatio;
    float maxRatio;
    float defaultRatio;
    std::string name;
};

// 扭矩分配结果结构体
struct TorqueDistribution {
    double engineTorque;
    double motorTorque;
    double totalTorque;
    double efficiency;
    uint64_t timestamp;
};

// 故障诊断相关结构体
struct FaultDiagnosis {
    uint32_t faultCode;
    std::string description;
    double severity;
    std::string component;
    uint64_t timestamp;
    bool isActive;
    std::string recommendedAction;
};

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

// 动力学相关结构体 - 统一定义，避免重复
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

// 预测性能结构体 - 统一定义，避免重复
struct PredictionPerformance {
    PredictionStrategy strategy;
    double averageError;
    double maxError;
    double computationalTime;
    double robustnessScore;
    uint32_t sampleCount;
    Timestamp lastUpdate;
};

} // namespace VCUCore
