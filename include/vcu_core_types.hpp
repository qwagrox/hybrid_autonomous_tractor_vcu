// include/vcu_core_types.hpp - 基于实际代码的完整修复版本
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <chrono>
#include <atomic>
#include <functional>

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

// CVT制造商参数结构体
struct CVTManufacturerParams {
    double minRatio;
    double maxRatio;
    double defaultRatio;
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

// 注意：不在这里重复定义以下结构体，因为它们在其他头文件中已定义
// - DynamicsStatistics (在vehicle_dynamics_model.hpp中定义)
// - DynamicsFault (在vehicle_dynamics_model.hpp中定义)
// - StabilityAssessment (在vehicle_dynamics_model.hpp中定义)
// - PredictionPerformance (在predictive_analytics.hpp中定义)

} // namespace VCUCore
