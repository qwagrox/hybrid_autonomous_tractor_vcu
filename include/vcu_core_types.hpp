// include/vcu_core_types.hpp - 完全修复版本
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

// 使用统一的数据类型（避免float/double混用）
using Vector3d = Eigen::Vector3d;  // 统一使用double
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

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

// 添加缺失的CVTManufacturer枚举
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
    double speed;  // 添加speed成员作为workingSpeed的别名
    double fieldEfficiency;
    double workedArea;
    uint32_t workingHours;
    double stabilityMargin;
    double wheelSpeed;  // 添加wheelSpeed成员
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
    
    // 添加缺失的成员
    double stabilityIndex;      // 稳定性指数
    double tractionEfficiency;  // 牵引效率
};

struct PredictionResult {
    std::vector<double> loadForecast;  // 使用double替代float
    TractorVehicleState predictedState;
    double confidence;
    uint64_t timestamp;
};

struct SensorData {
    Vector3d gnssPosition;
    Vector3d gnssVelocity;      // 添加缺失的gnssVelocity成员
    Vector3d imuAcceleration;
    Vector3d imuAngularRate;
    double wheelSpeed;
    std::vector<double> wheelSpeeds;  // 添加缺失的wheelSpeeds成员（多个车轮）
    double engineRPM;
    double engineTorque;
    double fuelLevel;
    double batteryVoltage;
    double hydraulicPressure;
    double oilTemperature;
    double coolantTemperature;
    uint64_t timestamp;
};

// 添加缺失的EngineData结构体
struct EngineData {
    double rpm;
    double torque;
    double power;
    double fuelConsumption;
    double temperature;
    double oilPressure;
    double throttlePosition;
    bool isRunning;
    uint64_t timestamp;
};

// 添加缺失的EngineState结构体
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

// 添加缺失的MotorState结构体
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

// 添加缺失的EnergyState结构体
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
    double stateOfHealth;  // 使用stateOfHealth替代soh
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
    double energy;              // 添加缺失的energy成员
    double internalResistance;  // 添加缺失的internalResistance成员
    double soc;                 // 添加缺失的soc成员
    std::vector<CellModel> cells;  // 添加缺失的cells成员
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
    double motorTorque;         // 添加缺失的motorTorque成员
    double implementForce;      // 添加缺失的implementForce成员
    double wheelSlip;           // 添加缺失的wheelSlip成员
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

// 控制命令结构体 - 添加缺失的成员
struct ControlCommands {
    double torqueRequest;
    double steeringAngleRequest;
    double brakeRequest;
    double cvtRatioRequest;
    double ptoSpeedRequest;
    double hydraulicPressureRequest;
    
    // 添加缺失的成员
    double engineTorqueRequest;   // 发动机扭矩请求
    double motorTorqueRequest;    // 电机扭矩请求
    
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
    bool isHealthy;  // 添加isHealthy成员
    std::vector<std::string> activeWarnings;
    std::vector<std::string> activeFaults;
    uint64_t timestamp;
};

// 系统相关结构体
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

// 动力学相关结构体
struct DynamicsStatistics {
    double averageSpeed;
    double maxSpeed;
    double averageAcceleration;
    double maxAcceleration;
    double totalDistance;
    double fuelEfficiency;
    uint64_t operatingTime;
};

struct DynamicsFault {
    uint32_t faultCode;
    std::string description;
    double severity;
    uint64_t timestamp;
    bool isActive;
};

struct StabilityAssessment {
    double stabilityMargin;
    double rolloverRisk;
    double tractionLoss;
    double lateralAcceleration;
    double longitudinalAcceleration;
    bool isStable;
    uint64_t timestamp;
};

// 预测性能结构体
struct PredictionPerformance {
    double accuracy;
    double precision;
    double recall;
    double f1Score;
    double computationTime;
    uint64_t timestamp;
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

} // namespace VCUCore
