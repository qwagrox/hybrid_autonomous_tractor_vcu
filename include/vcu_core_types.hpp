// include/vcu_core_types.hpp
#pragma once
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>

namespace VCUCore {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Timestamp = std::chrono::nanoseconds;

// 系统状态枚举
enum class SystemState : uint8_t {
    OFF, STANDBY, READY, RUNNING, 
    DEGRADED, FAULT, EMERGENCY, CALIBRATION
};

// 驱动模式枚举
enum class DriveMode : uint8_t {
    ECO, COMFORT, SPORT, 
    PLOWING, SEEDING, TRANSPORT,
    ROAD, FIELD, MANUAL,
    LOADING, UNLOADING, PARKING
};

// CVT制造商枚举
enum class CVTManufacturer {
    UNKNOWN, JOHN_DEERE, CASE_IH, CLAAS, 
    AGCO_FENDT, AGCO_MASSEY_FERGUSON, 
    NEW_HOLLAND, KUBOTA, CNH, DEUTZ_FAHR, SAME,
    VALTRA, ZETOR, YANMAR
};

// 负载变化类型
enum class LoadChangeType {
    NO_CHANGE, TORQUE_INCREASE, TORQUE_DECREASE,
    LOAD_SPIKE, LOAD_DROP, ENGINE_DERATE, OVERLOAD,
    WHEEL_SLIP, IMPLEMENT_BLOCKAGE, TERRAIN_CHANGE,
    SOIL_VARIATION, IMPLEMENT_ENGAGEMENT, IMPLEMENT_DISENGAGEMENT
};

// 负载趋势
enum class LoadTrend {
    STEADY, INCREASING, DECREASING, 
    OSCILLATING, STEP_CHANGE, RANDOM,
    CYCLIC, EXPONENTIAL, LOGARITHMIC
};

// 故障严重等级
enum class FaultSeverity : uint8_t {
    NONE, INFO, WARNING, ERROR, CRITICAL, FATAL
};

// 控制模式
enum class ControlMode : uint8_t {
    AUTO, MANUAL, SEMI_AUTO, DEGRADED, SAFETY
};

// 发动机数据结构
struct EngineData {
    float actualTorque;          // 实际扭矩 (Nm)
    float percentLoad;           // 负载百分比 (%)
    float speed;                 // 发动机转速 (rpm)
    float fuelRate;              // 燃油消耗率 (L/h)
    float boostPressure;         // 增压压力 (kPa)
    float temperature;           // 发动机温度 (°C)
    float oilPressure;           // 机油压力 (kPa)
    uint8_t derateStatus;        // 降功率状态
    uint16_t errorCodes;         // 错误代码
    uint32_t timestamp;          // 时间戳
    uint8_t operatingHours;      // 运行小时
    float efficiency;            // 效率 (%)
};

// 电机数据结构
struct MotorData {
    float actualTorque;          // 实际扭矩 (Nm)
    float speed;                 // 电机转速 (rpm)
    float power;                 // 功率 (kW)
    float voltage;               // 电压 (V)
    float current;               // 电流 (A)
    float temperature;           // 温度 (°C)
    float efficiency;            // 效率 (%)
    uint16_t errorCodes;         // 错误代码
    uint32_t timestamp;          // 时间戳
};

// 电池数据结构
struct BatteryData {
    float voltage;               // 电压 (V)
    float current;               // 电流 (A)
    float stateOfCharge;         // SOC (%)
    float stateOfHealth;         // SOH (%)
    float temperature;           // 温度 (°C)
    float power;                 // 功率 (kW)
    float capacity;              // 容量 (Ah)
    uint16_t errorCodes;         // 错误代码
    uint32_t timestamp;          // 时间戳
    uint8_t cycleCount;          // 循环次数
};

// CVT数据结构
struct CVTData {
    float actualRatio;           // 实际传动比
    float targetRatio;           // 目标传动比
    float inputSpeed;            // 输入转速 (rpm)
    float outputSpeed;           // 输出转速 (rpm)
    float hydraulicPressure;     // 液压压力 (bar)
    float oilTemperature;        // 油温 (°C)
    float efficiency;            // 效率 (%)
    float torqueTransmission;    // 传递扭矩 (Nm)
    uint16_t errorCodes;         // 错误代码
    uint32_t timestamp;          // 时间戳
    uint8_t statusFlags;         // 状态标志
};

// 农具数据结构
struct ImplementData {
    float depth;                 // 作业深度 (m)
    float draftForce;            // 牵引力 (N)
    float hydraulicPressure;     // 液压压力 (bar)
    float angle;                 // 角度 (°)
    float width;                 // 工作宽度 (m)
    uint16_t errorCodes;         // 错误代码
    uint32_t timestamp;          // 时间戳
    uint8_t implementType;       // 农具类型
    uint8_t workState;           // 工作状态
};

// 传感器数据结构
struct SensorData {
    Timestamp timestamp;
    Vector3d gnssPosition;          // WGS84坐标 (deg)
    Vector3d gnssVelocity;          // 速度向量 (m/s)
    Vector3d imuAcceleration;       // 加速度 (m/s²)
    Vector3d imuAngularRate;        // 角速度 (rad/s)
    Vector3d imuOrientation;        // 姿态角 (rad)
    std::array<float, 4> wheelSpeeds; // 轮速 (m/s)
    float engineRpm;                // 发动机转速 (rpm)
    float motorRpm;                 // 电机转速 (rpm)
    float batteryVoltage;           // 电池电压 (V)
    float batteryCurrent;           // 电池电流 (A)
    float batterySOC;               // 电量状态 (%)
    float batteryTemperature;       // 电池温度 (°C)
    float engineTemperature;        // 发动机温度 (°C)
    float motorTemperature;         // 电机温度 (°C)
    float hydraulicPressure;        // 液压压力 (bar)
    float implementDepth;           // 农具深度 (m)
    float implementForce;           // 农具力 (N)
    float soilMoisture;             // 土壤湿度 (%)
    float soilCompaction;           // 土壤压实度 (MPa)
    float fuelRate;                 // 燃油消耗率 (L/h)
    float ambientTemperature;       // 环境温度 (°C)
    float ambientHumidity;          // 环境湿度 (%)
    float windSpeed;                // 风速 (m/s)
};

// 拖拉机车辆状态结构 (专为农业拖拉机优化)
struct TractorVehicleState {
    // === 基础运动状态 ===
    Vector3d position;              // GNSS位置 (WGS84坐标)
    Vector3d velocity;              // 速度向量 (m/s)
    Vector3d acceleration;          // 加速度向量 (m/s²)
    
    // === 姿态信息 (对拖拉机稳定性至关重要) ===
    float heading;                  // 航向角 (rad)
    float pitch;                    // 俯仰角 (rad) - 影响农具深度控制
    float roll;                     // 横滚角 (rad) - 防侧翻关键参数
    float gradeAngle;               // 坡度角 (rad) - 爬坡能力评估
    
    // === 拖拉机核心牵引参数 ===
    float drawbarPull;              // 牵引力 (N) - 拖拉机最重要性能指标
    float drawbarPower;             // 牵引功率 (kW) - 有效功率输出
    float wheelSlipRatio;           // 轮滑率 - 牵引效率指标
    float tractionEfficiency;       // 牵引效率 (%) - 轮胎-土壤相互作用效率
    
    // === 质量和载荷分布 ===
    float estimatedMass;            // 总质量 (kg) - 包含农具和货物
    float frontAxleLoad;            // 前桥载荷 (N) - 影响转向和稳定性
    float rearAxleLoad;             // 后桥载荷 (N) - 影响牵引力
    float ballastMass;              // 配重质量 (kg) - 优化载荷分配
    
    // === 动力系统状态 ===
    float actualTorque;             // 实际输出扭矩 (Nm)
    float demandedTorque;           // 需求扭矩 (Nm)
    float engineLoad;               // 发动机负载率 (%)
    float pto_rpm;                  // PTO转速 (rpm) - 农具动力输出
    float pto_torque;               // PTO扭矩 (Nm) - 农具驱动扭矩
    
    // === 能耗和效率 (农业作业关键指标) ===
    float powerConsumption;         // 总功率消耗 (kW)
    float fuelConsumption;          // 燃油消耗率 (L/h)
    float energyEfficiency;         // 混合动力系统效率 (%)
    float specificFuelConsumption;  // 比油耗 (L/ha) - 农业作业效率指标
    
    // === 田间作业状态 ===
    float workingWidth;             // 作业幅宽 (m) - 单次通过作业宽度
    float workingDepth;             // 作业深度 (m) - 犁耕、播种深度
    float workingSpeed;             // 作业速度 (km/h) - 田间作业速度
    float fieldEfficiency;          // 田间效率 (%) - 有效作业时间比例
    float workedArea;               // 已作业面积 (ha) - 累计作业面积
    uint32_t workingHours;          // 累计作业时间 (h) - 发动机工作小时
    
    // === 稳定性和安全 (拖拉机安全关键) ===
    float stabilityMargin;          // 稳定性裕度 - 防侧翻安全系数
    float centerOfGravityHeight;    // 重心高度 (m) - 影响稳定性
    float turningRadius;            // 最小转弯半径 (m) - 机动性指标
    bool rolloverRisk;              // 侧翻风险警告 - 安全预警
    float groundClearance;          // 离地间隙 (m) - 通过性指标
    
    // === 液压系统 (农具控制核心) ===
    float hydraulicPressure;        // 主液压压力 (bar)
    float hydraulicFlowRate;        // 液压流量 (L/min)
    float hydraulicOilTemperature;  // 液压油温度 (°C)
    float hitchHeight;              // 三点悬挂高度 (m)
    
    // === 土壤和环境交互 ===
    float soilCompaction;           // 土壤压实度 (MPa) - 环境影响评估
    float soilMoisture;             // 土壤湿度 (%) - 作业条件评估
    float wheelLoadDistribution[4]; // 四轮载荷分配 (N) - [前左,前右,后左,后右]
    
    // === 系统状态和控制 ===
    Matrix3d estimationCovariance;  // 状态估计协方差矩阵
    SystemState systemState;        // 系统运行状态
    DriveMode driveMode;            // 驱动模式 (ECO/PLOWING/SEEDING等)
    ControlMode controlMode;        // 控制模式 (AUTO/MANUAL等)
    uint32_t timestamp;             // 时间戳 (ms)
    
    // === 作业模式标志 ===
    bool isWorking;                 // 是否在田间作业
    bool isTransporting;            // 是否在运输模式
    bool isTurning;                 // 是否在地头转弯
    bool isPTOEngaged;              // PTO是否接合
    bool isHydraulicActive;         // 液压系统是否激活
};

// 控制命令结构
struct ControlCommands {
    float engineTorqueRequest;      // 发动机扭矩请求 (Nm)
    float motorTorqueRequest;       // 电机扭矩请求 (Nm)
    int transmissionGearRequest;    // 变速箱档位请求
    float hydraulicPressureRequest; // 液压压力请求 (bar)
    bool implementLiftRequest;      // 农具提升请求
    float cvtRatioRequest;          // CVT传动比请求
    bool emergencyStop;             // 紧急停止
    uint8_t controlMode;            // 控制模式
    float maxTorqueLimit;           // 最大扭矩限制 (Nm)
    float minTorqueLimit;           // 最小扭矩限制 (Nm)
    float torqueChangeRate;         // 扭矩变化率 (Nm/s)
    float ratioChangeRate;          // 传动比变化率 (1/s)
    uint32_t timestamp;             // 时间戳
};

// 感知数据结构
struct PerceptionData {
    TractorVehicleState tractorState;  // 拖拉机状态
    float terrainSlope;             // 地形坡度 (rad)
    float soilResistance;           // 土壤阻力 (N)
    float rollingResistance;        // 滚动阻力 (N)
    float aerodynamicDrag;          // 空气阻力 (N)
    float loadFactor;               // 负载系数
    LoadChangeType loadChangeType;  // 负载变化类型
    LoadTrend loadTrend;            // 负载趋势
    float confidence;               // 置信度
    float stabilityIndex;           // 稳定性指数
    float tractionEfficiency;       // 牵引效率 (%)
    uint32_t timestamp;             // 时间戳
};

// 预测结果结构
struct PredictionResult {
    std::vector<float> loadForecast;    // 负载预测 (N)
    std::vector<float> energyDemand;    // 能量需求预测 (kWh)
    std::vector<Vector3d> pathProfile;  // 路径剖面
    std::vector<float> slopeProfile;    // 坡度剖面 (rad)
    std::vector<float> resistanceProfile; // 阻力剖面 (N)
    float predictedEfficiency;          // 预测效率 (%)
    float estimatedFuelConsumption;     // 估计燃油消耗 (L)
    float estimatedEnergyConsumption;   // 估计能量消耗 (kWh)
    float predictionHorizon;            // 预测时域 (s)
    float predictionConfidence;         // 预测置信度
    uint32_t timestamp;                 // 时间戳
};

// CVT控制参数
struct CVTControlParams {
    float minRatio;                 // 最小传动比
    float maxRatio;                 // 最大传动比
    float ratioChangeRate;          // 传动比变化率 (1/s)
    float torqueReserve;            // 扭矩储备 (%)
    float slipRatioTarget;          // 目标滑转率
    float efficiencyWeight;         // 效率权重
    float comfortWeight;            // 舒适性权重
    float responseWeight;           // 响应性权重
};

// 能量管理参数
struct EnergyManagementParams {
    float batterySOCMin;            // 电池SOC最小值 (%)
    float batterySOCMax;            // 电池SOC最大值 (%)
    float engineEfficiencyWeight;   // 发动机效率权重
    float motorEfficiencyWeight;    // 电机效率权重
    float batteryHealthWeight;      // 电池健康权重
    float fuelCostWeight;           // 燃油成本权重
    float electricityCostWeight;    // 电力成本权重
    float predictionHorizon;        // 预测时域 (s)
};

// 故障诊断结果
struct FaultDiagnosis {
    uint16_t faultCode;             // 故障代码
    FaultSeverity severity;         // 严重等级
    std::string description;        // 故障描述
    std::string component;          // 故障组件
    uint32_t timestamp;             // 时间戳
    uint32_t duration;              // 持续时间 (ms)
    bool isActive;                  // 是否活跃
    bool isRecoverable;             // 是否可恢复
    std::vector<std::string> recoverySteps; // 恢复步骤
};

// 系统健康状态
struct SystemHealthStatus {
    bool isHealthy;                 // 是否健康
    float overallHealth;            // 整体健康度 (%)
    std::vector<FaultDiagnosis> activeFaults; // 活跃故障
    std::vector<FaultDiagnosis> historicalFaults; // 历史故障
    uint32_t uptime;                // 运行时间 (s)
    uint32_t lastMaintenance;       // 上次维护时间 (s)
    uint32_t nextMaintenance;       // 下次维护时间 (s)
};

// 负载变化检测结果
struct LoadChangeResult {
    bool changeDetected;            // 是否检测到变化
    LoadChangeType changeType;      // 变化类型
    float changeMagnitude;          // 变化幅度
    float changeRate;               // 变化速率
    float confidence;               // 检测置信度
    uint32_t detectionTime;         // 检测时间戳
    std::vector<float> changeSignature; // 变化特征
};

// 配置参数结构
struct SystemConfig {
    // 系统参数
    std::string systemName;
    std::string systemVersion;
    uint32_t controlRate;           // 控制频率 (Hz)
    bool simulationMode;            // 仿真模式
    std::string logLevel;           // 日志级别
    std::string dataLogDirectory;   // 数据日志目录
    uint32_t maxLogSizeMB;          // 最大日志大小 (MB)
    uint32_t logRetentionDays;      // 日志保留天数

    // 车辆参数
    float vehicleMass;              // 车辆质量 (kg)
    float wheelRadius;              // 车轮半径 (m)
    float rollingResistanceCoeff;   // 滚动阻力系数
    float frontalArea;              // 迎风面积 (m²)
    float dragCoefficient;          // 风阻系数
    float maxSpeed;                 // 最大速度 (km/h)

    // 动力总成参数
    float engineMaxTorque;          // 发动机最大扭矩 (Nm)
    float engineMaxPower;           // 发动机最大功率 (kW)
    float engineIdleRpm;            // 发动机怠速 (rpm)
    float engineMaxRpm;             // 发动机最高转速 (rpm)
    float motorMaxTorque;           // 电机最大扭矩 (Nm)
    float motorMaxPower;            // 电机最大功率 (kW)
    float batteryCapacity;          // 电池容量 (kWh)
    float batteryNominalVoltage;    // 电池标称电压 (V)
    float batteryMaxChargePower;    // 电池最大充电功率 (kW)
    float batteryMaxDischargePower; // 电池最大放电功率 (kW)

    // 控制参数
    CVTControlParams cvtParams;     // CVT控制参数
    EnergyManagementParams energyParams; // 能量管理参数

    // 诊断参数
    uint32_t diagnosticUpdateRate;  // 诊断更新频率 (Hz)
    float overTemperatureThreshold; // 过热阈值 (°C)
    float overVoltageThreshold;     // 过压阈值 (V)
    float overCurrentThreshold;     // 过流阈值 (A)
    float wheelSlipThreshold;       // 轮滑阈值
};

// 性能统计结构
struct PerformanceStatistics {
    uint64_t totalCycles;           // 总控制周期数
    uint64_t missedDeadlines;       // 错过截止时间次数
    float averageCycleTime;         // 平均周期时间 (ms)
    float maxCycleTime;             // 最大周期时间 (ms)
    float minCycleTime;             // 最小周期时间 (ms)
    float cpuUsage;                 // CPU使用率 (%)
    float memoryUsage;              // 内存使用率 (%)
    float communicationLatency;     // 通信延迟 (ms)
    float controlAccuracy;          // 控制精度 (%)
    float energyEfficiency;         // 能量效率 (%)
    uint32_t timestamp;             // 时间戳
};

} // namespace VCUCore

// ====================================================================
// 农具控制相关类型 (Implement Control Types)
// ====================================================================

#include <string>
#include <vector>
#include <map>

/**
 * @enum ImplementState
 * @brief 定义农具的生命周期状态
 */
enum class ImplementState {
    UNKNOWN,       // 未知状态
    IDLE,          // 空闲状态
    CONFIGURED,    // 已配置
    ACTIVE,        // 作业中
    TRANSPORT,     // 运输模式
    FAULT,         // 故障状态
    EMERGENCY_STOP // 紧急停止
};

/**
 * @struct ImplementConfig
 * @brief 存储单个农具的配置参数
 */
struct ImplementConfig {
    std::string type;                     // 农具类型 (e.g., "Plow")
    std::string name;                     // 农具名称 (e.g., "John Deere 2720")
    double work_width;                    // 工作宽度 (米)
    std::map<std::string, double> params; // 其他特定参数 (e.g., {"max_depth", 0.4})
};

/**
 * @struct ImplementStatus
 * @brief 存储农具的实时状态
 */
struct ImplementStatus {
    ImplementState state = ImplementState::UNKNOWN;
    bool is_connected = false;
    double current_depth = 0.0;           // 当前深度 (米)
    double current_rate = 0.0;            // 当前速率 (e.g., kg/ha or L/min)
    std::vector<std::string> errors;      // 当前错误信息
};

/**
 * @struct DiagnosticReport
 * @brief 存储诊断测试的结果
 */
struct DiagnosticReport {
    bool passed = true;
    std::vector<std::string> findings; // 诊断发现
};

