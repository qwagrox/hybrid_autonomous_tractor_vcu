// include/models/vehicle_dynamics_model.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <deque>
#include <memory>

namespace VCUCore {

class VehicleDynamicsModel {
private:
    // 车辆参数
    struct VehicleParameters {
        // 质量参数
        float mass;                     // 总质量 (kg)
        float sprungMass;               // 簧载质量 (kg)
        float unsprungMass;             // 非簧载质量 (kg)
        float inertiaXX;                // 绕X轴转动惯量 (kg·m²)
        float inertiaYY;                // 绕Y轴转动惯量 (kg·m²) 
        float inertiaZZ;                // 绕Z轴转动惯量 (kg·m²)
        float cgHeight;                 // 重心高度 (m)
        float wheelbase;                // 轴距 (m)
        float trackWidth;               // 轮距 (m)
        
        // 轮胎参数
        float wheelRadius;              // 车轮半径 (m)
        float tireStiffness;            // 轮胎刚度 (N/m)
        float tireDamping;              // 轮胎阻尼 (N·s/m)
        float rollingResistance;        // 滚动阻力系数
        float maxTireForce;             // 最大轮胎力 (N)
        
        // 悬挂参数
        float springStiffness;          // 弹簧刚度 (N/m)
        float dampingCoefficient;       // 阻尼系数 (N·s/m)
        float suspensionTravel;         // 悬挂行程 (m)
        
        // 空气动力学
        float frontalArea;              // 迎风面积 (m²)
        float dragCoefficient;          // 风阻系数
        float liftCoefficient;          // 升力系数
        
        // 传动系统
        float finalDriveRatio;          // 主减速比
        float transmissionEfficiency;   // 传动效率
    };
    
    // 车辆状态
    struct TractorVehicleState {
        // 运动状态
        Eigen::Vector3d position;       // 位置 (m)
        Eigen::Vector3d velocity;       // 速度 (m/s)
        Eigen::Vector3d acceleration;   // 加速度 (m/s²)
        Eigen::Vector3d angularVelocity; // 角速度 (rad/s)
        
        // 姿态状态
        float roll;                     // 横滚角 (rad)
        float pitch;                    // 俯仰角 (rad)
        float yaw;                      // 偏航角 (rad)
        float rollRate;                 // 横滚角速度 (rad/s)
        float pitchRate;                // 俯仰角速度 (rad/s)
        float yawRate;                  // 偏航角速度 (rad/s)
        
        // 轮胎状态
        std::array<float, 4> wheelSlip;     // 车轮滑移率
        std::array<float, 4> wheelForce;    // 车轮力 (N)
        std::array<float, 4> wheelSpeed;    // 车轮速度 (rad/s)
        
        // 受力状态
        float totalForce;               // 总驱动力 (N)
        float rollingResistance;        // 滚动阻力 (N)
        float aerodynamicDrag;          // 空气阻力 (N)
        float gradeResistance;          // 坡度阻力 (N)
        float accelerationResistance;   // 加速阻力 (N)
        
        uint32_t timestamp;             // 时间戳
    };
    
    VehicleParameters params_;
    TractorVehicleState currentState_;
    TractorVehicleState previousState_;
    
    // 地形参数
    float currentSlope_;
    float currentFriction_;
    float roadRoughness_;
    
    // 模型数据
    Eigen::Matrix4f massMatrix_;
    Eigen::Matrix4f dampingMatrix_;
    Eigen::Matrix4f stiffnessMatrix_;
    
    // 历史数据
    std::deque<TractorVehicleState> stateHistory_;
    std::deque<Eigen::Vector3d> forceHistory_;
    std::deque<Eigen::Vector3d> momentHistory_;
    
    // 学习参数
    float adaptiveMass_;
    float adaptiveFriction_;
    float learningRate_;
    
    uint32_t maxHistorySize_;
    bool isInitialized_;

public:
    VehicleDynamicsModel(uint32_t historySize = 1000);
    
    // 初始化配置
    bool initialize(const std::string& configFile = "config/vehicle_params.yaml");
    bool loadParameters(const std::string& paramFile);
    void setTerrainParameters(float slope, float friction, float roughness);
    
    // 状态更新
    TractorVehicleState updateState(float engineTorque, float motorTorque, 
                           const Eigen::Vector3d& externalForces,
                           const Eigen::Vector3d& externalMoments,
                           float deltaTime, const PerceptionData& perception);
    
    // 力计算
    float calculateTotalTractiveForce(float engineTorque, float motorTorque, float gearRatio) const;
    float calculateRollingResistance(float normalForce) const;
    float calculateAerodynamicDrag(float velocity) const;
    float calculateGradeResistance(float slope) const;
    float calculateAccelerationResistance(float acceleration) const;
    
    // 轮胎模型
    float calculateTireForce(float slipRatio, float normalForce, float friction) const;
    float calculateSlipRatio(float wheelSpeed, float vehicleSpeed) const;
    float calculateNormalForce(int wheelIndex, float acceleration, float slope) const;
    
    // 稳定性计算
    float calculateStabilityFactor() const;
    float calculateRolloverRisk() const;
    float calculateWeightTransfer() const;
    bool checkStabilityLimits() const;
    
    // 预测功能
    TractorVehicleState predictState(float timeHorizon, const ControlCommands& commands) const;
    float predictStoppingDistance(float deceleration) const;
    float predictMaxSpeed(float slope, float headwind) const;
    float predictGradeability(float speed) const;
    
    // 性能计算
    float calculatePowerRequirements(const TractorVehicleState& state) const;
    float calculateEnergyConsumption(float power, float time, float efficiency) const;
    float calculateEnergyRecoveryPotential(float deceleration) const;
    
    // 获取状态
    TractorVehicleState getCurrentState() const;
    VehicleParameters getParameters() const;
    DynamicsStatistics getStatistics() const;
    
    // 诊断功能
    std::vector<DynamicsFault> detectDynamicsFaults() const;
    StabilityAssessment assessStability() const;
    
    // 校准功能
    void calibrateMass(float estimatedMass);
    void calibrateFriction(float estimatedFriction);
    void updateAdaptiveParameters(const PerceptionData& perception);

private:
    void initializeDefaultParameters();
    void initializeMatrices();
    void initializeState();
    
    // 动力学计算
    Eigen::Vector3d calculateAcceleration(const Eigen::Vector3d& forces, float mass) const;
    Eigen::Vector3d calculateAngularAcceleration(const Eigen::Vector3d& moments, 
                                               const Eigen::Matrix3f& inertia) const;
    void updateWheelDynamics(float deltaTime);
    void updateSuspensionDynamics(float deltaTime);
    
    // 数学模型
    Eigen::Matrix3f calculateInertiaTensor() const;
    Eigen::Matrix4f calculateTransformationMatrix(float roll, float pitch, float yaw) const;
    Eigen::Vector3d calculateCGPosition() const;
    
    // 轮胎模型
    float pacejkaTireModel(float slip, float normalForce, float friction) const;
    float linearTireModel(float slip, float normalForce, float friction) const;
    float dugoffTireModel(float slip, float normalForce, float friction) const;
    
    // 辅助函数
    bool validateInput(float torque, const Eigen::Vector3d& forces) const;
    void updateHistory(const TractorVehicleState& state);
    void applyConstraints(TractorVehicleState& state);
    void handleNumericalStability();
    
    // 学习算法
    void estimateVehicleMass();
    void estimateRoadFriction();
    void updateAdaptiveModel(const TractorVehicleState& measuredState);
};

// 动力学统计信息
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

// 动力学故障信息
struct DynamicsFault {
    uint16_t faultCode;
    FaultSeverity severity;
    std::string description;
    std::string component;
    uint32_t timestamp;
    float magnitude;
    bool isRecoverable;
};

// 稳定性评估
struct StabilityAssessment {
    float rollStability;
    float yawStability;
    float pitchStability;
    float overallStability;
    bool isStable;
    std::vector<std::string> stabilityIssues;
};

} // namespace VCUCore