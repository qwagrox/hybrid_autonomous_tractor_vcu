// include/models/vehicle_dynamics_model.hpp - 修复重复定义问题
#pragma once
#include "vcu_core_types.hpp"
#include <Eigen/Dense>
#include <memory>

namespace VCUCore {

// 移除重复的结构体定义，使用vcu_core_types.hpp中的定义：
// - DynamicsStatistics
// - DynamicsFault  
// - StabilityAssessment

// 车辆参数结构体
struct VehicleParameters {
    float mass;
    float wheelbase;
    float trackWidth;
    float centerOfGravityHeight;
    float frontAxleLoad;
    float rearAxleLoad;
    float momentOfInertiaX;
    float momentOfInertiaY;
    float momentOfInertiaZ;
    float dragCoefficient;
    float frontalArea;
    float rollingResistance;
    float maxSteeringAngle;
    float maxBrakingForce;
    float maxTractionForce;
};

// 轮胎模型参数
struct TireParameters {
    float corneringStiffnessFront;
    float corneringStiffnessRear;
    float longitudinalStiffness;
    float peakFrictionCoefficient;
    float slidingFrictionCoefficient;
    float relaxationLength;
    float verticalStiffness;
    float dampingCoefficient;
};

class VehicleDynamicsModel {
private:
    // 状态向量和矩阵
    Eigen::VectorXd state_;
    Eigen::MatrixXd stateTransitionMatrix_;
    Eigen::MatrixXd inputMatrix_;
    Eigen::MatrixXd outputMatrix_;
    Eigen::MatrixXd processNoiseCovariance_;
    Eigen::MatrixXd measurementNoiseCovariance_;
    
    // 车辆参数
    VehicleParameters vehicleParams_;
    TireParameters tireParams_;
    
    // 历史数据
    std::deque<TractorVehicleState> stateHistory_;
    std::deque<ControlCommands> inputHistory_;
    uint32_t maxHistorySize_;
    
    // 自适应参数
    float adaptiveMass_;
    float adaptiveFriction_;
    bool isAdaptiveEnabled_;
    
    // 统计信息
    DynamicsStatistics statistics_;
    std::vector<DynamicsFault> activeFaults_;

public:
    VehicleDynamicsModel();
    ~VehicleDynamicsModel() = default;
    
    // 初始化
    bool initialize(const VehicleParameters& params);
    void reset();
    
    // 状态更新
    TractorVehicleState updateState(const ControlCommands& commands, 
                                  const SensorData& sensors, 
                                  float deltaTime);
    
    // 预测功能
    std::vector<TractorVehicleState> predictTrajectory(
        const std::vector<ControlCommands>& commandSequence,
        float deltaTime, uint32_t steps);
    
    TractorVehicleState predictNextState(const ControlCommands& commands, 
                                       float deltaTime) const;
    
    // 参数管理
    void setVehicleParameters(const VehicleParameters& params);
    void setTireParameters(const TireParameters& params);
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
                                                const Eigen::Vector3d& inertia) const;
    
    // 力计算
    Eigen::Vector3d calculateTractionForces(const ControlCommands& commands) const;
    Eigen::Vector3d calculateBrakingForces(const ControlCommands& commands) const;
    Eigen::Vector3d calculateAerodynamicForces(const TractorVehicleState& state) const;
    Eigen::Vector3d calculateRollingResistance(const TractorVehicleState& state) const;
    Eigen::Vector3d calculateGravityForces(const TractorVehicleState& state) const;
    
    // 轮胎模型
    float calculateLongitudinalForce(float slip, float normalForce) const;
    float calculateLateralForce(float slipAngle, float normalForce) const;
    float calculateNormalForce(const TractorVehicleState& state, int wheelIndex) const;
    
    // 稳定性分析
    float calculateRollStability(const TractorVehicleState& state) const;
    float calculateYawStability(const TractorVehicleState& state) const;
    float calculatePitchStability(const TractorVehicleState& state) const;
    
    // 故障检测
    void updateFaultDetection(const TractorVehicleState& state, 
                            const ControlCommands& commands);
    bool checkStabilityLimits(const TractorVehicleState& state) const;
    bool checkPerformanceLimits(const TractorVehicleState& state) const;
    
    // 统计更新
    void updateStatistics(const TractorVehicleState& state);
    
    // 学习算法
    void estimateVehicleMass();
    void estimateRoadFriction();
    void updateAdaptiveModel(const TractorVehicleState& measuredState);
};

} // namespace VCUCore
