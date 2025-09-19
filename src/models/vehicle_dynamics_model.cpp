// src/models/vehicle_dynamics_model.cpp
#include "vehicle_dynamics_model.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace VCUCore {

VehicleDynamicsModel::VehicleDynamicsModel(uint32_t historySize)
    : maxHistorySize_(historySize), isInitialized_(false),
      currentSlope_(0.0f), currentFriction_(0.8f), roadRoughness_(0.0f),
      adaptiveMass_(0.0f), adaptiveFriction_(0.8f), learningRate_(0.05f) {
    
    initializeDefaultParameters();
    initializeMatrices();
    initializeState();
}

void VehicleDynamicsModel::initializeDefaultParameters() {
    // 典型的农业拖拉机参数
    params_ = {
        // 质量参数
        .mass = 8000.0f,                // 8吨
        .sprungMass = 6000.0f,          // 6吨簧载质量
        .unsprungMass = 2000.0f,        // 2吨非簧载质量
        .inertiaXX = 4000.0f,           // 绕X轴转动惯量
        .inertiaYY = 12000.0f,          // 绕Y轴转动惯量
        .inertiaZZ = 10000.0f,          // 绕Z轴转动惯量
        .cgHeight = 1.2f,               // 重心高度1.2m
        .wheelbase = 3.5f,              // 轴距3.5m
        .trackWidth = 2.0f,             // 轮距2.0m
        
        // 轮胎参数
        .wheelRadius = 0.9f,            // 车轮半径0.9m
        .tireStiffness = 300000.0f,     // 轮胎刚度
        .tireDamping = 5000.0f,         // 轮胎阻尼
        .rollingResistance = 0.08f,     // 滚动阻力系数
        .maxTireForce = 20000.0f,       // 最大轮胎力20kN
        
        // 悬挂参数
        .springStiffness = 150000.0f,   // 弹簧刚度
        .dampingCoefficient = 15000.0f, // 阻尼系数
        .suspensionTravel = 0.3f,       // 悬挂行程0.3m
        
        // 空气动力学
        .frontalArea = 6.5f,            // 迎风面积6.5m²
        .dragCoefficient = 0.8f,        // 风阻系数
        .liftCoefficient = 0.2f,        // 升力系数
        
        // 传动系统
        .finalDriveRatio = 15.0f,       // 主减速比
        .transmissionEfficiency = 0.92f // 传动效率
    };
    
    adaptiveMass_ = params_.mass;
}

void VehicleDynamicsModel::initializeMatrices() {
    // 初始化质量矩阵
    massMatrix_.setZero();
    massMatrix_(0, 0) = params_.mass;           // X方向质量
    massMatrix_(1, 1) = params_.mass;           // Y方向质量
    massMatrix_(2, 2) = params_.mass;           // Z方向质量
    massMatrix_(3, 3) = params_.inertiaZZ;      // 绕Z轴转动惯量
    
    // 初始化阻尼矩阵（简化）
    dampingMatrix_.setIdentity();
    dampingMatrix_ *= 1000.0f;
    
    // 初始化刚度矩阵（简化）
    stiffnessMatrix_.setIdentity();
    stiffnessMatrix_ *= 50000.0f;
}

void VehicleDynamicsModel::initializeState() {
    currentState_ = {
        .position = Eigen::Vector3d::Zero(),
        .velocity = Eigen::Vector3d::Zero(),
        .acceleration = Eigen::Vector3d::Zero(),
        .angularVelocity = Eigen::Vector3d::Zero(),
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
        .rollRate = 0.0f,
        .pitchRate = 0.0f,
        .yawRate = 0.0f,
        .wheelSlip = {0.0f, 0.0f, 0.0f, 0.0f},
        .wheelForce = {0.0f, 0.0f, 0.0f, 0.0f},
        .wheelSpeed = {0.0f, 0.0f, 0.0f, 0.0f},
        .totalForce = 0.0f,
        .rollingResistance = 0.0f,
        .aerodynamicDrag = 0.0f,
        .gradeResistance = 0.0f,
        .accelerationResistance = 0.0f,
        .timestamp = 0
    };
    
    previousState_ = currentState_;
}

bool VehicleDynamicsModel::initialize(const std::string& configFile) {
    try {
        // 加载配置文件
        if (!loadParameters(configFile)) {
            std::cerr << "Using default vehicle parameters" << std::endl;
        }
        
        isInitialized_ = true;
        std::cout << "Vehicle dynamics model initialized" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Vehicle dynamics model initialization failed: " << e.what() << std::endl;
        return false;
    }
}

TractorVehicleState VehicleDynamicsModel::updateState(float engineTorque, float motorTorque,
                                             const Eigen::Vector3d& externalForces,
                                             const Eigen::Vector3d& externalMoments,
                                             float deltaTime, const PerceptionData& perception) {
    
    if (!validateInput(engineTorque, externalForces)) {
        std::cerr << "Invalid input parameters" << std::endl;
        return currentState_;
    }
    
    try {
        // 保存先前状态
        previousState_ = currentState_;
        
        // 更新地形参数
        currentSlope_ = perception.terrainSlope;
        currentFriction_ = perception.soilResistance / 5000.0f; // 简化摩擦系数估计
        roadRoughness_ = perception.loadFactor * 0.1f;
        
        // 计算总驱动力
        float totalTorque = engineTorque + motorTorque;
        float gearRatio = perception.vehicleState.cvtRatio * params_.finalDriveRatio;
        currentState_.totalForce = calculateTotalTractiveForce(totalTorque, 0.0f, gearRatio);
        
        // 计算各种阻力
        float velocity = currentState_.velocity.norm();
        currentState_.rollingResistance = calculateRollingResistance(adaptiveMass_ * 9.81f);
        currentState_.aerodynamicDrag = calculateAerodynamicDrag(velocity);
        currentState_.gradeResistance = calculateGradeResistance(currentSlope_);
        
        // 计算净力
        float netForce = currentState_.totalForce - 
                        currentState_.rollingResistance - 
                        currentState_.aerodynamicDrag - 
                        currentState_.gradeResistance;
        
        // 计算加速度
        Eigen::Vector3d direction = currentState_.velocity.normalized();
        if (direction.norm() < 0.1f) {
            direction = Eigen::Vector3d(1.0f, 0.0f, 0.0f); // 默认前进方向
        }
        
        Eigen::Vector3d acceleration = calculateAcceleration(
            netForce * direction + externalForces, adaptiveMass_);
        
        // 计算角加速度
        Eigen::Vector3d angularAcceleration = calculateAngularAcceleration(
            externalMoments, calculateInertiaTensor());
        
        // 更新运动状态（欧拉积分）
        currentState_.velocity += acceleration * deltaTime;
        currentState_.position += currentState_.velocity * deltaTime;
        currentState_.acceleration = acceleration;
        
        // 更新角运动状态
        currentState_.angularVelocity += angularAcceleration * deltaTime;
        currentState_.rollRate = currentState_.angularVelocity.x();
        currentState_.pitchRate = currentState_.angularVelocity.y();
        currentState_.yawRate = currentState_.angularVelocity.z();
        
        currentState_.roll += currentState_.rollRate * deltaTime;
        currentState_.pitch += currentState_.pitchRate * deltaTime;
        currentState_.yaw += currentState_.yawRate * deltaTime;
        
        // 更新车轮动力学
        updateWheelDynamics(deltaTime);
        
        // 更新悬挂动力学
        updateSuspensionDynamics(deltaTime);
        
        // 应用物理约束
        applyConstraints(currentState_);
        
        // 更新时间戳
        currentState_.timestamp = getCurrentTime();
        
        // 更新历史记录
        updateHistory(currentState_);
        
        // 自适应学习
        updateAdaptiveParameters(perception);
        
        return currentState_;
        
    } catch (const std::exception& e) {
        std::cerr << "Vehicle state update error: " << e.what() << std::endl;
        return previousState_;
    }
}

float VehicleDynamicsModel::calculateTotalTractiveForce(float engineTorque, float motorTorque, 
                                                      float gearRatio) const {
    float totalTorque = engineTorque + motorTorque;
    float wheelTorque = totalTorque * gearRatio * params_.transmissionEfficiency;
    return wheelTorque / params_.wheelRadius;
}

float VehicleDynamicsModel::calculateRollingResistance(float normalForce) const {
    return normalForce * params_.rollingResistance * std::cos(currentSlope_);
}

float VehicleDynamicsModel::calculateAerodynamicDrag(float velocity) const {
    const float airDensity = 1.225f; // 空气密度 (kg/m³)
    return 0.5f * airDensity * velocity * velocity * 
           params_.frontalArea * params_.dragCoefficient;
}

float VehicleDynamicsModel::calculateGradeResistance(float slope) const {
    return adaptiveMass_ * 9.81f * std::sin(slope);
}

Eigen::Vector3d VehicleDynamicsModel::calculateAcceleration(const Eigen::Vector3d& forces, 
                                                          float mass) const {
    if (mass < 1.0f) mass = 1.0f; // 避免除零
    return forces / mass;
}

Eigen::Vector3d VehicleDynamicsModel::calculateAngularAcceleration(const Eigen::Vector3d& moments,
                                                                 const Eigen::Matrix3f& inertia) const {
    return inertia.inverse() * moments.cast<float>();
}

Eigen::Matrix3f VehicleDynamicsModel::calculateInertiaTensor() const {
    Eigen::Matrix3f inertia;
    inertia << params_.inertiaXX, 0.0f, 0.0f,
               0.0f, params_.inertiaYY, 0.0f,
               0.0f, 0.0f, params_.inertiaZZ;
    return inertia;
}

void VehicleDynamicsModel::updateWheelDynamics(float deltaTime) {
    float vehicleSpeed = currentState_.velocity.norm();
    
    for (int i = 0; i < 4; ++i) {
        // 计算车轮滑移率
        currentState_.wheelSlip[i] = calculateSlipRatio(
            currentState_.wheelSpeed[i], vehicleSpeed);
        
        // 计算法向力（考虑重量转移）
        float normalForce = calculateNormalForce(i, 
            currentState_.acceleration.norm(), currentSlope_);
        
        // 计算轮胎力
        currentState_.wheelForce[i] = calculateTireForce(
            currentState_.wheelSlip[i], normalForce, currentFriction_);
        
        // 更新车轮速度（简化）
        float wheelAcceleration = currentState_.wheelForce[i] / 
                                 (params_.wheelRadius * params_.unsprungMass / 4.0f);
        currentState_.wheelSpeed[i] += wheelAcceleration * deltaTime;
    }
}

float VehicleDynamicsModel::calculateTireForce(float slipRatio, float normalForce, 
                                             float friction) const {
    // Pacejka 魔术公式（简化版本）
    float B = 10.0f;  // 刚度因子
    float C = 1.9f;   // 形状因子
    float D = friction * normalForce; // 峰值因子
    float E = 0.97f;  // 曲率因子
    
    return D * std::sin(C * std::atan(B * slipRatio - E * (B * slipRatio - std::atan(B * slipRatio))));
}

float VehicleDynamicsModel::calculateSlipRatio(float wheelSpeed, float vehicleSpeed) const {
    if (vehicleSpeed < 0.1f) return 0.0f;
    
    float wheelLinearSpeed = wheelSpeed * params_.wheelRadius;
    return (wheelLinearSpeed - vehicleSpeed) / std::max(vehicleSpeed, 0.1f);
}

float VehicleDynamicsModel::calculateNormalForce(int wheelIndex, float acceleration, 
                                               float slope) const {
    // 静态重量分布
    float staticWeight = adaptiveMass_ * 9.81f / 4.0f;
    
    // 坡度影响
    float gradeEffect = staticWeight * std::sin(slope);
    
    // 加速度影响（重量转移）
    float weightTransfer = 0.0f;
    if (acceleration > 0) {
        // 加速时后轴加载，前轴减载
        weightTransfer = acceleration * params_.cgHeight / params_.wheelbase;
        if (wheelIndex < 2) { // 前轮
            weightTransfer = -weightTransfer;
        }
    }
    
    return staticWeight + gradeEffect + weightTransfer;
}

float VehicleDynamicsModel::calculateStabilityFactor() const {
    // 稳定性因子计算
    float rollStability = 1.0f - std::abs(currentState_.roll) / 0.3f; // 横滚角稳定性
    float yawStability = 1.0f - std::abs(currentState_.yawRate) / 1.0f; // 横摆稳定性
    float slipStability = 1.0f - (*std::max_element(currentState_.wheelSlip.begin(), 
                                                  currentState_.wheelSlip.end())) / 0.3f;
    
    return (rollStability + yawStability + slipStability) / 3.0f;
}

float VehicleDynamicsModel::calculateRolloverRisk() const {
    // 侧翻风险计算
    float lateralAcceleration = currentState_.acceleration.y();
    float criticalAcceleration = (params_.trackWidth / 2.0f) / params_.cgHeight * 9.81f;
    
    return std::abs(lateralAcceleration) / criticalAcceleration;
}

TractorVehicleState VehicleDynamicsModel::predictState(float timeHorizon, 
                                              const ControlCommands& commands) const {
    TractorVehicleState predictedState = currentState_;
    
    // 简化的预测模型
    float steps = timeHorizon / 0.1f; // 0.1秒时间步长
    for (int i = 0; i < steps; ++i) {
        // 基于当前命令预测状态
        float predictedAcceleration = commands.engineTorqueRequest / adaptiveMass_;
        predictedState.velocity.x() += predictedAcceleration * 0.1f;
        predictedState.position.x() += predictedState.velocity.x() * 0.1f;
    }
    
    return predictedState;
}

void VehicleDynamicsModel::updateAdaptiveParameters(const PerceptionData& perception) {
    // 基于感知数据更新自适应参数
    if (perception.vehicleState.estimatedMass > 0) {
        adaptiveMass_ = 0.9f * adaptiveMass_ + 0.1f * perception.vehicleState.estimatedMass;
    }
    
    // 基于车轮滑移估计路面摩擦
    float maxSlip = *std::max_element(currentState_.wheelSlip.begin(), 
                                    currentState_.wheelSlip.end());
    if (maxSlip > 0.1f) {
        adaptiveFriction_ = 0.95f * adaptiveFriction_ + 0.05f * (currentFriction_ / maxSlip);
    }
}

bool VehicleDynamicsModel::validateInput(float torque, const Eigen::Vector3d& forces) const {
    if (std::isnan(torque) || std::abs(torque) > 10000.0f) {
        return false;
    }
    
    if (forces.hasNaN() || forces.norm() > 100000.0f) {
        return false;
    }
    
    return true;
}

void VehicleDynamicsModel::applyConstraints(TractorVehicleState& state) {
    // 速度约束
    float maxSpeed = 40.0f / 3.6f; // 40 km/h -> m/s
    if (state.velocity.norm() > maxSpeed) {
        state.velocity = state.velocity.normalized() * maxSpeed;
    }
    
    // 角度约束
    state.roll = std::clamp(state.roll, -0.5f, 0.5f);  // ±30度
    state.pitch = std::clamp(state.pitch, -0.3f, 0.3f); // ±20度
    
    // 车轮力约束
    for (auto& force : state.wheelForce) {
        force = std::clamp(force, -params_.maxTireForce, params_.maxTireForce);
    }
}

uint32_t VehicleDynamicsModel::getCurrentTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

TractorVehicleState VehicleDynamicsModel::getCurrentState() const {
    return currentState_;
}

} // namespace VCUCore