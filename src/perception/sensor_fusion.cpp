// src/perception/sensor_fusion.cpp (更新版本)
#include "perception/sensor_fusion.hpp"
#include "utils/tractor_state_calculator.hpp"
#include <iostream>
#include <cmath>

namespace VCUCore {

SensorFusion::SensorFusion(size_t historySize, float innovationThreshold)
    : maxHistorySize_(historySize), innovationThreshold_(innovationThreshold) {
    initializeMatrices();
    reset();
}

void SensorFusion::initializeMatrices() {
    // 初始化状态转移矩阵
    H_.setZero();
    H_.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    H_.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();
    
    // 初始化测量噪声协方差
    R_.setIdentity();
    R_.block<3, 3>(0, 0) *= 0.1f;  // 位置噪声
    R_.block<3, 3>(3, 3) *= 0.5f;  // 速度噪声
    
    // 初始化过程噪声协方差
    Q_.setIdentity();
    Q_ *= 0.01f;
}

TractorVehicleState SensorFusion::fuseSensors(const SensorData& sensorData) {
    // 更新时间戳
    static Timestamp lastTime = sensorData.timestamp;
    float deltaTime = std::chrono::duration<float>(sensorData.timestamp - lastTime).count();
    lastTime = sensorData.timestamp;
    
    // 预测步骤
    predict(deltaTime);
    
    // 检查传感器健康状态
    if (checkSensorHealth(sensorData)) {
        // 更新步骤
        update(sensorData);
    }
    
    // 更新传感器历史
    sensorHistory_.push_back(sensorData);
    if (sensorHistory_.size() > maxHistorySize_) {
        sensorHistory_.pop_front();
    }
    
    return getCurrentState();
}

void SensorFusion::predict(float deltaTime) {
    if (deltaTime <= 0 || deltaTime > 1.0f) return;
    
    // 状态转移矩阵 F
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();
    
    // 位置 = 位置 + 速度 * dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * deltaTime;
    
    // 速度 = 速度 + 加速度 * dt
    F.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity() * deltaTime;
    
    // 预测状态
    x_ = F * x_;
    
    // 预测协方差
    P_ = F * P_ * F.transpose() + Q_;
}

void SensorFusion::update(const SensorData& data) {
    // 计算测量向量
    Eigen::VectorXf z = calculateMeasurementVector(data);
    
    // 计算预测测量
    Eigen::VectorXf h = H_ * x_;
    
    // 计算新息
    Eigen::VectorXf innovation = z - h;
    
    // 计算新息协方差
    Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
    
    // 验证新息
    if (!validateInnovation(innovation, S)) {
        return; // 跳过异常测量
    }
    
    // 计算卡尔曼增益
    Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();
    
    // 更新状态
    x_ = x_ + K * innovation;
    
    // 更新协方差
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> I = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity();
    P_ = (I - K * H_) * P_;
}

bool SensorFusion::validateInnovation(const Eigen::VectorXf& innovation, const Eigen::MatrixXf& innovationCovariance) {
    // 马氏距离检验
    float mahalanobisDistance = std::sqrt(innovation.transpose() * innovationCovariance.inverse() * innovation);
    return mahalanobisDistance < innovationThreshold_;
}

Eigen::VectorXf SensorFusion::calculateMeasurementVector(const SensorData& data) {
    Eigen::VectorXf z(6);
    z << data.gnssPosition.x(), data.gnssPosition.y(), data.gnssPosition.z(),
         data.gnssVelocity.x(), data.gnssVelocity.y(), data.gnssVelocity.z();
    return z;
}

TractorVehicleState SensorFusion::getCurrentState() const {
    TractorVehicleState state;
    
    // 基础运动状态
    state.position = Vector3d(x_(0), x_(1), x_(2));
    state.velocity = Vector3d(x_(3), x_(4), x_(5));
    state.acceleration = Vector3d(x_(6), x_(7), x_(8));
    state.heading = x_(9);
    state.pitch = x_(10);
    state.roll = x_(11);
    state.estimatedMass = x_(12);
    
    // 设置协方差矩阵
    state.estimationCovariance = P_.block<3, 3>(0, 0);
    
    // 设置时间戳
    state.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    return state;
}

PerceptionData SensorFusion::generatePerceptionData(const TractorVehicleState& state, const SensorData& sensorData) {
    PerceptionData perception;
    perception.tractorState = state;
    
    // 计算地形坡度 (基于加速度计和GPS)
    perception.terrainSlope = std::atan2(sensorData.imuAcceleration.x(), 
                                       std::sqrt(std::pow(sensorData.imuAcceleration.y(), 2) + 
                                               std::pow(sensorData.imuAcceleration.z(), 2)));
    
    // 计算土壤阻力 (基于牵引力和速度)
    float speed = state.velocity.norm();
    if (speed > 0.1f) {
        perception.soilResistance = state.drawbarPull * 0.7f; // 简化计算
    } else {
        perception.soilResistance = 0.0f;
    }
    
    // 计算滚动阻力
    perception.rollingResistance = state.estimatedMass * 9.81f * 0.08f; // 8%滚动阻力系数
    
    // 计算空气阻力
    float airDensity = 1.225f; // kg/m³
    float dragCoefficient = 0.8f; // 拖拉机阻力系数
    float frontalArea = 8.0f; // m² 拖拉机迎风面积
    perception.aerodynamicDrag = 0.5f * airDensity * dragCoefficient * frontalArea * speed * speed;
    
    // 计算负载系数
    float totalResistance = perception.soilResistance + perception.rollingResistance + perception.aerodynamicDrag;
    perception.loadFactor = totalResistance / (state.estimatedMass * 9.81f);
    
    // 负载变化检测
    static float lastLoadFactor = perception.loadFactor;
    float loadChange = perception.loadFactor - lastLoadFactor;
    
    if (std::abs(loadChange) > 0.1f) {
        if (loadChange > 0) {
            perception.loadChangeType = LoadChangeType::TORQUE_INCREASE;
        } else {
            perception.loadChangeType = LoadChangeType::TORQUE_DECREASE;
        }
    } else {
        perception.loadChangeType = LoadChangeType::NO_CHANGE;
    }
    
    lastLoadFactor = perception.loadFactor;
    
    // 负载趋势分析
    static std::deque<float> loadHistory;
    loadHistory.push_back(perception.loadFactor);
    if (loadHistory.size() > 10) {
        loadHistory.pop_front();
    }
    
    if (loadHistory.size() >= 5) {
        // 简单的趋势检测
        float trend = (loadHistory.back() - loadHistory.front()) / loadHistory.size();
        if (trend > 0.02f) {
            perception.loadTrend = LoadTrend::INCREASING;
        } else if (trend < -0.02f) {
            perception.loadTrend = LoadTrend::DECREASING;
        } else {
            perception.loadTrend = LoadTrend::STEADY;
        }
    } else {
        perception.loadTrend = LoadTrend::STEADY;
    }
    
    // 置信度评估
    float positionUncertainty = std::sqrt(state.estimationCovariance.trace());
    perception.confidence = std::max(0.0f, 1.0f - positionUncertainty / 10.0f);
    
    // 稳定性指数
    perception.stabilityIndex = TractorStateCalculator::calculateStabilityMargin(
        state.roll, state.pitch, state.centerOfGravityHeight);
    
    // 牵引效率
    if (speed > 0.1f && state.powerConsumption > 0) {
        perception.tractionEfficiency = TractorStateCalculator::calculateTractionEfficiency(
            state.drawbarPull, speed, state.powerConsumption);
    } else {
        perception.tractionEfficiency = 0.0f;
    }
    
    perception.timestamp = state.timestamp;
    
    return perception;
}

bool SensorFusion::checkSensorHealth(const SensorData& data) {
    // 检查GNSS数据有效性
    bool gnssValid = (data.gnssPosition.norm() > 0.1f && 
                     data.gnssPosition.norm() < 1e7f);
    
    // 检查IMU数据有效性
    bool imuValid = (data.imuAcceleration.norm() > 0.1f && 
                    data.imuAcceleration.norm() < 50.0f);
    
    // 检查速度数据合理性
    bool velocityValid = (data.gnssVelocity.norm() < 30.0f); // 最大30m/s
    
    // 检查轮速数据
    bool wheelSpeedValid = true;
    for (const auto& speed : data.wheelSpeeds) {
        if (speed < 0 || speed > 25.0f) { // 最大25m/s
            wheelSpeedValid = false;
            break;
        }
    }
    
    return gnssValid && imuValid && velocityValid && wheelSpeedValid;
}

void SensorFusion::reset() {
    x_.setZero();
    P_.setIdentity();
    P_ *= 100.0f; // 初始不确定性
    
    sensorHistory_.clear();
    sensorHealthStatus_.fill(true);
    innovationHistory_.fill(0.0f);
}

float SensorFusion::getEstimationUncertainty() const {
    return std::sqrt(P_.trace() / STATE_DIM);
}

} // namespace VCUCore
