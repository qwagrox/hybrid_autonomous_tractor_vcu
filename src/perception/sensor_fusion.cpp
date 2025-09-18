// src/perception/sensor_fusion.cpp
#include "sensor_fusion.hpp"
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

VehicleState SensorFusion::fuseSensors(const SensorData& sensorData) {
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
    updateSensorHistory(sensorData);
    
    return getCurrentState();
}

void SensorFusion::predict(float deltaTime) {
    // 状态转移矩阵
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F;
    F.setIdentity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * deltaTime;
    F.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity() * deltaTime;
    
    // 过程噪声协方差
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q_process;
    Q_process.setZero();
    Q_process.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 0.1f * deltaTime;
    
    // 状态预测
    x_ = F * x_;
    
    // 协方差预测
    P_ = F * P_ * F.transpose() + Q_process + Q_;
}

void SensorFusion::update(const SensorData& data) {
    Eigen::VectorXf z = calculateMeasurementVector(data);
    Eigen::VectorXf innovation = z - H_ * x_;
    Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
    
    // 创新序列验证
    if (!validateInnovation(innovation, S)) {
        return;
    }
    
    // 卡尔曼增益
    Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();
    
    // 状态更新
    x_ += K * innovation;
    
    // 协方差更新
    P_ = (StateCovariance::Identity() - K * H_) * P_;
}

bool SensorFusion::checkSensorHealth(const SensorData& data) {
    Eigen::VectorXf z = calculateMeasurementVector(data);
    Eigen::VectorXf innovation = z - H_ * x_;
    Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
    
    // 计算马氏距离
    float mahalanobisDist = innovation.transpose() * S.inverse() * innovation;
    
    // 检查各个传感器的创新序列
    for (int i = 0; i < 6; ++i) {
        float normalizedInnovation = std::abs(innovation(i)) / std::sqrt(S(i, i));
        innovationHistory_[i] = 0.9f * innovationHistory_[i] + 0.1f * normalizedInnovation;
        sensorHealthStatus_[i] = innovationHistory_[i] < innovationThreshold_;
    }
    
    // 至少需要4个健康的传感器
    int healthyCount = 0;
    for (bool healthy : sensorHealthStatus_) {
        if (healthy) healthyCount++;
    }
    
    return healthyCount >= 4;
}

bool SensorFusion::validateInnovation(const Eigen::VectorXf& innovation, const Eigen::MatrixXf& innovationCovariance) {
    // 马氏距离检查
    float mahalanobisDist = innovation.transpose() * innovationCovariance.inverse() * innovation;
    if (mahalanobisDist > innovationThreshold_ * innovationThreshold_) {
        return false;
    }
    
    // 各个维度的创新序列检查
    for (int i = 0; i < innovation.size(); ++i) {
        float normalizedInnovation = std::abs(innovation(i)) / std::sqrt(innovationCovariance(i, i));
        if (normalizedInnovation > innovationThreshold_) {
            return false;
        }
    }
    
    return true;
}

Eigen::VectorXf SensorFusion::calculateMeasurementVector(const SensorData& data) {
    Eigen::VectorXf z(6);
    z << data.gnssPosition.x(), data.gnssPosition.y(), data.gnssPosition.z(),
         data.gnssVelocity.x(), data.gnssVelocity.y(), data.gnssVelocity.z();
    return z;
}

VehicleState SensorFusion::getCurrentState() const {
    VehicleState state;
    state.position = Vector3d(x_(0), x_(1), x_(2));
    state.velocity = Vector3d(x_(3), x_(4), x_(5));
    state.acceleration = Vector3d(x_(6), x_(7), x_(8));
    state.heading = x_(9);
    state.estimatedMass = x_(12);
    state.estimationCovariance = P_.block<3, 3>(0, 0);
    
    return state;
}

PerceptionData SensorFusion::generatePerceptionData(const VehicleState& state, const SensorData& sensorData) {
    PerceptionData perception;
    perception.vehicleState = state;
    
    // 计算地形坡度 (基于加速度计和GPS)
    perception.terrainSlope = std::atan2(sensorData.imuAcceleration.x(), 
                                       sensorData.imuAcceleration.z());
    
    // 估计土壤阻力 (基于牵引力和速度)
    if (state.velocity.norm() > 0.1f) {
        perception.soilResistance = sensorData.implementForce / state.velocity.norm();
    }
    
    // 计算负载系数
    perception.loadFactor = sensorData.implementForce / (state.estimatedMass * 9.81f);
    
    return perception;
}

void SensorFusion::updateSensorHistory(const SensorData& data) {
    sensorHistory_.push_back(data);
    if (sensorHistory_.size() > maxHistorySize_) {
        sensorHistory_.pop_front();
    }
}

SensorData SensorFusion::getAverageSensorData() const {
    if (sensorHistory_.empty()) {
        return SensorData{};
    }
    
    SensorData average;
    size_t count = sensorHistory_.size();
    
    for (const auto& data : sensorHistory_) {
        average.gnssPosition += data.gnssPosition / count;
        average.gnssVelocity += data.gnssVelocity / count;
        average.imuAcceleration += data.imuAcceleration / count;
        average.imuAngularRate += data.imuAngularRate / count;
        average.implementForce += data.implementForce / count;
    }
    
    return average;
}

void SensorFusion::reset() {
    x_.setZero();
    P_.setIdentity();
    P_ *= 10.0f;
    
    std::fill(sensorHealthStatus_.begin(), sensorHealthStatus_.end(), true);
    std::fill(innovationHistory_.begin(), innovationHistory_.end(), 0.0f);
    sensorHistory_.clear();
}

float SensorFusion::getEstimationUncertainty() const {
    return P_.trace() / STATE_DIM;
}

} // namespace VCUCore