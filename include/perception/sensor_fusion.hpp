// include/perception/sensor_fusion.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <deque>

namespace VCUCore {

class SensorFusion {
private:
    // EKF状态向量: [x, y, z, vx, vy, vz, ax, ay, az, heading, pitch, roll, mass]
    static constexpr int STATE_DIM = 13;
    using StateVector = Eigen::Matrix<float, STATE_DIM, 1>;
    using StateCovariance = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
    
    StateVector x_;
    StateCovariance P_;
    Eigen::Matrix<float, 6, STATE_DIM> H_;
    Eigen::Matrix<float, 6, 6> R_;
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q_;
    
    std::deque<SensorData> sensorHistory_;
    size_t maxHistorySize_;
    
    // 故障检测参数
    float innovationThreshold_;
    std::array<bool, 6> sensorHealthStatus_;
    std::array<float, 6> innovationHistory_;

public:
    SensorFusion(size_t historySize = 1000, float innovationThreshold = 3.0f);
    
    TractorVehicleState fuseSensors(const SensorData& sensorData);
    PerceptionData generatePerceptionData(const TractorVehicleState& state, const SensorData& sensorData);
    
    bool checkSensorHealth(const SensorData& data);
    void reset();
    
    // 状态查询
    TractorVehicleState getCurrentState() const;
    float getEstimationUncertainty() const;
    
private:
    void initializeMatrices();
    void predict(float deltaTime);
    void update(const SensorData& data);
    bool validateInnovation(const Eigen::VectorXf& innovation, const Eigen::MatrixXf& innovationCovariance);
    
    Eigen::VectorXf calculateMeasurementVector(const SensorData& data);
    Eigen::MatrixXf calculateMeasurementJacobian();
    
    void updateSensorHistory(const SensorData& data);
    SensorData getAverageSensorData() const;
};

} // namespace VCUCore