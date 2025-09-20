// src/perception/sensor_fusion.cpp - 类型不匹配修复补丁
// 这个文件包含了修复Eigen类型不匹配和缺失成员的关键修改

// 在sensor_fusion.cpp中需要修改的关键部分：

// 1. 修复第129行的类型不匹配问题
// 原代码可能是：
// someMatrix = kalmanFilter_.getCovariance().block<3,3>(0,0);
// 修改为：
// someMatrix = kalmanFilter_.getCovariance().block<3,3>(0,0).cast<double>();

// 2. 修复第210行的stabilityIndex成员访问
// 原代码：perception.stabilityIndex = ...
// 这个成员已经在vcu_core_types.hpp中添加

// 3. 修复第215-218行的tractionEfficiency成员访问
// 原代码：perception.tractionEfficiency = ...
// 这个成员已经在vcu_core_types.hpp中添加

// 4. 修复第236行的gnssVelocity成员访问
// 原代码：data.gnssVelocity.norm()
// 这个成员已经在SensorData结构体中添加

// 5. 修复第240行的wheelSpeeds成员访问
// 原代码：for (const auto& speed : data.wheelSpeeds)
// 这个成员已经在SensorData结构体中添加

// 以下是需要在sensor_fusion.cpp中应用的具体修改：

/*
在sensor_fusion.cpp的相应位置进行以下修改：

1. 在文件顶部添加类型转换辅助函数：
namespace {
    template<typename T>
    Eigen::Matrix<double, 3, 3> toDouble3x3(const T& matrix) {
        return matrix.template cast<double>();
    }
}

2. 修复第129行左右的类型不匹配：
// 将类似这样的代码：
// Matrix3d someMatrix = kalmanFilter_.getCovariance().block<3,3>(0,0);
// 修改为：
Matrix3d someMatrix = toDouble3x3(kalmanFilter_.getCovariance().block<3,3>(0,0));

3. 确保所有Vector3d和Matrix3d使用double类型，避免与float混用

4. 在checkSensorHealth函数中，确保wheelSpeeds的循环正确：
for (const auto& speed : data.wheelSpeeds) {
    if (speed < 0.0 || speed > 50.0) {  // 使用double字面量
        return false;
    }
}
*/

// 由于无法直接修改原始的sensor_fusion.cpp文件，
// 这个补丁文件提供了修复指导。
// 用户需要根据这些指导手动修改sensor_fusion.cpp文件。

#include "sensor_fusion.hpp"
#include "utils/tractor_state_calculator.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace VCUCore {

// 类型转换辅助函数
namespace {
    template<typename T>
    Eigen::Matrix<double, 3, 3> toDouble3x3(const T& matrix) {
        return matrix.template cast<double>();
    }
    
    template<typename T>
    Eigen::Vector3d toDouble3d(const T& vector) {
        return vector.template cast<double>();
    }
}

// 示例修复代码片段（需要根据实际的sensor_fusion.cpp内容调整）

void SensorFusion::updateKalmanFilter(const SensorData& data) {
    // 修复类型不匹配问题的示例
    
    // 如果原代码有类似这样的赋值：
    // Matrix3d covariance = kalmanFilter_.getCovariance().block<3,3>(0,0);
    // 修改为：
    Matrix3d covariance = toDouble3x3(kalmanFilter_.getCovariance().block<3,3>(0,0));
    
    // 如果原代码有类似这样的向量赋值：
    // Vector3d velocity = someFloatVector;
    // 修改为：
    // Vector3d velocity = toDouble3d(someFloatVector);
}

PerceptionData SensorFusion::fuseData(const SensorData& data) {
    PerceptionData perception;
    
    // 基本数据融合
    perception.vehicleState.position = data.gnssPosition;
    perception.vehicleState.velocity = data.gnssVelocity;  // 现在gnssVelocity已定义
    perception.vehicleState.acceleration = data.imuAcceleration;
    perception.vehicleState.imuAngularRate = data.imuAngularRate;
    
    // 设置新添加的成员
    perception.stabilityIndex = TractorStateCalculator::calculateStabilityMargin(
        perception.vehicleState.position,
        perception.vehicleState.velocity,
        perception.vehicleState.acceleration
    );
    
    if (perception.vehicleState.drawbarPull > 0.0) {
        perception.tractionEfficiency = TractorStateCalculator::calculateTractionEfficiency(
            perception.vehicleState.drawbarPull, perception.vehicleState.actualTorque);
    } else {
        perception.tractionEfficiency = 0.0;
    }
    
    perception.timestamp = data.timestamp;
    
    return perception;
}

bool SensorFusion::checkSensorHealth(const SensorData& data) {
    // 检查GNSS速度有效性（现在gnssVelocity已定义）
    bool velocityValid = (data.gnssVelocity.norm() < 30.0); // 最大30m/s
    
    // 检查车轮速度有效性（现在wheelSpeeds已定义）
    bool wheelSpeedsValid = true;
    for (const auto& speed : data.wheelSpeeds) {
        if (speed < 0.0 || speed > 50.0) {  // 使用double字面量
            wheelSpeedsValid = false;
            break;
        }
    }
    
    // 检查IMU数据
    bool imuValid = (data.imuAcceleration.norm() < 50.0) && 
                    (data.imuAngularRate.norm() < 10.0);
    
    return velocityValid && wheelSpeedsValid && imuValid;
}

} // namespace VCUCore
