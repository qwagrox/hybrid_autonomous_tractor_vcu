// include/utils/tractor_state_calculator.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <cmath>

namespace VCUCore {

/**
 * @class TractorStateCalculator
 * @brief 拖拉机专用状态计算工具类
 * 
 * 提供拖拉机特有的状态计算功能，包括：
 * - 牵引效率计算
 * - 稳定性评估
 * - 田间作业效率计算
 * - 载荷分配优化
 */
class TractorStateCalculator {
public:
    /**
     * @brief 计算牵引效率
     * @param drawbarPull 牵引力 (N)
     * @param speed 作业速度 (m/s)
     * @param enginePower 发动机功率 (kW)
     * @return 牵引效率 (0-1)
     */
    static float calculateTractionEfficiency(float drawbarPull, float speed, float enginePower) {
        if (enginePower <= 0) return 0.0f;
        
        float drawbarPower = drawbarPull * speed / 1000.0f; // kW
        return std::min(1.0f, drawbarPower / enginePower);
    }
    
    /**
     * @brief 计算稳定性裕度
     * @param roll 横滚角 (rad)
     * @param pitch 俯仰角 (rad)
     * @param centerOfGravityHeight 重心高度 (m)
     * @param wheelbase 轴距 (m)
     * @return 稳定性裕度 (0-1，越大越稳定)
     */
    static float calculateStabilityMargin(float roll, float pitch, 
                                        float centerOfGravityHeight, 
                                        float wheelbase = 2.8f) {
        // 基于静态稳定性三角形的简化计算
        float rollStability = std::cos(std::abs(roll));
        float pitchStability = std::cos(std::abs(pitch));
        
        // 重心高度影响系数
        float heightFactor = std::max(0.1f, 1.0f - centerOfGravityHeight / 3.0f);
        
        return rollStability * pitchStability * heightFactor;
    }
    
    /**
     * @brief 计算田间效率
     * @param workingSpeed 作业速度 (km/h)
     * @param workingWidth 作业幅宽 (m)
     * @param fieldEfficiency 田间效率系数 (0-1)
     * @return 理论作业效率 (ha/h)
     */
    static float calculateFieldEfficiency(float workingSpeed, float workingWidth, 
                                        float fieldEfficiency = 0.85f) {
        // 理论作业效率 = 速度 × 幅宽 × 效率系数
        return workingSpeed * workingWidth * fieldEfficiency / 10.0f; // ha/h
    }
    
    /**
     * @brief 计算比油耗
     * @param fuelConsumption 燃油消耗率 (L/h)
     * @param workingSpeed 作业速度 (km/h)
     * @param workingWidth 作业幅宽 (m)
     * @param fieldEfficiency 田间效率 (0-1)
     * @return 比油耗 (L/ha)
     */
    static float calculateSpecificFuelConsumption(float fuelConsumption, 
                                                float workingSpeed, 
                                                float workingWidth,
                                                float fieldEfficiency = 0.85f) {
        float workRate = calculateFieldEfficiency(workingSpeed, workingWidth, fieldEfficiency);
        if (workRate <= 0) return 0.0f;
        
        return fuelConsumption / workRate;
    }
    
    /**
     * @brief 计算最优载荷分配
     * @param totalMass 总质量 (kg)
     * @param ballastMass 配重质量 (kg)
     * @param implementMass 农具质量 (kg)
     * @param wheelbase 轴距 (m)
     * @return 前后桥载荷分配 {前桥载荷, 后桥载荷} (N)
     */
    static std::pair<float, float> calculateOptimalLoadDistribution(
        float totalMass, float ballastMass, float implementMass, 
        float wheelbase = 2.8f) {
        
        const float g = 9.81f; // 重力加速度
        float totalWeight = totalMass * g;
        
        // 考虑农具对重心位置的影响
        float cgShift = implementMass * 0.5f / totalMass; // 简化计算
        
        // 理想前后桥载荷分配 (拖拉机通常后桥承重更多)
        float frontRatio = 0.35f + cgShift; // 前桥35%基础 + 农具影响
        float rearRatio = 0.65f - cgShift;  // 后桥65%基础 - 农具影响
        
        float frontLoad = totalWeight * frontRatio;
        float rearLoad = totalWeight * rearRatio;
        
        return {frontLoad, rearLoad};
    }
    
    /**
     * @brief 检查侧翻风险
     * @param roll 横滚角 (rad)
     * @param lateralAcceleration 侧向加速度 (m/s²)
     * @param speed 速度 (m/s)
     * @return 是否存在侧翻风险
     */
    static bool checkRolloverRisk(float roll, float lateralAcceleration, float speed) {
        const float ROLL_THRESHOLD = 0.35f; // 20度
        const float LATERAL_G_THRESHOLD = 0.4f * 9.81f; // 0.4g
        const float SPEED_THRESHOLD = 8.33f; // 30 km/h
        
        // 多因素综合判断
        bool rollRisk = std::abs(roll) > ROLL_THRESHOLD;
        bool lateralRisk = std::abs(lateralAcceleration) > LATERAL_G_THRESHOLD;
        bool speedRisk = speed > SPEED_THRESHOLD && rollRisk;
        
        return rollRisk || (lateralRisk && speedRisk);
    }
    
    /**
     * @brief 计算PTO功率需求
     * @param implementType 农具类型
     * @param workingWidth 作业幅宽 (m)
     * @param workingSpeed 作业速度 (km/h)
     * @param soilCondition 土壤条件系数 (1.0=标准)
     * @return PTO功率需求 (kW)
     */
    static float calculatePTOPowerDemand(const std::string& implementType,
                                       float workingWidth, 
                                       float workingSpeed,
                                       float soilCondition = 1.0f) {
        // 不同农具的功率需求系数 (kW/m)
        std::map<std::string, float> powerCoefficients = {
            {"plow", 15.0f},        // 犁具
            {"cultivator", 8.0f},   // 耕整机
            {"seeder", 5.0f},       // 播种机
            {"sprayer", 2.0f},      // 喷雾器
            {"mower", 12.0f},       // 割草机
            {"harvester", 25.0f}    // 收获机
        };
        
        auto it = powerCoefficients.find(implementType);
        if (it == powerCoefficients.end()) {
            return 10.0f * workingWidth; // 默认值
        }
        
        float basePower = it->second * workingWidth;
        float speedFactor = std::sqrt(workingSpeed / 8.0f); // 8 km/h为基准速度
        
        return basePower * speedFactor * soilCondition;
    }
    
    /**
     * @brief 更新拖拉机状态的计算字段
     * @param state 拖拉机状态（输入输出参数）
     * @param sensorData 传感器数据
     * @param engineData 发动机数据
     */
    static void updateCalculatedFields(TractorVehicleState& state, 
                                     const SensorData& sensorData,
                                     const EngineData& engineData) {
        // 计算牵引效率
        float speed = state.velocity.norm();
        state.tractionEfficiency = calculateTractionEfficiency(
            state.drawbarPull, speed, engineData.actualTorque * engineData.speed / 9549.0f);
        
        // 计算稳定性裕度
        state.stabilityMargin = calculateStabilityMargin(
            state.roll, state.pitch, state.centerOfGravityHeight);
        
        // 检查侧翻风险
        state.rolloverRisk = checkRolloverRisk(
            state.roll, state.acceleration.y(), speed);
        
        // 计算比油耗
        if (state.workingSpeed > 0 && state.workingWidth > 0) {
            state.specificFuelConsumption = calculateSpecificFuelConsumption(
                state.fuelConsumption, state.workingSpeed, 
                state.workingWidth, state.fieldEfficiency);
        }
        
        // 计算载荷分配
        auto [frontLoad, rearLoad] = calculateOptimalLoadDistribution(
            state.estimatedMass, state.ballastMass, 0.0f);
        state.frontAxleLoad = frontLoad;
        state.rearAxleLoad = rearLoad;
        
        // 更新作业状态标志
        state.isWorking = (state.workingSpeed > 1.0f && state.isPTOEngaged);
        state.isTransporting = (speed > 15.0f && !state.isPTOEngaged);
        
        // 计算牵引功率
        state.drawbarPower = state.drawbarPull * speed / 1000.0f; // kW
    }
};

} // namespace VCUCore
