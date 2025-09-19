// src/utils/tractor_state_calculator.cpp
#include "utils/tractor_state_calculator.hpp"
#include <algorithm>
#include <cmath>

namespace VCUCore {

// 该类主要使用静态方法，实现都在头文件中
// 这里可以添加一些复杂的计算实现

/**
 * @brief 高级稳定性分析
 * 考虑动态因素的稳定性计算
 */
class AdvancedStabilityAnalyzer {
public:
    /**
     * @brief 动态稳定性分析
     * 考虑速度、加速度、转向等动态因素
     */
    static float calculateDynamicStability(const TractorVehicleState& state) {
        // 基础静态稳定性
        float staticStability = TractorStateCalculator::calculateStabilityMargin(
            state.roll, state.pitch, state.centerOfGravityHeight);
        
        // 速度影响因子
        float speed = state.velocity.norm();
        float speedFactor = 1.0f - std::min(0.3f, speed / 50.0f); // 速度越高，稳定性越低
        
        // 加速度影响因子
        float lateralAccel = std::abs(state.acceleration.y());
        float accelFactor = 1.0f - std::min(0.4f, lateralAccel / 5.0f);
        
        // 转向影响因子
        float turningFactor = 1.0f;
        if (state.isTurning) {
            turningFactor = 0.8f; // 转弯时稳定性降低
        }
        
        // 载荷影响因子
        float loadFactor = 1.0f;
        if (state.estimatedMass > 15000.0f) { // 超过15吨
            loadFactor = 0.9f; // 重载时稳定性略降
        }
        
        return staticStability * speedFactor * accelFactor * turningFactor * loadFactor;
    }
    
    /**
     * @brief 预测性稳定性评估
     * 基于当前状态预测未来稳定性风险
     */
    static float predictStabilityRisk(const TractorVehicleState& state, float timeHorizon = 2.0f) {
        // 当前稳定性
        float currentStability = calculateDynamicStability(state);
        
        // 预测未来状态
        float futureRoll = state.roll + state.imuAngularRate.z() * timeHorizon;
        float futurePitch = state.pitch + state.imuAngularRate.y() * timeHorizon;
        
        // 预测稳定性
        float predictedStability = TractorStateCalculator::calculateStabilityMargin(
            futureRoll, futurePitch, state.centerOfGravityHeight);
        
        // 风险评估 (稳定性下降程度)
        float riskScore = std::max(0.0f, currentStability - predictedStability);
        
        return riskScore;
    }
};

/**
 * @brief 田间作业效率优化器
 */
class FieldEfficiencyOptimizer {
public:
    /**
     * @brief 计算最优作业速度
     * 在燃油经济性和作业效率之间找平衡点
     */
    static float calculateOptimalWorkingSpeed(const std::string& implementType,
                                            float workingWidth,
                                            float soilCondition,
                                            float fuelPrice = 1.5f) { // 元/升
        // 不同农具的最优速度范围
        std::map<std::string, std::pair<float, float>> speedRanges = {
            {"plow", {6.0f, 12.0f}},        // 犁具: 6-12 km/h
            {"cultivator", {8.0f, 15.0f}},  // 耕整机: 8-15 km/h
            {"seeder", {5.0f, 10.0f}},      // 播种机: 5-10 km/h
            {"sprayer", {12.0f, 25.0f}},    // 喷雾器: 12-25 km/h
            {"mower", {8.0f, 18.0f}}        // 割草机: 8-18 km/h
        };
        
        auto it = speedRanges.find(implementType);
        if (it == speedRanges.end()) {
            return 10.0f; // 默认速度
        }
        
        float minSpeed = it->second.first;
        float maxSpeed = it->second.second;
        
        // 根据土壤条件调整
        if (soilCondition > 1.2f) { // 困难土壤条件
            maxSpeed *= 0.8f;
        } else if (soilCondition < 0.8f) { // 良好土壤条件
            maxSpeed *= 1.2f;
        }
        
        // 简化的效率-速度关系
        // 通常在中等速度时效率最高
        float optimalSpeed = (minSpeed + maxSpeed) / 2.0f;
        
        return std::clamp(optimalSpeed, minSpeed, maxSpeed);
    }
    
    /**
     * @brief 计算作业成本
     * 包括燃油成本、时间成本等
     */
    static float calculateOperatingCost(float fuelConsumption,    // L/h
                                      float workingSpeed,       // km/h
                                      float workingWidth,       // m
                                      float fuelPrice = 1.5f,  // 元/L
                                      float timeCost = 50.0f) { // 元/h
        float workRate = TractorStateCalculator::calculateFieldEfficiency(
            workingSpeed, workingWidth);
        
        if (workRate <= 0) return 1000.0f; // 惩罚值
        
        float fuelCostPerHa = fuelConsumption * fuelPrice / workRate;
        float timeCostPerHa = timeCost / workRate;
        
        return fuelCostPerHa + timeCostPerHa; // 元/ha
    }
};

/**
 * @brief 负载管理优化器
 */
class LoadManagementOptimizer {
public:
    /**
     * @brief 优化配重分配
     * 根据作业类型和土壤条件优化配重
     */
    static float calculateOptimalBallast(const std::string& implementType,
                                       float implementWeight,
                                       float soilCondition,
                                       float tractorBaseWeight = 8000.0f) {
        // 不同作业的配重需求系数
        std::map<std::string, float> ballastRatios = {
            {"plow", 0.8f},         // 犁具需要较多配重
            {"cultivator", 0.6f},   // 耕整机中等配重
            {"seeder", 0.4f},       // 播种机较少配重
            {"sprayer", 0.2f},      // 喷雾器最少配重
            {"mower", 0.3f}         // 割草机少量配重
        };
        
        auto it = ballastRatios.find(implementType);
        float baseRatio = (it != ballastRatios.end()) ? it->second : 0.5f;
        
        // 根据土壤条件调整
        float soilFactor = std::clamp(soilCondition, 0.5f, 2.0f);
        
        // 根据农具重量调整
        float implementFactor = std::sqrt(implementWeight / 2000.0f); // 2吨为基准
        
        float optimalBallast = tractorBaseWeight * baseRatio * soilFactor * implementFactor;
        
        // 限制配重范围 (500kg - 3000kg)
        return std::clamp(optimalBallast, 500.0f, 3000.0f);
    }
    
    /**
     * @brief 计算轮胎压力建议
     * 根据载荷和土壤条件优化轮胎压力
     */
    static std::array<float, 4> calculateOptimalTirePressure(
        const std::array<float, 4>& wheelLoads, // N
        float soilCondition,
        bool isFieldWork = true) {
        
        std::array<float, 4> pressures;
        
        for (int i = 0; i < 4; ++i) {
            // 基础压力计算 (简化公式)
            float basePressure = wheelLoads[i] / 10000.0f + 1.0f; // bar
            
            // 田间作业时降低压力以减少土壤压实
            if (isFieldWork) {
                basePressure *= 0.8f;
            }
            
            // 根据土壤条件调整
            if (soilCondition < 0.8f) { // 软土
                basePressure *= 0.9f;
            } else if (soilCondition > 1.2f) { // 硬土
                basePressure *= 1.1f;
            }
            
            // 限制压力范围
            pressures[i] = std::clamp(basePressure, 0.8f, 2.5f);
        }
        
        return pressures;
    }
};

} // namespace VCUCore
