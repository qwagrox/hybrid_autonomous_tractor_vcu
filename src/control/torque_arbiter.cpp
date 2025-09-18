// src/control/torque_arbiter.cpp
#include "torque_arbiter.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>

namespace VCUCore {

TorqueArbiter::TorqueArbiter(uint32_t historySize)
    : maxHistorySize_(historySize) {
    
    // 初始化系统限制
    maxEngineTorque_ = 600.0f;
    maxMotorTorque_ = 400.0f;
    minEngineTorque_ = 50.0f;
    minMotorTorque_ = -200.0f; // 负值表示发电
    torqueChangeRateLimit_ = 100.0f; // Nm/s
    
    initializePolicies();
    
    // 初始化历史缓冲区
    torqueHistory_.reserve(maxHistorySize_);
}

void TorqueArbiter::initializePolicies() {
    // ECO模式：优先效率
    policyMap_[DriveMode::ECO] = {
        .enginePriority = 0.4f,
        .motorPriority = 0.6f,
        .efficiencyWeight = 0.8f,
        .responseWeight = 0.1f,
        .batterySOCWeight = 0.1f,
        .engineHealthWeight = 0.0f
    };
    
    // COMFORT模式：平衡效率和响应性
    policyMap_[DriveMode::COMFORT] = {
        .enginePriority = 0.5f,
        .motorPriority = 0.5f,
        .efficiencyWeight = 0.5f,
        .responseWeight = 0.4f,
        .batterySOCWeight = 0.1f,
        .engineHealthWeight = 0.0f
    };
    
    // SPORT模式：优先响应性
    policyMap_[DriveMode::SPORT] = {
        .enginePriority = 0.6f,
        .motorPriority = 0.4f,
        .efficiencyWeight = 0.2f,
        .responseWeight = 0.7f,
        .batterySOCWeight = 0.1f,
        .engineHealthWeight = 0.0f
    };
    
    // 犁地模式：优先牵引力
    policyMap_[DriveMode::PLOWING] = {
        .enginePriority = 0.7f,
        .motorPriority = 0.3f,
        .efficiencyWeight = 0.3f,
        .responseWeight = 0.6f,
        .batterySOCWeight = 0.1f,
        .engineHealthWeight = 0.0f
    };
    
    // 播种模式：优先稳定性
    policyMap_[DriveMode::SEEDING] = {
        .enginePriority = 0.55f,
        .motorPriority = 0.45f,
        .efficiencyWeight = 0.6f,
        .responseWeight = 0.2f,
        .batterySOCWeight = 0.2f,
        .engineHealthWeight = 0.0f
    };
    
    // 运输模式
    policyMap_[DriveMode::TRANSPORT] = {
        .enginePriority = 0.65f,
        .motorPriority = 0.35f,
        .efficiencyWeight = 0.4f,
        .responseWeight = 0.4f,
        .batterySOCWeight = 0.2f,
        .engineHealthWeight = 0.0f
    };
    
    // 默认策略
    currentPolicy_ = policyMap_[DriveMode::COMFORT];
}

TorqueSplit TorqueArbiter::decideDistribution(const PerceptionData& perception,
                                            const PredictionResult& prediction) {
    
    if (!validateInput(perception, prediction)) {
        std::cerr << "Invalid input data, using conservative split" << std::endl;
        return createConservativeSplit();
    }
    
    try {
        // 计算总扭矩需求
        float totalTorque = calculateTotalTorqueDemand(perception, prediction);
        
        // 计算基础分配
        TorqueSplit baseSplit = calculateBaseSplit(totalTorque, perception);
        
        // 多目标优化
        TorqueSplit optimizedSplit = optimizeTorqueSplit(baseSplit, perception, prediction);
        
        // 应用限制和保护
        TorqueSplit finalSplit = applyTorqueLimits(optimizedSplit);
        finalSplit = applySafetyLimits(finalSplit, perception);
        
        // 计算效率和其他指标
        finalSplit.efficiency = calculateSplitEfficiency(finalSplit, perception);
        finalSplit.responseTime = calculateResponseTime(finalSplit);
        finalSplit.timestamp = getCurrentTime();
        
        // 更新历史记录
        updateTorqueHistory(finalSplit);
        
        return finalSplit;
        
    } catch (const std::exception& e) {
        std::cerr << "Torque arbitration error: " << e.what() << std::endl;
        return createEmergencySplit();
    }
}

bool TorqueArbiter::validateInput(const PerceptionData& perception,
                                const PredictionResult& prediction) const {
    
    // 检查感知数据有效性
    if (perception.vehicleState.velocity.norm() < 0.0f ||
        perception.vehicleState.velocity.norm() > 50.0f) {
        return false;
    }
    
    if (perception.vehicleState.actualTorque < -1000.0f ||
        perception.vehicleState.actualTorque > 1000.0f) {
        return false;
    }
    
    if (perception.vehicleState.batterySOC < 0.0f ||
        perception.vehicleState.batterySOC > 1.0f) {
        return false;
    }
    
    // 检查预测数据有效性
    if (!prediction.loadForecast.empty() && 
        (prediction.loadForecast[0] < 0.0f || prediction.loadForecast[0] > 20000.0f)) {
        return false;
    }
    
    return true;
}

float TorqueArbiter::calculateTotalTorqueDemand(const PerceptionData& perception,
                                              const PredictionResult& prediction) const {
    
    float baseTorque = perception.vehicleState.drawbarPull;
    
    // 考虑地形坡度
    float gradeTorque = perception.vehicleState.estimatedMass * 9.81f * 
                       std::sin(perception.terrainSlope);
    
    // 考虑加速度需求
    float accelerationTorque = perception.vehicleState.estimatedMass * 
                              perception.vehicleState.acceleration.norm();
    
    // 考虑滚动阻力
    float rollingResistanceTorque = perception.vehicleState.estimatedMass * 9.81f *
                                   perception.rollingResistance *
                                   std::cos(perception.terrainSlope);
    
    // 考虑空气阻力
    float aerodynamicTorque = perception.aerodynamicDrag * 0.5f;
    
    // 基础总扭矩
    float totalTorque = baseTorque + gradeTorque + accelerationTorque + 
                       rollingResistanceTorque + aerodynamicTorque;
    
    // 考虑预测负载
    if (!prediction.loadForecast.empty()) {
        float predictedTorque = prediction.loadForecast[0];
        float forecastWeight = 0.3f; // 30%的前瞻权重
        totalTorque = totalTorque * (1.0f - forecastWeight) + predictedTorque * forecastWeight;
    }
    
    // 确保正值
    return std::max(0.0f, totalTorque);
}

TorqueSplit TorqueArbiter::calculateBaseSplit(float totalTorque, const PerceptionData& perception) const {
    TorqueSplit split;
    split.totalTorque = totalTorque;
    
    // 基于策略优先级的基础分配
    float engineRatio = currentPolicy_.enginePriority / 
                       (currentPolicy_.enginePriority + currentPolicy_.motorPriority);
    
    split.engineTorque = totalTorque * engineRatio;
    split.motorTorque = totalTorque * (1.0f - engineRatio);
    
    // 考虑电池SOC调整
    split = adjustForBatterySOC(split, perception.vehicleState.batterySOC);
    
    return split;
}

TorqueSplit TorqueArbiter::optimizeTorqueSplit(const TorqueSplit& baseSplit,
                                             const PerceptionData& perception,
                                             const PredictionResult& prediction) const {
    
    TorqueSplit bestSplit = baseSplit;
    float bestScore = -std::numeric_limits<float>::max();
    
    // 生成多个候选方案
    std::vector<TorqueSplit> candidates = generateCandidateSplits(baseSplit, perception);
    
    for (auto& candidate : candidates) {
        if (!checkTorqueLimits(candidate.engineTorque, candidate.motorTorque)) {
            continue;
        }
        
        float score = evaluateSplitScore(candidate, perception, prediction);
        
        if (score > bestScore) {
            bestScore = score;
            bestSplit = candidate;
        }
    }
    
    return bestSplit;
}

std::vector<TorqueSplit> TorqueArbiter::generateCandidateSplits(const TorqueSplit& baseSplit,
                                                              const PerceptionData& perception) const {
    
    std::vector<TorqueSplit> candidates;
    const int steps = 8;
    
    // 生成基于不同权重的候选方案
    for (int i = 0; i <= steps; ++i) {
        TorqueSplit candidate = baseSplit;
        float ratio = static_cast<float>(i) / steps;
        
        // 效率优化候选
        if (i % 2 == 0) {
            candidate = optimizeForEfficiency(candidate, perception);
        }
        // 响应性优化候选
        else if (i % 2 == 1) {
            candidate = optimizeForResponse(candidate, perception);
        }
        // 电池优化候选
        else {
            candidate = optimizeForBattery(candidate, perception);
        }
        
        candidates.push_back(candidate);
    }
    
    return candidates;
}

float TorqueArbiter::evaluateSplitScore(const TorqueSplit& split,
                                      const PerceptionData& perception,
                                      const PredictionResult& prediction) const {
    
    float efficiencyScore = calculateEfficiencyScore(split, perception);
    float responseScore = calculateResponseScore(split);
    float batteryScore = calculateBatteryScore(split, perception);
    float predictionScore = calculatePredictionScore(split, prediction);
    
    return currentPolicy_.efficiencyWeight * efficiencyScore +
           currentPolicy_.responseWeight * responseScore +
           currentPolicy_.batterySOCWeight * batteryScore +
           0.1f * predictionScore; // 预测权重
}

TorqueSplit TorqueArbiter::optimizeForEfficiency(const TorqueSplit& baseSplit,
                                               const PerceptionData& perception) const {
    
    TorqueSplit optimizedSplit = baseSplit;
    float bestEfficiency = 0.0f;
    
    const int steps = 10;
    for (int i = 0; i <= steps; ++i) {
        float engineTorque = baseSplit.engineTorque * (0.5f + 0.5f * i / steps);
        float motorTorque = baseSplit.totalTorque - engineTorque;
        
        if (!checkTorqueLimits(engineTorque, motorTorque)) {
            continue;
        }
        
        float engineEff = calculateEngineEfficiency(engineTorque, perception.vehicleState.engineRpm);
        float motorEff = calculateMotorEfficiency(motorTorque, perception.vehicleState.motorRpm, 
                                                perception.vehicleState.batterySOC);
        
        float totalEfficiency = (engineTorque * engineEff + motorTorque * motorEff) / 
                               baseSplit.totalTorque;
        
        if (totalEfficiency > bestEfficiency) {
            bestEfficiency = totalEfficiency;
            optimizedSplit.engineTorque = engineTorque;
            optimizedSplit.motorTorque = motorTorque;
        }
    }
    
    optimizedSplit.efficiency = bestEfficiency;
    return optimizedSplit;
}

TorqueSplit TorqueArbiter::optimizeForResponse(const TorqueSplit& baseSplit,
                                             const PerceptionData& perception) const {
    
    TorqueSplit optimizedSplit = baseSplit;
    
    // 响应性优化：优先使用电机（响应更快）
    float motorCapability = std::min(maxMotorTorque_, baseSplit.totalTorque);
    optimizedSplit.motorTorque = motorCapability;
    optimizedSplit.engineTorque = baseSplit.totalTorque - motorCapability;
    
    // 确保发动机在高效区工作
    if (optimizedSplit.engineTorque < minEngineTorque_) {
        optimizedSplit.engineTorque = minEngineTorque_;
        optimizedSplit.motorTorque = baseSplit.totalTorque - minEngineTorque_;
    }
    
    return optimizedSplit;
}

TorqueSplit TorqueArbiter::optimizeForBattery(const TorqueSplit& baseSplit,
                                            const PerceptionData& perception) const {
    
    TorqueSplit optimizedSplit = baseSplit;
    float batterySOC = perception.vehicleState.batterySOC;
    
    // 电池SOC低时，减少电机使用，增加发动机发电
    if (batterySOC < 0.3f) {
        float chargeTorque = 50.0f; // 发电扭矩
        optimizedSplit.engineTorque += chargeTorque;
        optimizedSplit.motorTorque = -chargeTorque; // 电机发电
    }
    // 电池SOC高时，优先使用电机
    else if (batterySOC > 0.8f) {
        float motorAdditional = std::min(100.0f, maxMotorTorque_ - optimizedSplit.motorTorque);
        optimizedSplit.motorTorque += motorAdditional;
        optimizedSplit.engineTorque -= motorAdditional;
    }
    
    return optimizedSplit;
}

TorqueSplit TorqueArbiter::adjustForBatterySOC(const TorqueSplit& split, float batterySOC) const {
    TorqueSplit adjusted = split;
    
    // SOC低于20%时减少电机使用
    if (batterySOC < 0.2f) {
        float reduction = (0.2f - batterySOC) * 2.0f; // 线性减少
        adjusted.motorTorque *= (1.0f - reduction);
        adjusted.engineTorque = adjusted.totalTorque - adjusted.motorTorque;
    }
    // SOC高于80%时增加电机使用
    else if (batterySOC > 0.8f) {
        float increase = (batterySOC - 0.8f) * 2.0f; // 线性增加
        adjusted.motorTorque = std::min(adjusted.motorTorque * (1.0f + increase), maxMotorTorque_);
        adjusted.engineTorque = adjusted.totalTorque - adjusted.motorTorque;
    }
    
    return adjusted;
}

TorqueSplit TorqueArbiter::applyTorqueLimits(const TorqueSplit& split) const {
    TorqueSplit limitedSplit = split;
    
    limitedSplit.engineTorque = std::clamp(limitedSplit.engineTorque, 
                                         minEngineTorque_, maxEngineTorque_);
    limitedSplit.motorTorque = std::clamp(limitedSplit.motorTorque,
                                        minMotorTorque_, maxMotorTorque_);
    
    // 确保总和不变（在限制范围内）
    float total = limitedSplit.engineTorque + limitedSplit.motorTorque;
    if (std::abs(total - split.totalTorque) > 1.0f) {
        float scale = split.totalTorque / total;
        limitedSplit.engineTorque *= scale;
        limitedSplit.motorTorque *= scale;
    }
    
    return limitedSplit;
}

TorqueSplit TorqueArbiter::applySafetyLimits(const TorqueSplit& split, const PerceptionData& perception) const {
    TorqueSplit safeSplit = split;
    
    // 温度限制
    if (perception.vehicleState.engineTemperature > 100.0f) {
        safeSplit.engineTorque *= 0.8f; // 高温降额
    }
    
    if (perception.vehicleState.motorTemperature > 100.0f) {
        safeSplit.motorTorque *= 0.8f; // 高温降额
    }
    
    // 电池限制
    if (perception.vehicleState.batterySOC < 0.15f) {
        safeSplit.motorTorque = std::max(safeSplit.motorTorque, -50.0f); // 限制发电
    }
    
    return safeSplit;
}

float TorqueArbiter::calculateEngineEfficiency(float torque, float rpm) const {
    // 发动机效率模型（基于万有特性）
    float optimalRpm = 1800.0f;
    float optimalTorque = 300.0f;
    
    float rpmEfficiency = 1.0f - 0.5f * std::abs(rpm - optimalRpm) / optimalRpm;
    float torqueEfficiency = 1.0f - 0.3f * std::abs(torque - optimalTorque) / optimalTorque;
    
    // 基础效率曲线
    float baseEfficiency = 0.35f + 0.1f * (torque / maxEngineTorque_);
    
    return baseEfficiency * rpmEfficiency * torqueEfficiency;
}

float TorqueArbiter::calculateMotorEfficiency(float torque, float rpm, float batterySOC) const {
    // 电机效率模型
    float baseEfficiency = 0.92f;
    
    // 负载影响
    float loadFactor = 1.0f - 0.2f * std::abs(torque) / maxMotorTorque_;
    
    // 转速影响
    float optimalRpm = 2500.0f;
    float rpmFactor = 1.0f - 0.1f * std::abs(rpm - optimalRpm) / optimalRpm;
    
    // SOC影响
    float socFactor = 0.9f + 0.1f * batterySOC;
    
    return baseEfficiency * loadFactor * rpmFactor * socFactor;
}

float TorqueArbiter::calculateSplitEfficiency(const TorqueSplit& split,
                                            const PerceptionData& perception) const {
    
    float engineEff = calculateEngineEfficiency(split.engineTorque, perception.vehicleState.engineRpm);
    float motorEff = calculateMotorEfficiency(split.motorTorque, perception.vehicleState.motorRpm,
                                            perception.vehicleState.batterySOC);
    
    return (split.engineTorque * engineEff + split.motorTorque * motorEff) / split.totalTorque;
}

float TorqueArbiter::calculateResponseTime(const TorqueSplit& split) const {
    // 发动机响应时间模型
    float engineResponse = 0.2f + 0.1f * (split.engineTorque / maxEngineTorque_);
    
    // 电机响应时间模型
    float motorResponse = 0.05f + 0.02f * (std::abs(split.motorTorque) / maxMotorTorque_);
    
    // 总响应时间（加权平均）
    float engineWeight = split.engineTorque / (split.engineTorque + std::abs(split.motorTorque));
    return engineWeight * engineResponse + (1.0f - engineWeight) * motorResponse;
}

float TorqueArbiter::calculateEfficiencyScore(const TorqueSplit& split,
                                            const PerceptionData& perception) const {
    
    float efficiency = calculateSplitEfficiency(split, perception);
    return efficiency / 0.9f; // 归一化到0-1范围
}

float TorqueArbiter::calculateResponseScore(const TorqueSplit& split) const {
    float responseTime = calculateResponseTime(split);
    return 1.0f / (1.0f + responseTime); // 响应时间越短，分数越高
}

float TorqueArbiter::calculateBatteryScore(const TorqueSplit& split,
                                         const PerceptionData& perception) const {
    
    float batterySOC = perception.vehicleState.batterySOC;
    
    // 充电时正分，放电时负分（基于SOC水平）
    if (split.motorTorque < 0.0f) { // 发电
        return (1.0f - batterySOC) * 0.5f; // 低SOC时发电得分更高
    } else { // 用电
        return batterySOC * 0.3f; // 高SOC时用电得分更高
    }
}

float TorqueArbiter::calculatePredictionScore(const TorqueSplit& split,
                                            const PredictionResult& prediction) const {
    
    if (prediction.loadForecast.empty()) {
        return 0.5f;
    }
    
    // 基于预测负载的评分
    float predictedLoad = prediction.loadForecast[0];
    float currentLoad = split.totalTorque;
    
    // 预测负载与当前负载的匹配程度
    float matchScore = 1.0f - std::abs(predictedLoad - currentLoad) / predictedLoad;
    
    return std::max(0.0f, matchScore);
}

bool TorqueArbiter::checkTorqueLimits(float engineTorque, float motorTorque) const {
    return (engineTorque >= minEngineTorque_) && (engineTorque <= maxEngineTorque_) &&
           (motorTorque >= minMotorTorque_) && (motorTorque <= maxMotorTorque_) &&
           (engineTorque + motorTorque <= maxEngineTorque_ + maxMotorTorque_);
}

void TorqueArbiter::updateTorqueHistory(const TorqueSplit& split) {
    torqueHistory_.push_back(split);
    if (torqueHistory_.size() > maxHistorySize_) {
        torqueHistory_.pop_front();
    }
}

uint32_t TorqueArbiter::getCurrentTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

TorqueSplit TorqueArbiter::createConservativeSplit() const {
    return {
        .engineTorque = 100.0f,
        .motorTorque = 50.0f,
        .totalTorque = 150.0f,
        .efficiency = 0.7f,
        .responseTime = 0.15f,
        .timestamp = getCurrentTime()
    };
}

TorqueSplit TorqueArbiter::createEmergencySplit() const {
    return {
        .engineTorque = 80.0f,
        .motorTorque = 0.0f,
        .totalTorque = 80.0f,
        .efficiency = 0.6f,
        .responseTime = 0.2f,
        .timestamp = getCurrentTime()
    };
}

void TorqueArbiter::updateArbitrationPolicy(DriveMode mode) {
    if (policyMap_.find(mode) != policyMap_.end()) {
        currentPolicy_ = policyMap_[mode];
        std::cout << "Torque arbitration policy updated for mode: " << static_cast<int>(mode) << std::endl;
    }
}

void TorqueArbiter::learnFromFeedback(const TorqueSplit& actualSplit, 
                                   const PerceptionData& resultingPerception) {
    
    // 计算实际效率
    float actualEfficiency = resultingPerception.vehicleState.energyEfficiency;
    
    // 更新效率模型
    updateEfficiencyModel(actualSplit, actualEfficiency);
    
    // 更新响应模型
    float actualResponse = calculateActualResponseTime(actualSplit, resultingPerception);
    updateResponseModel(actualSplit, actualResponse);
    
    // 调整策略权重
    adjustPolicyWeights(actualSplit, resultingPerception);
}

float TorqueArbiter::calculateActualResponseTime(const TorqueSplit& split,
                                               const PerceptionData& perception) const {
    
    // 简化实现：基于扭矩变化率和加速度计算实际响应时间
    float torqueChange = std::abs(split.engineTorque - perception.vehicleState.actualTorque);
    float acceleration = perception.vehicleState.acceleration.norm();
    
    if (acceleration > 0.1f) {
        return torqueChange / (acceleration * perception.vehicleState.estimatedMass);
    }
    
    return 0.2f; // 默认值
}

void TorqueArbiter::updateEfficiencyModel(const TorqueSplit& split, float actualEfficiency) {
    float predictedEfficiency = calculateSplitEfficiency(split, PerceptionData{});
    float error = actualEfficiency - predictedEfficiency;
    
    // 根据误差调整效率计算参数
    if (std::abs(error) > 0.05f) {
        // 这里可以更新内部效率模型的参数
        // 简化实现：只调整策略权重
        if (error > 0.0f) {
            currentPolicy_.efficiencyWeight *= 1.05f;
        } else {
            currentPolicy_.efficiencyWeight *= 0.95f;
        }
    }
}

void TorqueArbiter::updateResponseModel(const TorqueSplit& split, float actualResponse) {
    float predictedResponse = calculateResponseTime(split);
    float error = actualResponse - predictedResponse;
    
    // 根据误差调整响应时间计算参数
    if (std::abs(error) > 0.02f) {
        if (error > 0.0f) {
            currentPolicy_.responseWeight *= 0.95f;
        } else {
            currentPolicy_.responseWeight *= 1.05f;
        }
    }
}

void TorqueArbiter::adjustPolicyWeights(const TorqueSplit& split,
                                      const PerceptionData& perception) {
    
    // 权重归一化
    float total = currentPolicy_.efficiencyWeight + currentPolicy_.responseWeight + 
                 currentPolicy_.batterySOCWeight + currentPolicy_.engineHealthWeight;
    
    currentPolicy_.efficiencyWeight /= total;
    currentPolicy_.responseWeight /= total;
    currentPolicy_.batterySOCWeight /= total;
    currentPolicy_.engineHealthWeight /= total;
}

float TorqueArbiter::calculateAverageEfficiency() const {
    if (torqueHistory_.empty()) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (const auto& split : torqueHistory_) {
        sum += split.efficiency;
    }
    
    return sum / torqueHistory_.size();
}

float TorqueArbiter::calculateResponseQuality() const {
    if (torqueHistory_.empty()) {
        return 0.0f;
    }
    
    // 计算响应时间的倒数（质量指标）
    float sum = 0.0f;
    for (const auto& split : torqueHistory_) {
        sum += 1.0f / (split.responseTime + 0.001f); // 避免除零
    }
    
    return sum / torqueHistory_.size();
}

} // namespace VCUCore