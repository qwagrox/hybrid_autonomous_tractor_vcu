// src/control/cvt_controller.cpp
#include "cvt_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>

namespace VCUCore {

CVTController::CVTController(uint32_t historySize)
    : maxHistorySize_(historySize), currentManufacturer_(CVTManufacturer::UNKNOWN) {
    
    // 初始化状态
    currentState_.currentRatio = 1.0f;
    currentState_.targetRatio = 1.0f;
    currentState_.ratioRate = 0.0f;
    currentState_.isShifting = false;
    currentState_.shiftProgress = 0.0f;
    currentState_.shiftStartTime = 0;
    
    initializeModeParams();
    initializeManufacturerParams();
    
    // 默认控制参数
    controlParams_ = modeParams_[DriveMode::COMFORT];
}

void CVTController::initializeModeParams() {
    // ECO模式：优先效率
    modeParams_[DriveMode::ECO] = {
        .minRatio = 0.6f,
        .maxRatio = 2.5f,
        .ratioChangeRate = 0.3f,
        .slipRatioTarget = 0.12f,
        .efficiencyWeight = 0.8f,
        .comfortWeight = 0.1f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.5f
    };
    
    // COMFORT模式：平衡舒适性
    modeParams_[DriveMode::COMFORT] = {
        .minRatio = 0.5f,
        .maxRatio = 2.8f,
        .ratioChangeRate = 0.5f,
        .slipRatioTarget = 0.15f,
        .efficiencyWeight = 0.4f,
        .comfortWeight = 0.5f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.7f
    };
    
    // SPORT模式：优先响应性
    modeParams_[DriveMode::SPORT] = {
        .minRatio = 0.4f,
        .maxRatio = 3.0f,
        .ratioChangeRate = 0.8f,
        .slipRatioTarget = 0.18f,
        .efficiencyWeight = 0.2f,
        .comfortWeight = 0.2f,
        .responseWeight = 0.6f,
        .terrainAdaptationGain = 0.3f
    };
    
    // 犁地模式：优先牵引力
    modeParams_[DriveMode::PLOWING] = {
        .minRatio = 0.5f,
        .maxRatio = 2.2f,
        .ratioChangeRate = 0.4f,
        .slipRatioTarget = 0.16f,
        .efficiencyWeight = 0.3f,
        .comfortWeight = 0.3f,
        .responseWeight = 0.4f,
        .terrainAdaptationGain = 0.8f
    };
    
    // 播种模式：优先速度稳定性
    modeParams_[DriveMode::SEEDING] = {
        .minRatio = 0.7f,
        .maxRatio = 2.0f,
        .ratioChangeRate = 0.2f,
        .slipRatioTarget = 0.10f,
        .efficiencyWeight = 0.6f,
        .comfortWeight = 0.8f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.6f
    };
    
    // 运输模式
    modeParams_[DriveMode::TRANSPORT] = {
        .minRatio = 0.8f,
        .maxRatio = 3.0f,
        .ratioChangeRate = 0.6f,
        .slipRatioTarget = 0.14f,
        .efficiencyWeight = 0.5f,
        .comfortWeight = 0.3f,
        .responseWeight = 0.2f,
        .terrainAdaptationGain = 0.4f
    };
}

void CVTController::initializeManufacturerParams() {
    // John Deere参数
    manufacturerParams_[CVTManufacturer::JOHN_DEERE] = {
        .minRatio = 0.5f,
        .maxRatio = 3.0f,
        .ratioChangeRate = 0.6f,
        .slipRatioTarget = 0.15f,
        .efficiencyWeight = 0.5f,
        .comfortWeight = 0.4f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.7f
    };
    
    // Case IH参数
    manufacturerParams_[CVTManufacturer::CASE_IH] = {
        .minRatio = 0.6f,
        .maxRatio = 2.8f,
        .ratioChangeRate = 0.5f,
        .slipRatioTarget = 0.14f,
        .efficiencyWeight = 0.6f,
        .comfortWeight = 0.3f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.6f
    };
    
    // CLAAS参数
    manufacturerParams_[CVTManufacturer::CLAAS] = {
        .minRatio = 0.55f,
        .maxRatio = 3.2f,
        .ratioChangeRate = 0.7f,
        .slipRatioTarget = 0.16f,
        .efficiencyWeight = 0.4f,
        .comfortWeight = 0.5f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.8f
    };
    
    // AGCO Fendt参数
    manufacturerParams_[CVTManufacturer::AGCO_FENDT] = {
        .minRatio = 0.52f,
        .maxRatio = 3.1f,
        .ratioChangeRate = 0.65f,
        .slipRatioTarget = 0.145f,
        .efficiencyWeight = 0.55f,
        .comfortWeight = 0.35f,
        .responseWeight = 0.1f,
        .terrainAdaptationGain = 0.75f
    };
}

float CVTController::calculateOptimalRatio(const PerceptionData& perception,
                                         const PredictionResult& prediction) {
    if (!validatePerceptionData(perception)) {
        std::cerr << "Invalid perception data, using default ratio" << std::endl;
        return 1.0f;
    }
    
    try {
        // 计算基础传动比
        float baseRatio = calculateBaseRatio(perception);
        
        // 根据模式优化传动比
        float optimizedRatio = baseRatio;
        
        // 基于权重优化
        optimizedRatio = optimizeWithWeights(optimizedRatio, perception);
        
        // 场景特定优化
        optimizedRatio = applyScenarioOptimization(optimizedRatio, perception);
        
        // 应用预测信息
        optimizedRatio = applyPrediction(optimizedRatio, perception, prediction);
        
        // 应用限制
        optimizedRatio = std::clamp(optimizedRatio, controlParams_.minRatio, controlParams_.maxRatio);
        optimizedRatio = applyRateLimits(optimizedRatio);
        
        // 更新状态
        updateShiftState(optimizedRatio);
        
        // 记录历史
        ratioHistory_.push_back(optimizedRatio);
        if (ratioHistory_.size() > maxHistorySize_) {
            ratioHistory_.pop_front();
        }
        
        return optimizedRatio;
        
    } catch (const std::exception& e) {
        std::cerr << "CVT ratio calculation error: " << e.what() << std::endl;
        return currentState_.currentRatio; // 保持当前比率
    }
}

float CVTController::calculateBaseRatio(const PerceptionData& perception) const {
    const float speed = perception.vehicleState.velocity.norm();
    const float torque = perception.vehicleState.actualTorque;
    
    if (speed < 0.1f || torque < 1.0f) {
        return 1.0f; // 默认值
    }
    
    // 计算功率需求
    const float powerDemand = torque * speed / 9549.3f; // kW
    
    // 计算理论最优传动比
    const float optimalEngineRpm = calculateOptimalEngineRpm(powerDemand);
    const float wheelRpm = speed * 60.0f / (2.0f * M_PI * 0.9f); // 车轮转速
    
    if (wheelRpm < 0.1f) {
        return 1.0f;
    }
    
    return optimalEngineRpm / wheelRpm;
}

float CVTController::calculateOptimalEngineRpm(float powerDemand) const {
    // 基于功率需求计算最优发动机转速
    // 简化模型：在中等负载时选择最佳效率转速
    
    if (powerDemand < 20.0f) {
        return 1500.0f; // 低负载
    } else if (powerDemand < 80.0f) {
        return 1800.0f; // 中等负载（最佳效率）
    } else if (powerDemand < 150.0f) {
        return 2000.0f; // 高负载
    } else {
        return 2200.0f; // 峰值负载
    }
}

float CVTController::optimizeWithWeights(float baseRatio, const PerceptionData& perception) const {
    float optimizedRatio = baseRatio;
    
    // 效率优化
    if (controlParams_.efficiencyWeight > 0.0f) {
        float efficiencyOpt = optimizeForEfficiency(baseRatio, perception);
        optimizedRatio = weightedAverage(optimizedRatio, efficiencyOpt, controlParams_.efficiencyWeight);
    }
    
    // 牵引力优化
    if (controlParams_.responseWeight > 0.0f) {
        float tractionOpt = optimizeForTraction(baseRatio, perception);
        optimizedRatio = weightedAverage(optimizedRatio, tractionOpt, controlParams_.responseWeight);
    }
    
    // 舒适性优化
    if (controlParams_.comfortWeight > 0.0f) {
        float comfortOpt = optimizeForComfort(baseRatio, perception);
        optimizedRatio = weightedAverage(optimizedRatio, comfortOpt, controlParams_.comfortWeight);
    }
    
    return optimizedRatio;
}

float CVTController::optimizeForEfficiency(float baseRatio, const PerceptionData& perception) const {
    float bestRatio = baseRatio;
    float bestEfficiency = calculateTheoreticalEfficiency(baseRatio, perception);
    
    // 在基础传动比附近搜索最优效率
    const float searchRange = 0.3f;
    const int steps = 5;
    
    for (int i = -steps; i <= steps; ++i) {
        float testRatio = baseRatio + (searchRange * i / steps);
        
        if (!checkRatioLimits(testRatio)) {
            continue;
        }
        
        float efficiency = calculateTheoreticalEfficiency(testRatio, perception);
        
        if (efficiency > bestEfficiency) {
            bestEfficiency = efficiency;
            bestRatio = testRatio;
        }
    }
    
    return bestRatio;
}

float CVTController::optimizeForTraction(float baseRatio, const PerceptionData& perception) const {
    float currentSlip = perception.vehicleState.wheelSlipRatio;
    float targetSlip = controlParams_.slipRatioTarget;
    
    // PID控制滑转率
    float slipError = currentSlip - targetSlip;
    float ratioAdjustment = -slipError * 0.5f; // 比例增益
    
    float optimizedRatio = baseRatio + ratioAdjustment;
    
    // 地形适应增益
    float terrainAdjustment = perception.terrainSlope * controlParams_.terrainAdaptationGain;
    optimizedRatio += terrainAdjustment;
    
    // 土壤阻力影响
    float soilEffect = perception.soilResistance / 5000.0f * 0.1f;
    optimizedRatio -= soilEffect;
    
    return optimizedRatio;
}

float CVTController::optimizeForComfort(float baseRatio, const PerceptionData& perception) const {
    float optimizedRatio = baseRatio;
    
    // 减少频繁变化
    if (!ratioHistory_.empty()) {
        float lastRatio = ratioHistory_.back();
        float change = optimizedRatio - lastRatio;
        
        // 限制变化率
        float maxChange = controlParams_.ratioChangeRate * 0.1f;
        if (std::abs(change) > maxChange) {
            optimizedRatio = lastRatio + std::copysign(maxChange, change);
        }
    }
    
    // 坡度补偿
    float gradeCompensation = -perception.terrainSlope * 0.2f;
    optimizedRatio += gradeCompensation;
    
    return optimizedRatio;
}

float CVTController::applyScenarioOptimization(float ratio, const PerceptionData& perception) const {
    switch (perception.vehicleState.driveMode) {
        case DriveMode::PLOWING:
            return calculatePlowingRatio(perception);
        case DriveMode::SEEDING:
            return calculateSeedingRatio(perception);
        case DriveMode::TRANSPORT:
            return calculateTransportRatio(perception);
        case DriveMode::ROAD:
            return calculateRoadRatio(perception);
        case DriveMode::FIELD:
            return calculateFieldRatio(perception);
        default:
            return ratio;
    }
}

float CVTController::calculatePlowingRatio(const PerceptionData& perception) const {
    float baseRatio = calculateBaseRatio(perception);
    
    // 犁地模式：较低传动比，更高扭矩
    float soilFactor = 1.0f - (0.2f * perception.soilResistance / 5000.0f);
    float draftFactor = 1.0f - (0.1f * perception.implementForce / 8000.0f);
    
    float optimizedRatio = baseRatio * soilFactor * draftFactor;
    
    return std::clamp(optimizedRatio, 0.5f, 2.2f);
}

float CVTController::calculateSeedingRatio(const PerceptionData& perception) const {
    float baseRatio = calculateBaseRatio(perception);
    
    // 播种模式：稳定传动比，精确速度控制
    if (!ratioHistory_.empty()) {
        float averageRatio = std::accumulate(ratioHistory_.begin(), ratioHistory_.end(), 0.0f) /
                           ratioHistory_.size();
        float smoothingFactor = 0.8f;
        baseRatio = smoothingFactor * averageRatio + (1.0f - smoothingFactor) * baseRatio;
    }
    
    // 速度稳定性补偿
    float speedVariance = calculateSpeedVariance();
    float stabilityCompensation = -speedVariance * 0.1f;
    
    return baseRatio + stabilityCompensation;
}

float CVTController::calculateTransportRatio(const PerceptionData& perception) const {
    float baseRatio = calculateBaseRatio(perception);
    
    // 运输模式：较高传动比，更高速度
    float speedFactor = 1.0f + (0.2f * perception.vehicleState.velocity.norm() / 20.0f);
    float optimizedRatio = baseRatio * speedFactor;
    
    // 道路坡度补偿
    float roadCompensation = -perception.terrainSlope * 0.15f;
    optimizedRatio += roadCompensation;
    
    return std::clamp(optimizedRatio, 0.8f, 3.0f);
}

float CVTController::applyPrediction(float ratio, const PerceptionData& perception,
                                   const PredictionResult& prediction) const {
    if (prediction.loadForecast.empty() || prediction.slopeProfile.empty()) {
        return ratio;
    }
    
    // 基于预测负载调整
    float predictedLoad = prediction.loadForecast[0];
    float loadAdjustment = (predictedLoad - perception.implementForce) / 10000.0f * 0.1f;
    
    // 基于预测坡度调整
    float predictedSlope = prediction.slopeProfile[0];
    float slopeAdjustment = -predictedSlope * 0.2f;
    
    return ratio + loadAdjustment + slopeAdjustment;
}

float CVTController::calculateTheoreticalEfficiency(float ratio, const PerceptionData& perception) const {
    // 传动效率模型
    float optimalRatio = 1.2f;
    float ratioEfficiency = 1.0f - 0.3f * std::abs(ratio - optimalRatio) / optimalRatio;
    
    // 负载效率影响
    float loadEfficiency = 1.0f - 0.2f * (perception.loadFactor / 0.8f);
    
    // 速度效率影响
    float speed = perception.vehicleState.velocity.norm();
    float speedEfficiency = 1.0f - 0.1f * std::abs(speed - 15.0f) / 15.0f;
    
    // 温度效率影响（简化）
    float tempEfficiency = 1.0f - 0.05f * (perception.vehicleState.motorTemperature - 80.0f) / 20.0f;
    
    return 0.92f * ratioEfficiency * loadEfficiency * speedEfficiency * tempEfficiency;
}

bool CVTController::checkRatioLimits(float ratio) const {
    return ratio >= controlParams_.minRatio && ratio <= controlParams_.maxRatio;
}

float CVTController::applyRateLimits(float targetRatio) const {
    if (currentState_.isShifting) {
        // 换挡过程中限制变化率
        float maxChange = controlParams_.ratioChangeRate * currentState_.shiftProgress;
        float current = currentState_.currentRatio;
        
        return current + std::clamp(targetRatio - current, -maxChange, maxChange);
    }
    
    // 正常情况下的速率限制
    float maxChange = controlParams_.ratioChangeRate;
    float current = currentState_.currentRatio;
    
    return current + std::clamp(targetRatio - current, -maxChange, maxChange);
}

void CVTController::updateShiftState(float newRatio) {
    float change = std::abs(newRatio - currentState_.currentRatio);
    
    if (change > 0.01f && !currentState_.isShifting) {
        // 开始换挡
        currentState_.isShifting = true;
        currentState_.targetRatio = newRatio;
        currentState_.shiftStartTime = getCurrentTime();
        currentState_.shiftProgress = 0.0f;
    }
    
    if (currentState_.isShifting) {
        // 更新换挡进度
        uint32_t currentTime = getCurrentTime();
        float elapsed = (currentTime - currentState_.shiftStartTime) / 1000.0f; // 转换为秒
        
        currentState_.shiftProgress = std::min(1.0f, elapsed / 0.5f); // 假设0.5秒完成换挡
        
        // 更新当前传动比
        currentState_.currentRatio = currentState_.currentRatio + 
            (currentState_.targetRatio - currentState_.currentRatio) * currentState_.shiftProgress;
        
        // 检查换挡是否完成
        if (currentState_.shiftProgress >= 1.0f) {
            completeShift();
        }
    }
}

void CVTController::completeShift() {
    currentState_.isShifting = false;
    currentState_.currentRatio = currentState_.targetRatio;
    currentState_.shiftProgress = 1.0f;
    currentState_.ratioRate = 0.0f;
}

uint32_t CVTController::getCurrentTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

float CVTController::calculateSpeedVariance() const {
    if (ratioHistory_.size() < 2) {
        return 0.0f;
    }
    
    // 计算速度变化的方差
    float sum = 0.0f;
    float sumSq = 0.0f;
    
    for (float ratio : ratioHistory_) {
        sum += ratio;
        sumSq += ratio * ratio;
    }
    
    float mean = sum / ratioHistory_.size();
    float variance = (sumSq / ratioHistory_.size()) - (mean * mean);
    
    return std::sqrt(variance);
}

float CVTController::weightedAverage(float a, float b, float weight) const {
    return a * (1.0f - weight) + b * weight;
}

bool CVTController::validatePerceptionData(const PerceptionData& perception) const {
    if (perception.vehicleState.velocity.norm() < 0.0f ||
        perception.vehicleState.velocity.norm() > 50.0f) {
        return false;
    }
    
    if (perception.vehicleState.actualTorque < -1000.0f ||
        perception.vehicleState.actualTorque > 1000.0f) {
        return false;
    }
    
    if (std::isnan(perception.terrainSlope) || std::isinf(perception.terrainSlope)) {
        return false;
    }
    
    return true;
}

void CVTController::updateControlParams(DriveMode mode) {
    if (modeParams_.find(mode) != modeParams_.end()) {
        controlParams_ = modeParams_[mode];
        std::cout << "CVT control parameters updated for mode: " << static_cast<int>(mode) << std::endl;
    }
}

void CVTController::adaptToManufacturer(CVTManufacturer manufacturer) {
    if (manufacturerParams_.find(manufacturer) != manufacturerParams_.end()) {
        controlParams_ = manufacturerParams_[manufacturer];
        currentManufacturer_ = manufacturer;
        std::cout << "CVT parameters adapted to manufacturer: " << static_cast<int>(manufacturer) << std::endl;
    }
}

void CVTController::learnFromExperience(float actualRatio, float actualEfficiency) {
    if (std::isnan(actualRatio) || std::isnan(actualEfficiency)) {
        return;
    }
    
    // 保存历史数据
    ratioHistory_.push_back(actualRatio);
    efficiencyHistory_.push_back(actualEfficiency);
    
    if (ratioHistory_.size() > maxHistorySize_) {
        ratioHistory_.pop_front();
    }
    if (efficiencyHistory_.size() > maxHistorySize_) {
        efficiencyHistory_.pop_front();
    }
    
    // 简单的学习：调整效率权重
    if (efficiencyHistory_.size() > 10) {
        float predictedEfficiency = calculateTheoreticalEfficiency(actualRatio, PerceptionData{});
        float error = actualEfficiency - predictedEfficiency;
        
        if (error > 0.05f) {
            controlParams_.efficiencyWeight *= 1.1f;
        } else if (error < -0.05f) {
            controlParams_.efficiencyWeight *= 0.9f;
        }
        
        // 权重归一化
        float total = controlParams_.efficiencyWeight + controlParams_.comfortWeight + 
                     controlParams_.responseWeight;
        
        controlParams_.efficiencyWeight /= total;
        controlParams_.comfortWeight /= total;
        controlParams_.responseWeight /= total;
    }
}

CVTState CVTController::getCurrentState() const {
    return currentState_;
}

float CVTController::getCurrentEfficiency() const {
    if (efficiencyHistory_.empty()) {
        return 0.0f;
    }
    return efficiencyHistory_.back();
}

bool CVTController::isShifting() const {
    return currentState_.isShifting;
}

} // namespace VCUCore