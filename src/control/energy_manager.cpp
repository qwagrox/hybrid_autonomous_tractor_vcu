// src/control/energy_manager.cpp
#include "energy_manager.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

namespace VCUCore {

EnergyManager::EnergyManager(uint32_t historySize) 
    : maxHistorySize_(historySize), currentFuelCost_(0.8f), currentElectricityCost_(0.12f) {
    
    initializeOptimizationParams();
    
    // 初始化模型
    batteryModel_ = std::make_unique<BatteryModel>();
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    
    // 初始化每日能量模式（24小时）
    std::fill(dailyEnergyPattern_.begin(), dailyEnergyPattern_.end(), 0.5f);
    std::fill(weeklyEnergyPattern_.begin(), weeklyEnergyPattern_.end(), 0.5f);
}

void EnergyManager::initializeOptimizationParams() {
    params_ = {
        .batterySOCMin = 0.2f,
        .batterySOCMax = 0.9f,
        .batterySOCOptimal = 0.7f,
        .engineEfficiencyWeight = 0.4f,
        .motorEfficiencyWeight = 0.3f,
        .batteryHealthWeight = 0.2f,
        .fuelCostWeight = 0.6f,
        .electricityCostWeight = 0.4f,
        .predictionConfidenceWeight = 0.8f,
        .emergencyPowerReserve = 10.0f // kW
    };
}

EnergyOptimization EnergyManager::optimizeEnergyUsage(const ControlCommands& currentCommands,
                                                    const PerceptionData& perception,
                                                    const PredictionResult& prediction) {
    
    EnergyOptimization optimization;
    
    try {
        // 获取当前能量状态
        EnergyState currentState;
        currentState.batterySOC = perception.vehicleState.batterySOC;
        currentState.fuelLevel = perception.vehicleState.fuelLevel;
        currentState.powerDemand = perception.vehicleState.powerConsumption;
        currentState.timestamp = perception.timestamp;
        
        // 生成能量预测
        EnergyForecast forecast = generateEnergyForecast(perception);
        
        // 解决优化问题
        optimization = solveOptimizationProblem(currentState, forecast);
        
        // 应用安全限制
        if (!checkEnergyLimits(optimization.powerFlow)) {
            optimization.powerFlow = applyEnergyLimits(optimization.powerFlow);
            optimization.limitsApplied = true;
        }
        
        // 更新历史记录
        updateEnergyHistory(currentState);
        
    } catch (const std::exception& e) {
        std::cerr << "Energy optimization failed: " << e.what() << std::endl;
        // 返回保守的默认策略
        optimization.powerFlow = calculateOptimalPowerFlow(
            perception.vehicleState.powerConsumption,
            perception.vehicleState.batterySOC,
            perception.vehicleState.fuelLevel,
            perception.ambientTemperature
        );
        optimization.optimizationSuccessful = false;
    }
    
    return optimization;
}

PowerFlow EnergyManager::calculateOptimalPowerFlow(float powerDemand, float batterySOC,
                                                 float fuelLevel, float ambientTemp) {
    
    PowerFlow flow;
    
    // 基于SOC的策略选择
    if (batterySOC > params_.batterySOCOptimal + 0.1f) {
        // 高SOC：优先使用电力
        flow.motorPower = std::min(powerDemand, motorModel_->getMaxPower());
        flow.enginePower = std::max(0.0f, powerDemand - flow.motorPower);
        flow.batteryPower = -flow.motorPower; // 放电
        flow.chargingPower = 0.0f;
        
    } else if (batterySOC < params_.batterySOCOptimal - 0.1f) {
        // 低SOC：优先使用发动机，同时充电
        flow.enginePower = std::min(powerDemand + 20.0f, engineModel_->getMaxPower()); // 额外功率用于充电
        flow.motorPower = std::max(0.0f, powerDemand - flow.enginePower);
        flow.batteryPower = flow.enginePower - powerDemand; // 充电功率
        flow.chargingPower = flow.batteryPower;
        
    } else {
        // 最优SOC范围：混合模式
        float engineRatio = 0.6f; // 发动机承担60%负载
        flow.enginePower = powerDemand * engineRatio;
        flow.motorPower = powerDemand * (1.0f - engineRatio);
        flow.batteryPower = -flow.motorPower;
        flow.chargingPower = 0.0f;
    }
    
    // 考虑温度影响
    if (ambientTemp < 0.0f) {
        // 低温环境：增加发动机使用以提供热量
        flow.enginePower = std::min(flow.enginePower + 10.0f, engineModel_->getMaxPower());
        flow.motorPower = std::max(0.0f, powerDemand - flow.enginePower);
    }
    
    return flow;
}

EnergyOptimization EnergyManager::solveOptimizationProblem(const EnergyState& currentState,
                                                         const EnergyForecast& forecast) {
    
    EnergyOptimization optimization;
    
    // 计算基础功率流
    PowerFlow baseFlow = calculateOptimalPowerFlow(
        currentState.powerDemand,
        currentState.batterySOC,
        currentState.fuelLevel,
        25.0f // 默认环境温度
    );
    
    // 多目标优化
    std::vector<PowerFlow> candidateFlows;
    candidateFlows.push_back(optimizeForCost(baseFlow));
    candidateFlows.push_back(optimizeForEfficiency(baseFlow));
    candidateFlows.push_back(optimizeForBatteryLife(baseFlow));
    
    // 选择最佳方案（加权评分）
    float bestScore = -std::numeric_limits<float>::max();
    PowerFlow bestFlow = baseFlow;
    
    for (const auto& flow : candidateFlows) {
        float costScore = calculateOperatingCost(flow) * params_.fuelCostWeight;
        float efficiencyScore = (flow.enginePower * 0.4f + flow.motorPower * 0.3f) * params_.engineEfficiencyWeight;
        float batteryScore = calculateBatteryHealthScore(flow.batteryPower, currentState.batterySOC, 25.0f) * 
                           params_.batteryHealthWeight;
        
        float totalScore = costScore + efficiencyScore + batteryScore;
        
        if (totalScore > bestScore) {
            bestScore = totalScore;
            bestFlow = flow;
        }
    }
    
    optimization.powerFlow = bestFlow;
    optimization.optimizationScore = bestScore;
    optimization.optimizationSuccessful = true;
    
    return optimization;
}

float EnergyManager::calculateOperatingCost(const PowerFlow& flow) const {
    float fuelCost = flow.enginePower * 0.08f * currentFuelCost_; // 假设0.08 L/kWh
    float electricityCost = std::max(0.0f, flow.chargingPower) * currentElectricityCost_;
    float regenerationCredit = std::min(0.0f, flow.batteryPower) * currentElectricityCost_ * -0.8f; // 回馈能量价值
    
    return fuelCost + electricityCost + regenerationCredit;
}

bool EnergyManager::checkEnergyLimits(const PowerFlow& flow) const {
    // 检查发动机限制
    if (flow.enginePower > engineModel_->getMaxPower() * 1.05f) {
        return false;
    }
    
    // 检查电机限制
    if (flow.motorPower > motorModel_->getMaxPower() * 1.05f) {
        return false;
    }
    
    // 检查电池限制
    if (std::abs(flow.batteryPower) > batteryModel_->getMaxPower() * 1.05f) {
        return false;
    }
    
    // 检查总功率平衡
    if (std::abs(flow.enginePower + flow.motorPower - flow.batteryPower - flow.chargingPower) > 1.0f) {
        return false;
    }
    
    return true;
}

PowerFlow EnergyManager::applyEnergyLimits(const PowerFlow& flow) const {
    PowerFlow limitedFlow = flow;
    
    // 应用发动机限制
    limitedFlow.enginePower = std::min(limitedFlow.enginePower, engineModel_->getMaxPower());
    
    // 应用电机限制
    limitedFlow.motorPower = std::min(limitedFlow.motorPower, motorModel_->getMaxPower());
    
    // 应用电池限制
    float maxBatteryPower = batteryModel_->getMaxPower();
    limitedFlow.batteryPower = std::clamp(limitedFlow.batteryPower, -maxBatteryPower, maxBatteryPower);
    limitedFlow.chargingPower = std::clamp(limitedFlow.chargingPower, 0.0f, maxBatteryPower);
    
    // 确保功率平衡
    float totalGeneration = limitedFlow.enginePower + limitedFlow.motorPower;
    float totalConsumption = std::abs(limitedFlow.batteryPower) + limitedFlow.chargingPower;
    
    if (totalGeneration > totalConsumption) {
        // 减少发电
        float scale = totalConsumption / totalGeneration;
        limitedFlow.enginePower *= scale;
        limitedFlow.motorPower *= scale;
    }
    
    return limitedFlow;
}

void EnergyManager::updateEnergyHistory(const EnergyState& state) {
    energyHistory_.push_back(state);
    if (energyHistory_.size() > maxHistorySize_) {
        energyHistory_.pop_front();
    }
    
    // 更新每日模式
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm* localTime = std::localtime(&time);
    int hour = localTime->tm_hour;
    
    dailyEnergyPattern_[hour] = 0.9f * dailyEnergyPattern_[hour] + 0.1f * (state.powerDemand / 100.0f);
}

EnergyForecast EnergyManager::generateEnergyForecast(const PerceptionData& perception) const {
    EnergyForecast forecast;
    
    // 基于历史模式的简单预测
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm* localTime = std::localtime(&time);
    int currentHour = localTime->tm_hour;
    
    for (int i = 0; i < 24; ++i) {
        int forecastHour = (currentHour + i) % 24;
        forecast.hourlyDemand[i] = dailyEnergyPattern_[forecastHour] * 100.0f; // 转换为kW
    }
    
    forecast.confidence = 0.7f;
    forecast.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    return forecast;
}

} // namespace VCUCore