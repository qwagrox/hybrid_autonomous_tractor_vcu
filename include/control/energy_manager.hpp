// include/control/energy_manager.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "models/battery_model.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include <memory>
#include <deque>

namespace VCUCore {

class EnergyManager {
private:
    struct EnergyOptimizationParams {
        float batterySOCMin;
        float batterySOCMax;
        float batterySOCOptimal;
        float engineEfficiencyWeight;
        float motorEfficiencyWeight;
        float batteryHealthWeight;
        float fuelCostWeight;
        float electricityCostWeight;
        float predictionConfidenceWeight;
        float emergencyPowerReserve;
    };

    EnergyOptimizationParams params_;
    std::unique_ptr<BatteryModel> batteryModel_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    
    std::deque<EnergyState> energyHistory_;
    std::deque<OptimizationResult> optimizationHistory_;
    
    uint32_t maxHistorySize_;
    float currentFuelCost_;
    float currentElectricityCost_;
    
    // 学习参数
    std::array<float, 24> dailyEnergyPattern_;
    std::array<float, 7> weeklyEnergyPattern_;

public:
    EnergyManager(uint32_t historySize = 1000);
    
    EnergyOptimization optimizeEnergyUsage(const ControlCommands& currentCommands,
                                         const PerceptionData& perception,
                                         const PredictionResult& prediction);
    
    PowerFlow calculateOptimalPowerFlow(float powerDemand, float batterySOC,
                                      float fuelLevel, float ambientTemp);
    
    ChargingStrategy determineChargingStrategy(float batterySOC, float powerDemand,
                                             const PredictionResult& prediction);
    
    // 成本优化
    float calculateOperatingCost(const PowerFlow& flow) const;
    void updateCostParameters(float fuelCost, float electricityCost);
    
    // 预测集成
    void integrateEnergyPrediction(const PredictionResult& prediction);
    void updateOptimizationHistory(const EnergyOptimization& result);
    EnergyForecast generateEnergyForecast(const PerceptionData& perception) const;
    
    // 学习功能
    void learnFromHistoricalData(const std::deque<EnergyState>& history);
    void updateEnergyPatterns();
    
    // 限制管理
    bool checkEnergyLimits(const PowerFlow& flow) const;
    PowerFlow applyEnergyLimits(const PowerFlow& flow) const;

private:
    void initializeOptimizationParams();
    EnergyOptimization solveOptimizationProblem(const EnergyState& currentState,
                                              const EnergyForecast& forecast);
    
    float calculateEngineEfficiencyScore(float torque, float rpm) const;
    float calculateMotorEfficiencyScore(float torque, float rpm, float batterySOC) const;
    float calculateBatteryHealthScore(float power, float soc, float temp) const;
    
    PowerFlow optimizeForCost(const PowerFlow& baseFlow) const;
    PowerFlow optimizeForEfficiency(const PowerFlow& baseFlow) const;
    PowerFlow optimizeForBatteryLife(const PowerFlow& baseFlow) const;
    
    void updateEnergyHistory(const EnergyState& state);
    EnergyState calculateAverageEnergyState() const;
};

} // namespace VCUCore