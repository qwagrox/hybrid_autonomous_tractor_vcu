// include/models/engine_model.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>

namespace VCUCore {

class EngineModel {
private:
    // 发动机万有特性数据
    Eigen::MatrixXf torqueMap_;          // 扭矩映射表 [rpm][fuel]
    Eigen::MatrixXf fuelConsumptionMap_; // 燃油消耗率映射表 [rpm][torque]
    Eigen::MatrixXf efficiencyMap_;      // 效率映射表 [rpm][torque]
    Eigen::MatrixXf emissionMap_;        // 排放映射表 [rpm][torque]
    
    // 发动机参数
    struct EngineParameters {
        float displacement;          // 排量 (L)
        float compressionRatio;      // 压缩比
        float maxTorque;             // 最大扭矩 (Nm)
        float maxPower;              // 最大功率 (kW)
        float idleSpeed;             // 怠速转速 (rpm)
        float ratedSpeed;            // 额定转速 (rpm)
        float peakEfficiencyRpm;     // 峰值效率转速 (rpm)
        float peakEfficiencyTorque;  // 峰值效率扭矩 (Nm)
        float mechanicalLosses;      // 机械损失 (Nm)
        float thermalMass;           // 热质量 (J/K)
        float heatTransferCoeff;     // 传热系数 (W/K)
    };
    
    EngineParameters params_;
    
    // 动态状态
    struct EngineDynamicState {
        float currentRpm;
        float currentTorque;
        float currentFuelRate;
        float currentEfficiency;
        float coolantTemperature;
        float oilTemperature;
        float exhaustTemperature;
        float accumulatedFuel;
        float accumulatedWork;
        uint32_t operatingHours;
    };
    
    EngineDynamicState currentState_;
    
    // 温度模型参数
    struct ThermalModel {
        float coolantTempBase;
        float oilTempBase;
        float exhaustTempBase;
        float coolantTempGain;
        float oilTempGain;
        float exhaustTempGain;
        float ambientTempInfluence;
    };
    
    ThermalModel thermalParams_;
    
    // 排放模型参数
    struct EmissionModel {
        float noxBaseRate;
        float pmBaseRate;
        float coBaseRate;
        float hcBaseRate;
        float noxLoadFactor;
        float pmLoadFactor;
        float coLoadFactor;
        float hcLoadFactor;
        float noxTempFactor;
        float pmTempFactor;
    };
    
    EmissionModel emissionParams_;

public:
    EngineModel();
    
    bool loadEngineMap(const std::string& mapFile);
    bool loadEngineParameters(const std::string& paramFile);
    
    // 发动机动力学计算
    float calculateTorque(float fuelRate, float rpm, float coolantTemp);
    float calculateFuelConsumption(float torque, float rpm, float coolantTemp);
    float calculateEfficiency(float torque, float rpm);
    float calculateMechanicalLosses(float rpm, float oilTemp);
    
    // 温度模型
    void updateTemperatureModel(float deltaTime, float ambientTemp, 
                               float currentLoad, float coolantFlowRate);
    float calculateCoolantTemperature(float heatInput, float flowRate);
    float calculateOilTemperature(float frictionWork, float rpm);
    float calculateExhaustTemperature(float fuelRate, float airFuelRatio);
    
    // 排放计算
    Emissions calculateEmissions(float torque, float rpm, float exhaustTemp);
    float calculateNOxEmission(float load, float rpm, float exhaustTemp);
    float calculatePMEmission(float load, float rpm, float exhaustTemp);
    float calculateCOEmission(float load, float rpm, float airFuelRatio);
    float calculateHCEmission(float load, float rpm, float airFuelRatio);
    
    // 状态更新
    void updateEngineState(float fuelRate, float rpm, float deltaTime, 
                          float ambientTemp, float coolantFlowRate);
    
    // 查询函数
    float getCurrentEfficiency() const { return currentState_.currentEfficiency; }
    float getCurrentFuelRate() const { return currentState_.currentFuelRate; }
    float getCurrentTorque() const { return currentState_.currentTorque; }
    float getCoolantTemperature() const { return currentState_.coolantTemperature; }
    
    // 性能分析
    float calculateBSFC(float torque, float rpm) const;
    float calculateBMEP(float torque) const;
    float calculateVolumetricEfficiency(float rpm, float boostPressure) const;
    
    // 限制和保护
    bool checkOperatingLimits(float torque, float rpm, float coolantTemp) const;
    float calculateDerateFactor(float coolantTemp, float oilTemp) const;

private:
    void initializeDefaultParameters();
    void initializeThermalModel();
    void initializeEmissionModel();
    
    float interpolateMap(const Eigen::MatrixXf& map, float x, float y) const;
    void updateEfficiencyMap();
    
    // 物理计算
    float calculateIndicatedWork(float fuelEnergy) const;
    float calculateFrictionWork(float rpm, float oilTemp) const;
    float calculateHeatRejection(float fuelEnergy, float efficiency) const;
    
    // 数据验证
    bool validateEngineMap() const;
    bool validateOperatingPoint(float torque, float rpm) const;
};

struct Emissions {
    float nox;      // 氮氧化物 (g/kWh)
    float pm;       // 颗粒物 (g/kWh)
    float co;       // 一氧化碳 (g/kWh)
    float hc;       // 碳氢化合物 (g/kWh)
    float co2;      // 二氧化碳 (g/kWh)
};

} // namespace VCUCore