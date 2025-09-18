// include/models/motor_model.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>

namespace VCUCore {

class MotorModel {
private:
    // 电机效率映射表
    Eigen::MatrixXf efficiencyMap_;      // 效率映射表 [torque][rpm]
    Eigen::MatrixXf torqueMap_;          // 扭矩能力映射表 [rpm][voltage]
    Eigen::MatrixXf lossMap_;            // 损失映射表 [torque][rpm]
    
    // 电机参数
    struct MotorParameters {
        float ratedPower;            // 额定功率 (kW)
        float peakPower;             // 峰值功率 (kW)
        float ratedTorque;           // 额定扭矩 (Nm)
        float peakTorque;            // 峰值扭矩 (Nm)
        float maxSpeed;              // 最高转速 (rpm)
        float baseSpeed;             // 基速 (rpm)
        float statorResistance;      // 定子电阻 (Ω)
        float rotorResistance;       // 转子电阻 (Ω)
        float inductance;            // 电感 (H)
        float backEMFConstant;       // 反电动势常数 (V/rad/s)
        float thermalResistance;     // 热阻 (K/W)
        float thermalCapacitance;    // 热容 (J/K)
        float mechanicalLosses;      // 机械损失 (Nm)
    };
    
    MotorParameters params_;
    
    // 动态状态
    struct MotorDynamicState {
        float currentTorque;
        float currentRpm;
        float currentVoltage;
        float currentCurrent;
        float currentEfficiency;
        float windingTemperature;
        float magnetTemperature;
        float coolantTemperature;
        float accumulatedEnergy;
        float accumulatedLosses;
        uint32_t operatingHours;
    };
    
    MotorDynamicState currentState_;
    
    // 温度模型参数
    struct ThermalModel {
        float windingTempBase;
        float magnetTempBase;
        float coolantTempBase;
        float windingTempGain;
        float magnetTempGain;
        float coolantTempGain;
        float ambientTempInfluence;
    };
    
    ThermalModel thermalParams_;
    
    // 电池影响参数
    struct BatteryInfluence {
        float voltageEffect;
        float socEffect;
        float temperatureEffect;
        float internalResistanceEffect;
    };
    
    BatteryInfluence batteryParams_;

public:
    MotorModel();
    
    bool loadMotorMap(const std::string& mapFile);
    bool loadMotorParameters(const std::string& paramFile);
    
    // 电机特性计算
    float calculateTorque(float current, float rpm, float batteryVoltage);
    float calculateCurrent(float torque, float rpm, float batteryVoltage);
    float calculateEfficiency(float torque, float rpm, float batteryVoltage, float batterySOC);
    float calculateLosses(float torque, float rpm, float batteryVoltage);
    
    // 温度模型
    void updateTemperatureModel(float deltaTime, float ambientTemp, 
                               float coolantFlowRate, float currentLoad);
    float calculateWindingTemperature(float copperLosses, float coolantTemp);
    float calculateMagnetTemperature(float eddyLosses, float coolantTemp);
    float calculateCoolantTemperature(float totalLosses, float flowRate);
    
    // 电池影响计算
    float calculateVoltageEffect(float batteryVoltage, float nominalVoltage);
    float calculateSOCEffect(float batterySOC);
    float calculateTemperatureEffect(float batteryTemp);
    
    // 状态更新
    void updateMotorState(float torque, float rpm, float batteryVoltage, 
                         float batterySOC, float batteryTemp, float deltaTime,
                         float ambientTemp, float coolantFlowRate);
    
    // 查询函数
    float getCurrentEfficiency() const { return currentState_.currentEfficiency; }
    float getWindingTemperature() const { return currentState_.windingTemperature; }
    float getCurrentCurrent() const { return currentState_.currentCurrent; }
    
    // 性能分析
    float calculatePowerLoss(float torque, float rpm) const;
    float calculateThermalLimit(float currentTemp, float maxTemp) const;
    float calculateVoltageRequirement(float torque, float rpm) const;
    
    // 限制和保护
    bool checkOperatingLimits(float torque, float rpm, float batteryVoltage, 
                             float windingTemp) const;
    float calculateDerateFactor(float windingTemp, float magnetTemp) const;

private:
    void initializeDefaultParameters();
    void initializeThermalModel();
    void initializeBatteryInfluence();
    
    float interpolateMap(const Eigen::MatrixXf& map, float x, float y) const;
    void updateEfficiencyMap();
    
    // 物理计算
    float calculateCopperLosses(float current) const;
    float calculateIronLosses(float rpm, float flux) const;
    float calculateMechanicalLosses(float rpm) const;
    float calculateStrayLosses(float torque, float rpm) const;
    
    // 电磁计算
    float calculateBackEMF(float rpm) const;
    float calculateFluxLinkage(float current) const;
    float calculateTorqueConstant(float current) const;
    
    // 数据验证
    bool validateMotorMap() const;
    bool validateOperatingPoint(float torque, float rpm) const;
};

} // namespace VCUCore