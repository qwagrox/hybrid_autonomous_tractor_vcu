// src/models/engine_model.cpp
#include "engine_model.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace VCUCore {

EngineModel::EngineModel() {
    initializeDefaultParameters();
    initializeThermalModel();
    initializeEmissionModel();
    
    // 初始化映射表
    torqueMap_ = Eigen::MatrixXf::Zero(100, 100);
    fuelConsumptionMap_ = Eigen::MatrixXf::Zero(100, 100);
    efficiencyMap_ = Eigen::MatrixXf::Zero(100, 100);
    emissionMap_ = Eigen::MatrixXf::Zero(100, 100);
    
    // 初始化状态
    currentState_ = {
        .currentRpm = 800.0f,
        .currentTorque = 0.0f,
        .currentFuelRate = 0.0f,
        .currentEfficiency = 0.0f,
        .coolantTemperature = 90.0f,
        .oilTemperature = 85.0f,
        .exhaustTemperature = 400.0f,
        .accumulatedFuel = 0.0f,
        .accumulatedWork = 0.0f,
        .operatingHours = 0
    };
}

void EngineModel::initializeDefaultParameters() {
    // 典型的6缸柴油发动机参数
    params_ = {
        .displacement = 6.7f,           // 6.7L
        .compressionRatio = 17.0f,      // 17:1
        .maxTorque = 600.0f,            // 600 Nm
        .maxPower = 235.0f,             // 235 kW
        .idleSpeed = 800.0f,            // 800 rpm
        .ratedSpeed = 2200.0f,          // 2200 rpm
        .peakEfficiencyRpm = 1800.0f,   // 1800 rpm
        .peakEfficiencyTorque = 300.0f, // 300 Nm
        .mechanicalLosses = 15.0f,      // 15 Nm
        .thermalMass = 50000.0f,        // 50 kJ/K
        .heatTransferCoeff = 1000.0f    // 1 kW/K
    };
}

void EngineModel::initializeThermalModel() {
    thermalParams_ = {
        .coolantTempBase = 90.0f,
        .oilTempBase = 85.0f,
        .exhaustTempBase = 400.0f,
        .coolantTempGain = 0.05f,       // °C per % load
        .oilTempGain = 0.03f,           // °C per % load
        .exhaustTempGain = 2.0f,        // °C per % load
        .ambientTempInfluence = 0.3f     // 30% ambient influence
    };
}

void EngineModel::initializeEmissionModel() {
    emissionParams_ = {
        .noxBaseRate = 3.5f,            // g/kWh
        .pmBaseRate = 0.02f,            // g/kWh
        .coBaseRate = 1.5f,             // g/kWh
        .hcBaseRate = 0.15f,            // g/kWh
        .noxLoadFactor = 0.8f,
        .pmLoadFactor = 1.2f,
        .coLoadFactor = 0.7f,
        .hcLoadFactor = 0.6f,
        .noxTempFactor = 0.005f,        // per °C
        .pmTempFactor = -0.003f         // per °C
    };
}

bool EngineModel::loadEngineMap(const std::string& mapFile) {
    std::ifstream file(mapFile);
    if (!file.is_open()) {
        std::cerr << "Error opening engine map file: " << mapFile << std::endl;
        return false;
    }
    
    try {
        std::string line;
        int row = 0;
        
        while (std::getline(file, line) && row < torqueMap_.rows()) {
            std::istringstream iss(line);
            int col = 0;
            float value;
            
            while (iss >> value && col < torqueMap_.cols()) {
                torqueMap_(row, col) = value;
                col++;
            }
            row++;
        }
        
        // 验证映射表数据
        if (!validateEngineMap()) {
            std::cerr << "Engine map validation failed" << std::endl;
            return false;
        }
        
        // 更新效率映射表
        updateEfficiencyMap();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing engine map: " << e.what() << std::endl;
        return false;
    }
}

bool EngineModel::loadEngineParameters(const std::string& paramFile) {
    std::ifstream file(paramFile);
    if (!file.is_open()) {
        std::cerr << "Error opening engine parameter file: " << paramFile << std::endl;
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            float value;
            
            if (iss >> key >> value) {
                if (key == "displacement") params_.displacement = value;
                else if (key == "compression_ratio") params_.compressionRatio = value;
                else if (key == "max_torque") params_.maxTorque = value;
                else if (key == "max_power") params_.maxPower = value;
                else if (key == "idle_speed") params_.idleSpeed = value;
                else if (key == "rated_speed") params_.ratedSpeed = value;
                else if (key == "mechanical_losses") params_.mechanicalLosses = value;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing engine parameters: " << e.what() << std::endl;
        return false;
    }
}

float EngineModel::calculateTorque(float fuelRate, float rpm, float coolantTemp) {
    if (!validateOperatingPoint(0, rpm)) {
        return 0.0f;
    }
    
    // 计算指示扭矩（基于燃油能量）
    float fuelEnergy = fuelRate * 42.5f; // 柴油热值 ~42.5 MJ/kg
    float indicatedWork = calculateIndicatedWork(fuelEnergy);
    
    // 计算摩擦损失
    float frictionLoss = calculateMechanicalLosses(rpm, currentState_.oilTemperature);
    
    // 计算净扭矩
    float netTorque = indicatedWork - frictionLoss;
    
    // 温度降额
    float derateFactor = calculateDerateFactor(coolantTemp, currentState_.oilTemperature);
    netTorque *= derateFactor;
    
    // 应用扭矩限制
    netTorque = std::clamp(netTorque, 0.0f, params_.maxTorque);
    
    return netTorque;
}

float EngineModel::calculateFuelConsumption(float torque, float rpm, float coolantTemp) {
    if (!validateOperatingPoint(torque, rpm)) {
        return 0.0f;
    }
    
    // 从映射表插值获取BSFC
    float bsfc = interpolateMap(fuelConsumptionMap_, rpm, torque);
    if (bsfc <= 0.0f) {
        // 计算理论BSFC
        float efficiency = calculateEfficiency(torque, rpm);
        if (efficiency > 0.0f) {
            bsfc = 3600.0f / (42.5f * efficiency); // g/kWh
        } else {
            bsfc = 250.0f; // 默认值
        }
    }
    
    // 计算功率 (kW)
    float power = (torque * rpm * 2.0f * M_PI) / 60000.0f;
    
    // 计算燃油消耗率 (g/h)
    float fuelRate = (bsfc * power) / 1000.0f;
    
    // 温度影响
    float derateFactor = calculateDerateFactor(coolantTemp, currentState_.oilTemperature);
    fuelRate /= derateFactor;
    
    return fuelRate;
}

float EngineModel::calculateEfficiency(float torque, float rpm) {
    // 从效率映射表插值
    float efficiency = interpolateMap(efficiencyMap_, rpm, torque);
    
    if (efficiency <= 0.0f) {
        // 计算理论效率
        float indicatedWork = torque + calculateMechanicalLosses(rpm, currentState_.oilTemperature);
        float fuelEnergy = indicatedWork * (2.0f * M_PI * rpm) / 60000.0f;
        
        if (fuelEnergy > 0.0f) {
            efficiency = (torque * (2.0f * M_PI * rpm) / 60000.0f) / fuelEnergy;
        }
    }
    
    return std::clamp(efficiency, 0.0f, 0.45f); // 柴油机最大效率约45%
}

float EngineModel::calculateMechanicalLosses(float rpm, float oilTemp) {
    // 机械损失模型：泵气损失 + 摩擦损失 + 附件损失
    float pumpingLoss = 0.1f * params_.maxTorque * std::pow(rpm / params_.ratedSpeed, 2.0f);
    float frictionLoss = params_.mechanicalLosses * (rpm / params_.idleSpeed) * 
                        (1.0f + 0.002f * (oilTemp - 85.0f));
    float accessoryLoss = 0.05f * params_.maxTorque * (rpm / params_.ratedSpeed);
    
    return pumpingLoss + frictionLoss + accessoryLoss;
}

void EngineModel::updateTemperatureModel(float deltaTime, float ambientTemp, 
                                       float currentLoad, float coolantFlowRate) {
    // 计算热量输入
    float fuelEnergy = currentState_.currentFuelRate * 42.5f * 1000.0f; // kJ/h
    float heatInput = fuelEnergy * (1.0f - currentState_.currentEfficiency);
    
    // 更新冷却液温度
    currentState_.coolantTemperature = calculateCoolantTemperature(heatInput, coolantFlowRate);
    
    // 更新机油温度
    float frictionWork = calculateMechanicalLosses(currentState_.currentRpm, 
                                                 currentState_.oilTemperature);
    currentState_.oilTemperature = calculateOilTemperature(frictionWork, currentState_.currentRpm);
    
    // 更新排气温度
    float airFuelRatio = 18.0f; // 典型柴油机空燃比
    currentState_.exhaustTemperature = calculateExhaustTemperature(currentState_.currentFuelRate, 
                                                                 airFuelRatio);
    
    // 环境温度影响
    currentState_.coolantTemperature += thermalParams_.ambientTempInfluence * 
                                      (ambientTemp - 25.0f);
    currentState_.oilTemperature += thermalParams_.ambientTempInfluence * 
                                  (ambientTemp - 25.0f);
}

float EngineModel::calculateCoolantTemperature(float heatInput, float flowRate) {
    // 冷却系统热平衡方程
    float heatTransfer = thermalParams_.heatTransferCoeff * 
                       (currentState_.coolantTemperature - thermalParams_.coolantTempBase);
    float tempChange = (heatInput - heatTransfer) / params_.thermalMass;
    
    return currentState_.coolantTemperature + tempChange;
}

float EngineModel::calculateOilTemperature(float frictionWork, float rpm) {
    // 机油温度模型
    float heatGeneration = frictionWork * rpm * 2.0f * M_PI / 60000.0f; // kW
    float heatDissipation = 0.5f * (currentState_.oilTemperature - thermalParams_.oilTempBase);
    
    float tempChange = (heatGeneration - heatDissipation) / (params_.thermalMass * 0.1f);
    
    return currentState_.oilTemperature + tempChange;
}

float EngineModel::calculateExhaustTemperature(float fuelRate, float airFuelRatio) {
    // 排气温度模型
    float stoichiometricTemp = 800.0f; // 理论燃烧温度
    float excessAirFactor = airFuelRatio / 14.7f; // 过量空气系数
    
    float exhaustTemp = stoichiometricTemp * std::exp(-0.2f * (excessAirFactor - 1.0f));
    exhaustTemp += thermalParams_.exhaustTempGain * (currentState_.currentTorque / params_.maxTorque);
    
    return exhaustTemp;
}

Emissions EngineModel::calculateEmissions(float torque, float rpm, float exhaustTemp) {
    Emissions emissions;
    
    float load = torque / params_.maxTorque;
    
    emissions.nox = calculateNOxEmission(load, rpm, exhaustTemp);
    emissions.pm = calculatePMEmission(load, rpm, exhaustTemp);
    emissions.co = calculateCOEmission(load, rpm, 18.0f);
    emissions.hc = calculateHCEmission(load, rpm, 18.0f);
    emissions.co2 = 3.17f * currentState_.currentFuelRate; // 柴油CO2排放系数
    
    return emissions;
}

float EngineModel::calculateNOxEmission(float load, float rpm, float exhaustTemp) {
    // NOx排放模型
    float baseNOx = emissionParams_.noxBaseRate;
    float loadEffect = emissionParams_.noxLoadFactor * load;
    float tempEffect = emissionParams_.noxTempFactor * (exhaustTemp - 400.0f);
    float rpmEffect = 0.5f * (rpm - 1800.0f) / 1000.0f;
    
    return baseNOx * (1.0f + loadEffect + tempEffect + rpmEffect);
}

float EngineModel::calculatePMEmission(float load, float rpm, float exhaustTemp) {
    // PM排放模型
    float basePM = emissionParams_.pmBaseRate;
    float loadEffect = emissionParams_.pmLoadFactor * load;
    float tempEffect = emissionParams_.pmTempFactor * (exhaustTemp - 400.0f);
    
    return basePM * (1.0f + loadEffect + tempEffect);
}

float EngineModel::interpolateMap(const Eigen::MatrixXf& map, float x, float y) const {
    // 双线性插值
    int x1 = static_cast<int>(x);
    int x2 = x1 + 1;
    int y1 = static_cast<int>(y);
    int y2 = y1 + 1;
    
    if (x1 < 0 || x2 >= map.rows() || y1 < 0 || y2 >= map.cols()) {
        return 0.0f;
    }
    
    float q11 = map(x1, y1);
    float q12 = map(x1, y2);
    float q21 = map(x2, y1);
    float q22 = map(x2, y2);
    
    float dx = x - x1;
    float dy = y - y1;
    
    return (1.0f - dx) * (1.0f - dy) * q11 +
           (1.0f - dx) * dy * q12 +
           dx * (1.0f - dy) * q21 +
           dx * dy * q22;
}

bool EngineModel::validateEngineMap() const {
    // 检查映射表数据有效性
    if (torqueMap_.rows() == 0 || torqueMap_.cols() == 0) {
        return false;
    }
    
    if (torqueMap_.minCoeff() < 0.0f || fuelConsumptionMap_.minCoeff() < 0.0f) {
        return false;
    }
    
    return true;
}

bool EngineModel::validateOperatingPoint(float torque, float rpm) const {
    if (rpm < params_.idleSpeed || rpm > params_.ratedSpeed * 1.1f) {
        return false;
    }
    
    if (torque < 0.0f || torque > params_.maxTorque * 1.1f) {
        return false;
    }
    
    return true;
}

float EngineModel::calculateBSFC(float torque, float rpm) const {
    float efficiency = calculateEfficiency(torque, rpm);
    if (efficiency > 0.0f) {
        return 3600.0f / (42.5f * efficiency); // g/kWh
    }
    return 0.0f;
}

float EngineModel::calculateBMEP(float torque) const {
    // 制动平均有效压力 (bar)
    return (torque * 4.0f * M_PI) / (params_.displacement * 1e5f);
}

float EngineModel::calculateVolumetricEfficiency(float rpm, float boostPressure) const {
    // 容积效率模型
    float veBase = 0.85f;
    float rpmEffect = -0.2f * (rpm - 1800.0f) / 1000.0f;
    float boostEffect = 0.1f * (boostPressure - 1.0f);
    
    return std::clamp(veBase + rpmEffect + boostEffect, 0.7f, 0.95f);
}

bool EngineModel::checkOperatingLimits(float torque, float rpm, float coolantTemp) const {
    if (rpm > params_.ratedSpeed * 1.1f) {
        return false; // 超速
    }
    
    if (torque > params_.maxTorque * 1.1f) {
        return false; // 过载
    }
    
    if (coolantTemp > 105.0f) {
        return false; // 过热
    }
    
    if (currentState_.oilTemperature > 120.0f) {
        return false; // 机油过热
    }
    
    return true;
}

float EngineModel::calculateDerateFactor(float coolantTemp, float oilTemp) const {
    float derate = 1.0f;
    
    // 冷却液温度降额
    if (coolantTemp > 95.0f) {
        derate *= 1.0f - 0.02f * (coolantTemp - 95.0f);
    }
    
    // 机油温度降额
    if (oilTemp > 100.0f) {
        derate *= 1.0f - 0.015f * (oilTemp - 100.0f);
    }
    
    return std::max(0.5f, derate); // 最低降额到50%
}

void EngineModel::updateEngineState(float fuelRate, float rpm, float deltaTime, 
                                  float ambientTemp, float coolantFlowRate) {
    // 更新当前状态
    currentState_.currentRpm = rpm;
    currentState_.currentFuelRate = fuelRate;
    currentState_.currentTorque = calculateTorque(fuelRate, rpm, currentState_.coolantTemperature);
    currentState_.currentEfficiency = calculateEfficiency(currentState_.currentTorque, rpm);
    
    // 更新温度模型
    float currentLoad = currentState_.currentTorque / params_.maxTorque;
    updateTemperatureModel(deltaTime, ambientTemp, currentLoad, coolantFlowRate);
    
    // 累计统计
    currentState_.accumulatedFuel += fuelRate * deltaTime / 3600.0f; // kg
    currentState_.accumulatedWork += currentState_.currentTorque * rpm * 2.0f * M_PI * 
                                   deltaTime / 60000.0f; // kJ
    currentState_.operatingHours += static_cast<uint32_t>(deltaTime / 3600.0f);
}

} // namespace VCUCore