// src/models/motor_model.cpp
#include "motor_model.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace VCUCore {

MotorModel::MotorModel() {
    initializeDefaultParameters();
    initializeThermalModel();
    initializeBatteryInfluence();
    
    // 初始化映射表
    efficiencyMap_ = Eigen::MatrixXf::Zero(100, 100);
    torqueMap_ = Eigen::MatrixXf::Zero(100, 100);
    lossMap_ = Eigen::MatrixXf::Zero(100, 100);
    
    // 初始化状态
    currentState_ = {
        .currentTorque = 0.0f,
        .currentRpm = 0.0f,
        .currentVoltage = 0.0f,
        .currentCurrent = 0.0f,
        .currentEfficiency = 0.0f,
        .windingTemperature = 25.0f,
        .magnetTemperature = 25.0f,
        .coolantTemperature = 25.0f,
        .accumulatedEnergy = 0.0f,
        .accumulatedLosses = 0.0f,
        .operatingHours = 0
    };
}

void MotorModel::initializeDefaultParameters() {
    // 典型的永磁同步电机参数
    params_ = {
        .ratedPower = 150.0f,           // 150 kW
        .peakPower = 200.0f,            // 200 kW (30秒)
        .ratedTorque = 400.0f,          // 400 Nm
        .peakTorque = 600.0f,           // 600 Nm (30秒)
        .maxSpeed = 6000.0f,            // 6000 rpm
        .baseSpeed = 2000.0f,           // 2000 rpm
        .statorResistance = 0.02f,      // 0.02 Ω
        .rotorResistance = 0.01f,       // 0.01 Ω
        .inductance = 0.001f,           // 1 mH
        .backEMFConstant = 0.5f,        // 0.5 V/rad/s
        .thermalResistance = 0.1f,      // 0.1 K/W
        .thermalCapacitance = 5000.0f,  // 5 kJ/K
        .mechanicalLosses = 5.0f        // 5 Nm
    };
}

void MotorModel::initializeThermalModel() {
    thermalParams_ = {
        .windingTempBase = 25.0f,
        .magnetTempBase = 25.0f,
        .coolantTempBase = 25.0f,
        .windingTempGain = 0.8f,        // °C per kW loss
        .magnetTempGain = 0.3f,         // °C per kW loss
        .coolantTempGain = 0.2f,        // °C per kW loss
        .ambientTempInfluence = 0.5f     // 50% ambient influence
    };
}

void MotorModel::initializeBatteryInfluence() {
    batteryParams_ = {
        .voltageEffect = 0.005f,        // 每伏特电压变化的影响
        .socEffect = 0.1f,              // SOC变化10%的影响
        .temperatureEffect = 0.002f,    // 每°C电池温度的影响
        .internalResistanceEffect = 0.01f // 内阻变化的影响
    };
}

bool MotorModel::loadMotorMap(const std::string& mapFile) {
    std::ifstream file(mapFile);
    if (!file.is_open()) {
        std::cerr << "Error opening motor map file: " << mapFile << std::endl;
        return false;
    }
    
    try {
        std::string line;
        int row = 0;
        
        while (std::getline(file, line) && row < efficiencyMap_.rows()) {
            std::istringstream iss(line);
            int col = 0;
            float value;
            
            while (iss >> value && col < efficiencyMap_.cols()) {
                efficiencyMap_(row, col) = value;
                col++;
            }
            row++;
        }
        
        // 验证映射表数据
        if (!validateMotorMap()) {
            std::cerr << "Motor map validation failed" << std::endl;
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing motor map: " << e.what() << std::endl;
        return false;
    }
}

bool MotorModel::loadMotorParameters(const std::string& paramFile) {
    std::ifstream file(paramFile);
    if (!file.is_open()) {
        std::cerr << "Error opening motor parameter file: " << paramFile << std::endl;
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            float value;
            
            if (iss >> key >> value) {
                if (key == "rated_power") params_.ratedPower = value;
                else if (key == "peak_power") params_.peakPower = value;
                else if (key == "rated_torque") params_.ratedTorque = value;
                else if (key == "peak_torque") params_.peakTorque = value;
                else if (key == "max_speed") params_.maxSpeed = value;
                else if (key == "base_speed") params_.baseSpeed = value;
                else if (key == "stator_resistance") params_.statorResistance = value;
                else if (key == "back_emf_constant") params_.backEMFConstant = value;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing motor parameters: " << e.what() << std::endl;
        return false;
    }
}

float MotorModel::calculateTorque(float current, float rpm, float batteryVoltage) {
    if (!validateOperatingPoint(0, rpm)) {
        return 0.0f;
    }
    
    // 计算电磁扭矩
    float backEMF = calculateBackEMF(rpm);
    float terminalVoltage = batteryVoltage - current * params_.statorResistance;
    float flux = calculateFluxLinkage(current);
    
    float electromagneticTorque = (3.0f / 2.0f) * params_.backEMFConstant * flux * current;
    
    // 减去机械损失
    float mechanicalLoss = calculateMechanicalLosses(rpm);
    float netTorque = electromagneticTorque - mechanicalLoss;
    
    // 电压影响
    float voltageEffect = calculateVoltageEffect(batteryVoltage, 650.0f);
    netTorque *= voltageEffect;
    
    // 应用扭矩限制
    netTorque = std::clamp(netTorque, -params_.peakTorque, params_.peakTorque);
    
    return netTorque;
}

float MotorModel::calculateCurrent(float torque, float rpm, float batteryVoltage) {
    if (!validateOperatingPoint(torque, rpm)) {
        return 0.0f;
    }
    
    // 计算所需电流
    float mechanicalLoss = calculateMechanicalLosses(rpm);
    float totalTorque = torque + mechanicalLoss;
    
    if (std::abs(totalTorque) < 0.1f) {
        return 0.0f;
    }
    
    float flux = calculateFluxLinkage(0.0f); // 使用标称磁链
    float current = totalTorque / (1.5f * params_.backEMFConstant * flux);
    
    // 考虑反电动势
    float backEMF = calculateBackEMF(rpm);
    float requiredVoltage = current * params_.statorResistance + backEMF;
    
    // 检查电压限制
    if (requiredVoltage > batteryVoltage * 0.95f) {
        // 电压不足，需要场弱化
        current *= batteryVoltage * 0.95f / requiredVoltage;
    }
    
    return current;
}

float MotorModel::calculateEfficiency(float torque, float rpm, float batteryVoltage, float batterySOC) {
    // 从效率映射表插值
    float efficiency = interpolateMap(efficiencyMap_, rpm, torque);
    
    if (efficiency <= 0.0f) {
        // 计算理论效率
        float inputPower = std::abs(torque) * rpm * 2.0f * M_PI / 60.0f;
        float losses = calculateLosses(torque, rpm, batteryVoltage);
        
        if (inputPower + losses > 0.0f) {
            efficiency = inputPower / (inputPower + losses);
        }
    }
    
    // 电池影响
    efficiency *= calculateSOCEffect(batterySOC);
    efficiency *= calculateTemperatureEffect(currentState_.windingTemperature);
    
    return std::clamp(efficiency, 0.0f, 0.97f); // 电机最大效率约97%
}

float MotorModel::calculateLosses(float torque, float rpm, float batteryVoltage) {
    float current = calculateCurrent(torque, rpm, batteryVoltage);
    
    // 铜损
    float copperLosses = calculateCopperLosses(current);
    
    // 铁损
    float flux = calculateFluxLinkage(current);
    float ironLosses = calculateIronLosses(rpm, flux);
    
    // 机械损失
    float mechanicalLosses = calculateMechanicalLosses(rpm);
    
    // 杂散损失
    float strayLosses = calculateStrayLosses(torque, rpm);
    
    return copperLosses + ironLosses + mechanicalLosses + strayLosses;
}

void MotorModel::updateTemperatureModel(float deltaTime, float ambientTemp, 
                                      float coolantFlowRate, float currentLoad) {
    // 计算总损失
    float totalLosses = calculateLosses(currentState_.currentTorque, 
                                      currentState_.currentRpm,
                                      currentState_.currentVoltage);
    
    // 更新绕组温度
    currentState_.windingTemperature = calculateWindingTemperature(totalLosses * 0.6f, 
                                                                 currentState_.coolantTemperature);
    
    // 更新磁体温度
    currentState_.magnetTemperature = calculateMagnetTemperature(totalLosses * 0.2f,
                                                               currentState_.coolantTemperature);
    
    // 更新冷却液温度
    currentState_.coolantTemperature = calculateCoolantTemperature(totalLosses,
                                                                 coolantFlowRate);
    
    // 环境温度影响
    currentState_.windingTemperature += thermalParams_.ambientTempInfluence * 
                                      (ambientTemp - 25.0f);
    currentState_.magnetTemperature += thermalParams_.ambientTempInfluence * 
                                     (ambientTemp - 25.0f);
    currentState_.coolantTemperature += thermalParams_.ambientTempInfluence * 
                                      (ambientTemp - 25.0f);
}

float MotorModel::calculateWindingTemperature(float copperLosses, float coolantTemp) {
    // 热平衡方程
    float heatTransfer = thermalParams_.thermalResistance * 
                       (currentState_.windingTemperature - coolantTemp);
    float tempChange = (copperLosses - heatTransfer) / params_.thermalCapacitance;
    
    return currentState_.windingTemperature + tempChange;
}

float MotorModel::calculateMagnetTemperature(float eddyLosses, float coolantTemp) {
    // 磁体温度模型
    float heatTransfer = 0.5f * thermalParams_.thermalResistance * 
                       (currentState_.magnetTemperature - coolantTemp);
    float tempChange = (eddyLosses - heatTransfer) / (params_.thermalCapacitance * 0.5f);
    
    return currentState_.magnetTemperature + tempChange;
}

float MotorModel::calculateCoolantTemperature(float totalLosses, float flowRate) {
    // 冷却系统模型
    float heatTransfer = 2.0f * flowRate * 
                       (currentState_.coolantTemperature - thermalParams_.coolantTempBase);
    float tempChange = (totalLosses - heatTransfer) / (params_.thermalCapacitance * 2.0f);
    
    return currentState_.coolantTemperature + tempChange;
}

float MotorModel::calculateVoltageEffect(float batteryVoltage, float nominalVoltage) {
    // 电压对性能的影响
    float voltageRatio = batteryVoltage / nominalVoltage;
    return std::clamp(0.8f + 0.2f * voltageRatio, 0.8f, 1.2f);
}

float MotorModel::calculateSOCEffect(float batterySOC) {
    // SOC对效率的影响
    return 0.9f + 0.1f * batterySOC; // SOC越高，效率越高
}

float MotorModel::calculateTemperatureEffect(float batteryTemp) {
    // 温度对性能的影响
    float optimalTemp = 25.0f;
    return 1.0f - 0.002f * std::abs(batteryTemp - optimalTemp);
}

void MotorModel::updateMotorState(float torque, float rpm, float batteryVoltage, 
                                float batterySOC, float batteryTemp, float deltaTime,
                                float ambientTemp, float coolantFlowRate) {
    // 更新当前状态
    currentState_.currentTorque = torque;
    currentState_.currentRpm = rpm;
    currentState_.currentVoltage = batteryVoltage;
    currentState_.currentCurrent = calculateCurrent(torque, rpm, batteryVoltage);
    currentState_.currentEfficiency = calculateEfficiency(torque, rpm, batteryVoltage, batterySOC);
    
    // 更新温度模型
    updateTemperatureModel(deltaTime, ambientTemp, coolantFlowRate, torque / params_.ratedTorque);
    
    // 累计统计
    float power = std::abs(torque) * rpm * 2.0f * M_PI / 60000.0f; // kW
    currentState_.accumulatedEnergy += power * deltaTime / 3600.0f; // kWh
    currentState_.accumulatedLosses += calculateLosses(torque, rpm, batteryVoltage) * 
                                     deltaTime / 3600.0f; // kWh
    currentState_.operatingHours += static_cast<uint32_t>(deltaTime / 3600.0f);
}

float MotorModel::interpolateMap(const Eigen::MatrixXf& map, float x, float y) const {
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

bool MotorModel::validateMotorMap() const {
    // 检查映射表数据有效性
    if (efficiencyMap_.rows() == 0 || efficiencyMap_.cols() == 0) {
        return false;
    }
    
    if (efficiencyMap_.minCoeff() < 0.0f || efficiencyMap_.maxCoeff() > 1.0f) {
        return false;
    }
    
    return true;
}

bool MotorModel::validateOperatingPoint(float torque, float rpm) const {
    if (rpm < 0.0f || rpm > params_.maxSpeed * 1.1f) {
        return false;
    }
    
    if (torque < -params_.peakTorque * 1.1f || torque > params_.peakTorque * 1.1f) {
        return false;
    }
    
    return true;
}

float MotorModel::calculateCopperLosses(float current) const {
    return 3.0f * params_.statorResistance * current * current;
}

float MotorModel::calculateIronLosses(float rpm, float flux) const {
    // 铁损 = 磁滞损失 + 涡流损失
    float hysteresisLoss = 0.5f * rpm * flux * flux;
    float eddyCurrentLoss = 0.01f * rpm * rpm * flux * flux;
    return hysteresisLoss + eddyCurrentLoss;
}

float MotorModel::calculateMechanicalLosses(float rpm) const {
    return params_.mechanicalLosses * (rpm / params_.baseSpeed);
}

float MotorModel::calculateStrayLosses(float torque, float rpm) const {
    return 0.02f * std::abs(torque) * (rpm / params_.baseSpeed);
}

float MotorModel::calculateBackEMF(float rpm) const {
    return params_.backEMFConstant * rpm * 2.0f * M_PI / 60.0f;
}

float MotorModel::calculateFluxLinkage(float current) const {
    // 简化磁链模型
    return 1.0f - 0.001f * std::abs(current);
}

float MotorModel::calculateTorqueConstant(float current) const {
    return 1.5f * params_.backEMFConstant * calculateFluxLinkage(current);
}

float MotorModel::calculatePowerLoss(float torque, float rpm) const {
    return calculateLosses(torque, rpm, currentState_.currentVoltage);
}

float MotorModel::calculateThermalLimit(float currentTemp, float maxTemp, float time) const {
    float tempRise = maxTemp - currentTemp;
    float maxPowerLoss = tempRise / (thermalParams_.thermalResistance * time);
    return maxPowerLoss;
}

MotorState MotorModel::getMotorState() const {
    return currentState_;
}

MotorParameters MotorModel::getMotorParameters() const {
    return params_;
}

void MotorModel::resetAccumulatedStats() {
    currentState_.accumulatedEnergy = 0.0f;
    currentState_.accumulatedLosses = 0.0f;
    currentState_.operatingHours = 0;
}

void MotorModel::setMotorParameters(const MotorParameters& params) {
    params_ = params;
    initializeThermalModel(); // 重新初始化热模型
}

} // namespace VCUCore