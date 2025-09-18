// src/models/battery_model.cpp
#include "battery_model.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace VCUCore {

BatteryModel::BatteryModel(uint32_t cellCount, uint32_t historySize)
    : cellCount_(cellCount), maxHistorySize_(historySize), isInitialized_(false),
      capacityEstimate_(0.0f), resistanceEstimate_(0.0f), learningRate_(0.01f) {
    
    initializeDefaultParameters();
    initializeCellStates();
    initializeOCVCurve();
    initializeAgingModel();
}

void BatteryModel::initializeDefaultParameters() {
    // 典型的锂离子电池参数（96串系统）
    params_ = {
        .nominalCapacity = 200.0f,         // 200 Ah
        .nominalVoltage = 3.2f * 96,       // 307.2 V (3.2V * 96串)
        .maxVoltage = 3.65f * 96,          // 350.4 V
        .minVoltage = 2.8f * 96,           // 268.8 V
        .maxChargeCurrent = 100.0f,        // 100 A
        .maxDischargeCurrent = 300.0f,     // 300 A
        .internalResistance = 0.05f,       // 50 mΩ
        .capacityFadeFactor = 1e-6f,       // 容量衰减系数
        .resistanceGrowthFactor = 1e-7f,   // 内阻增长系数
        .thermalMass = 50000.0f,           // 50 kJ/K
        .heatTransferCoeff = 100.0f,       // 100 W/K
        .selfDischargeRate = 2.0f          // 2% 每月
    };
    
    // 初始化电池状态
    currentState_ = {
        .voltage = params_.nominalVoltage,
        .current = 0.0f,
        .soc = 50.0f,
        .soh = 100.0f,
        .temperature = 25.0f,
        .power = 0.0f,
        .energy = params_.nominalCapacity * params_.nominalVoltage / 1000.0f * 0.5f,
        .internalResistance = params_.internalResistance,
        .timestamp = 0,
        .cycleCount = 0,
        .cells = std::vector<CellModel>(cellCount_)
    };
    
    previousState_ = currentState_;
}

void BatteryModel::initializeCellStates() {
    for (auto& cell : currentState_.cells) {
        cell = {
            .voltage = 3.2f,
            .soc = 50.0f,
            .temperature = 25.0f,
            .internalResistance = 0.0005f, // 0.5 mΩ
            .capacity = params_.nominalCapacity / cellCount_,
            .cycleCount = 0
        };
    }
}

void BatteryModel::initializeOCVCurve() {
    // 初始化SOC-OCV曲线（典型磷酸铁锂电池）
    // SOC从0到100%，每10%一个点
    socOcvCurve_ = Eigen::MatrixXf(11, 2);
    
    socOcvCurve_ << 
        0.0f, 2.8f,    // 0% SOC -> 2.8V
        10.0f, 3.0f,    // 10% SOC -> 3.0V
        20.0f, 3.1f,    // 20% SOC -> 3.1V
        30.0f, 3.2f,    // 30% SOC -> 3.2V
        40.0f, 3.25f,   // 40% SOC -> 3.25V
        50.0f, 3.3f,    // 50% SOC -> 3.3V
        60.0f, 3.32f,   // 60% SOC -> 3.32V
        70.0f, 3.35f,   // 70% SOC -> 3.35V
        80.0f, 3.4f,    // 80% SOC -> 3.4V
        90.0f, 3.45f,   // 90% SOC -> 3.45V
        100.0f, 3.5f;   // 100% SOC -> 3.5f
}

void BatteryModel::initializeAgingModel() {
    // 初始化老化模型参数
    agingModel_ = Eigen::MatrixXf(3, 4); // 温度, SOC, 电流, 衰减率
    
    agingModel_ <<
        // 温度(°C), SOC(%), 电流(C-rate), 衰减率
        25.0f, 50.0f, 0.5f, 1.0f,    // 基准条件
        35.0f, 50.0f, 0.5f, 1.5f,    // 高温
        45.0f, 50.0f, 0.5f, 2.0f,    // 更高温
        25.0f, 80.0f, 0.5f, 1.2f,    // 高SOC
        25.0f, 20.0f, 0.5f, 1.1f,    // 低SOC
        25.0f, 50.0f, 1.0f, 1.3f,    // 大电流
        25.0f, 50.0f, 2.0f, 1.8f;    // 极大电流
}

bool BatteryModel::initialize(const std::string& configFile) {
    try {
        // 加载配置文件
        if (!loadParameters(configFile)) {
            std::cerr << "Using default battery parameters" << std::endl;
        }
        
        // 加载特性数据（如果有）
        std::string dataFile = configFile.substr(0, configFile.find_last_of('.')) + "_data.csv";
        loadCharacterizationData(dataFile);
        
        isInitialized_ = true;
        std::cout << "Battery model initialized with " << cellCount_ << " cells" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Battery model initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool BatteryModel::loadParameters(const std::string& paramFile) {
    std::ifstream file(paramFile);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            std::string key;
            float value;
            
            if (iss >> key >> value) {
                if (key == "nominal_capacity") params_.nominalCapacity = value;
                else if (key == "nominal_voltage") params_.nominalVoltage = value;
                else if (key == "max_voltage") params_.maxVoltage = value;
                else if (key == "min_voltage") params_.minVoltage = value;
                else if (key == "max_charge_current") params_.maxChargeCurrent = value;
                else if (key == "max_discharge_current") params_.maxDischargeCurrent = value;
                else if (key == "internal_resistance") params_.internalResistance = value;
                else if (key == "thermal_mass") params_.thermalMass = value;
                else if (key == "heat_transfer_coeff") params_.heatTransferCoeff = value;
                else if (key == "self_discharge_rate") params_.selfDischargeRate = value;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading battery parameters: " << e.what() << std::endl;
        return false;
    }
}

BatteryState BatteryModel::updateState(float current, float voltage, float temperature, float deltaTime) {
    if (!validateInput(current, voltage, temperature)) {
        std::cerr << "Invalid input parameters" << std::endl;
        return currentState_;
    }
    
    try {
        // 保存先前状态
        previousState_ = currentState_;
        
        // 更新基本状态
        currentState_.current = current;
        currentState_.voltage = voltage;
        currentState_.temperature = temperature;
        currentState_.power = current * voltage;
        currentState_.timestamp = getCurrentTime();
        
        // 更新SOC（库仑计数 + OCV校正）
        updateSOC(deltaTime);
        
        // 更新热模型
        updateThermalModel(25.0f, 0.0f, deltaTime); // 默认环境温度25°C，无冷却
        
        // 更新老化模型
        updateAgingModel(deltaTime);
        
        // 更新电芯状态
        updateIndividualCells(deltaTime);
        
        // 检查安全限制
        if (!checkSafetyLimits()) {
            handleSafetyViolation();
        }
        
        // 更新历史记录
        updateHistory(currentState_);
        
        // 在线参数估计
        performOnlineParameterEstimation();
        
        return currentState_;
        
    } catch (const std::exception& e) {
        std::cerr << "Battery state update error: " << e.what() << std::endl;
        return previousState_;
    }
}

void BatteryModel::updateSOC(float deltaTime) {
    // 库仑计数法
    float deltaSOC = (currentState_.current * deltaTime) / (params_.nominalCapacity * 3600.0f) * 100.0f;
    currentState_.soc = previousState_.soc - deltaSOC;
    
    // OCV校正（每60秒或SOC变化较大时）
    static uint32_t lastOCVCorrection = 0;
    uint32_t currentTime = getCurrentTime();
    
    if ((currentTime - lastOCVCorrection > 60000) || std::abs(deltaSOC) > 5.0f) {
        float ocv = calculateOCV(currentState_.soc, currentState_.temperature);
        float measuredOCV = currentState_.voltage - (currentState_.current * currentState_.internalResistance);
        
        // SOC校正（如果OCV偏差较大）
        if (std::abs(ocv - measuredOCV) > 0.1f) {
            float correctedSOC = calculateSOC(measuredOCV, 0.0f, currentState_.temperature);
            currentState_.soc = 0.9f * currentState_.soc + 0.1f * correctedSOC;
        }
        
        lastOCVCorrection = currentTime;
    }
    
    // 边界限制
    currentState_.soc = std::clamp(currentState_.soc, 0.0f, 100.0f);
    
    // 更新剩余能量
    currentState_.energy = (currentState_.soc / 100.0f) * params_.nominalCapacity * 
                          params_.nominalVoltage / 1000.0f;
}

void BatteryModel::updateThermalModel(float ambientTemp, float coolantFlowRate, float deltaTime) {
    // 计算热量产生
    float heatGeneration = calculateHeatGeneration(currentState_.current, currentState_.internalResistance);
    
    // 计算热量耗散
    float heatDissipation = calculateHeatDissipation(currentState_.temperature, ambientTemp, coolantFlowRate);
    
    // 计算温度变化
    float tempChange = calculateTemperatureChange(heatGeneration, heatDissipation, deltaTime);
    
    // 更新温度
    currentState_.temperature += tempChange;
    
    // 更新电芯温度（简化：假设均匀分布）
    for (auto& cell : currentState_.cells) {
        cell.temperature = currentState_.temperature;
    }
}

void BatteryModel::updateAgingModel(float deltaTime) {
    // 计算容量衰减
    float capacityFade = calculateCapacityFade(currentState_.temperature, 
                                             currentState_.soc, 
                                             currentState_.current / params_.nominalCapacity,
                                             deltaTime);
    
    // 计算内阻增长
    float resistanceGrowth = calculateResistanceGrowth(currentState_.temperature,
                                                     currentState_.soc,
                                                     currentState_.current / params_.nominalCapacity,
                                                     deltaTime);
    
    // 计算循环老化
    float cycleAging = calculateCycleAging(std::abs(currentState_.soc - previousState_.soc), deltaTime);
    
    // 更新参数
    params_.nominalCapacity *= (1.0f - capacityFade);
    params_.internalResistance *= (1.0f + resistanceGrowth);
    
    // 更新健康状态
    currentState_.soh = calculateSOH();
    currentState_.internalResistance = params_.internalResistance;
    
    // 更新循环计数
    if (std::abs(currentState_.soc - previousState_.soc) > 80.0f) {
        currentState_.cycleCount++;
        for (auto& cell : currentState_.cells) {
            cell.cycleCount++;
        }
    }
}

float BatteryModel::calculateOCV(float soc, float temperature) const {
    // 双线性插值计算OCV
    float baseOCV = interpolateOCV(soc, 25.0f); // 基准温度25°C
    
    // 温度补偿（-0.2mV/°C per cell）
    float tempCompensation = (temperature - 25.0f) * -0.0002f * cellCount_;
    
    return baseOCV + tempCompensation;
}

float BatteryModel::calculateSOC(float voltage, float current, float temperature) const {
    // 考虑内阻压降的OCV计算
    float ocv = voltage + (current * currentState_.internalResistance);
    
    // 在OCV曲线上查找对应的SOC
    for (int i = 0; i < socOcvCurve_.rows() - 1; ++i) {
        if (ocv >= socOcvCurve_(i, 1) && ocv <= socOcvCurve_(i + 1, 1)) {
            float socLow = socOcvCurve_(i, 0);
            float socHigh = socOcvCurve_(i + 1, 0);
            float ocvLow = socOcvCurve_(i, 1);
            float ocvHigh = socOcvCurve_(i + 1, 1);
            
            return socLow + (socHigh - socLow) * (ocv - ocvLow) / (ocvHigh - ocvLow);
        }
    }
    
    return 50.0f; // 默认值
}

float BatteryModel::calculateHeatGeneration(float current, float internalResistance) const {
    // I²R 损失
    return current * current * internalResistance;
}

float BatteryModel::calculateHeatDissipation(float temperature, float ambientTemp, float flowRate) const {
    // 对流散热 + 强制冷却
    float naturalConvection = params_.heatTransferCoeff * (temperature - ambientTemp);
    float forcedCooling = flowRate * 1000.0f * (temperature - ambientTemp) * 4.18f; // 水冷
    
    return naturalConvection + forcedCooling;
}

float BatteryModel::calculateSOH() const {
    // 基于容量和内阻的健康状态计算
    float capacityHealth = params_.nominalCapacity / 200.0f * 100.0f; // 相对于标称容量
    float resistanceHealth = (0.05f / currentState_.internalResistance) * 100.0f; // 相对于标称内阻
    
    return 0.7f * capacityHealth + 0.3f * resistanceHealth;
}

bool BatteryModel::checkSafetyLimits() const {
    return checkVoltageLimits() && checkCurrentLimits() && checkThermalLimits();
}

bool BatteryModel::checkVoltageLimits() const {
    return currentState_.voltage >= params_.minVoltage && 
           currentState_.voltage <= params_.maxVoltage;
}

bool BatteryModel::checkCurrentLimits() const {
    if (currentState_.current > 0) { // 放电
        return currentState_.current <= params_.maxDischargeCurrent;
    } else { // 充电
        return std::abs(currentState_.current) <= params_.maxChargeCurrent;
    }
}

bool BatteryModel::checkThermalLimits() const {
    return currentState_.temperature >= -20.0f && 
           currentState_.temperature <= 60.0f;
}

float BatteryModel::predictRemainingCapacity(float currentLoad) const {
    if (std::abs(currentLoad) < 0.1f) {
        return currentState_.energy;
    }
    
    // 基于当前负载预测剩余容量
    float remainingEnergy = currentState_.energy;
    float timeToEmpty = remainingEnergy * 1000.0f / std::abs(currentLoad); // 小时
    
    // 考虑效率损失（约95%）
    return timeToEmpty * 0.95f * 3600.0f; // 转换为秒
}

float BatteryModel::estimateMaxPower(float duration) const {
    // 基于时间 duration（秒）估算最大功率
    float availableEnergy = currentState_.energy * 0.8f; // 保留20%容量
    float maxPower = availableEnergy * 1000.0f / (duration / 3600.0f);
    
    // 考虑电流限制
    float currentLimitedPower = params_.maxDischargeCurrent * currentState_.voltage;
    
    return std::min(maxPower, currentLimitedPower);
}

void BatteryModel::calibrateSOC(float measuredVoltage, float measuredCurrent) {
    float ocv = measuredVoltage + (measuredCurrent * currentState_.internalResistance);
    float calibratedSOC = calculateSOC(ocv, 0.0f, currentState_.temperature);
    
    // 平滑过渡到校准值
    currentState_.soc = 0.3f * currentState_.soc + 0.7f * calibratedSOC;
}

void BatteryModel::calibrateCapacity(float measuredCapacity) {
    // 更新容量估计
    capacityEstimate_ = 0.9f * capacityEstimate_ + 0.1f * measuredCapacity;
    params_.nominalCapacity = capacityEstimate_;
}

uint32_t BatteryModel::getCurrentTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

float BatteryModel::interpolateOCV(float soc, float temperature) const {
    // 在OCV曲线上线性插值
    for (int i = 0; i < socOcvCurve_.rows() - 1; ++i) {
        if (soc >= socOcvCurve_(i, 0) && soc <= socOcvCurve_(i + 1, 0)) {
            float socLow = socOcvCurve_(i, 0);
            float socHigh = socOcvCurve_(i + 1, 0);
            float ocvLow = socOcvCurve_(i, 1);
            float ocvHigh = socOcvCurve_(i + 1, 1);
            
            return ocvLow + (ocvHigh - ocvLow) * (soc - socLow) / (socHigh - socLow);
        }
    }
    
    return 3.2f; // 默认值
}

bool BatteryModel::validateInput(float current, float voltage, float temperature) const {
    if (std::isnan(current) || std::isinf(current) || std::abs(current) > 1000.0f) {
        return false;
    }
    
    if (std::isnan(voltage) || std::isinf(voltage) || voltage < 0.0f || voltage > 500.0f) {
        return false;
    }
    
    if (std::isnan(temperature) || std::isinf(temperature) || temperature < -50.0f || temperature > 100.0f) {
        return false;
    }
    
    return true;
}

void BatteryModel::updateHistory(const BatteryState& state) {
    history_.push_back(state);
    voltageHistory_.push_back(state.voltage);
    currentHistory_.push_back(state.current);
    temperatureHistory_.push_back(state.temperature);
    
    if (history_.size() > maxHistorySize_) {
        history_.pop_front();
        voltageHistory_.pop_front();
        currentHistory_.pop_front();
        temperatureHistory_.pop_front();
    }
}

BatteryState BatteryModel::getCurrentState() const {
    return currentState_;
}

BatteryHealth BatteryModel::getHealthStatus() const {
    BatteryHealth health;
    
    health.overallHealth = currentState_.soh;
    health.capacityHealth = (params_.nominalCapacity / 200.0f) * 100.0f;
    health.resistanceHealth = (0.05f / currentState_.internalResistance) * 100.0f;
    health.voltageHealth = checkCellVoltages() ? 100.0f : 80.0f;
    health.temperatureHealth = (60.0f - currentState_.temperature) / 60.0f * 100.0f;
    
    // 预测寿命（简化模型）
    health.predictedLifeCycles = static_cast<uint32_t>((100.0f - currentState_.soh) * 100);
    health.predictedLifeDays = health.predictedLifeCycles / 2; // 假设每天2个循环
    
    return health;
}

} // namespace VCUCore