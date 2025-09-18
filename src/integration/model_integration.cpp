// src/integration/model_integration.cpp
#include "model_integration.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

namespace VCUCore {

ModelIntegration::ModelIntegration() 
    : isInitialized_(false), 
      isRunning_(false),
      updateRate_(100), // 100 Hz
      lastUpdateTime_(0) {
    
    // 初始化互斥锁
    pthread_mutex_init(&dataMutex_, nullptr);
    pthread_cond_init(&dataCondition_, nullptr);
    
    // 创建模型实例
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    predictiveAnalytics_ = std::make_unique<PredictiveAnalytics>();
    
    // 初始化默认配置
    initializeDefaultConfig();
}

ModelIntegration::~ModelIntegration() {
    stop();
    
    pthread_mutex_destroy(&dataMutex_);
    pthread_cond_destroy(&dataCondition_);
}

bool ModelIntegration::initialize(const std::string& configFile) {
    if (isInitialized_) {
        std::cout << "Model integration already initialized" << std::endl;
        return true;
    }
    
    try {
        // 加载配置文件
        if (!loadConfiguration(configFile)) {
            std::cerr << "Failed to load configuration file: " << configFile << std::endl;
            return false;
        }
        
        // 初始化发动机模型
        if (!engineModel_->initialize(config_.engineModelConfig)) {
            std::cerr << "Failed to initialize engine model" << std::endl;
            return false;
        }
        
        // 初始化电机模型
        if (!motorModel_->loadMotorParameters(config_.motorModelConfig)) {
            std::cerr << "Failed to initialize motor model" << std::endl;
            return false;
        }
        
        // 初始化预测分析
        if (!predictiveAnalytics_->initialize(config_.predictionConfig)) {
            std::cerr << "Failed to initialize predictive analytics" << std::endl;
            return false;
        }
        
        // 初始化数据缓冲区
        initializeDataBuffers();
        
        isInitialized_ = true;
        std::cout << "Model integration initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing model integration: " << e.what() << std::endl;
        return false;
    }
}

void ModelIntegration::initializeDefaultConfig() {
    // 默认配置参数
    config_ = {
        .updateRate = 100,
        .predictionHorizon = 5.0f,
        .controlHorizon = 2.0f,
        .maxTorqueError = 50.0f,
        .maxSpeedError = 100.0f,
        .thermalSafetyMargin = 0.9f,
        .batterySafetyMargin = 0.85f,
        .emergencyShutdownTemp = 150.0f,
        .minBatterySOC = 0.2f,
        .engineModelConfig = "config/engine_params.cfg",
        .motorModelConfig = "config/motor_params.cfg",
        .predictionConfig = "config/prediction_params.cfg"
    };
}

bool ModelIntegration::loadConfiguration(const std::string& configFile) {
    std::ifstream file(configFile);
    if (!file.is_open()) {
        std::cerr << "Cannot open configuration file: " << configFile << std::endl;
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            // 跳过注释和空行
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            std::string key;
            std::string value;
            
            if (iss >> key >> value) {
                if (key == "update_rate") config_.updateRate = std::stoi(value);
                else if (key == "prediction_horizon") config_.predictionHorizon = std::stof(value);
                else if (key == "control_horizon") config_.controlHorizon = std::stof(value);
                else if (key == "max_torque_error") config_.maxTorqueError = std::stof(value);
                else if (key == "max_speed_error") config_.maxSpeedError = std::stof(value);
                else if (key == "thermal_safety_margin") config_.thermalSafetyMargin = std::stof(value);
                else if (key == "battery_safety_margin") config_.batterySafetyMargin = std::stof(value);
                else if (key == "emergency_shutdown_temp") config_.emergencyShutdownTemp = std::stof(value);
                else if (key == "min_battery_soc") config_.minBatterySOC = std::stof(value);
                else if (key == "engine_config") config_.engineModelConfig = value;
                else if (key == "motor_config") config_.motorModelConfig = value;
                else if (key == "prediction_config") config_.predictionConfig = value;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing configuration: " << e.what() << std::endl;
        return false;
    }
}

void ModelIntegration::initializeDataBuffers() {
    // 初始化环形缓冲区
    sensorBuffer_.resize(config_.updateRate * 10); // 10秒数据
    controlBuffer_.resize(config_.updateRate * 5);  // 5秒控制数据
    predictionBuffer_.resize(config_.updateRate * 2); // 2秒预测数据
    
    // 初始化统计计数器
    performanceStats_ = {
        .totalUpdates = 0,
        .failedUpdates = 0,
        .averageUpdateTime = 0.0f,
        .maxUpdateTime = 0.0f,
        .minUpdateTime = std::numeric_limits<float>::max(),
        .modelConvergenceRate = 0.0f,
        .predictionAccuracy = 0.0f,
        .energyEfficiency = 0.0f
    };
}

bool ModelIntegration::start() {
    if (!isInitialized_) {
        std::cerr << "Model integration not initialized" << std::endl;
        return false;
    }
    
    if (isRunning_) {
        std::cout << "Model integration already running" << std::endl;
        return true;
    }
    
    isRunning_ = true;
    
    // 创建更新线程
    updateThread_ = std::thread(&ModelIntegration::updateLoop, this);
    
    // 创建监控线程
    monitorThread_ = std::thread(&ModelIntegration::monitorLoop, this);
    
    std::cout << "Model integration started" << std::endl;
    return true;
}

bool ModelIntegration::stop() {
    if (!isRunning_) {
        return true;
    }
    
    isRunning_ = false;
    
    // 通知所有等待的线程
    pthread_cond_broadcast(&dataCondition_);
    
    // 等待线程结束
    if (updateThread_.joinable()) {
        updateThread_.join();
    }
    
    if (monitorThread_.joinable()) {
        monitorThread_.join();
    }
    
    std::cout << "Model integration stopped" << std::endl;
    return true;
}

void ModelIntegration::updateLoop() {
    auto lastTime = std::chrono::high_resolution_clock::now();
    
    while (isRunning_) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastTime).count();
        
        float deltaTime = static_cast<float>(elapsed) / 1000.0f;
        
        if (deltaTime >= 1.0f / config_.updateRate) {
            auto startTime = std::chrono::high_resolution_clock::now();
            
            try {
                // 执行模型更新
                if (!updateModels(deltaTime)) {
                    performanceStats_.failedUpdates++;
                    std::cerr << "Model update failed" << std::endl;
                }
                
                // 更新性能统计
                auto endTime = std::chrono::high_resolution_clock::now();
                float updateTime = std::chrono::duration_cast<std::chrono::microseconds>(
                    endTime - startTime).count() / 1000.0f;
                
                updatePerformanceStats(updateTime);
                
            } catch (const std::exception& e) {
                std::cerr << "Error in update loop: " << e.what() << std::endl;
                performanceStats_.failedUpdates++;
            }
            
            lastTime = currentTime;
        }
        
        // 休眠剩余时间
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ModelIntegration::monitorLoop() {
    while (isRunning_) {
        // 检查系统健康状态
        checkSystemHealth();
        
        // 检查模型收敛性
        checkModelConvergence();
        
        // 检查资源使用
        checkResourceUsage();
        
        // 休眠1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool ModelIntegration::updateModels(float deltaTime) {
    pthread_mutex_lock(&dataMutex_);
    
    try {
        // 获取最新传感器数据
        SensorData sensorData = getLatestSensorData();
        BatteryData batteryData = getLatestBatteryData();
        TerrainData terrainData = getLatestTerrainData();
        
        // 创建感知数据
        PerceptionData perception = createPerceptionData(sensorData, batteryData, terrainData);
        
        // 执行预测分析
        PredictionResult prediction = predictiveAnalytics_->predict(perception);
        
        // 更新发动机模型
        EngineState engineState = engineModel_->updateState(
            sensorData.engineRpm,
            sensorData.engineLoad,
            sensorData.engineTemp,
            batteryData.voltage,
            deltaTime
        );
        
        // 更新电机模型
        motorModel_->updateMotorState(
            sensorData.motorTorque,
            sensorData.motorRpm,
            batteryData.voltage,
            batteryData.soc,
            batteryData.temperature,
            deltaTime,
            sensorData.ambientTemp,
            sensorData.coolantFlowRate
        );
        
        // 计算最优控制策略
        ControlStrategy strategy = calculateOptimalStrategy(perception, prediction);
        
        // 生成控制命令
        ControlCommands commands = generateControlCommands(strategy, perception, prediction);
        
        // 验证控制命令
        if (!validateControlCommands(commands)) {
            std::cerr << "Control command validation failed" << std::endl;
            activateFallbackMode();
            pthread_mutex_unlock(&dataMutex_);
            return false;
        }
        
        // 保存数据和命令
        saveToBuffer(sensorData, perception, prediction, commands);
        lastControlCommands_ = commands;
        
        // 发布控制命令
        publishControlCommands(commands);
        
        pthread_mutex_unlock(&dataMutex_);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error updating models: " << e.what() << std::endl;
        pthread_mutex_unlock(&dataMutex_);
        return false;
    }
}

PerceptionData ModelIntegration::createPerceptionData(const SensorData& sensor,
                                                    const BatteryData& battery,
                                                    const TerrainData& terrain) const {
    PerceptionData perception;
    
    // 车辆状态
    perception.vehicleState.position = Vector3d(sensor.gpsX, sensor.gpsY, sensor.gpsZ);
    perception.vehicleState.velocity = Vector3d(sensor.velocityX, sensor.velocityY, sensor.velocityZ);
    perception.vehicleState.acceleration = Vector3d(sensor.accelX, sensor.accelY, sensor.accelZ);
    perception.vehicleState.heading = sensor.heading;
    perception.vehicleState.estimatedMass = estimateVehicleMass(sensor);
    
    // 动力系统状态
    perception.vehicleState.fuelConsumption = sensor.fuelConsumption;
    perception.vehicleState.energyEfficiency = calculateEnergyEfficiency(sensor, battery);
    perception.vehicleState.powerConsumption = sensor.enginePower + sensor.motorPower;
    
    // 地形信息
    perception.terrainSlope = terrain.slope;
    perception.soilResistance = terrain.soilResistance;
    perception.rollingResistance = terrain.rollingResistance;
    perception.aerodynamicDrag = terrain.aerodynamicDrag;
    
    // 负载信息
    perception.implementForce = sensor.implementForce;
    perception.loadFactor = sensor.loadFactor;
    perception.loadTrend = estimateLoadTrend();
    perception.loadChangeType = estimateLoadChangeType();
    
    return perception;
}

ControlStrategy ModelIntegration::calculateOptimalStrategy(const PerceptionData& perception,
                                                         const PredictionResult& prediction) {
    ControlStrategy strategy;
    
    // 基于预测结果计算最优策略
    float predictedLoad = prediction.loadForecast.empty() ? 0.0f : prediction.loadForecast[0];
    float predictedEnergy = prediction.energyDemand.empty() ? 0.0f : prediction.energyDemand[0];
    
    // 获取当前系统状态
    EngineState engineState = engineModel_->getEngineState();
    MotorState motorState = motorModel_->getMotorState();
    
    // 计算能量需求
    EnergyRequirements energyReq = calculateEnergyRequirements(perception, prediction);
    
    // 确定工作模式
    strategy.operatingMode = determineOperatingMode(energyReq, engineState, motorState);
    
    // 计算扭矩分配
    strategy.torqueSplit = calculateTorqueSplit(strategy.operatingMode, energyReq, 
                                              engineState, motorState);
    
    // 计算变速策略
    strategy.transmissionStrategy = calculateTransmissionStrategy(perception, prediction);
    
    // 计算能量管理策略
    strategy.energyManagement = calculateEnergyManagementStrategy(energyReq, batteryState_);
    
    // 设置安全限制
    applySafetyLimits(strategy, engineState, motorState);
    
    return strategy;
}

ControlCommands ModelIntegration::generateControlCommands(const ControlStrategy& strategy,
                                                        const PerceptionData& perception,
                                                        const PredictionResult& prediction) {
    ControlCommands commands;
    
    // 生成发动机控制命令
    commands.engineTorque = strategy.torqueSplit.engineTorque;
    commands.engineSpeed = calculateOptimalEngineSpeed(strategy, perception);
    commands.fuelInjection = calculateFuelInjection(commands.engineTorque, commands.engineSpeed);
    
    // 生成电机控制命令
    commands.motorTorque = strategy.torqueSplit.motorTorque;
    commands.motorSpeed = calculateOptimalMotorSpeed(strategy, perception);
    commands.motorMode = determineMotorMode(strategy.operatingMode);
    
    // 生成变速器控制命令
    commands.gearRatio = calculateOptimalGearRatio(strategy, perception);
    commands.cvtMode = determineCVTMode(strategy.transmissionStrategy);
    
    // 生成能量管理命令
    commands.chargingPower = strategy.energyManagement.chargingPower;
    commands.dischargingPower = strategy.energyManagement.dischargingPower;
    commands.energyMode = strategy.energyManagement.energyMode;
    
    // 设置时间戳
    commands.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    return commands;
}

bool ModelIntegration::validateControlCommands(const ControlCommands& commands) const {
    // 检查发动机限制
    if (commands.engineTorque > engineModel_->getMaxAvailableTorque() * 1.1f) {
        std::cerr << "Engine torque command exceeds limits" << std::endl;
        return false;
    }
    
    if (commands.engineSpeed > engineModel_->getMaxRPM() * 1.1f) {
        std::cerr << "Engine speed command exceeds limits" << std::endl;
        return false;
    }
    
    // 检查电机限制
    if (commands.motorTorque > motorModel_->getMotorParameters().peakTorque * 1.1f) {
        std::cerr << "Motor torque command exceeds limits" << std::endl;
        return false;
    }
    
    if (commands.motorSpeed > motorModel_->getMotorParameters().maxSpeed * 1.1f) {
        std::cerr << "Motor speed command exceeds limits" << std::endl;
        return false;
    }
    
    // 检查能量管理
    if (commands.chargingPower < 0 || commands.dischargingPower < 0) {
        std::cerr << "Invalid power command" << std::endl;
        return false;
    }
    
    return true;
}

void ModelIntegration::activateFallbackMode() {
    std::cerr << "Activating fallback control mode" << std::endl;
    
    // 使用保守的控制策略
    ControlCommands fallbackCommands;
    
    // 设置安全的默认值
    fallbackCommands.engineTorque = engineModel_->getRatedTorque() * 0.5f;
    fallbackCommands.engineSpeed = engineModel_->getOptimalEfficiencySpeed();
    fallbackCommands.motorTorque = 0.0f;
    fallbackCommands.motorSpeed = 0.0f;
    fallbackCommands.gearRatio = 1.0f;
    fallbackCommands.chargingPower = 0.0f;
    fallbackCommands.dischargingPower = 0.0f;
    fallbackCommands.energyMode = EnergyMode::CONSERVATIVE;
    
    lastControlCommands_ = fallbackCommands;
    publishControlCommands(fallbackCommands);
}

void ModelIntegration::updatePerformanceStats(float updateTime) {
    performanceStats_.totalUpdates++;
    performanceStats_.averageUpdateTime = 
        (performanceStats_.averageUpdateTime * (performanceStats_.totalUpdates - 1) + updateTime) /
        performanceStats_.totalUpdates;
    
    performanceStats_.maxUpdateTime = std::max(performanceStats_.maxUpdateTime, updateTime);
    performanceStats_.minUpdateTime = std::min(performanceStats_.minUpdateTime, updateTime);
}

void ModelIntegration::checkSystemHealth() {
    // 检查温度限制
    EngineState engineState = engineModel_->getEngineState();
    MotorState motorState = motorModel_->getMotorState();
    
    if (engineState.currentTemperature > config_.emergencyShutdownTemp * 0.9f ||
        motorState.windingTemperature > config_.emergencyShutdownTemp * 0.9f) {
        std::cerr << "Warning: High temperature detected" << std::endl;
        // 可以触发降级模式或报警
    }
    
    // 检查电池状态
    BatteryData battery = getLatestBatteryData();
    if (battery.soc < config_.minBatterySOC) {
        std::cerr << "Warning: Low battery SOC" << std::endl;
    }
    
    // 检查模型收敛性
    if (performanceStats_.modelConvergenceRate < 0.8f) {
        std::cerr << "Warning: Model convergence rate low" << std::endl;
    }
}

void ModelIntegration::checkModelConvergence() {
    // 检查模型收敛状态
    int successfulUpdates = performanceStats_.totalUpdates - performanceStats_.failedUpdates;
    performanceStats_.modelConvergenceRate = 
        static_cast<float>(successfulUpdates) / performanceStats_.totalUpdates;
}

void ModelIntegration::checkResourceUsage() {
    // 检查CPU和内存使用（简化实现）
    // 在实际系统中，这里会调用系统API获取资源使用情况
    
    static int checkCount = 0;
    checkCount++;
    
    if (checkCount % 60 == 0) { // 每分钟记录一次
        std::cout << "System resource check - Updates: " << performanceStats_.totalUpdates 
                  << ", Failed: " << performanceStats_.failedUpdates
                  << ", Convergence: " << performanceStats_.modelConvergenceRate << std::endl;
    }
}

float ModelIntegration::estimateVehicleMass(const SensorData& sensor) const {
    // 基于加速度和力的质量估计
    float totalForce = sensor.engineForce + sensor.motorForce;
    float acceleration = sensor.accelX; // 主要运动方向
    
    if (std::abs(acceleration) > 0.1f) {
        return totalForce / acceleration;
    }
    
    return 5000.0f; // 默认质量
}

float ModelIntegration::calculateEnergyEfficiency(const SensorData& sensor, 
                                                const BatteryData& battery) const {
    float totalPower = sensor.enginePower + sensor.motorPower;
    float fuelEnergy = sensor.fuelConsumption * 10.0f; // 柴油热值约10 kWh/L
    float batteryEnergy = battery.power;
    
    if (totalPower > 0.0f) {
        return (fuelEnergy + batteryEnergy) / totalPower;
    }
    
    return 0.0f;
}

OperatingMode ModelIntegration::determineOperatingMode(const EnergyRequirements& energyReq,
                                                     const EngineState& engineState,
                                                     const MotorState& motorState) const {
    // 基于能量需求和系统状态确定工作模式
    if (energyReq.totalPower < 20.0f) {
        return OperatingMode::ELECTRIC_ONLY;
    } else if (energyReq.totalPower < 50.0f) {
        return OperatingMode::HYBRID_ECONOMY;
    } else if (energyReq.totalPower < 100.0f) {
        return OperatingMode::HYBRID_POWER;
    } else {
        return OperatingMode::POWER_BOOST;
    }
}

TorqueSplit ModelIntegration::calculateTorqueSplit(OperatingMode mode,
                                                 const EnergyRequirements& energyReq,
                                                 const EngineState& engineState,
                                                 const MotorState& motorState) const {
    TorqueSplit split;
    
    switch (mode) {
        case OperatingMode::ELECTRIC_ONLY:
            split.engineTorque = 0.0f;
            split.motorTorque = energyReq.totalTorque;
            break;
            
        case OperatingMode::HYBRID_ECONOMY:
            split.engineTorque = energyReq.totalTorque * 0.7f;
            split.motorTorque = energyReq.totalTorque * 0.3f;
            break;
            
        case OperatingMode::HYBRID_POWER:
            split.engineTorque = energyReq.totalTorque * 0.5f;
            split.motorTorque = energyReq.totalTorque * 0.5f;
            break;
            
        case OperatingMode::POWER_BOOST:
            split.engineTorque = std::min(energyReq.totalTorque, engineState.maxAvailableTorque);
            split.motorTorque = energyReq.totalTorque - split.engineTorque;
            break;
    }
    
    return split;
}

EnergyRequirements ModelIntegration::calculateEnergyRequirements(const PerceptionData& perception,
                                                               const PredictionResult& prediction) const {
    EnergyRequirements req;
    
    // 计算当前能量需求
    req.currentPower = perception.vehicleState.powerConsumption;
    req.currentTorque = perception.implementForce;
    
    // 预测未来需求
    if (!prediction.energyDemand.empty()) {
        req.predictedPower = prediction.energyDemand[0];
    } else {
        req.predictedPower = req.currentPower;
    }
    
    if (!prediction.loadForecast.empty()) {
        req.predictedTorque = prediction.loadForecast[0];
    } else {
        req.predictedTorque = req.currentTorque;
    }
    
    // 总需求
    req.totalPower = std::max(req.currentPower, req.predictedPower);
    req.totalTorque = std::max(req.currentTorque, req.predictedTorque);
    
    return req;
}

void ModelIntegration::applySafetyLimits(ControlStrategy& strategy,
                                       const EngineState& engineState,
                                       const MotorState& motorState) const {
    // 应用发动机安全限制
    strategy.torqueSplit.engineTorque = std::min(
        strategy.torqueSplit.engineTorque,
        engineState.maxAvailableTorque * config_.thermalSafetyMargin
    );
    
    // 应用电机安全限制
    strategy.torqueSplit.motorTorque = std::min(
        strategy.torqueSplit.motorTorque,
        motorModel_->getMotorParameters().peakTorque * config_.thermalSafetyMargin
    );
    
    // 应用电池安全限制
    if (strategy.energyManagement.chargingPower > 0) {
        strategy.energyManagement.chargingPower *= config_.batterySafetyMargin;
    }
    
    if (strategy.energyManagement.dischargingPower > 0) {
        strategy.energyManagement.dischargingPower *= config_.batterySafetyMargin;
    }
}

void ModelIntegration::saveToBuffer(const SensorData& sensor, const PerceptionData& perception,
                                  const PredictionResult& prediction, const ControlCommands& commands) {
    // 保存到环形缓冲区
    sensorBuffer_.push_back(sensor);
    perceptionBuffer_.push_back(perception);
    predictionBuffer_.push_back(prediction);
    controlBuffer_.push_back(commands);
}

void ModelIntegration::publishControlCommands(const ControlCommands& commands) {
    // 在实际系统中，这里会将控制命令发布到CAN总线或其他通信接口
    // 简化实现：只更新最后命令
    
    lastControlCommands_ = commands;
    
    if (commandCallback_) {
        commandCallback_(commands);
    }
}

ControlCommands ModelIntegration::getLastControlCommands() const {
    pthread_mutex_lock(&dataMutex_);
    ControlCommands commands = lastControlCommands_;
    pthread_mutex_unlock(&dataMutex_);
    return commands;
}

IntegrationPerformance ModelIntegration::getPerformanceStats() const {
    return performanceStats_;
}

void ModelIntegration::setCommandCallback(CommandCallback callback) {
    commandCallback_ = callback;
}

bool ModelIntegration::saveDataToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        pthread_mutex_lock(&dataMutex_);
        
        // 写入标题
        file << "Timestamp,EngineTorque,MotorTorque,EngineRPM,MotorRPM,"
             << "FuelConsumption,BatterySOC,Efficiency\n";
        
        // 写入数据
        for (const auto& cmd : controlBuffer_) {
            file << cmd.timestamp.count() << ","
                 << cmd.engineTorque << ","
                 << cmd.motorTorque << ","
                 // 这里需要对应的传感器数据，简化实现
                 << "0,0,0,0,0\n";
        }
        
        pthread_mutex_unlock(&dataMutex_);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving data: " << e.what() << std::endl;
        pthread_mutex_unlock(&dataMutex_);
        return false;
    }
}

} // namespace VCUCore