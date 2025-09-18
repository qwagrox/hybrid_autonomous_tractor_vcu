// src/system_integration.cpp
#include "system_integration.hpp"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace VCUCore {

SystemIntegration::SystemIntegration() 
    : isInitialized_(false), isRunning_(false) {
    
    // 初始化组件
    canInterface_ = std::make_unique<CANBusInterface>();
    sensorFusion_ = std::make_unique<SensorFusion>();
    loadDetector_ = std::make_unique<LoadDetector>();
    predictiveAnalytics_ = std::make_unique<PredictiveAnalytics>();
    torqueArbiter_ = std::make_unique<TorqueArbiter>();
    cvtController_ = std::make_unique<CVTController>();
    energyManager_ = std::make_unique<EnergyManager>();
    actuatorInterface_ = std::make_unique<ActuatorInterface>();
    safetyMonitor_ = std::make_unique<SafetyMonitor>();
    faultHandler_ = std::make_unique<FaultHandler>();
    healthMonitor_ = std::make_unique<HealthMonitor>();
    dataLogger_ = std::make_unique<DataLogger>();
    adaptiveLearner_ = std::make_unique<AdaptiveLearner>();
}

bool SystemIntegration::initialize(const std::string& configPath) {
    if (isInitialized_) {
        return true;
    }
    
    try {
        std::cout << "Initializing VCU System..." << std::endl;
        
        // 加载系统配置
        if (!loadSystemConfig(configPath + "/system_config.yaml")) {
            throw std::runtime_error("Failed to load system configuration");
        }
        
        // 初始化硬件接口
        std::cout << "Initializing hardware interfaces..." << std::endl;
        if (!initializeHardware()) {
            throw std::runtime_error("Hardware initialization failed");
        }
        
        // 初始化算法模块
        std::cout << "Initializing algorithm modules..." << std::endl;
        if (!initializeAlgorithms()) {
            throw std::runtime_error("Algorithm initialization failed");
        }
        
        // 初始化诊断系统
        std::cout << "Initializing diagnostic systems..." << std::endl;
        if (!initializeDiagnostics()) {
            throw std::runtime_error("Diagnostic initialization failed");
        }
        
        // 启动数据记录
        std::cout << "Starting data logging..." << std::endl;
        if (!startDataLogging()) {
            throw std::runtime_error("Data logging startup failed");
        }
        
        isInitialized_ = true;
        std::cout << "VCU System initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "System initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool SystemIntegration::loadSystemConfig(const std::string& configFile) {
    try {
        YAML::Node config = YAML::LoadFile(configFile);
        
        // 加载系统参数
        systemConfig_.systemName = config["system"]["name"].as<std::string>();
        systemConfig_.systemVersion = config["system"]["version"].as<std::string>();
        systemConfig_.controlRate = config["system"]["control_rate"].as<uint32_t>();
        systemConfig_.simulationMode = config["system"]["simulation_mode"].as<bool>();
        
        // 加载车辆参数
        systemConfig_.vehicleMass = config["vehicle"]["mass"].as<float>();
        systemConfig_.wheelRadius = config["vehicle"]["wheel_radius"].as<float>();
        systemConfig_.rollingResistanceCoeff = config["vehicle"]["rolling_resistance"].as<float>();
        
        // 加载动力总成参数
        systemConfig_.engineMaxTorque = config["powertrain"]["engine"]["max_torque"].as<float>();
        systemConfig_.engineMaxPower = config["powertrain"]["engine"]["max_power"].as<float>();
        systemConfig_.motorMaxTorque = config["powertrain"]["motor"]["max_torque"].as<float>();
        systemConfig_.batteryCapacity = config["powertrain"]["battery"]["capacity"].as<float>();
        
        // 加载控制参数
        systemConfig_.cvtParams.minRatio = config["control"]["cvt_controller"]["min_ratio"].as<float>();
        systemConfig_.cvtParams.maxRatio = config["control"]["cvt_controller"]["max_ratio"].as<float>();
        
        std::cout << "System configuration loaded successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading system config: " << e.what() << std::endl;
        return false;
    }
}

bool SystemIntegration::initializeHardware() {
    try {
        // 初始化CAN接口
        if (!canInterface_->initialize("can0")) {
            throw std::runtime_error("CAN interface initialization failed");
        }
        
        // 初始化执行器接口
        if (!actuatorInterface_->initialize()) {
            throw std::runtime_error("Actuator interface initialization failed");
        }
        
        // 注册CAN消息处理器
        canInterface_->registerMessageHandler(0x0CF00400, [this](const can_frame& frame) {
            this->handleEngineData(frame);
        });
        
        canInterface_->registerMessageHandler(0x0CFF1000, [this](const can_frame& frame) {
            this->handleMotorData(frame);
        });
        
        std::cout << "Hardware interfaces initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Hardware initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool SystemIntegration::initializeAlgorithms() {
    try {
        // 初始化传感器融合
        if (!sensorFusion_->initialize()) {
            throw std::runtime_error("Sensor fusion initialization failed");
        }
        
        // 初始化预测分析
        if (!predictiveAnalytics_->initialize(5.0f, 0.1f)) {
            throw std::runtime_error("Predictive analytics initialization failed");
        }
        
        // 初始化能量管理器
        if (!energyManager_->initialize()) {
            throw std::runtime_error("Energy manager initialization failed");
        }
        
        std::cout << "Algorithm modules initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Algorithm initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool SystemIntegration::initializeDiagnostics() {
    try {
        // 初始化故障处理器
        if (!faultHandler_->initialize()) {
            throw std::runtime_error("Fault handler initialization failed");
        }
        
        // 初始化健康监控
        if (!healthMonitor_->initialize()) {
            throw std::runtime_error("Health monitor initialization failed");
        }
        
        // 初始化自适应学习器
        if (!adaptiveLearner_->initialize()) {
            throw std::runtime_error("Adaptive learner initialization failed");
        }
        
        std::cout << "Diagnostic systems initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Diagnostic initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool SystemIntegration::startDataLogging() {
    try {
        DataLogger::LogConfig logConfig;
        logConfig.directory = "/var/log/vcu";
        logConfig.maxFileSizeMB = 100;
        logConfig.maxFiles = 10;
        logConfig.flushIntervalMs = 1000;
        
        if (!dataLogger_->initialize(logConfig)) {
            throw std::runtime_error("Data logger initialization failed");
        }
        
        if (!dataLogger_->start()) {
            throw std::runtime_error("Data logger startup failed");
        }
        
        std::cout << "Data logging started successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Data logging startup error: " << e.what() << std::endl;
        return false;
    }
}

void SystemIntegration::runMainLoop() {
    if (!isInitialized_) {
        std::cerr << "System not initialized" << std::endl;
        return;
    }
    
    isRunning_ = true;
    std::cout << "Starting main control loop..." << std::endl;
    
    auto lastTime = std::chrono::high_resolution_clock::now();
    const auto cycleTime = std::chrono::milliseconds(1000 / systemConfig_.controlRate);
    
    while (isRunning_) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);
        
        if (elapsed >= cycleTime) {
            try {
                executeControlCycle();
                lastTime = currentTime;
            } catch (const std::exception& e) {
                std::cerr << "Control cycle error: " << e.what() << std::endl;
                handleControlError(e);
            }
        }
        
        // 休眠剩余时间
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SystemIntegration::executeControlCycle() {
    // 1. 数据采集
    SensorData sensorData = acquireSensorData();
    
    // 2. 感知和状态估计
    PerceptionData perception = perceiveEnvironment(sensorData);
    
    // 3. 安全检查
    SafetyCheckResult safetyCheck = safetyMonitor_->checkSafety(
        lastControlCommands_, perception.vehicleState, healthMonitor_->checkSystemHealth());
    
    if (!safetyCheck.overallSafe) {
        handleSafetyViolation(safetyCheck);
        return;
    }
    
    // 4. 预测分析
    PredictionResult prediction = predictiveAnalytics_->analyzeFuture(perception);
    
    // 5. 控制决策
    ControlCommands commands = makeControlDecisions(perception, prediction);
    
    // 6. 能量管理
    EnergyOptimization energyOpt = energyManager_->optimizeEnergyUsage(
        commands, perception, prediction);
    commands = applyEnergyOptimization(commands, energyOpt);
    
    // 7. 执行控制
    if (actuatorInterface_->sendControlCommands(commands)) {
        lastControlCommands_ = commands;
    }
    
    // 8. 数据记录
    logCycleData(sensorData, perception, commands, energyOpt);
    
    // 9. 学习更新
    adaptiveLearner_->updateModels({sensorData}, {perception.vehicleState}, {commands});
    
    // 10. 诊断检查
    performDiagnosticChecks();
}

SensorData SystemIntegration::acquireSensorData() {
    SensorData data;
    
    // 从CAN总线获取数据
    auto canFrames = canInterface_->receiveCANFrames(5);
    for (const auto& frame : canFrames) {
        processCANFrame(frame, data);
    }
    
    // 设置时间戳
    data.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    return data;
}

PerceptionData SystemIntegration::perceiveEnvironment(const SensorData& sensorData) {
    PerceptionData perception;
    
    // 传感器融合
    perception.vehicleState = sensorFusion_->fuseSensors(sensorData);
    
    // 负载检测
    auto loadChange = loadDetector_->detectLoadChange(sensorData, perception.vehicleState);
    perception.loadChangeType = loadChange.changeType;
    perception.loadTrend = loadChange.trend;
    
    // 地形分析
    perception.terrainSlope = calculateTerrainSlope(sensorData);
    perception.soilResistance = estimateSoilResistance(sensorData, perception.vehicleState);
    
    return perception;
}

ControlCommands SystemIntegration::makeControlDecisions(const PerceptionData& perception,
                                                      const PredictionResult& prediction) {
    
    ControlCommands commands;
    
    // 扭矩分配决策
    auto torqueSplit = torqueArbiter_->decideDistribution(perception, prediction);
    commands.engineTorqueRequest = torqueSplit.engineTorque;
    commands.motorTorqueRequest = torqueSplit.motorTorque;
    
    // CVT控制决策
    commands.cvtRatioRequest = cvtController_->calculateOptimalRatio(perception, prediction);
    
    // 设置时间戳
    commands.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    return commands;
}

void SystemIntegration::handleSafetyViolation(const SafetyCheckResult& safetyCheck) {
    std::cerr << "Safety violation detected!" << std::endl;
    
    // 记录安全违规
    for (const auto& violation : safetyCheck.violations) {
        FaultDiagnosis fault = {
            .faultCode = 0x4000 + static_cast<uint16_t>(violation.type),
            .severity = FaultSeverity::HIGH,
            .description = "Safety violation: " + violation.component,
            .component = "SafetySystem",
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch()).count(),
            .duration = 0,
            .isActive = true,
            .isRecoverable = true,
            .recoverySteps = {"Check safety limits", "Verify system operation"}
        };
        faultHandler_->handleFaults({fault});
    }
    
    // 应用安全限制
    ControlCommands safeCommands = safetyMonitor_->applySafetyLimits(
        lastControlCommands_, perception_.vehicleState);
    
    if (actuatorInterface_->sendControlCommands(safeCommands)) {
        lastControlCommands_ = safeCommands;
    }
}

void SystemIntegration::performDiagnosticChecks() {
    // 检查系统健康
    SystemHealthStatus healthStatus = healthMonitor_->checkSystemHealth();
    
    // 检测和处理故障
    auto faults = faultHandler_->detectFaults(perception_.vehicleState, healthStatus);
    if (!faults.empty()) {
        faultHandler_->handleFaults(faults);
    }
    
    // 更新自适应学习
    if (healthStatus.isHealthy) {
        adaptiveLearner_->updateModels(
            {sensorData_}, {perception_.vehicleState}, {lastControlCommands_});
    }
}

void SystemIntegration::emergencyStop() {
    std::cout << "EMERGENCY STOP ACTIVATED" << std::endl;
    
    // 停止所有执行器
    actuatorInterface_->emergencyStop();
    
    // 更新系统状态
    systemState_ = SystemState::EMERGENCY;
    
    // 记录紧急事件
    FaultDiagnosis emergencyFault = {
        .faultCode = 0xFFFF,
        .severity = FaultSeverity::CRITICAL,
        .description = "Emergency stop activated",
        .component = "System",
        .timestamp = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count(),
        .duration = 0,
        .isActive = true,
        .isRecoverable = true,
        .recoverySteps = {"Investigate emergency cause", "Perform system reset"}
    };
    faultHandler_->handleFaults({emergencyFault});
}

void SystemIntegration::shutdown() {
    std::cout << "Shutting down VCU System..." << std::endl;
    
    isRunning_ = false;
    
    // 安全停止所有组件
    actuatorInterface_->safeShutdown();
    dataLogger_->stop();
    
    // 保存学习状态
    adaptiveLearner_->saveLearningState("/var/lib/vcu/learning_state.bin");
    
    std::cout << "VCU System shutdown complete" << std::endl;
}

} // namespace VCUCore