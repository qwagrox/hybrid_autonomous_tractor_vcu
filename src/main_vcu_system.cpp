// src/main_vcu_system.cpp
#include "vcu_core_types.hpp"
#include "can_bus_interface.hpp"
#include "perception/sensor_fusion.hpp"
#include "perception/load_detector.hpp"
#include "prediction/predictive_analytics.hpp"
#include "prediction/load_forecaster.hpp"
#include "prediction/energy_predictor.hpp"
#include "control/torque_arbiter.hpp"
#include "control/cvt_controller.hpp"
#include "control/energy_manager.hpp"
#include "execution/actuator_interface.hpp"
#include "execution/safety_monitor.hpp"
#include "execution/fault_handler.hpp"
#include "diagnostic/health_monitor.hpp"
#include "diagnostic/data_logger.hpp"
#include "diagnostic/adaptive_learner.hpp"
#include "hardware/can_driver.hpp"
#include "hardware/gpio_driver.hpp"
#include "hardware/adc_driver.hpp"

#include <iostream>
#include <fstream>
#include <signal.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <sys/time.h>

namespace VCUCore {

class MainVCUSystem {
private:
    // 系统组件
    std::unique_ptr<CANBusInterface> canInterface_;
    std::unique_ptr<SensorFusion> sensorFusion_;
    std::unique_ptr<LoadDetector> loadDetector_;
    std::unique_ptr<PredictiveAnalytics> predictiveAnalytics_;
    std::unique_ptr<LoadForecaster> loadForecaster_;
    std::unique_ptr<EnergyPredictor> energyPredictor_;
    std::unique_ptr<TorqueArbiter> torqueArbiter_;
    std::unique_ptr<CVTController> cvtController_;
    std::unique_ptr<EnergyManager> energyManager_;
    std::unique_ptr<ActuatorInterface> actuatorInterface_;
    std::unique_ptr<SafetyMonitor> safetyMonitor_;
    std::unique_ptr<FaultHandler> faultHandler_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    std::unique_ptr<DataLogger> dataLogger_;
    std::unique_ptr<AdaptiveLearner> adaptiveLearner_;
    
    // 硬件驱动
    std::unique_ptr<CANDriver> canDriver_;
    std::unique_ptr<GPIODriver> gpioDriver_;
    std::unique_ptr<ADCDriver> adcDriver_;

    // 系统状态
    std::atomic<bool> running_{false};
    std::atomic<bool> emergencyStop_{false};
    std::thread controlThread_;
    std::thread monitoringThread_;
    std::thread diagnosticThread_;
    
    SystemConfig systemConfig_;
    VehicleState currentState_;
    ControlCommands currentCommands_;
    SystemHealthStatus healthStatus_;
    PerformanceStatistics performanceStats_;
    
    // 时间和同步
    std::chrono::steady_clock::time_point startTime_;
    uint64_t cycleCount_{0};
    std::mutex dataMutex_;
    std::condition_variable cv_;

    // 数据缓冲区
    std::deque<SensorData> sensorDataBuffer_;
    std::deque<VehicleState> stateBuffer_;
    std::deque<ControlCommands> commandBuffer_;

public:
    MainVCUSystem(const std::string& configPath = "config/") {
        initializeSystem(configPath);
    }
    
    ~MainVCUSystem() {
        stop();
    }
    
    bool initializeSystem(const std::string& configPath) {
        try {
            std::cout << "Initializing VCU System..." << std::endl;
            
            // 加载配置
            if (!loadConfiguration(configPath)) {
                throw std::runtime_error("Failed to load configuration");
            }
            
            // 初始化硬件驱动
            if (!initializeHardware()) {
                throw std::runtime_error("Failed to initialize hardware");
            }
            
            // 初始化算法模块
            if (!initializeAlgorithms()) {
                throw std::runtime_error("Failed to initialize algorithms");
            }
            
            // 启动看门狗
            startWatchdog();
            
            // 初始化数据记录
            initializeDataLogging();
            
            std::cout << "VCU System initialized successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "System initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void start() {
        if (running_) {
            std::cout << "System is already running" << std::endl;
            return;
        }
        
        running_ = true;
        startTime_ = std::chrono::steady_clock::now();
        
        // 启动控制线程
        controlThread_ = std::thread(&MainVCUSystem::controlLoop, this);
        
        // 启动监控线程
        monitoringThread_ = std::thread(&MainVCUSystem::monitoringLoop, this);
        
        // 启动诊断线程
        diagnosticThread_ = std::thread(&MainVCUSystem::diagnosticLoop, this);
        
        std::cout << "VCU System started successfully" << std::endl;
    }
    
    void stop() {
        if (!running_) {
            return;
        }
        
        running_ = false;
        cv_.notify_all();
        
        if (controlThread_.joinable()) {
            controlThread_.join();
        }
        
        if (monitoringThread_.joinable()) {
            monitoringThread_.join();
        }
        
        if (diagnosticThread_.joinable()) {
            diagnosticThread_.join();
        }
        
        // 安全关闭所有组件
        shutdownSystem();
        
        std::cout << "VCU System stopped" << std::endl;
    }
    
    // 系统状态查询
    SystemState getSystemState() const {
        std::lock_guard<std::mutex> lock(dataMutex_);
        return currentState_.systemState;
    }
    
    SystemHealthStatus getHealthStatus() const {
        std::lock_guard<std::mutex> lock(dataMutex_);
        return healthStatus_;
    }
    
    PerformanceStatistics getPerformanceStats() const {
        std::lock_guard<std::mutex> lock(dataMutex_);
        return performanceStats_;
    }
    
    void emergencyStop() {
        emergencyStop_ = true;
        if (actuatorInterface_) {
            actuatorInterface_->emergencyStop();
        }
    }
    
    void resumeFromEmergency() {
        emergencyStop_ = false;
        if (actuatorInterface_) {
            actuatorInterface_->resumeFromEmergency();
        }
    }

private:
    bool loadConfiguration(const std::string& configPath) {
        try {
            std::cout << "Loading configuration from: " << configPath << std::endl;
            
            // 加载主配置文件
            YAML::Node config = YAML::LoadFile(configPath + "/system_config.yaml");
            
            // 解析系统配置
            systemConfig_.systemName = config["system"]["name"].as<std::string>();
            systemConfig_.systemVersion = config["system"]["version"].as<std::string>();
            systemConfig_.controlRate = config["system"]["control_rate"].as<uint32_t>();
            systemConfig_.simulationMode = config["system"]["simulation_mode"].as<bool>();
            systemConfig_.logLevel = config["system"]["log_level"].as<std::string>();
            
            // 解析车辆配置
            systemConfig_.vehicleMass = config["vehicle"]["mass"].as<float>();
            systemConfig_.wheelRadius = config["vehicle"]["wheel_radius"].as<float>();
            systemConfig_.rollingResistanceCoeff = config["vehicle"]["rolling_resistance"].as<float>();
            
            // 解析动力总成配置
            systemConfig_.engineMaxTorque = config["powertrain"]["engine"]["max_torque"].as<float>();
            systemConfig_.engineMaxPower = config["powertrain"]["engine"]["max_power"].as<float>();
            systemConfig_.batteryCapacity = config["powertrain"]["battery"]["capacity"].as<float>();
            
            std::cout << "Configuration loaded successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading configuration: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initializeHardware() {
        try {
            std::cout << "Initializing hardware..." << std::endl;
            
            // 初始化CAN驱动
            canDriver_ = std::make_unique<CANDriver>();
            if (!canDriver_->initialize("can0", 500000)) {
                throw std::runtime_error("Failed to initialize CAN driver");
            }
            
            // 初始化GPIO驱动
            gpioDriver_ = std::make_unique<GPIODriver>();
            if (!gpioDriver_->initialize()) {
                throw std::runtime_error("Failed to initialize GPIO driver");
            }
            
            // 初始化ADC驱动
            adcDriver_ = std::make_unique<ADCDriver>();
            if (!adcDriver_->initialize()) {
                throw std::runtime_error("Failed to initialize ADC driver");
            }
            
            // 初始化CAN总线接口
            canInterface_ = std::make_unique<CANBusInterface>("can0");
            if (!canInterface_->initialize()) {
                throw std::runtime_error("Failed to initialize CAN bus interface");
            }
            
            std::cout << "Hardware initialized successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Hardware initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initializeAlgorithms() {
        try {
            std::cout << "Initializing algorithms..." << std::endl;
            
            // 初始化感知算法
            sensorFusion_ = std::make_unique<SensorFusion>(1000, 3.0f);
            loadDetector_ = std::make_unique<LoadDetector>(500, 30.0f, 1000.0f, 0.05f, 10.0f);
            
            // 初始化预测算法
            predictiveAnalytics_ = std::make_unique<PredictiveAnalytics>();
            loadForecaster_ = std::make_unique<LoadForecaster>();
            energyPredictor_ = std::make_unique<EnergyPredictor>();
            
            // 初始化控制算法
            torqueArbiter_ = std::make_unique<TorqueArbiter>();
            cvtController_ = std::make_unique<CVTController>();
            energyManager_ = std::make_unique<EnergyManager>();
            
            // 初始化执行层
            actuatorInterface_ = std::make_unique<ActuatorInterface>();
            safetyMonitor_ = std::make_unique<SafetyMonitor>();
            faultHandler_ = std::make_unique<FaultHandler>();
            
            // 初始化诊断层
            healthMonitor_ = std::make_unique<HealthMonitor>();
            dataLogger_ = std::make_unique<DataLogger>("/var/log/vcu");
            adaptiveLearner_ = std::make_unique<AdaptiveLearner>();
            
            std::cout << "Algorithms initialized successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Algorithm initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void initializeDataLogging() {
        if (dataLogger_) {
            dataLogger_->setLogLevel(systemConfig_.logLevel);
            dataLogger_->setMaxFileSize(systemConfig_.maxLogSizeMB);
            dataLogger_->setRetentionDays(systemConfig_.logRetentionDays);
            dataLogger_->startLogging();
        }
    }
    
    void startWatchdog() {
        // 启动硬件看门狗
        if (gpioDriver_) {
            gpioDriver_->startWatchdog(5000); // 5秒超时
        }
    }
    
    void controlLoop() {
        auto nextCycleTime = std::chrono::steady_clock::now();
        const auto cycleDuration = std::chrono::milliseconds(1000 / systemConfig_.controlRate);
        
        std::cout << "Control loop started at " << systemConfig_.controlRate << "Hz" << std::endl;
        
        while (running_) {
            auto cycleStart = std::chrono::steady_clock::now();
            
            try {
                if (emergencyStop_) {
                    handleEmergencyState();
                } else {
                    executeControlCycle();
                }
                
                updatePerformanceStatistics(cycleStart);
                
            } catch (const std::exception& e) {
                handleControlCycleError(e);
            }
            
            cycleCount_++;
            
            // 精确的时间控制
            nextCycleTime += cycleDuration;
            std::this_thread::sleep_until(nextCycleTime);
            
            // 检查实时性约束
            if (!checkRealtimePerformance(cycleStart)) {
                handleRealtimeViolation();
            }
        }
    }
    
    void executeControlCycle() {
        // 1. 数据采集
        SensorData sensorData = acquireSensorData();
        
        // 2. 感知和状态估计
        PerceptionData perception = perceiveEnvironment(sensorData);
        
        // 3. 预测分析
        PredictionResult prediction = predictFuture(perception);
        
        // 4. 决策控制
        ControlCommands commands = makeControlDecisions(perception, prediction);
        
        // 5. 安全检查和执行
        if (safetyMonitor_->validateCommands(commands, currentState_)) {
            executeCommands(commands);
        } else {
            handleSafetyViolation(commands);
        }
        
        // 6. 更新系统状态
        updateSystemState(perception, commands);
        
        // 7. 数据记录
        logCycleData(sensorData, perception, commands);
    }
    
    SensorData acquireSensorData() {
        SensorData data;
        
        // 从CAN总线获取数据
        if (canInterface_) {
            canInterface_->requestCriticalEngineParameters();
            auto frames = canInterface_->receiveCANFrames(5);
            
            for (const auto& frame : frames) {
                // 解析发动机数据
                if ((frame.can_id & 0x1FFFF00) == 0x0CF00400) {
                    auto engineData = canInterface_->parseEngineData(frame);
                    data.engineRpm = engineData.speed;
                    data.fuelRate = engineData.fuelRate;
                }
            }
        }
        
        // 从ADC获取模拟传感器数据
        if (adcDriver_) {
            data.batteryVoltage = adcDriver_->readVoltage(0);
            data.batteryCurrent = adcDriver_->readCurrent(1);
            data.hydraulicPressure = adcDriver_->readPressure(2);
        }
        
        // 设置时间戳
        data.timestamp = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch());
            
        return data;
    }
    
    PerceptionData perceiveEnvironment(const SensorData& sensorData) {
        PerceptionData perception;
        
        // 传感器融合
        if (sensorFusion_) {
            VehicleState state = sensorFusion_->fuseSensors(sensorData);
            perception = sensorFusion_->generatePerceptionData(state, sensorData);
            
            // 更新当前状态
            std::lock_guard<std::mutex> lock(dataMutex_);
            currentState_ = state;
        }
        
        // 负载检测
        if (loadDetector_) {
            auto loadChange = loadDetector_->detectLoadChange(sensorData, currentState_);
            perception.loadChangeType = loadChange.changeType;
            perception.confidence = loadChange.confidence;
        }
        
        return perception;
    }
    
    PredictionResult predictFuture(const PerceptionData& perception) {
        PredictionResult prediction;
        
        if (predictiveAnalytics_ && loadForecaster_ && energyPredictor_) {
            // 负载预测
            prediction.loadForecast = loadForecaster_->predictLoad(perception);
            
            // 能量需求预测
            prediction.energyDemand = energyPredictor_->predictEnergyDemand(perception);
            
            // 综合预测分析
            prediction = predictiveAnalytics_->analyzeFuture(perception, prediction);
        }
        
        return prediction;
    }
    
    ControlCommands makeControlDecisions(const PerceptionData& perception, 
                                       const PredictionResult& prediction) {
        ControlCommands commands;
        
        // 扭矩分配决策
        if (torqueArbiter_) {
            auto torqueSplit = torqueArbiter_->decideDistribution(perception, prediction);
            commands.engineTorqueRequest = torqueSplit.engineTorque;
            commands.motorTorqueRequest = torqueSplit.motorTorque;
        }
        
        // CVT控制决策
        if (cvtController_) {
            commands.cvtRatioRequest = cvtController_->calculateOptimalRatio(perception, prediction);
        }
        
        // 能量管理决策
        if (energyManager_) {
            commands = energyManager_->optimizeEnergyUsage(commands, perception, prediction);
        }
        
        // 设置时间戳
        commands.timestamp = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch());
            
        return commands;
    }
    
    void executeCommands(const ControlCommands& commands) {
        if (actuatorInterface_) {
            // 发送控制命令
            actuatorInterface_->sendTorqueCommand(commands.engineTorqueRequest, commands.motorTorqueRequest);
            actuatorInterface_->sendCVTRatioCommand(commands.cvtRatioRequest);
            actuatorInterface_->sendImplementCommand(commands.implementLiftRequest);
            
            // 更新当前命令
            std::lock_guard<std::mutex> lock(dataMutex_);
            currentCommands_ = commands;
        }
    }
    
    void updateSystemState(const PerceptionData& perception, const ControlCommands& commands) {
        std::lock_guard<std::mutex> lock(dataMutex_);
        
        // 更新车辆状态
        currentState_ = perception.vehicleState;
        
        // 更新系统状态机
        updateSystemStateMachine();
    }
    
    void updateSystemStateMachine() {
        // 基于当前条件和故障状态更新系统状态
        if (emergencyStop_) {
            currentState_.systemState = SystemState::EMERGENCY;
        } 
        else if (healthStatus_.activeFaults.size() > 0) {
            // 检查故障严重程度
            bool hasCriticalFault = false;
            for (const auto& fault : healthStatus_.activeFaults) {
                if (fault.severity >= FaultSeverity::CRITICAL) {
                    hasCriticalFault = true;
                    break;
                }
            }
            
            if (hasCriticalFault) {
                currentState_.systemState = SystemState::FAULT;
            } else {
                currentState_.systemState = SystemState::DEGRADED;
            }
        }
        else if (currentState_.systemState == SystemState::STANDBY && 
                 currentState_.velocity.norm() > 0.1f) {
            currentState_.systemState = SystemState::RUNNING;
        }
    }
    
    void monitoringLoop() {
        std::cout << "Monitoring loop started" << std::endl;
        
        while (running_) {
            try {
                // 系统健康监控
                if (healthMonitor_) {
                    healthStatus_ = healthMonitor_->checkSystemHealth();
                }
                
                // 实时性能监控
                monitorSystemPerformance();
                
                // 自适应学习
                if (adaptiveLearner_ && healthStatus_.isHealthy) {
                    adaptiveLearner_->updateModels(sensorDataBuffer_, stateBuffer_, commandBuffer_);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
            } catch (const std::exception& e) {
                std::cerr << "Monitoring loop error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    void diagnosticLoop() {
        std::cout << "Diagnostic loop started" << std::endl;
        
        while (running_) {
            try {
                // 故障检测和处理
                if (faultHandler_) {
                    auto faults = faultHandler_->detectFaults(currentState_, healthStatus_);
                    if (!faults.empty()) {
                        faultHandler_->handleFaults(faults);
                    }
                }
                
                // 数据记录和诊断
                if (dataLogger_) {
                    dataLogger_->logDiagnosticData(healthStatus_, performanceStats_);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
            } catch (const std::exception& e) {
                std::cerr << "Diagnostic loop error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
    
    void logCycleData(const SensorData& sensorData, const PerceptionData& perception, 
                     const ControlCommands& commands) {
        if (dataLogger_) {
            // 记录传感器数据
            dataLogger_->logSensorData(sensorData);
            
            // 记录感知数据
            dataLogger_->logPerceptionData(perception);
            
            // 记录控制命令
            dataLogger_->logControlCommands(commands);
            
            // 记录性能数据
            dataLogger_->logPerformanceData(performanceStats_);
        }
        
        // 更新数据缓冲区
        sensorDataBuffer_.push_back(sensorData);
        stateBuffer_.push_back(currentState_);
        commandBuffer_.push_back(commands);
        
        // 保持缓冲区大小
        if (sensorDataBuffer_.size() > 1000) {
            sensorDataBuffer_.pop_front();
        }
        if (stateBuffer_.size() > 1000) {
            stateBuffer_.pop_front();
        }
        if (commandBuffer_.size() > 1000) {
            commandBuffer_.pop_front();
        }
    }
    
    void updatePerformanceStatistics(const std::chrono::steady_clock::time_point& cycleStart) {
        auto cycleEnd = std::chrono::steady_clock::now();
        auto cycleTime = std::chrono::duration_cast<std::chrono::microseconds>(cycleEnd - cycleStart);
        
        performanceStats_.totalCycles = cycleCount_;
        performanceStats_.averageCycleTime = 
            (performanceStats_.averageCycleTime * (cycleCount_ - 1) + cycleTime.count()) / cycleCount_;
        
        if (cycleTime.count() > performanceStats_.maxCycleTime) {
            performanceStats_.maxCycleTime = cycleTime.count();
        }
        
        if (cycleCount_ == 1 || cycleTime.count() < performanceStats_.minCycleTime) {
            performanceStats_.minCycleTime = cycleTime.count();
        }
        
        // 检查截止时间是否满足
        const uint64_t maxCycleTimeUs = 1000000 / systemConfig_.controlRate;
        if (cycleTime.count() > maxCycleTimeUs) {
            performanceStats_.missedDeadlines++;
        }
        
        performanceStats_.timestamp = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    bool checkRealtimePerformance(const std::chrono::steady_clock::time_point& cycleStart) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - cycleStart);
        
        const uint64_t maxAllowedTime = 900000 / systemConfig_.controlRate; // 90% of cycle time
        
        return elapsed.count() <= maxAllowedTime;
    }
    
    void handleRealtimeViolation() {
        std::cerr << "Realtime performance violation detected!" << std::endl;
        
        if (faultHandler_) {
            FaultDiagnosis fault;
            fault.faultCode = 0x1001;
            fault.severity = FaultSeverity::WARNING;
            fault.description = "Realtime performance violation";
            fault.component = "System Scheduler";
            faultHandler_->handleFaults({fault});
        }
    }
    
    void handleControlCycleError(const std::exception& e) {
        std::cerr << "Control cycle error: " << e.what() << std::endl;
        
        if (faultHandler_) {
            FaultDiagnosis fault;
            fault.faultCode = 0x1002;
            fault.severity = FaultSeverity::ERROR;
            fault.description = "Control cycle execution failed: " + std::string(e.what());
            fault.component = "Control System";
            faultHandler_->handleFaults({fault});
        }
    }
    
    void handleSafetyViolation(const ControlCommands& commands) {
        std::cerr << "Safety violation detected in commands!" << std::endl;
        
        // 执行安全限制的命令
        ControlCommands safeCommands = safetyMonitor_->applySafetyLimits(commands, currentState_);
        executeCommands(safeCommands);
        
        if (faultHandler_) {
            FaultDiagnosis fault;
            fault.faultCode = 0x1003;
            fault.severity = FaultSeverity::WARNING;
            fault.description = "Safety violation in control commands";
            fault.component = "Safety System";
            faultHandler_->handleFaults({fault});
        }
    }
    
    void handleEmergencyState() {
        // 紧急停止状态处理
        if (actuatorInterface_) {
            actuatorInterface_->emergencyStop();
        }
        
        // 更新系统状态
        std::lock_guard<std::mutex> lock(dataMutex_);
        currentState_.systemState = SystemState::EMERGENCY;
        
        // 等待紧急状态解除
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    void monitorSystemPerformance() {
        // 监控CPU使用率
        std::ifstream statFile("/proc/stat");
        if (statFile.is_open()) {
            std::string line;
            std::getline(statFile, line);
            // 解析CPU使用率信息
            // 实际实现需要更复杂的CPU使用率计算
        }
        
        // 监控内存使用
        std::ifstream memInfo("/proc/meminfo");
        if (memInfo.is_open()) {
            std::string line;
            // 解析内存信息
        }
    }
    
    void shutdownSystem() {
        std::cout << "Shutting down system components..." << std::endl;
        
        // 安全停止所有执行器
        if (actuatorInterface_) {
            actuatorInterface_->safeShutdown();
        }
        
        // 停止数据记录
        if (dataLogger_) {
            dataLogger_->stopLogging();
        }
        
        // 关闭硬件驱动
        if (canDriver_) {
            canDriver_->shutdown();
        }
        
        if (gpioDriver_) {
            gpioDriver_->shutdown();
        }
        
        if (adcDriver_) {
            adcDriver_->shutdown();
        }
        
        std::cout << "System shutdown complete" << std::endl;
    }
    
    void performAdaptiveLearning() {
        if (adaptiveLearner_ && !sensorDataBuffer_.empty() && !stateBuffer_.empty()) {
            // 使用最近的数据进行学习
            std::vector<SensorData> recentSensors(
                sensorDataBuffer_.begin(), 
                sensorDataBuffer_.end()
            );
            
            std::vector<VehicleState> recentStates(
                stateBuffer_.begin(), 
                stateBuffer_.end()
            );
            
            std::vector<ControlCommands> recentCommands(
                commandBuffer_.begin(), 
                commandBuffer_.end()
            );
            
            adaptiveLearner_->updateModels(recentSensors, recentStates, recentCommands);
        }
    }
};

} // namespace VCUCore

// 全局变量和信号处理
std::atomic<bool> shutdownRequested{false};
std::unique_ptr<VCUCore::MainVCUSystem> vcuSystem;

void signalHandler(int signal) {
    std::cout << "Received signal: " << signal << std::endl;
    
    switch (signal) {
        case SIGINT:  // Ctrl+C
        case SIGTERM: // Termination request
            shutdownRequested = true;
            break;
            
        case SIGUSR1: // Emergency stop
            if (vcuSystem) {
                vcuSystem->emergencyStop();
            }
            break;
            
        case SIGUSR2: // Resume from emergency
            if (vcuSystem) {
                vcuSystem->resumeFromEmergency();
            }
            break;
    }
}

void setupSignalHandlers() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGUSR1, signalHandler);
    std::signal(SIGUSR2, signalHandler);
    std::signal(SIGPIPE, SIG_IGN); // Ignore broken pipe signals
}

int main(int argc, char* argv[]) {
    std::cout << "=== Autonomous Tractor VCU System ===" << std::endl;
    std::cout << "Version: 1.0.0" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    // 设置信号处理
    setupSignalHandlers();
    
    try {
        // 解析命令行参数
        std::string configPath = "config/";
        bool simulationMode = false;
        std::string logLevel = "info";
        
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--config" && i + 1 < argc) {
                configPath = argv[++i];
            } else if (arg == "--simulation") {
                simulationMode = true;
            } else if (arg == "--log-level" && i + 1 < argc) {
                logLevel = argv[++i];
            } else if (arg == "--help") {
                std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << "  --config <path>     Configuration directory path" << std::endl;
                std::cout << "  --simulation        Enable simulation mode" << std::endl;
                std::cout << "  --log-level <level> Log level (debug, info, warning, error)" << std::endl;
                std::cout << "  --help              Show this help message" << std::endl;
                return 0;
            }
        }
        
        // 创建系统实例
        vcuSystem = std::make_unique<VCUCore::MainVCUSystem>(configPath);
        
        // 启动系统
        vcuSystem->start();
        
        std::cout << "System is running. Press Ctrl+C to stop." << std::endl;
        
        // 主循环
        while (!shutdownRequested) {
            // 显示系统状态
            auto state = vcuSystem->getSystemState();
            auto health = vcuSystem->getHealthStatus();
            auto stats = vcuSystem->getPerformanceStats();
            
            std::cout << "\rSystem State: " << static_cast<int>(state)
                      << " | Health: " << (health.isHealthy ? "OK" : "DEGRADED")
                      << " | Cycles: " << stats.totalCycles
                      << " | Avg Time: " << stats.averageCycleTime << "μs"
                      << std::flush;
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // 停止系统
        std::cout << "\nShutting down system..." << std::endl;
        vcuSystem->stop();
        
        std::cout << "System shutdown complete. Goodbye!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}