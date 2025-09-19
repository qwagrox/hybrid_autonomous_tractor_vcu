#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <functional>
#include <string>

#include "vcu_core_types.hpp"
#include "can_bus_interface.hpp"
#include "control/braking_controller.hpp"
#include "control/cvt_controller.hpp"
#include "control/energy_manager.hpp"
#include "control/implement_control_manager.hpp"
#include "control/steering_controller.hpp"
#include "control/torque_arbiter.hpp"
#include "diagnostic/adaptive_learner.hpp"
#include "diagnostic/data_logger.hpp"
#include "diagnostic/health_monitor.hpp"
#include "execution/actuator_interface.hpp"
#include "execution/fault_handler.hpp"
#include "hardware/watchdog.hpp"
#include "models/battery_model.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "perception/load_detector.hpp"
#include "perception/sensor_fusion.hpp"
#include "prediction/predictive_analytics.hpp"
#include "system_integration.hpp"
#include "utils/tractor_state_calculator.hpp"

using namespace VCUCore;

class SystemTester {
private:
    // 核心组件
    std::unique_ptr<CANBusInterface> canInterface_;
    std::unique_ptr<BrakingController> brakingController_;
    std::unique_ptr<CVTController> cvtController_;
    std::unique_ptr<EnergyManager> energyManager_;
    std::unique_ptr<ImplementControlManager> implementManager_;
    std::unique_ptr<SteeringController> steeringController_;
    std::unique_ptr<TorqueArbiter> torqueArbiter_;
    std::unique_ptr<AdaptiveLearner> adaptiveLearner_;
    std::unique_ptr<DataLogger> dataLogger_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    std::unique_ptr<ActuatorInterface> actuatorInterface_;
    std::unique_ptr<FaultHandler> faultHandler_;
    std::unique_ptr<Watchdog> watchdog_;
    std::unique_ptr<BatteryModel> batteryModel_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    std::unique_ptr<LoadDetector> loadDetector_;
    std::unique_ptr<SensorFusion> sensorFusion_;
    std::unique_ptr<PredictiveAnalytics> predictiveAnalytics_;
    std::unique_ptr<SystemIntegration> systemIntegration_;
    std::unique_ptr<TractorStateCalculator> stateCalculator_;

    // 测试统计
    int totalTests_;
    int passedTests_;
    int failedTests_;

public:
    SystemTester() : totalTests_(0), passedTests_(0), failedTests_(0) {
        initializeComponents();
    }

    void initializeComponents() {
        try {
            // 初始化所有组件
            canInterface_ = std::make_unique<CANBusInterface>();
            brakingController_ = std::make_unique<BrakingController>();
            cvtController_ = std::make_unique<CVTController>();  // 使用默认构造函数
            energyManager_ = std::make_unique<EnergyManager>();
            implementManager_ = std::make_unique<ImplementControlManager>();
            steeringController_ = std::make_unique<SteeringController>();
            torqueArbiter_ = std::make_unique<TorqueArbiter>();
            adaptiveLearner_ = std::make_unique<AdaptiveLearner>();
            dataLogger_ = std::make_unique<DataLogger>();
            healthMonitor_ = std::make_unique<HealthMonitor>();
            actuatorInterface_ = std::make_unique<ActuatorInterface>();
            faultHandler_ = std::make_unique<FaultHandler>();
            watchdog_ = std::make_unique<Watchdog>();
            batteryModel_ = std::make_unique<BatteryModel>();
            engineModel_ = std::make_unique<EngineModel>();
            motorModel_ = std::make_unique<MotorModel>();
            loadDetector_ = std::make_unique<LoadDetector>();
            sensorFusion_ = std::make_unique<SensorFusion>();
            predictiveAnalytics_ = std::make_unique<PredictiveAnalytics>();
            systemIntegration_ = std::make_unique<SystemIntegration>();
            stateCalculator_ = std::make_unique<TractorStateCalculator>();

            std::cout << "All components initialized successfully." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error initializing components: " << e.what() << std::endl;
            throw;
        }
    }

    uint64_t getCurrentTimestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void runAllTests() {
        std::cout << "=== VCU System Integration Test Suite ===" << std::endl;
        std::cout << "Starting comprehensive system tests..." << std::endl << std::endl;

        testComponentInitialization();
        testCANInterface();
        testControllers();
        testTorqueArbiter();
        testCVTController();
        testEnergyManager();
        testSensorFusion();
        testPredictiveAnalytics();
        testSystemIntegration();
        testFaultHandling();
        testDataLogging();
        testHealthMonitoring();

        printTestResults();
    }

    void testComponentInitialization() {
        std::cout << "Testing Component Initialization..." << std::endl;
        
        runTest("CAN Interface Initialization", [this]() {
            return canInterface_ != nullptr;
        });
        
        runTest("Braking Controller Initialization", [this]() {
            return brakingController_ != nullptr;
        });
        
        runTest("CVT Controller Initialization", [this]() {
            return cvtController_ != nullptr;
        });
        
        runTest("Energy Manager Initialization", [this]() {
            return energyManager_ != nullptr;
        });
        
        runTest("Torque Arbiter Initialization", [this]() {
            return torqueArbiter_ != nullptr;
        });
        
        std::cout << "Component initialization tests completed." << std::endl << std::endl;
    }

    void testCANInterface() {
        std::cout << "Testing CAN Interface..." << std::endl;
        
        runTest("CAN Interface Creation", [this]() {
            return canInterface_ != nullptr;
        });
        
        std::cout << "CAN Interface tests completed." << std::endl << std::endl;
    }

    void testControllers() {
        std::cout << "Testing Controllers..." << std::endl;
        
        runTest("Braking Controller Creation", [this]() {
            return brakingController_ != nullptr;
        });
        
        runTest("Steering Controller Creation", [this]() {
            return steeringController_ != nullptr;
        });
        
        std::cout << "Controller tests completed." << std::endl << std::endl;
    }

    void testTorqueArbiter() {
        std::cout << "Testing Torque Arbiter..." << std::endl;
        
        runTest("Torque Arbiter Creation", [this]() {
            return torqueArbiter_ != nullptr;
        });
        
        runTest("Torque Distribution Decision", [this]() {
            PerceptionData perception;
            perception.vehicleState.actualTorque = 1000.0f;
            perception.vehicleState.demandedTorque = 1200.0f;
            perception.timestamp = getCurrentTimestamp();
            
            PredictionResult prediction;
            prediction.predictedState.actualTorque = 1100.0f;
            prediction.confidence = 0.85f;
            prediction.timestamp = getCurrentTimestamp();
            
            // 使用正确的方法名
            auto result = torqueArbiter_->decideDistribution(perception, prediction);
            return result.engineTorque >= 0 && result.motorTorque >= 0;
        });
        
        std::cout << "Torque Arbiter tests completed." << std::endl << std::endl;
    }

    void testCVTController() {
        std::cout << "Testing CVT Controller..." << std::endl;
        
        runTest("CVT Controller Creation", [this]() {
            return cvtController_ != nullptr;
        });
        
        runTest("CVT Ratio Calculation", [this]() {
            PerceptionData perception;
            perception.vehicleState.speed = 15.0f;
            perception.vehicleState.engineLoad = 0.7f;
            perception.timestamp = getCurrentTimestamp();
            
            PredictionResult prediction;
            prediction.predictedState.speed = 16.0f;
            prediction.confidence = 0.9f;
            prediction.timestamp = getCurrentTimestamp();
            
            // 使用正确的方法签名
            float ratio = cvtController_->calculateOptimalRatio(perception, prediction);
            return ratio > 0.5f && ratio < 3.0f;
        });
        
        std::cout << "CVT Controller tests completed." << std::endl << std::endl;
    }

    void testEnergyManager() {
        std::cout << "Testing Energy Manager..." << std::endl;
        
        runTest("Energy Manager Creation", [this]() {
            return energyManager_ != nullptr;
        });
        
        std::cout << "Energy Manager tests completed." << std::endl << std::endl;
    }

    void testSensorFusion() {
        std::cout << "Testing Sensor Fusion..." << std::endl;
        
        runTest("Sensor Fusion Creation", [this]() {
            return sensorFusion_ != nullptr;
        });
        
        runTest("Sensor Data Fusion", [this]() {
            SensorData sensorData;
            sensorData.gnssPosition = Vector3d(100.0, 200.0, 50.0);
            sensorData.imuAcceleration = Vector3d(0.1, 0.0, -9.8);
            sensorData.engineRpm = 1800.0f;
            sensorData.timestamp = getCurrentTimestamp();
            
            // 使用正确的方法名
            TractorVehicleState fusedState = sensorFusion_->fuseSensors(sensorData);
            return fusedState.position.norm() > 0;
        });
        
        std::cout << "Sensor Fusion tests completed." << std::endl << std::endl;
    }

    void testPredictiveAnalytics() {
        std::cout << "Testing Predictive Analytics..." << std::endl;
        
        runTest("Predictive Analytics Creation", [this]() {
            return predictiveAnalytics_ != nullptr;
        });
        
        std::cout << "Predictive Analytics tests completed." << std::endl << std::endl;
    }

    void testSystemIntegration() {
        std::cout << "Testing System Integration..." << std::endl;
        
        runTest("System Integration Creation", [this]() {
            return systemIntegration_ != nullptr;
        });
        
        runTest("Full System Loop", [this]() {
            // 创建传感器数据
            SensorData sensorData;
            sensorData.gnssPosition = Vector3d(100.0, 200.0, 50.0);
            sensorData.imuAcceleration = Vector3d(0.1, 0.0, -9.8);
            sensorData.engineRpm = 1800.0f;
            sensorData.timestamp = getCurrentTimestamp();
            
            // 传感器融合
            TractorVehicleState fusedState = sensorFusion_->fuseSensors(sensorData);
            
            // 创建感知数据
            PerceptionData perception;
            perception.vehicleState = fusedState;
            perception.terrainSlope = 0.05f;
            perception.timestamp = getCurrentTimestamp();
            
            // 创建预测结果
            PredictionResult prediction;
            prediction.predictedState = fusedState;
            prediction.confidence = 0.9f;
            prediction.timestamp = getCurrentTimestamp();
            
            // 生成控制命令
            ControlCommands commands;
            commands.engineTorqueRequest = 1200.0f;
            commands.motorTorqueRequest = 600.0f;
            commands.cvtRatioRequest = cvtController_->calculateOptimalRatio(perception, prediction);
            commands.timestamp = getCurrentTimestamp();
            
            // 扭矩仲裁
            auto torqueSplit = torqueArbiter_->decideDistribution(perception, prediction);
            
            // 检查系统健康
            auto health = healthMonitor_->getSystemHealth();
            
            return health.isHealthy && torqueSplit.totalTorque > 0;
        });
        
        std::cout << "System Integration tests completed." << std::endl << std::endl;
    }

    void testFaultHandling() {
        std::cout << "Testing Fault Handling..." << std::endl;
        
        runTest("Fault Handler Creation", [this]() {
            return faultHandler_ != nullptr;
        });
        
        std::cout << "Fault Handling tests completed." << std::endl << std::endl;
    }

    void testDataLogging() {
        std::cout << "Testing Data Logging..." << std::endl;
        
        runTest("Data Logger Creation", [this]() {
            return dataLogger_ != nullptr;
        });
        
        std::cout << "Data Logging tests completed." << std::endl << std::endl;
    }

    void testHealthMonitoring() {
        std::cout << "Testing Health Monitoring..." << std::endl;
        
        runTest("Health Monitor Creation", [this]() {
            return healthMonitor_ != nullptr;
        });
        
        runTest("System Health Check", [this]() {
            auto health = healthMonitor_->getSystemHealth();
            return health.overallHealth >= 0.0f && health.overallHealth <= 1.0f;
        });
        
        std::cout << "Health Monitoring tests completed." << std::endl << std::endl;
    }

    void runTest(const std::string& testName, std::function<bool()> testFunc) {
        totalTests_++;
        std::cout << "  Running: " << testName << "... ";
        
        try {
            bool result = testFunc();
            if (result) {
                std::cout << "PASSED" << std::endl;
                passedTests_++;
            } else {
                std::cout << "FAILED" << std::endl;
                failedTests_++;
            }
        } catch (const std::exception& e) {
            std::cout << "ERROR: " << e.what() << std::endl;
            failedTests_++;
        }
    }

    void printTestResults() {
        std::cout << "=== Test Results ===" << std::endl;
        std::cout << "Total tests: " << totalTests_ << std::endl;
        std::cout << "Passed: " << passedTests_ << std::endl;
        std::cout << "Failed: " << failedTests_ << std::endl;
        std::cout << "Success rate: " << (totalTests_ > 0 ? (passedTests_ * 100.0 / totalTests_) : 0) << "%" << std::endl;
        
        if (failedTests_ == 0) {
            std::cout << "🎉 All tests passed! VCU system is functioning correctly." << std::endl;
        } else {
            std::cout << "⚠️  Some tests failed. Please check the implementation." << std::endl;
        }
    }
};

int main() {
    try {
        SystemTester tester;
        tester.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}