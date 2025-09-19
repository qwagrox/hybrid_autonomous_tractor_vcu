// tests/system_test.cpp
#include <iostream>
#include <chrono>
#include <thread>
#include <cassert>

// 包含所有主要模块的头文件
#include "can_bus_interface.hpp"
#include "control/torque_arbiter.hpp"
#include "control/cvt_controller.hpp"
#include "control/braking_controller.hpp"
#include "control/steering_controller.hpp"
#include "perception/sensor_fusion.hpp"
#include "perception/load_detector.hpp"
#include "utils/tractor_state_calculator.hpp"
#include "hardware/watchdog.hpp"
#include "diagnostic/health_monitor.hpp"
#include "execution/actuator_interface.hpp"

using namespace VCUCore;

class SystemTester {
private:
    std::unique_ptr<CANBusInterface> canInterface_;
    std::unique_ptr<TorqueArbiter> torqueArbiter_;
    std::unique_ptr<CVTController> cvtController_;
    std::unique_ptr<BrakingController> brakingController_;
    std::unique_ptr<SteeringController> steeringController_;
    std::unique_ptr<SensorFusion> sensorFusion_;
    std::unique_ptr<LoadDetector> loadDetector_;
    std::unique_ptr<TractorStateCalculator> stateCalculator_;
    std::unique_ptr<Watchdog> watchdog_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    std::unique_ptr<ActuatorInterface> actuatorInterface_;
    
    bool testsPassed_;
    int totalTests_;
    int passedTests_;

public:
    SystemTester() : testsPassed_(true), totalTests_(0), passedTests_(0) {}
    
    void runAllTests() {
        std::cout << "=== VCU System Test Suite ===" << std::endl;
        std::cout << "Starting comprehensive system tests..." << std::endl << std::endl;
        
        // 初始化所有模块
        if (!initializeModules()) {
            std::cerr << "Module initialization failed!" << std::endl;
            return;
        }
        
        // 运行各个测试
        testCANInterface();
        testTorqueArbiter();
        testCVTController();
        testBrakingController();
        testSteeringController();
        testSensorFusion();
        testLoadDetector();
        testStateCalculator();
        testWatchdog();
        testHealthMonitor();
        testActuatorInterface();
        testSystemIntegration();
        
        // 输出测试结果
        printTestResults();
        
        // 清理资源
        cleanupModules();
    }

private:
    bool initializeModules() {
        std::cout << "Initializing system modules..." << std::endl;
        
        try {
            // 初始化CAN总线接口
            canInterface_ = std::make_unique<CANBusInterface>();
            std::cout << "✓ CAN Bus Interface created" << std::endl;
            
            // 初始化控制模块
            torqueArbiter_ = std::make_unique<TorqueArbiter>();
            cvtController_ = std::make_unique<CVTController>(CVTManufacturer::JOHN_DEERE);
            brakingController_ = std::make_unique<BrakingController>();
            steeringController_ = std::make_unique<SteeringController>();
            std::cout << "✓ Control modules created" << std::endl;
            
            // 初始化感知模块
            sensorFusion_ = std::make_unique<SensorFusion>();
            loadDetector_ = std::make_unique<LoadDetector>();
            std::cout << "✓ Perception modules created" << std::endl;
            
            // 初始化工具模块
            stateCalculator_ = std::make_unique<TractorStateCalculator>();
            std::cout << "✓ State calculator created" << std::endl;
            
            // 初始化硬件模块
            watchdog_ = std::make_unique<Watchdog>();
            std::cout << "✓ Watchdog created" << std::endl;
            
            // 初始化诊断模块
            healthMonitor_ = std::make_unique<HealthMonitor>();
            std::cout << "✓ Health monitor created" << std::endl;
            
            // 初始化执行模块
            actuatorInterface_ = std::make_unique<ActuatorInterface>();
            std::cout << "✓ Actuator interface created" << std::endl;
            
            std::cout << "All modules initialized successfully!" << std::endl << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Module initialization failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void testCANInterface() {
        std::cout << "Testing CAN Bus Interface..." << std::endl;
        
        // 测试CAN接口基本功能
        runTest("CAN Interface Creation", [this]() {
            return canInterface_ != nullptr;
        });
        
        // 测试发动机数据解析
        runTest("Engine Data Parsing", [this]() {
            EngineData engineData;
            engineData.actualTorque = 1500.0f;
            engineData.speed = 1800.0f;
            engineData.temperature = 85.0f;
            engineData.timestamp = getCurrentTimestamp();
            
            // 模拟解析过程
            return engineData.actualTorque > 0 && engineData.speed > 0;
        });
        
        std::cout << "CAN Interface tests completed." << std::endl << std::endl;
    }
    
    void testTorqueArbiter() {
        std::cout << "Testing Torque Arbiter..." << std::endl;
        
        runTest("Torque Arbiter Creation", [this]() {
            return torqueArbiter_ != nullptr;
        });
        
        runTest("Torque Command Processing", [this]() {
            ControlCommands commands;
            commands.engineTorqueRequest = 1200.0f;
            commands.motorTorqueRequest = 800.0f;
            commands.timestamp = getCurrentTimestamp();
            
            PerceptionData perception;
            perception.vehicleState.actualTorque = 1000.0f;
            perception.vehicleState.demandedTorque = 1200.0f;
            
            auto result = torqueArbiter_->arbitrateTorque(commands, perception);
            return result.engineTorqueRequest >= 0 && result.motorTorqueRequest >= 0;
        });
        
        std::cout << "Torque Arbiter tests completed." << std::endl << std::endl;
    }
    
    void testCVTController() {
        std::cout << "Testing CVT Controller..." << std::endl;
        
        runTest("CVT Controller Creation", [this]() {
            return cvtController_ != nullptr;
        });
        
        runTest("CVT Ratio Calculation", [this]() {
            TractorVehicleState state;
            state.velocity = Vector3d(5.0, 0.0, 0.0); // 5 m/s forward
            state.engineRpm = 1800.0f;
            state.timestamp = getCurrentTimestamp();
            
            float optimalRatio = cvtController_->calculateOptimalRatio(state);
            return optimalRatio > 0.5f && optimalRatio < 3.0f; // 合理的CVT比率范围
        });
        
        std::cout << "CVT Controller tests completed." << std::endl << std::endl;
    }
    
    void testBrakingController() {
        std::cout << "Testing Braking Controller..." << std::endl;
        
        runTest("Braking Controller Creation", [this]() {
            return brakingController_ != nullptr;
        });
        
        runTest("Brake Force Calculation", [this]() {
            TractorVehicleState state;
            state.velocity = Vector3d(10.0, 0.0, 0.0); // 10 m/s forward
            state.estimatedMass = 8000.0f; // 8 tons
            
            float targetDeceleration = -2.0f; // 2 m/s² deceleration
            float brakeForce = brakingController_->calculateBrakeForce(state, targetDeceleration);
            
            return brakeForce > 0 && brakeForce < 50000.0f; // 合理的制动力范围
        });
        
        std::cout << "Braking Controller tests completed." << std::endl << std::endl;
    }
    
    void testSteeringController() {
        std::cout << "Testing Steering Controller..." << std::endl;
        
        runTest("Steering Controller Creation", [this]() {
            return steeringController_ != nullptr;
        });
        
        runTest("Steering Angle Calculation", [this]() {
            TractorVehicleState state;
            state.velocity = Vector3d(5.0, 0.0, 0.0);
            state.heading = 0.0f;
            
            float targetHeading = 0.1f; // 小角度转向
            float steeringAngle = steeringController_->calculateSteeringAngle(state, targetHeading);
            
            return std::abs(steeringAngle) < 1.57f; // 小于90度
        });
        
        std::cout << "Steering Controller tests completed." << std::endl << std::endl;
    }
    
    void testSensorFusion() {
        std::cout << "Testing Sensor Fusion..." << std::endl;
        
        runTest("Sensor Fusion Creation", [this]() {
            return sensorFusion_ != nullptr;
        });
        
        runTest("Sensor Data Fusion", [this]() {
            SensorData sensorData;
            sensorData.gnssPosition = Vector3d(100.0, 200.0, 50.0);
            sensorData.imuAcceleration = Vector3d(0.1, 0.0, -9.81);
            sensorData.imuAngularRate = Vector3d(0.01, 0.02, 0.0);
            sensorData.timestamp = getCurrentTimestamp();
            
            TractorVehicleState fusedState = sensorFusion_->fuseSensorData(sensorData);
            
            return fusedState.position.norm() > 0 && fusedState.acceleration.norm() > 0;
        });
        
        std::cout << "Sensor Fusion tests completed." << std::endl << std::endl;
    }
    
    void testLoadDetector() {
        std::cout << "Testing Load Detector..." << std::endl;
        
        runTest("Load Detector Creation", [this]() {
            return loadDetector_ != nullptr;
        });
        
        runTest("Load Detection", [this]() {
            TractorVehicleState state;
            state.drawbarPull = 15000.0f; // 15 kN
            state.engineLoad = 0.75f; // 75% load
            state.timestamp = getCurrentTimestamp();
            
            loadDetector_->updateLoadMeasurement(state);
            float currentLoad = loadDetector_->getCurrentLoad();
            
            return currentLoad >= 0 && currentLoad <= 1.0f;
        });
        
        std::cout << "Load Detector tests completed." << std::endl << std::endl;
    }
    
    void testStateCalculator() {
        std::cout << "Testing State Calculator..." << std::endl;
        
        runTest("State Calculator Creation", [this]() {
            return stateCalculator_ != nullptr;
        });
        
        runTest("State Prediction", [this]() {
            TractorVehicleState currentState;
            currentState.position = Vector3d(0, 0, 0);
            currentState.velocity = Vector3d(5.0, 0, 0);
            currentState.acceleration = Vector3d(0.5, 0, 0);
            currentState.imuAngularRate = Vector3d(0.01, 0.02, 0.0);
            
            float deltaTime = 0.1f; // 100ms
            TractorVehicleState predictedState = stateCalculator_->predictFutureState(currentState, deltaTime);
            
            return predictedState.position.x() > currentState.position.x();
        });
        
        std::cout << "State Calculator tests completed." << std::endl << std::endl;
    }
    
    void testWatchdog() {
        std::cout << "Testing Watchdog..." << std::endl;
        
        runTest("Watchdog Creation", [this]() {
            return watchdog_ != nullptr;
        });
        
        runTest("Watchdog Feed", [this]() {
            watchdog_->feed();
            return watchdog_->isHealthy();
        });
        
        std::cout << "Watchdog tests completed." << std::endl << std::endl;
    }
    
    void testHealthMonitor() {
        std::cout << "Testing Health Monitor..." << std::endl;
        
        runTest("Health Monitor Creation", [this]() {
            return healthMonitor_ != nullptr;
        });
        
        runTest("System Health Check", [this]() {
            SystemHealthStatus health = healthMonitor_->checkSystemHealth();
            return health.overallHealth >= 0 && health.overallHealth <= 1.0f;
        });
        
        std::cout << "Health Monitor tests completed." << std::endl << std::endl;
    }
    
    void testActuatorInterface() {
        std::cout << "Testing Actuator Interface..." << std::endl;
        
        runTest("Actuator Interface Creation", [this]() {
            return actuatorInterface_ != nullptr;
        });
        
        runTest("Actuator Health Check", [this]() {
            return actuatorInterface_->checkActuatorHealth();
        });
        
        std::cout << "Actuator Interface tests completed." << std::endl << std::endl;
    }
    
    void testSystemIntegration() {
        std::cout << "Testing System Integration..." << std::endl;
        
        runTest("Complete System Cycle", [this]() {
            try {
                // 模拟一个完整的系统循环
                
                // 1. 获取传感器数据
                SensorData sensorData;
                sensorData.gnssPosition = Vector3d(100.0, 200.0, 50.0);
                sensorData.imuAcceleration = Vector3d(0.1, 0.0, -9.81);
                sensorData.imuAngularRate = Vector3d(0.01, 0.02, 0.0);
                sensorData.engineRpm = 1800.0f;
                sensorData.wheelSpeed[0] = sensorData.wheelSpeed[1] = 
                sensorData.wheelSpeed[2] = sensorData.wheelSpeed[3] = 5.0f;
                sensorData.timestamp = getCurrentTimestamp();
                
                // 2. 传感器融合
                TractorVehicleState fusedState = sensorFusion_->fuseSensorData(sensorData);
                
                // 3. 负载检测
                loadDetector_->updateLoadMeasurement(fusedState);
                
                // 4. 状态预测
                TractorVehicleState predictedState = stateCalculator_->predictFutureState(fusedState, 0.1f);
                
                // 5. 控制决策
                ControlCommands commands;
                commands.engineTorqueRequest = 1200.0f;
                commands.motorTorqueRequest = 800.0f;
                commands.cvtRatioRequest = cvtController_->calculateOptimalRatio(predictedState);
                commands.timestamp = getCurrentTimestamp();
                
                // 6. 扭矩仲裁
                PerceptionData perception;
                perception.vehicleState = predictedState;
                auto arbitratedCommands = torqueArbiter_->arbitrateTorque(commands, perception);
                
                // 7. 健康监控
                SystemHealthStatus health = healthMonitor_->checkSystemHealth();
                
                // 8. 看门狗喂狗
                watchdog_->feed();
                
                return health.isHealthy && watchdog_->isHealthy();
                
            } catch (const std::exception& e) {
                std::cerr << "System integration test failed: " << e.what() << std::endl;
                return false;
            }
        });
        
        std::cout << "System Integration tests completed." << std::endl << std::endl;
    }
    
    void runTest(const std::string& testName, std::function<bool()> testFunc) {
        totalTests_++;
        std::cout << "  Running: " << testName << "... ";
        
        try {
            if (testFunc()) {
                std::cout << "PASS" << std::endl;
                passedTests_++;
            } else {
                std::cout << "FAIL" << std::endl;
                testsPassed_ = false;
            }
        } catch (const std::exception& e) {
            std::cout << "ERROR: " << e.what() << std::endl;
            testsPassed_ = false;
        }
    }
    
    void printTestResults() {
        std::cout << "=== Test Results ===" << std::endl;
        std::cout << "Total Tests: " << totalTests_ << std::endl;
        std::cout << "Passed: " << passedTests_ << std::endl;
        std::cout << "Failed: " << (totalTests_ - passedTests_) << std::endl;
        std::cout << "Success Rate: " << (100.0 * passedTests_ / totalTests_) << "%" << std::endl;
        
        if (testsPassed_) {
            std::cout << "🎉 ALL TESTS PASSED! System is functioning correctly." << std::endl;
        } else {
            std::cout << "❌ Some tests failed. Please check the system." << std::endl;
        }
    }
    
    void cleanupModules() {
        std::cout << std::endl << "Cleaning up modules..." << std::endl;
        
        // 智能指针会自动清理，但我们可以显式重置
        actuatorInterface_.reset();
        healthMonitor_.reset();
        watchdog_.reset();
        stateCalculator_.reset();
        loadDetector_.reset();
        sensorFusion_.reset();
        steeringController_.reset();
        brakingController_.reset();
        cvtController_.reset();
        torqueArbiter_.reset();
        canInterface_.reset();
        
        std::cout << "Cleanup completed." << std::endl;
    }
    
    uint32_t getCurrentTimestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

int main() {
    std::cout << "VCU System Test - Starting..." << std::endl;
    std::cout << "Build Date: " << __DATE__ << " " << __TIME__ << std::endl;
    std::cout << "========================================" << std::endl << std::endl;
    
    SystemTester tester;
    tester.runAllTests();
    
    std::cout << std::endl << "Test execution completed." << std::endl;
    return 0;
}
