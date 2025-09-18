// include/execution/actuator_interface.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "hardware/can_driver.hpp"
#include "hardware/gpio_driver.hpp"
#include <memory>
#include <atomic>
#include <mutex>

namespace VCUCore {

class ActuatorInterface {
private:
    struct ActuatorState {
        float engineTorqueCommand;
        float motorTorqueCommand;
        float cvtRatioCommand;
        float hydraulicPressure;
        bool implementLiftState;
        bool emergencyStopActive;
        uint32_t lastUpdateTime;
        uint32_t errorCount;
    };
    
    ActuatorState currentState_;
    std::unique_ptr<CANDriver> canDriver_;
    std::unique_ptr<GPIODriver> gpioDriver_;
    
    std::atomic<bool> isInitialized_;
    std::atomic<bool> emergencyStop_;
    mutable std::mutex stateMutex_;
    
    // 校准参数
    struct CalibrationParams {
        float engineTorqueOffset;
        float motorTorqueOffset;
        float cvtRatioOffset;
        float hydraulicOffset;
        std::array<float, 10> torqueCalibration;
        std::array<float, 10> ratioCalibration;
    };
    
    CalibrationParams calibration_;
    std::deque<ActuatorCommand> commandHistory_;

public:
    ActuatorInterface();
    ~ActuatorInterface();
    
    bool initialize(const std::string& canInterface = "can0");
    bool shutdown();
    
    // 命令执行
    bool sendTorqueCommand(float engineTorque, float motorTorque);
    bool sendCVTRatioCommand(float ratio);
    bool sendHydraulicCommand(float pressure);
    bool sendImplementCommand(bool lift);
    
    // 紧急操作
    bool emergencyStop();
    bool resumeFromEmergency();
    bool isEmergencyActive() const;
    
    // 状态查询
    ActuatorState getCurrentState() const;
    bool checkActuatorHealth() const;
    float getCommandExecutionTime() const;
    
    // 校准功能
    bool calibrateActuators();
    bool loadCalibration(const std::string& calibrationFile);
    bool saveCalibration(const std::string& calibrationFile);
    
    // 诊断功能
    std::vector<ActuatorDiagnostic> runDiagnostics() const;
    bool resetActuatorFaults();

private:
    bool sendCANCommand(uint32_t canId, const std::vector<uint8_t>& data);
    bool sendGPIOCommand(uint8_t pin, bool state);
    bool validateCommand(const ActuatorCommand& command) const;
    
    void updateCommandHistory(const ActuatorCommand& command);
    void monitorActuatorPerformance();
    void handleActuatorFailure(uint8_t actuatorId, uint16_t errorCode);
    
    bool initializeCAN();
    bool initializeGPIO();
    bool initializeCalibration();
    
    float applyCalibration(float value, float offset, const std::array<float, 10>& calibration) const;
};

} // namespace VCUCore