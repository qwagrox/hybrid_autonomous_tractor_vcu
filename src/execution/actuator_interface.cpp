// src/execution/actuator_interface.cpp
#include "actuator_interface.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fstream>

namespace VCUCore {

ActuatorInterface::ActuatorInterface() 
    : isInitialized_(false), emergencyStop_(false) {
    
    currentState_ = {
        .engineTorqueCommand = 0.0f,
        .motorTorqueCommand = 0.0f,
        .cvtRatioCommand = 1.0f,
        .hydraulicPressure = 0.0f,
        .implementLiftState = false,
        .emergencyStopActive = false,
        .lastUpdateTime = 0,
        .errorCount = 0
    };
    
    initializeCalibration();
}

ActuatorInterface::~ActuatorInterface() {
    shutdown();
}

bool ActuatorInterface::initialize(const std::string& canInterface) {
    if (isInitialized_) {
        return true;
    }
    
    try {
        // 初始化CAN驱动
        canDriver_ = std::make_unique<CANDriver>();
        if (!canDriver_->initialize(canInterface, 500000)) {
            throw std::runtime_error("CAN driver initialization failed");
        }
        
        // 初始化GPIO驱动
        gpioDriver_ = std::make_unique<GPIODriver>();
        if (!gpioDriver_->initialize()) {
            throw std::runtime_error("GPIO driver initialization failed");
        }
        
        // 加载校准数据
        if (!loadCalibration("/etc/vcu/actuator_calibration.cfg")) {
            std::cout << "Using default actuator calibration" << std::endl;
        }
        
        isInitialized_ = true;
        emergencyStop_ = false;
        
        std::cout << "Actuator interface initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Actuator interface initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool ActuatorInterface::shutdown() {
    if (!isInitialized_) {
        return true;
    }
    
    try {
        // 发送安全关闭命令
        sendTorqueCommand(0.0f, 0.0f);
        sendCVTRatioCommand(1.0f);
        sendHydraulicCommand(0.0f);
        
        // 关闭驱动
        if (canDriver_) {
            canDriver_->shutdown();
        }
        
        if (gpioDriver_) {
            gpioDriver_->shutdown();
        }
        
        isInitialized_ = false;
        std::cout << "Actuator interface shutdown successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Actuator interface shutdown failed: " << e.what() << std::endl;
        return false;
    }
}

bool ActuatorInterface::sendTorqueCommand(float engineTorque, float motorTorque) {
    if (!isInitialized_ || emergencyStop_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 应用校准
        float calibratedEngineTorque = applyCalibration(engineTorque, 
            calibration_.engineTorqueOffset, calibration_.torqueCalibration);
        
        float calibratedMotorTorque = applyCalibration(motorTorque,
            calibration_.motorTorqueOffset, calibration_.torqueCalibration);
        
        // 发送发动机扭矩命令 (J1939 PGN 0x00F003)
        std::vector<uint8_t> engineData(8);
        int16_t torqueValue = static_cast<int16_t>(calibratedEngineTorque * 10.0f); // 0.1 Nm resolution
        engineData[0] = (torqueValue >> 8) & 0xFF;
        engineData[1] = torqueValue & 0xFF;
        
        if (!sendCANCommand(0x00F00300, engineData)) {
            throw std::runtime_error("Engine torque command failed");
        }
        
        // 发送电机扭矩命令 (自定义协议)
        std::vector<uint8_t> motorData(8);
        int16_t motorTorqueValue = static_cast<int16_t>(calibratedMotorTorque * 10.0f);
        motorData[0] = (motorTorqueValue >> 8) & 0xFF;
        motorData[1] = motorTorqueValue & 0xFF;
        
        if (!sendCANCommand(0x0CFF1000, motorData)) {
            throw std::runtime_error("Motor torque command failed");
        }
        
        // 更新状态
        currentState_.engineTorqueCommand = calibratedEngineTorque;
        currentState_.motorTorqueCommand = calibratedMotorTorque;
        currentState_.lastUpdateTime = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        // 记录命令历史
        ActuatorCommand command;
        command.timestamp = currentState_.lastUpdateTime;
        command.engineTorque = calibratedEngineTorque;
        command.motorTorque = calibratedMotorTorque;
        updateCommandHistory(command);
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Torque command failed: " << e.what() << std::endl;
        currentState_.errorCount++;
        return false;
    }
}

bool ActuatorInterface::sendCVTRatioCommand(float ratio) {
    if (!isInitialized_ || emergencyStop_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 应用校准
        float calibratedRatio = applyCalibration(ratio,
            calibration_.cvtRatioOffset, calibration_.ratioCalibration);
        
        // 发送CVT比率命令 (J1939 PGN 0xFEFF00)
        std::vector<uint8_t> ratioData(8);
        uint16_t ratioValue = static_cast<uint16_t>(calibratedRatio * 1000.0f); // 0.001 resolution
        ratioData[0] = (ratioValue >> 8) & 0xFF;
        ratioData[1] = ratioValue & 0xFF;
        
        if (!sendCANCommand(0xFEFF0000, ratioData)) {
            throw std::runtime_error("CVT ratio command failed");
        }
        
        // 更新状态
        currentState_.cvtRatioCommand = calibratedRatio;
        currentState_.lastUpdateTime = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "CVT ratio command failed: " << e.what() << std::endl;
        currentState_.errorCount++;
        return false;
    }
}

bool ActuatorInterface::sendHydraulicCommand(float pressure) {
    if (!isInitialized_ || emergencyStop_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 应用校准
        float calibratedPressure = applyCalibration(pressure,
            calibration_.hydraulicOffset, calibration_.torqueCalibration);
        
        // 发送液压压力命令 (自定义协议)
        std::vector<uint8_t> pressureData(8);
        uint16_t pressureValue = static_cast<uint16_t>(calibratedPressure * 10.0f); // 0.1 bar resolution
        pressureData[0] = (pressureValue >> 8) & 0xFF;
        pressureData[1] = pressureValue & 0xFF;
        
        if (!sendCANCommand(0x0CFF2000, pressureData)) {
            throw std::runtime_error("Hydraulic command failed");
        }
        
        // 更新状态
        currentState_.hydraulicPressure = calibratedPressure;
        currentState_.lastUpdateTime = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Hydraulic command failed: " << e.what() << std::endl;
        currentState_.errorCount++;
        return false;
    }
}

bool ActuatorInterface::sendImplementCommand(bool lift) {
    if (!isInitialized_ || emergencyStop_) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 使用GPIO控制农具提升
        if (!sendGPIOCommand(GPIO_PIN_IMPLEMENT_LIFT, lift)) {
            throw std::runtime_error("Implement command failed");
        }
        
        // 更新状态
        currentState_.implementLiftState = lift;
        currentState_.lastUpdateTime = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Implement command failed: " << e.what() << std::endl;
        currentState_.errorCount++;
        return false;
    }
}

bool ActuatorInterface::emergencyStop() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 发送紧急停止命令
        std::vector<uint8_t> emergencyData(8);
        emergencyData[0] = 0x01; // 紧急停止命令
        
        if (!sendCANCommand(0x0CFFFFFF, emergencyData)) {
            throw std::runtime_error("Emergency stop command failed");
        }
        
        // 设置所有GPIO输出为安全状态
        for (int pin = 0; pin < 8; ++pin) {
            sendGPIOCommand(pin, false);
        }
        
        // 更新状态
        emergencyStop_ = true;
        currentState_.emergencyStopActive = true;
        currentState_.engineTorqueCommand = 0.0f;
        currentState_.motorTorqueCommand = 0.0f;
        currentState_.hydraulicPressure = 0.0f;
        
        std::cout << "Emergency stop activated" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Emergency stop failed: " << e.what() << std::endl;
        return false;
    }
}

bool ActuatorInterface::resumeFromEmergency() {
    std::lock_guard<std::mutex> lock(stateMutex_);
    
    try {
        // 发送恢复命令
        std::vector<uint8_t> resumeData(8);
        resumeData[0] = 0x02; // 恢复命令
        
        if (!sendCANCommand(0x0CFFFFFE, resumeData)) {
            throw std::runtime_error("Resume command failed");
        }
        
        // 更新状态
        emergencyStop_ = false;
        currentState_.emergencyStopActive = false;
        
        std::cout << "Resumed from emergency stop" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Resume from emergency failed: " << e.what() << std::endl;
        return false;
    }
}

bool ActuatorInterface::sendCANCommand(uint32_t canId, const std::vector<uint8_t>& data) {
    if (!canDriver_) {
        return false;
    }
    
    struct can_frame frame;
    frame.can_id = canId;
    frame.can_dlc = data.size() < 8 ? data.size() : 8;
    
    for (int i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = data[i];
    }
    
    return canDriver_->sendFrame(frame);
}

bool ActuatorInterface::sendGPIOCommand(uint8_t pin, bool state) {
    if (!gpioDriver_) {
        return false;
    }
    
    return gpioDriver_->setPinState(pin, state);
}

void ActuatorInterface::initializeCalibration() {
    // 默认校准参数
    calibration_ = {
        .engineTorqueOffset = 0.0f,
        .motorTorqueOffset = 0.0f,
        .cvtRatioOffset = 0.0f,
        .hydraulicOffset = 0.0f,
        .torqueCalibration = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
        .ratioCalibration = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}
    };
}

float ActuatorInterface::applyCalibration(float value, float offset, 
                                        const std::array<float, 10>& calibration) const {
    float calibrated = value + offset;
    
    // 应用非线性校准（分段线性）
    int index = static_cast<int>(std::abs(value) / 100.0f);
    if (index >= 0 && index < 10) {
        calibrated *= calibration[index];
    }
    
    return calibrated;
}

bool ActuatorInterface::loadCalibration(const std::string& calibrationFile) {
    std::ifstream file(calibrationFile);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            float value;
            
            if (iss >> key >> value) {
                if (key == "engine_offset") calibration_.engineTorqueOffset = value;
                else if (key == "motor_offset") calibration_.motorTorqueOffset = value;
                else if (key == "cvt_offset") calibration_.cvtRatioOffset = value;
                else if (key == "hydraulic_offset") calibration_.hydraulicOffset = value;
                // 可以添加更多校准参数...
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading calibration: " << e.what() << std::endl;
        return false;
    }
}

ActuatorState ActuatorInterface::getCurrentState() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return currentState_;
}

bool ActuatorInterface::checkActuatorHealth() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return currentState_.errorCount < 10 && isInitialized_ && !emergencyStop_;
}

} // namespace VCUCore