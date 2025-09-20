#include "system_integration.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace VCUCore {

SystemIntegration::SystemIntegration() 
    : initialized_(false), running_(false) {
    // 初始化系统状态
    currentStatus_.isOperational = false;
    currentStatus_.errorCode = 0;
    currentStatus_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 初始化系统参数
    systemParams_.maxSpeed = 50.0f;
    systemParams_.maxTorque = 1000.0f;
    systemParams_.safetyTimeout = 5000; // 5秒
}

SystemIntegration::~SystemIntegration() {
    if (running_) {
        stop();
    }
}

bool SystemIntegration::initialize() {
    std::cout << "Initializing System Integration..." << std::endl;
    
    if (initialized_) {
        std::cout << "System already initialized" << std::endl;
        return true;
    }
    
    // 初始化各个子系统
    if (!initializeSubsystems()) {
        std::cout << "Failed to initialize subsystems" << std::endl;
        return false;
    }
    
    initialized_ = true;
    currentStatus_.isOperational = true;
    currentStatus_.errorCode = 0;
    
    std::cout << "System Integration initialized successfully" << std::endl;
    return true;
}

bool SystemIntegration::run() {
    if (!initialized_) {
        std::cout << "System not initialized" << std::endl;
        return false;
    }
    
    if (running_) {
        std::cout << "System already running" << std::endl;
        return true;
    }
    
    std::cout << "Starting System Integration main loop..." << std::endl;
    running_ = true;
    
    // 模拟系统运行
    int cycles = 0;
    while (running_ && cycles < 10) { // 限制循环次数用于测试
        updateSystemStatus();
        
        // 模拟工作负载
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cycles++;
        
        if (cycles % 5 == 0) {
            std::cout << "System running, cycle: " << cycles << std::endl;
        }
    }
    
    std::cout << "System Integration main loop completed" << std::endl;
    return true;
}

void SystemIntegration::stop() {
    std::cout << "Stopping System Integration..." << std::endl;
    running_ = false;
    currentStatus_.isOperational = false;
    std::cout << "System Integration stopped" << std::endl;
}

SystemStatus SystemIntegration::getSystemStatus() const {
    return currentStatus_;
}

void SystemIntegration::setSystemParameters(const SystemParameters& params) {
    systemParams_ = params;
    std::cout << "System parameters updated" << std::endl;
}

SystemParameters SystemIntegration::getSystemParameters() const {
    return systemParams_;
}

void SystemIntegration::emergencyStop() {
    std::cout << "EMERGENCY STOP activated!" << std::endl;
    running_ = false;
    currentStatus_.isOperational = false;
    currentStatus_.errorCode = 999; // 紧急停止错误代码
}

bool SystemIntegration::isSystemHealthy() const {
    return initialized_ && currentStatus_.isOperational && currentStatus_.errorCode == 0;
}

bool SystemIntegration::initializeSubsystems() {
    std::cout << "Initializing subsystems..." << std::endl;
    
    // 模拟子系统初始化
    std::cout << "  - CAN Bus Interface: OK" << std::endl;
    std::cout << "  - Sensor Fusion: OK" << std::endl;
    std::cout << "  - Control Systems: OK" << std::endl;
    std::cout << "  - Safety Systems: OK" << std::endl;
    
    return true;
}

void SystemIntegration::updateSystemStatus() {
    // 更新时间戳
    currentStatus_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 模拟状态检查
    if (running_ && initialized_) {
        currentStatus_.isOperational = true;
        currentStatus_.errorCode = 0;
    }
}

void SystemIntegration::handleSystemError(int errorCode) {
    std::cout << "Handling system error: " << errorCode << std::endl;
    currentStatus_.errorCode = errorCode;
    
    if (errorCode >= 900) { // 严重错误
        emergencyStop();
    }
}

} // namespace VCUCore
