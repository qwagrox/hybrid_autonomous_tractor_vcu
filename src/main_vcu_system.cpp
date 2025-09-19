// src/main_vcu_system.cpp
#include "vcu_core_types.hpp"
#include "can_bus_interface.hpp"
#include "control/braking_controller.hpp"
#include "control/steering_controller.hpp"
#include "control/cvt_controller.hpp"
#include "hardware/watchdog.hpp"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

using namespace VCUCore;

int main() {
    std::cout << "Starting VCU Main System..." << std::endl;
    
    // 创建核心组件
    auto canBus = std::make_unique<CANBusInterface>("can0");
    auto brakingController = std::make_unique<BrakingController>();
    auto steeringController = std::make_unique<SteeringController>();
    auto cvtController = std::make_unique<CVTController>();
    auto watchdog = std::make_unique<Watchdog>(5000); // 5秒超时
    
    // 初始化所有组件
    std::cout << "Initializing components..." << std::endl;
    
    if (!canBus->initialize()) {
        std::cerr << "Warning: Failed to initialize CAN bus interface (expected in simulation)" << std::endl;
        // 在仿真环境中继续运行
    }
    
    if (!brakingController->initialize()) {
        std::cerr << "Failed to initialize braking controller" << std::endl;
        return -1;
    }
    
    if (!steeringController->initialize()) {
        std::cerr << "Failed to initialize steering controller" << std::endl;
        return -1;
    }
    
    if (!watchdog->initialize()) {
        std::cerr << "Failed to initialize watchdog" << std::endl;
        return -1;
    }
    
    // 启用看门狗
    watchdog->enable();
    
    std::cout << "All components initialized successfully" << std::endl;
    std::cout << "VCU system is running..." << std::endl;
    
    // 主控制循环
    int loopCount = 0;
    while (loopCount < 10) { // 运行10个周期后退出
        // 喂狗
        watchdog->kick();
        
        // 更新控制器
        steeringController->update();
        
        // 检查系统状态
        if (watchdog->checkTimeout()) {
            std::cerr << "Watchdog timeout! Emergency shutdown!" << std::endl;
            break;
        }
        
        // 模拟一些控制操作
        if (loopCount == 3) {
            std::cout << "Testing steering control..." << std::endl;
            steeringController->setSteeringAngle(10.0f);
        }
        
        if (loopCount == 6) {
            std::cout << "Testing braking control..." << std::endl;
            brakingController->applyBraking(1000.0f);
        }
        
        // 打印状态信息
        std::cout << "Loop " << loopCount + 1 << ": "
                  << "Steering angle: " << steeringController->getCurrentSteeringAngle() << "°, "
                  << "Braking force: " << brakingController->getCurrentBrakingForce() << "N"
                  << std::endl;
        
        // 等待下一个周期
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        loopCount++;
    }
    
    std::cout << "Shutting down VCU system..." << std::endl;
    
    // 关闭所有组件
    brakingController->releaseBraking();
    brakingController->shutdown();
    steeringController->shutdown();
    watchdog->shutdown();
    // canBus->shutdown();  // CAN总线接口没有shutdown方法
    
    std::cout << "VCU system shutdown complete" << std::endl;
    return 0;
}
