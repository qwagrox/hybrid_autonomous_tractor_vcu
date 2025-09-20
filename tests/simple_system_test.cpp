#include <iostream>
#include <memory>
#include <cassert>

// 包含核心头文件
#include "vcu_core_types.hpp"
#include "system_integration.hpp"

// 简单的测试函数
bool testSystemIntegration() {
    std::cout << "Testing System Integration..." << std::endl;
    
    try {
        VCUCore::SystemIntegration system;
        
        // 测试初始化
        if (!system.initialize()) {
            std::cout << "❌ System initialization failed" << std::endl;
            return false;
        }
        std::cout << "✓ System initialization successful" << std::endl;
        
        // 测试状态获取
        auto status = system.getSystemStatus();
        if (!status.isOperational) {
            std::cout << "❌ System not operational after initialization" << std::endl;
            return false;
        }
        std::cout << "✓ System status check successful" << std::endl;
        
        // 测试参数设置
        VCUCore::SystemParameters params;
        params.maxSpeed = 60.0f;
        params.maxTorque = 1200.0f;
        params.safetyTimeout = 3000;
        
        system.setSystemParameters(params);
        auto retrievedParams = system.getSystemParameters();
        
        if (retrievedParams.maxSpeed != params.maxSpeed) {
            std::cout << "❌ Parameter setting failed" << std::endl;
            return false;
        }
        std::cout << "✓ Parameter setting successful" << std::endl;
        
        // 测试健康检查
        if (!system.isSystemHealthy()) {
            std::cout << "❌ System health check failed" << std::endl;
            return false;
        }
        std::cout << "✓ System health check successful" << std::endl;
        
        // 测试运行
        if (!system.run()) {
            std::cout << "❌ System run failed" << std::endl;
            return false;
        }
        std::cout << "✓ System run successful" << std::endl;
        
        // 测试停止
        system.stop();
        std::cout << "✓ System stop successful" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Exception during testing: " << e.what() << std::endl;
        return false;
    }
}

bool testCoreTypes() {
    std::cout << "Testing Core Types..." << std::endl;
    
    try {
        // 测试基本数据结构
        VCUCore::TractorVehicleState state;
        state.speed = 25.5f;
        state.engineRPM = 1800;
        state.fuelLevel = 75.0f;
        
        if (state.speed != 25.5f) {
            std::cout << "❌ TractorVehicleState assignment failed" << std::endl;
            return false;
        }
        std::cout << "✓ TractorVehicleState test successful" << std::endl;
        
        // 测试控制命令
        VCUCore::ControlCommands commands;
        commands.throttlePosition = 0.6f;
        commands.brakePosition = 0.0f;
        commands.steeringAngle = 15.0f;
        
        if (commands.throttlePosition != 0.6f) {
            std::cout << "❌ ControlCommands assignment failed" << std::endl;
            return false;
        }
        std::cout << "✓ ControlCommands test successful" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Exception during core types testing: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "=== VCU System Test Suite ===" << std::endl;
    
    int passed = 0;
    int total = 0;
    
    // 运行测试
    total++;
    if (testCoreTypes()) {
        passed++;
        std::cout << "✓ Core Types Test: PASSED" << std::endl;
    } else {
        std::cout << "❌ Core Types Test: FAILED" << std::endl;
    }
    
    total++;
    if (testSystemIntegration()) {
        passed++;
        std::cout << "✓ System Integration Test: PASSED" << std::endl;
    } else {
        std::cout << "❌ System Integration Test: FAILED" << std::endl;
    }
    
    // 输出结果
    std::cout << std::endl;
    std::cout << "=== Test Results ===" << std::endl;
    std::cout << "Passed: " << passed << "/" << total << std::endl;
    std::cout << "Success Rate: " << (passed * 100 / total) << "%" << std::endl;
    
    if (passed == total) {
        std::cout << "🎉 All tests passed!" << std::endl;
        return 0;
    } else {
        std::cout << "⚠️ Some tests failed" << std::endl;
        return 1;
    }
}
