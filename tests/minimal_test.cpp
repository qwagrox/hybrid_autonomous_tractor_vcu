// tests/minimal_test.cpp
// 最简化的测试程序，只验证核心类型定义修复

#include "vcu_core_types.hpp"
#include <iostream>
#include <vector>

using namespace VCUCore;

int main() {
    std::cout << "=== VCU Minimal Type Test ===" << std::endl;
    
    try {
        // 测试1: BatteryState结构体及其新增成员
        std::cout << "Testing BatteryState structure..." << std::endl;
        BatteryState batteryState;
        
        // 测试原有成员
        batteryState.stateOfCharge = 80.0f;
        batteryState.stateOfHealth = 95.0f;
        batteryState.voltage = 48.0f;
        batteryState.current = 10.0f;
        batteryState.temperature = 25.0f;
        batteryState.power = 480.0f;
        batteryState.cycleCount = 100;
        batteryState.isCharging = false;
        batteryState.isDischarging = true;
        batteryState.timestamp = 1234567890;
        
        // 测试新增成员
        batteryState.energy = 50.0f;
        batteryState.internalResistance = 0.05f;
        batteryState.soc = batteryState.stateOfCharge; // 测试soc别名
        
        std::cout << "  ✓ Basic members: OK" << std::endl;
        std::cout << "  ✓ New members (energy, internalResistance, soc): OK" << std::endl;
        
        // 测试2: CellModel结构体
        std::cout << "Testing CellModel structure..." << std::endl;
        CellModel cell;
        cell.voltage = 3.2f;
        cell.soc = 80.0f;
        cell.temperature = 25.0f;
        cell.internalResistance = 0.001f;
        cell.capacity = 2.5f;
        cell.cycleCount = 100;
        
        // 测试cells向量
        batteryState.cells.push_back(cell);
        batteryState.cells.push_back(cell);
        
        std::cout << "  ✓ CellModel structure: OK" << std::endl;
        std::cout << "  ✓ BatteryState.cells vector: OK (size=" << batteryState.cells.size() << ")" << std::endl;
        
        // 测试3: LoadSignature结构体及其扩展成员
        std::cout << "Testing LoadSignature structure..." << std::endl;
        LoadSignature loadSig;
        
        // 原有成员
        loadSig.engineTorque = 1500.0f;
        loadSig.groundSpeed = 15.0f;
        loadSig.fuelRate = 12.5f;
        loadSig.hydraulicPressure = 180.0f;
        loadSig.timestamp = 1234567890;
        
        // 新增成员
        loadSig.motorTorque = 200.0f;
        loadSig.implementForce = 5000.0f;
        loadSig.wheelSlip = 0.05f;
        loadSig.powerConsumption = 75.0f;
        loadSig.torqueDerivative = 0.1f;
        loadSig.forceDerivative = 0.2f;
        loadSig.frequencyComponents = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
        
        std::cout << "  ✓ Basic LoadSignature members: OK" << std::endl;
        std::cout << "  ✓ Extended LoadSignature members: OK" << std::endl;
        std::cout << "  ✓ Eigen::Vector3f integration: OK" << std::endl;
        
        // 测试4: 其他核心类型
        std::cout << "Testing other core types..." << std::endl;
        
        BatteryHealth batteryHealth;
        batteryHealth.stateOfHealth = 95.0f;
        batteryHealth.stateOfCharge = 80.0f;
        batteryHealth.internalResistance = 0.05f;
        
        BatteryFault batteryFault;
        batteryFault.faultCode = 0x1001;
        batteryFault.description = "Test fault";
        
        BatteryStatistics batteryStats;
        batteryStats.totalEnergyCharged = 1000.0f;
        batteryStats.totalEnergyDischarged = 800.0f;
        batteryStats.cycleCount = 100;
        
        std::cout << "  ✓ BatteryHealth: OK" << std::endl;
        std::cout << "  ✓ BatteryFault: OK" << std::endl;
        std::cout << "  ✓ BatteryStatistics: OK" << std::endl;
        
        // 测试5: 枚举类型
        std::cout << "Testing enum types..." << std::endl;
        
        LoadChangeType changeType = LoadChangeType::GRADUAL;
        LoadType loadType = LoadType::HEAVY_IMPLEMENT;
        TrendDirection trendDir = TrendDirection::INCREASING;
        
        std::cout << "  ✓ LoadChangeType: OK" << std::endl;
        std::cout << "  ✓ LoadType: OK" << std::endl;
        std::cout << "  ✓ TrendDirection: OK" << std::endl;
        
        std::cout << std::endl;
        std::cout << "=== Test Summary ===" << std::endl;
        std::cout << "✅ All core type definitions are working correctly!" << std::endl;
        std::cout << std::endl;
        std::cout << "Fixed Issues:" << std::endl;
        std::cout << "  • BatteryState: Added energy, internalResistance, soc, cells members" << std::endl;
        std::cout << "  • CellModel: Complete structure definition added" << std::endl;
        std::cout << "  • LoadSignature: Extended with motorTorque, implementForce, etc." << std::endl;
        std::cout << "  • BatteryHealth/Fault/Statistics: All type definitions working" << std::endl;
        std::cout << "  • Enum types: LoadChangeType, LoadType, TrendDirection working" << std::endl;
        std::cout << std::endl;
        std::cout << "🎉 Type definition fixes verified successfully!" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }
}
