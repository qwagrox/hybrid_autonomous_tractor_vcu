// src/diagnostic/health_monitor.cpp
#include "health_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCUCore {

HealthMonitor::HealthMonitor(uint32_t historySize, uint32_t checkInterval)
    : maxHistorySize_(historySize), checkIntervalMs_(checkInterval) {
    
    initializeComponentHealth();
    
    // 初始化模型
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    batteryModel_ = std::make_unique<BatteryModel>();
    
    // 初始化组件寿命
    componentLifespans_ = {
        {"engine", 10000},     // 10,000小时
        {"motor", 20000},      // 20,000小时  
        {"battery", 5000},     // 5,000循环
        {"transmission", 15000}, // 15,000小时
        {"hydraulics", 8000}   // 8,000小时
    };
}

void HealthMonitor::initializeComponentHealth() {
    // 初始化所有组件健康状态
    componentHealth_ = {
        {"engine", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0, 
                   .operatingHours = 0, .lastMaintenance = 0}},
        {"motor", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                  .operatingHours = 0, .lastMaintenance = 0}},
        {"battery", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                    .operatingHours = 0, .lastMaintenance = 0}},
        {"transmission", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                         .operatingHours = 0, .lastMaintenance = 0}},
        {"hydraulics", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                       .operatingHours = 0, .lastMaintenance = 0}},
        {"sensors", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                    .operatingHours = 0, .lastMaintenance = 0}},
        {"can_bus", {.healthScore = 1.0f, .errorCount = 0, .warningCount = 0,
                    .operatingHours = 0, .lastMaintenance = 0}}
    };
}

SystemHealthStatus HealthMonitor::checkSystemHealth() {
    SystemHealthStatus status;
    status.timestamp = std::chrono::duration_cast<Timestamp>(
        std::chrono::system_clock::now().time_since_epoch());
    
    try {
        // 检查各组件健康
        for (auto& [component, health] : componentHealth_) {
            ComponentHealth compHealth = checkComponentHealth(component);
            health = compHealth;
            
            status.componentHealth[component] = compHealth.healthScore;
            status.activeFaults.insert(status.activeFaults.end(),
                                      compHealth.activeFaults.begin(),
                                      compHealth.activeFaults.end());
        }
        
        // 计算整体健康度
        status.overallHealth = calculateSystemHealth();
        status.isHealthy = status.overallHealth > 0.7f && status.activeFaults.empty();
        
        // 更新运行时间
        status.uptime = 0; // 需要从系统获取
        status.lastMaintenance = getLastMaintenanceTime();
        status.nextMaintenance = estimateNextMaintenance();
        
        // 更新历史记录
        updateHealthHistory(status);
        
    } catch (const std::exception& e) {
        std::cerr << "Health check failed: " << e.what() << std::endl;
        status.isHealthy = false;
        status.overallHealth = 0.3f;
    }
    
    return status;
}

ComponentHealth HealthMonitor::checkComponentHealth(const std::string& component) {
    ComponentHealth health = componentHealth_[component];
    
    // 模拟健康度计算（实际中需要从传感器和模型获取数据）
    if (component == "engine") {
        EngineState state = engineModel_->getEngineState();
        health.healthScore = calculateEngineHealth(state);
        health.operatingHours = state.operatingHours;
        
    } else if (component == "motor") {
        MotorState state = motorModel_->getMotorState();
        health.healthScore = calculateMotorHealth(state);
        health.operatingHours = state.operatingHours;
        
    } else if (component == "battery") {
        BatteryState state; // 需要从BMS获取
        health.healthScore = calculateBatteryHealth(state);
        
    } else {
        // 通用健康度衰减模型
        health.healthScore = 1.0f - (health.operatingHours / 20000.0f);
        health.healthScore = std::max(0.1f, health.healthScore);
    }
    
    // 检测故障
    detectComponentFaults(component, health);
    
    // 预测维护需求
    if (health.healthScore < 0.6f && !isMaintenanceDue(component)) {
        MaintenanceItem item = {
            .component = component,
            .description = "Predictive maintenance needed",
            .severity = MaintenanceSeverity::MEDIUM,
            .estimatedCost = 500.0f,
            .estimatedTime = 4,
            .dueDate = static_cast<uint32_t>(time(nullptr)) + 7 * 24 * 3600 // 1周后
        };
        health.maintenanceItems.push_back(item);
        notifyMaintenanceNeed(item);
    }
    
    return health;
}

float HealthMonitor::calculateEngineHealth(const EngineState& state) const {
    float health = 1.0f;
    
    // 基于运行时间的衰减
    health -= state.operatingHours / 20000.0f;
    
    // 基于温度的惩罚
    if (state.coolantTemperature > 95.0f) {
        health -= 0.01f * (state.coolantTemperature - 95.0f);
    }
    
    // 基于负载的惩罚
    float loadFactor = state.currentTorque / state.maxAvailableTorque;
    health -= 0.0001f * loadFactor * state.operatingHours;
    
    return std::max(0.1f, health);
}

float HealthMonitor::calculateMotorHealth(const MotorState& state) const {
    float health = 1.0f;
    
    // 基于运行时间的衰减
    health -= state.operatingHours / 30000.0f;
    
    // 基于温度的惩罚
    if (state.windingTemperature > 100.0f) {
        health -= 0.02f * (state.windingTemperature - 100.0f);
    }
    
    // 基于电流的惩罚
    health -= 0.00005f * std::abs(state.currentCurrent) * state.operatingHours;
    
    return std::max(0.1f, health);
}

float HealthMonitor::calculateSystemHealth() const {
    float totalHealth = 0.0f;
    int count = 0;
    
    for (const auto& [component, health] : componentHealth_) {
        totalHealth += health.healthScore;
        count++;
    }
    
    return count > 0 ? totalHealth / count : 0.5f;
}

void HealthMonitor::detectComponentFaults(const std::string& component, ComponentHealth& health) {
    // 清除旧的故障
    health.activeFaults.erase(
        std::remove_if(health.activeFaults.begin(), health.activeFaults.end(),
            [](const FaultDiagnosis& fault) { return !fault.isActive; }),
        health.activeFaults.end()
    );
    
    // 检测新故障（简化实现）
    if (health.healthScore < 0.3f) {
        FaultDiagnosis fault = {
            .faultCode = 0x3000, // 通用健康故障
            .severity = FaultSeverity::HIGH,
            .description = "Component health critically low: " + component,
            .component = component,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch()).count(),
            .duration = 0,
            .isActive = true,
            .isRecoverable = true,
            .recoverySteps = {"Perform maintenance", "Replace if necessary"}
        };
        health.activeFaults.push_back(fault);
    }
    else if (health.healthScore < 0.6f) {
        FaultDiagnosis fault = {
            .faultCode = 0x3001,
            .severity = FaultSeverity::MEDIUM,
            .description = "Component health degraded: " + component,
            .component = component,
            .timestamp = std::chrono::duration_cast<Timestamp>(
                std::chrono::system_clock::now().time_since_epoch()).count(),
            .duration = 0,
            .isActive = true,
            .isRecoverable = true,
            .recoverySteps = {"Schedule maintenance", "Monitor closely"}
        };
        health.activeFaults.push_back(fault);
    }
}

std::vector<MaintenanceItem> HealthMonitor::getMaintenanceSchedule() const {
    std::vector<MaintenanceItem> schedule;
    
    for (const auto& [component, health] : componentHealth_) {
        for (const auto& item : health.maintenanceItems) {
            schedule.push_back(item);
        }
    }
    
    // 按紧急程度排序
    std::sort(schedule.begin(), schedule.end(),
        [](const MaintenanceItem& a, const MaintenanceItem& b) {
            return a.severity > b.severity;
        });
    
    return schedule;
}

bool HealthMonitor::isMaintenanceDue(const std::string& component) const {
    if (!componentHealth_.count(component)) {
        return false;
    }
    
    const auto& health = componentHealth_.at(component);
    return !health.maintenanceItems.empty();
}

void HealthMonitor::updateHealthHistory(const SystemHealthStatus& status) {
    healthHistory_.push_back(status);
    if (healthHistory_.size() > maxHistorySize_) {
        healthHistory_.pop_front();
    }
}

void HealthMonitor::notifyMaintenanceNeed(const MaintenanceItem& item) {
    std::cout << "MAINTENANCE NEEDED: " << item.component 
              << " - " << item.description
              << " [Severity: " << static_cast<int>(item.severity) << "]" << std::endl;
    
    // 这里可以集成通知系统（邮件、短信、日志等）
}

} // namespace VCUCore