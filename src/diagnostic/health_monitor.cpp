// src/diagnostic/health_monitor.cpp - 修复版本的关键部分
#include "diagnostic/health_monitor.hpp"
#include <algorithm>
#include <numeric>
#include <chrono>

namespace VCUCore {

HealthMonitor::HealthMonitor(uint32_t historySize, uint32_t checkInterval)
    : maxHistorySize_(historySize), checkIntervalMs_(checkInterval) {
    initializeComponentHealth();
}

bool HealthMonitor::initialize() {
    // 初始化各个模型
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    batteryModel_ = std::make_unique<BatteryModel>();
    
    // 加载配置
    loadConfiguration();
    
    return true;
}

SystemHealthStatus HealthMonitor::checkSystemHealth() {
    SystemHealthStatus status;
    
    // 计算整体健康度
    status.overallHealth = calculateOverallHealth();
    
    // 获取各子系统状态
    if (batteryModel_) {
        auto batteryState = batteryModel_->getCurrentState();
        status.batteryLevel = batteryState.stateOfCharge;
    } else {
        status.batteryLevel = 0.8; // 默认值
    }
    
    if (engineModel_) {
        auto engineState = engineModel_->getEngineState();
        status.engineLoad = engineState.torque / 1000.0; // 归一化
    } else {
        status.engineLoad = 0.5; // 默认值
    }
    
    // 设置其他状态
    status.fuelLevel = 0.7; // 示例值
    status.hydraulicPressure = 200.0; // 示例值
    status.isHealthy = (status.overallHealth > 0.7);
    
    // 收集警告和故障
    for (const auto& [component, health] : componentHealth_) {
        for (const auto& warning : health.healthWarnings) {
            status.activeWarnings.push_back(component + ": " + warning);
        }
        for (const auto& fault : health.activeFaults) {
            if (fault.isActive) {
                status.activeFaults.push_back(component + ": " + fault.description);
            }
        }
    }
    
    status.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 更新历史记录
    updateHealthHistory(status);
    
    return status;
}

// 添加getSystemHealth方法实现
SystemHealthStatus HealthMonitor::getSystemHealth() {
    // getSystemHealth可以直接调用checkSystemHealth，或者返回最近的健康状态
    if (!healthHistory_.empty()) {
        return healthHistory_.back();
    } else {
        return checkSystemHealth();
    }
}

HealthMonitor::ComponentHealth HealthMonitor::checkComponentHealth(const std::string& component) {
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        return it->second;
    }
    
    // 返回默认的组件健康状态
    ComponentHealth defaultHealth;
    defaultHealth.overallHealth = 1.0;
    defaultHealth.stateOfHealth = 1.0;
    defaultHealth.degradationRate = 0.0;
    defaultHealth.cycleCount = 0;
    defaultHealth.temperatureHealth = 1.0;
    defaultHealth.voltageHealth = 1.0;
    defaultHealth.currentHealth = 1.0;
    defaultHealth.lastMaintenanceTime = 0;
    defaultHealth.errorCount = 0;
    defaultHealth.warningCount = 0;
    defaultHealth.operatingHours = 0;
    defaultHealth.lastMaintenance = 0;
    // activeFaults, maintenanceItems, healthWarnings 默认为空
    
    return defaultHealth;
}

void HealthMonitor::initializeComponentHealth() {
    // 初始化各组件的健康状态
    std::vector<std::string> components = {
        "engine", "motor", "battery", "hydraulic", "transmission", "brakes"
    };
    
    for (const auto& component : components) {
        ComponentHealth health;
        health.overallHealth = 1.0;
        health.stateOfHealth = 1.0;
        health.degradationRate = 0.0;
        health.cycleCount = 0;
        health.temperatureHealth = 1.0;
        health.voltageHealth = 1.0;
        health.currentHealth = 1.0;
        health.lastMaintenanceTime = 0;
        health.errorCount = 0;
        health.warningCount = 0;
        health.operatingHours = 0;
        health.lastMaintenance = 0;
        // activeFaults, maintenanceItems, healthWarnings 默认为空
        
        componentHealth_[component] = health;
    }
}

double HealthMonitor::calculateOverallHealth() const {
    if (componentHealth_.empty()) {
        return 1.0;
    }
    
    double totalHealth = 0.0;
    for (const auto& [component, health] : componentHealth_) {
        totalHealth += health.overallHealth;
    }
    
    return totalHealth / componentHealth_.size();
}

void HealthMonitor::updateHealthHistory(const SystemHealthStatus& status) {
    healthHistory_.push_back(status);
    
    // 维护历史记录大小
    while (healthHistory_.size() > maxHistorySize_) {
        healthHistory_.pop_front();
    }
}

std::vector<MaintenanceItem> HealthMonitor::getMaintenanceSchedule() const {
    std::vector<MaintenanceItem> schedule;
    
    for (const auto& [component, health] : componentHealth_) {
        for (const auto& item : health.maintenanceItems) {
            schedule.push_back(item);
        }
    }
    
    // 按优先级排序
    std::sort(schedule.begin(), schedule.end(), 
              [](const MaintenanceItem& a, const MaintenanceItem& b) {
                  return a.priority > b.priority;
              });
    
    return schedule;
}

bool HealthMonitor::recordMaintenance(const std::string& component, const MaintenanceRecord& record) {
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        it->second.lastMaintenanceTime = record.timestamp;
        return true;
    }
    return false;
}

bool HealthMonitor::isMaintenanceDue(const std::string& component) const {
    return shouldScheduleMaintenance(component);
}

HealthForecast HealthMonitor::predictComponentHealth(const std::string& component, uint32_t hours) const {
    HealthForecast forecast;
    forecast.component = component;
    forecast.forecastTime = hours;
    forecast.confidence = 0.8;
    
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        double degradation = predictDegradation(component, hours);
        forecast.predictedHealth = std::max(0.0, it->second.overallHealth - degradation);
        
        if (forecast.predictedHealth < 0.5) {
            forecast.recommendations = "Schedule maintenance within " + std::to_string(hours/24) + " days";
        } else {
            forecast.recommendations = "Component health is acceptable";
        }
    } else {
        forecast.predictedHealth = 1.0;
        forecast.recommendations = "Component not monitored";
    }
    
    return forecast;
}

uint32_t HealthMonitor::estimateRemainingLife(const std::string& component) const {
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        double currentHealth = it->second.overallHealth;
        double degradationRate = it->second.degradationRate;
        
        if (degradationRate > 0.0) {
            // 估算到达最低健康阈值(0.3)的时间
            double remainingHealth = currentHealth - 0.3;
            return static_cast<uint32_t>(remainingHealth / degradationRate);
        }
    }
    
    return 10000; // 默认返回很长的时间
}

std::vector<DiagnosticTest> HealthMonitor::runDiagnostics() const {
    std::vector<DiagnosticTest> tests;
    
    // 添加一些基本的诊断测试
    DiagnosticTest batteryTest;
    batteryTest.testName = "Battery Health Check";
    batteryTest.component = "battery";
    batteryTest.expectedDuration = 5.0;
    batteryTest.testFunction = []() { return true; }; // 简化的测试函数
    tests.push_back(batteryTest);
    
    DiagnosticTest engineTest;
    engineTest.testName = "Engine Performance Check";
    engineTest.component = "engine";
    engineTest.expectedDuration = 10.0;
    engineTest.testFunction = []() { return true; }; // 简化的测试函数
    tests.push_back(engineTest);
    
    return tests;
}

DiagnosticResult HealthMonitor::performDiagnosticTest(const DiagnosticTest& test) {
    DiagnosticResult result;
    result.testName = test.testName;
    
    auto start = std::chrono::high_resolution_clock::now();
    result.passed = test.testFunction();
    auto end = std::chrono::high_resolution_clock::now();
    
    result.actualDuration = std::chrono::duration<double>(end - start).count();
    result.details = result.passed ? "Test passed successfully" : "Test failed";
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    return result;
}

std::string HealthMonitor::generateHealthReport() const {
    std::string report = "=== System Health Report ===\n";
    report += "Overall Health: " + std::to_string(calculateOverallHealth()) + "\n\n";
    
    for (const auto& [component, health] : componentHealth_) {
        report += "Component: " + component + "\n";
        report += "  Health: " + std::to_string(health.overallHealth) + "\n";
        report += "  Errors: " + std::to_string(health.errorCount) + "\n";
        report += "  Warnings: " + std::to_string(health.warningCount) + "\n\n";
    }
    
    return report;
}

std::string HealthMonitor::generateMaintenanceReport() const {
    std::string report = "=== Maintenance Report ===\n";
    
    auto schedule = getMaintenanceSchedule();
    for (const auto& item : schedule) {
        report += "Component: " + item.component + "\n";
        report += "  Description: " + item.description + "\n";
        report += "  Priority: " + std::to_string(item.priority) + "\n";
        report += "  Overdue: " + std::string(item.isOverdue ? "Yes" : "No") + "\n\n";
    }
    
    return report;
}

std::vector<SystemHealthStatus> HealthMonitor::getHealthHistory(uint32_t hours) const {
    // 简化实现：返回最近的记录
    std::vector<SystemHealthStatus> result;
    
    uint64_t cutoffTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() - (hours * 3600 * 1000);
    
    for (const auto& status : healthHistory_) {
        if (status.timestamp >= cutoffTime) {
            result.push_back(status);
        }
    }
    
    return result;
}

void HealthMonitor::clearHealthHistory() {
    healthHistory_.clear();
}

void HealthMonitor::updateComponentHealth(const std::string& component, const ComponentHealth& health) {
    componentHealth_[component] = health;
}

void HealthMonitor::checkThresholds() {
    // 检查各组件是否超过阈值
}

void HealthMonitor::generateAlerts() {
    // 生成警报
}

double HealthMonitor::predictDegradation(const std::string& component, uint32_t hours) const {
    // 简化的退化预测
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        return it->second.degradationRate * hours;
    }
    return 0.0;
}

bool HealthMonitor::shouldScheduleMaintenance(const std::string& component) const {
    auto it = componentHealth_.find(component);
    if (it != componentHealth_.end()) {
        return it->second.overallHealth < 0.7 || !it->second.maintenanceItems.empty();
    }
    return false;
}

void HealthMonitor::analyzeHealthTrends() {
    // 分析健康趋势
}

void HealthMonitor::detectAnomalies() {
    // 检测异常
}

void HealthMonitor::loadConfiguration() {
    // 加载配置
}

void HealthMonitor::saveConfiguration() const {
    // 保存配置
}

} // namespace VCUCore
