// include/diagnostic/health_monitor.hpp - 修复版本
#pragma once
#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/battery_model.hpp"
#include <memory>
#include <map>
#include <deque>

namespace VCUCore {

class HealthMonitor {
private:
    struct ComponentHealth {
        double overallHealth;
        double stateOfHealth;
        double degradationRate;
        uint32_t cycleCount;
        double temperatureHealth;
        double voltageHealth;
        double currentHealth;
        uint64_t lastMaintenanceTime;
        uint32_t errorCount;
        uint32_t warningCount;
        uint64_t operatingHours;
        uint32_t lastMaintenance;
        std::vector<FaultDiagnosis> activeFaults;
        std::vector<MaintenanceItem> maintenanceItems;
        std::vector<std::string> healthWarnings;
    };
    
    std::map<std::string, ComponentHealth> componentHealth_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    std::unique_ptr<BatteryModel> batteryModel_;
    
    std::deque<SystemHealthStatus> healthHistory_;
    uint32_t maxHistorySize_;
    uint32_t checkIntervalMs_;
    
    // 预测性维护
    std::map<std::string, uint32_t> componentLifespans_;
    std::map<std::string, uint32_t> componentUsage_;

public:
    HealthMonitor(uint32_t historySize = 1000, uint32_t checkInterval = 1000);
    
    bool initialize();
    SystemHealthStatus checkSystemHealth();
    SystemHealthStatus getSystemHealth();  // 添加getSystemHealth方法
    ComponentHealth checkComponentHealth(const std::string& component);
    
    // 维护管理
    std::vector<MaintenanceItem> getMaintenanceSchedule() const;
    bool recordMaintenance(const std::string& component, const MaintenanceRecord& record);
    bool isMaintenanceDue(const std::string& component) const;
    
    // 健康预测
    HealthForecast predictComponentHealth(const std::string& component, uint32_t hours) const;
    uint32_t estimateRemainingLife(const std::string& component) const;
    
    // 诊断功能
    std::vector<DiagnosticTest> runDiagnostics() const;
    DiagnosticResult performDiagnosticTest(const DiagnosticTest& test);
    
    // 报告生成
    std::string generateHealthReport() const;
    std::string generateMaintenanceReport() const;
    
    // 历史数据管理
    void updateHealthHistory(const SystemHealthStatus& status);
    std::vector<SystemHealthStatus> getHealthHistory(uint32_t hours) const;
    void clearHealthHistory();

private:
    void initializeComponentHealth();
    void updateComponentHealth(const std::string& component, const ComponentHealth& health);
    double calculateOverallHealth() const;
    void checkThresholds();
    void generateAlerts();
    
    // 预测性维护算法
    double predictDegradation(const std::string& component, uint32_t hours) const;
    bool shouldScheduleMaintenance(const std::string& component) const;
    
    // 数据分析
    void analyzeHealthTrends();
    void detectAnomalies();
    
    // 配置管理
    void loadConfiguration();
    void saveConfiguration() const;
};

} // namespace VCUCore
