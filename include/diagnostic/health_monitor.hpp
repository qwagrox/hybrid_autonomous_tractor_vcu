// include/diagnostic/health_monitor.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/battery_model.hpp"
#include <memory>
#include <map>

namespace VCUCore {

class HealthMonitor {
private:
    struct ComponentHealth {
        float healthScore;
        uint32_t errorCount;
        uint32_t warningCount;
        uint64_t operatingHours;
        uint32_t lastMaintenance;
        std::vector<FaultDiagnosis> activeFaults;
        std::vector<MaintenanceItem> maintenanceItems;
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
    HealthReport generateHealthReport() const;
    std::vector<HealthTrend> analyzeHealthTrends() const;

private:
    void initializeComponentHealth();
    void updateComponentHealth(const std::string& component, float deltaHealth);
    void detectComponentFaults(const std::string& component, ComponentHealth& health);
    
    float calculateEngineHealth(const EngineState& state) const;
    float calculateMotorHealth(const MotorState& state) const;
    float calculateBatteryHealth(const BatteryState& state) const;
    float calculateSystemHealth() const;
    
    void updateHealthHistory(const SystemHealthStatus& status);
    void predictMaintenanceNeeds();
    
    bool checkWearAndTear(const std::string& component) const;
    bool checkPerformanceDegradation(const std::string& component) const;
    
    void notifyMaintenanceNeed(const MaintenanceItem& item);
    void escalateCriticalHealthIssue(const std::string& component, float healthScore);
};

} // namespace VCUCore