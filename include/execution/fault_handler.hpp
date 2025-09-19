// include/execution/fault_handler.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "diagnostic/health_monitor.hpp"
#include <memory>
#include <map>
#include <queue>

namespace VCUCore {

class FaultHandler {
private:
    struct FaultRule {
        uint16_t faultCode;
        std::string condition;
        FaultSeverity severity;
        std::string recoveryProcedure;
        uint32_t timeoutMs;
        bool autoRecoverable;
    };
    
    std::map<uint16_t, FaultRule> faultRules_;
    std::unique_ptr<HealthMonitor> healthMonitor_;
    
    std::queue<FaultDiagnosis> faultQueue_;
    std::map<uint16_t, FaultDiagnosis> activeFaults_;
    std::vector<FaultDiagnosis> historicalFaults_;
    
    uint32_t maxHistorySize_;
    uint32_t processingIntervalMs_;
    
    // 恢复策略
    std::map<uint16_t, std::function<bool()>> recoveryStrategies_;
    std::map<uint16_t, uint32_t> recoveryAttempts_;

public:
    FaultHandler(uint32_t historySize = 1000, uint32_t processingInterval = 100);
    
    bool initialize(const std::string& ruleFile = "config/fault_rules.cfg");
    
    // 故障处理
    std::vector<FaultDiagnosis> detectFaults(const TractorVehicleState& state, 
                                           const SystemHealthStatus& healthStatus);
    bool handleFaults(const std::vector<FaultDiagnosis>& faults);
    
    // 恢复管理
    bool attemptRecovery(uint16_t faultCode);
    bool executeRecoveryProcedure(uint16_t faultCode);
    void monitorRecoveryProgress();
    
    // 规则管理
    bool addFaultRule(const FaultRule& rule);
    bool removeFaultRule(uint16_t faultCode);
    bool loadFaultRules(const std::string& ruleFile);
    bool saveFaultRules(const std::string& ruleFile);
    
    // 状态查询
    std::vector<FaultDiagnosis> getActiveFaults() const;
    std::vector<FaultDiagnosis> getHistoricalFaults() const;
    bool isFaultActive(uint16_t faultCode) const;
    
    // 诊断功能
    FaultStatistics getFaultStatistics() const;
    std::vector<FaultTrend> analyzeFaultTrends() const;

private:
    void initializeDefaultRules();
    FaultDiagnosis createFaultDiagnosis(uint16_t code, FaultSeverity severity,
                                      const std::string& description, const std::string& component);
    
    bool checkFaultCondition(const FaultRule& rule, const TractorVehicleState& state,
                           const SystemHealthStatus& healthStatus) const;
    
    void updateFaultState(const FaultDiagnosis& fault);
    void escalateFault(const FaultDiagnosis& fault);
    void resolveFault(uint16_t faultCode);
    
    void logFault(const FaultDiagnosis& fault);
    void notifyFault(const FaultDiagnosis& fault);
    
    bool canAutoRecover(uint16_t faultCode) const;
    uint32_t getRecoveryTimeout(uint16_t faultCode) const;
};

} // namespace VCUCore

bool handleSingleFault(const FaultDiagnosis& fault);

