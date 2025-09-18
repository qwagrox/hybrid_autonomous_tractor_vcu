// src/execution/fault_handler.cpp
#include "fault_handler.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>

namespace VCUCore {

FaultHandler::FaultHandler(uint32_t historySize, uint32_t processingInterval)
    : maxHistorySize_(historySize), processingIntervalMs_(processingInterval) {
    
    initializeDefaultRules();
    healthMonitor_ = std::make_unique<HealthMonitor>();
}

void FaultHandler::initializeDefaultRules() {
    // 默认故障规则
    faultRules_ = {
        {0x1001, {
            .faultCode = 0x1001,
            .condition = "engine_temperature > 105",
            .severity = FaultSeverity::HIGH,
            .recoveryProcedure = "Reduce load and cool engine",
            .timeoutMs = 30000,
            .autoRecoverable = true
        }},
        {0x1002, {
            .faultCode = 0x1002,
            .condition = "battery_voltage < 480",
            .severity = FaultSeverity::MEDIUM,
            .recoveryProcedure = "Reduce electrical load and charge battery",
            .timeoutMs = 60000,
            .autoRecoverable = true
        }},
        {0x1003, {
            .faultCode = 0x1003,
            .condition = "wheel_slip > 0.4",
            .severity = FaultSeverity::MEDIUM,
            .recoveryProcedure = "Reduce torque and check terrain",
            .timeoutMs = 10000,
            .autoRecoverable = true
        }},
        {0x2001, {
            .faultCode = 0x2001,
            .condition = "can_bus_errors > 10",
            .severity = FaultSeverity::LOW,
            .recoveryProcedure = "Reset CAN bus and check connections",
            .timeoutMs = 5000,
            .autoRecoverable = true
        }}
    };
}

bool FaultHandler::initialize(const std::string& ruleFile) {
    // 加载故障规则
    if (!loadFaultRules(ruleFile)) {
        std::cout << "Using default fault rules" << std::endl;
    }
    
    // 初始化健康监控
    if (!healthMonitor_->initialize()) {
        std::cerr << "Health monitor initialization failed" << std::endl;
        return false;
    }
    
    return true;
}

std::vector<FaultDiagnosis> FaultHandler::detectFaults(const VehicleState& state,
                                                     const SystemHealthStatus& healthStatus) {
    
    std::vector<FaultDiagnosis> detectedFaults;
    
    // 检查所有故障规则
    for (const auto& [faultCode, rule] : faultRules_) {
        if (checkFaultCondition(rule, state, healthStatus)) {
            FaultDiagnosis fault = createFaultDiagnosis(
                faultCode,
                rule.severity,
                "Fault detected: " + rule.condition,
                "System"
            );
            
            detectedFaults.push_back(fault);
        }
    }
    
    // 检查健康监控器的故障
    auto healthFaults = healthMonitor_->checkSystemHealth().activeFaults;
    detectedFaults.insert(detectedFaults.end(), healthFaults.begin(), healthFaults.end());
    
    return detectedFaults;
}

bool FaultHandler::handleFaults(const std::vector<FaultDiagnosis>& faults) {
    bool allHandled = true;
    
    for (const auto& fault : faults) {
        if (!handleSingleFault(fault)) {
            allHandled = false;
        }
    }
    
    return allHandled;
}

bool FaultHandler::handleSingleFault(const FaultDiagnosis& fault) {
    // 更新故障状态
    updateFaultState(fault);
    
    // 记录故障
    logFault(fault);
    
    // 通知相关人员
    notifyFault(fault);
    
    // 尝试自动恢复
    if (faultRules_.count(fault.faultCode) && faultRules_[fault.faultCode].autoRecoverable) {
        if (attemptRecovery(fault.faultCode)) {
            std::cout << "Auto-recovery initiated for fault: " << fault.faultCode << std::endl;
            return true;
        }
    }
    
    // 需要人工干预的故障
    if (fault.severity >= FaultSeverity::HIGH) {
        escalateFault(fault);
    }
    
    return false;
}

bool FaultHandler::attemptRecovery(uint16_t faultCode) {
    if (!canAutoRecover(faultCode)) {
        return false;
    }
    
    if (recoveryStrategies_.count(faultCode)) {
        // 执行自定义恢复策略
        return recoveryStrategies_[faultCode]();
    }
    
    // 执行默认恢复程序
    return executeRecoveryProcedure(faultCode);
}

bool FaultHandler::executeRecoveryProcedure(uint16_t faultCode) {
    if (!faultRules_.count(faultCode)) {
        return false;
    }
    
    const auto& rule = faultRules_[faultCode];
    
    try {
        // 根据故障代码执行相应的恢复操作
        switch (faultCode) {
            case 0x1001: // 发动机过热
                // 减少发动机负载，增加冷却
                std::cout << "Executing engine cooling procedure" << std::endl;
                break;
                
            case 0x1002: // 电池电压低
                // 减少电气负载，启动充电
                std::cout << "Executing battery charging procedure" << std::endl;
                break;
                
            case 0x1003: // 轮滑
                // 减少扭矩输出
                std::cout << "Executing wheel slip recovery" << std::endl;
                break;
                
            case 0x2001: // CAN总线错误
                // 重置CAN总线
                std::cout << "Resetting CAN bus" << std::endl;
                break;
                
            default:
                std::cout << "Executing generic recovery procedure" << std::endl;
                break;
        }
        
        // 记录恢复尝试
        recoveryAttempts_[faultCode]++;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Recovery procedure failed: " << e.what() << std::endl;
        return false;
    }
}

void FaultHandler::updateFaultState(const FaultDiagnosis& fault) {
    // 添加到活动故障列表
    activeFaults_[fault.faultCode] = fault;
    
    // 添加到历史记录
    historicalFaults_.push_back(fault);
    if (historicalFaults_.size() > maxHistorySize_) {
        historicalFaults_.erase(historicalFaults_.begin());
    }
    
    // 更新故障队列
    faultQueue_.push(fault);
    while (faultQueue_.size() > 100) {
        faultQueue_.pop();
    }
}

void FaultHandler::escalateFault(const FaultDiagnosis& fault) {
    // 严重故障升级处理
    std::cout << "ESCALATING FAULT: " << fault.description 
              << " (Severity: " << static_cast<int>(fault.severity) << ")" << std::endl;
    
    // 这里可以集成通知系统（邮件、短信等）
    // 或者触发更高级别的恢复程序
}

void FaultHandler::resolveFault(uint16_t faultCode) {
    if (activeFaults_.count(faultCode)) {
        // 标记故障为已解决
        activeFaults_[faultCode].isActive = false;
        activeFaults_[faultCode].duration = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count() - 
            activeFaults_[faultCode].timestamp;
        
        // 从活动故障中移除
        activeFaults_.erase(faultCode);
    }
}

bool FaultHandler::checkFaultCondition(const FaultRule& rule, const VehicleState& state,
                                    const SystemHealthStatus& healthStatus) const {
    
    // 简化实现：实际中需要解析条件表达式
    if (rule.condition.find("engine_temperature") != std::string::npos) {
        // 检查发动机温度
        for (const auto& fault : healthStatus.activeFaults) {
            if (fault.component == "Engine" && fault.description.find("temperature") != std::string::npos) {
                return true;
            }
        }
    }
    
    if (rule.condition.find("battery_voltage") != std::string::npos) {
        // 检查电池电压
        for (const auto& fault : healthStatus.activeFaults) {
            if (fault.component == "Battery" && fault.description.find("voltage") != std::string::npos) {
                return true;
            }
        }
    }
    
    if (rule.condition.find("wheel_slip") != std::string::npos) {
        // 检查轮滑
        return state.wheelSlipRatio > 0.4f;
    }
    
    if (rule.condition.find("can_bus_errors") != std::string::npos) {
        // 检查CAN总线错误
        for (const auto& fault : healthStatus.activeFaults) {
            if (fault.component == "CAN" && fault.severity >= FaultSeverity::MEDIUM) {
                return true;
            }
        }
    }
    
    return false;
}

FaultDiagnosis FaultHandler::createFaultDiagnosis(uint16_t code, FaultSeverity severity,
                                                const std::string& description, const std::string& component) {
    
    return {
        .faultCode = code,
        .severity = severity,
        .description = description,
        .component = component,
        .timestamp = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch()).count(),
        .duration = 0,
        .isActive = true,
        .isRecoverable = true,
        .recoverySteps = {"Automatic recovery attempt", "Manual intervention if needed"}
    };
}

void FaultHandler::logFault(const FaultDiagnosis& fault) {
    std::cout << "FAULT: " << fault.faultCode << " - " << fault.description 
              << " [Severity: " << static_cast<int>(fault.severity) << "]" << std::endl;
}

bool FaultHandler::loadFaultRules(const std::string& ruleFile) {
    std::ifstream file(ruleFile);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            FaultRule rule;
            std::string severityStr;
            
            if (iss >> std::hex >> rule.faultCode >> severityStr >> rule.timeoutMs >> rule.autoRecoverable) {
                std::getline(iss, rule.condition);
                std::getline(iss, rule.recoveryProcedure);
                
                // 转换严重性字符串
                if (severityStr == "LOW") rule.severity = FaultSeverity::LOW;
                else if (severityStr == "MEDIUM") rule.severity = FaultSeverity::MEDIUM;
                else if (severityStr == "HIGH") rule.severity = FaultSeverity::HIGH;
                else if (severityStr == "CRITICAL") rule.severity = FaultSeverity::CRITICAL;
                
                faultRules_[rule.faultCode] = rule;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading fault rules: " << e.what() << std::endl;
        return false;
    }
}

std::vector<FaultDiagnosis> FaultHandler::getActiveFaults() const {
    std::vector<FaultDiagnosis> active;
    for (const auto& [code, fault] : activeFaults_) {
        active.push_back(fault);
    }
    return active;
}

bool FaultHandler::isFaultActive(uint16_t faultCode) const {
    return activeFaults_.count(faultCode) > 0 && activeFaults_.at(faultCode).isActive;
}

} // namespace VCUCore