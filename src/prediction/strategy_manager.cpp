// src/prediction/strategy_manager.cpp
#include "predictive_analytics.hpp"
#include <algorithm>
#include <thread>

namespace VCUCore {

bool PredictiveAnalytics::validateStrategySwitch(PredictionStrategy newStrategy) const {
    const auto& newConfig = strategyConfigs_.at(newStrategy);
    const auto& currentConfig = strategyConfigs_.at(currentStrategy_);
    
    // 检查最小样本要求
    auto performance = evaluateStrategyPerformance(newStrategy);
    if (performance.sampleCount < newConfig.minSamplesRequired && 
        newStrategy != PredictionStrategy::FALLBACK) {
        std::cerr << "Insufficient samples for strategy switch: " 
                  << performance.sampleCount << " < " << newConfig.minSamplesRequired << std::endl;
        return false;
    }
    
    // 检查性能阈值
    if (performance.averageError > newConfig.activationThreshold &&
        newStrategy != PredictionStrategy::FALLBACK) {
        std::cerr << "Performance below activation threshold: " 
                  << performance.averageError << " > " << newConfig.activationThreshold << std::endl;
        return false;
    }
    
    // 检查校准要求
    if (newConfig.requiresCalibration && !mlModel_->isCalibrated()) {
        std::cerr << "Strategy requires calibration" << std::endl;
        return false;
    }
    
    return true;
}

void PredictiveAnalytics::performStrategySwitch(PredictionStrategy newStrategy) {
    switch (newStrategy) {
        case PredictionStrategy::MACHINE_LEARNING:
            // 停止NMPC求解器
            if (nmpcSolver_) {
                nmpcSolver_->shutdown();
            }
            // 激活ML模型
            mlModel_->setActive(true);
            break;
            
        case PredictionStrategy::NMPC_OPTIMIZATION:
            // 初始化NMPC求解器
            if (!nmpcSolver_->isInitialized()) {
                initializeNMPCSolver();
            }
            // 停用ML模型
            mlModel_->setActive(false);
            break;
            
        case PredictionStrategy::HYBRID:
            // 保持两者都激活
            if (!nmpcSolver_->isInitialized()) {
                initializeNMPCSolver();
            }
            mlModel_->setActive(true);
            break;
            
        case PredictionStrategy::FALLBACK:
            // 使用降级模式
            if (nmpcSolver_) {
                nmpcSolver_->shutdown();
            }
            mlModel_->setActive(false);
            break;
    }
}

void PredictiveAnalytics::saveStrategyState(PredictionStrategy strategy) {
    // 保存当前策略状态以便恢复
    switch (strategy) {
        case PredictionStrategy::NMPC_OPTIMIZATION:
        case PredictionStrategy::HYBRID:
            // 保存NMPC状态
            if (nmpcController_) {
                nmpcStateCache_ = nmpcController_->getState();
            }
            break;
            
        case PredictionStrategy::MACHINE_LEARNING:
            // 保存ML模型状态
            mlModel_->saveState("ml_model_state.bin");
            break;
            
        default:
            break;
    }
}

void PredictiveAnalytics::restoreStrategyState(PredictionStrategy strategy) {
    // 恢复策略状态
    switch (strategy) {
        case PredictionStrategy::NMPC_OPTIMIZATION:
        case PredictionStrategy::HYBRID:
            // 恢复NMPC状态
            if (nmpcController_ && !nmpcStateCache_.empty()) {
                nmpcController_->setState(nmpcStateCache_);
            }
            break;
            
        case PredictionStrategy::MACHINE_LEARNING:
            // 恢复ML模型状态
            mlModel_->loadState("ml_model_state.bin");
            break;
            
        default:
            break;
    }
}

} // namespace VCUCore