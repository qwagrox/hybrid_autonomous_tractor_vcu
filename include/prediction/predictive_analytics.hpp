// include/prediction/predictive_analytics.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/vehicle_dynamics_model.hpp"
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <memory>
#include <deque>

namespace VCUCore {

// 预测策略枚举
enum class PredictionStrategy {
    MACHINE_LEARNING,    // 机器学习预测
    NMPC_OPTIMIZATION,   // 非线性模型预测控制
    HYBRID,              // 混合模式
    FALLBACK             // 降级模式
};

// NMPC优化状态
struct NMPCState {
    ACADO::VariablesGrid states;
    ACADO::VariablesGrid controls;
    ACADO::VariablesGrid parameters;
    double optimizationTime;
    double costValue;
    int iterations;
    bool converged;
};

class PredictiveAnalytics {
private:
    // 预测策略
    PredictionStrategy currentStrategy_;
    std::atomic<bool> strategySwitchRequested_;
    PredictionStrategy requestedStrategy_;
    
    // NMPC相关成员
    std::unique_ptr<ACADO::RealTimeAlgorithm> nmpcSolver_;
    std::unique_ptr<ACADO::StaticReferenceTrajectory> referenceTrajectory_;
    std::unique_ptr<ACADO::Controller> nmpcController_;
    
    // NMPC参数
    ACADO::DifferentialEquation systemDynamics_;
    ACADO::Function systemModel_;
    ACADO::OCP nmpcProblem_;
    
    // 优化问题参数
    int predictionHorizonSteps_;
    double controlInterval_;
    ACADO::VariablesGrid nmpcWeights_;
    
    // 机器学习模型
    std::unique_ptr<MLModel> mlModel_;
    std::unique_ptr<MLModel> fallbackModel_;
    
    // 模型切换逻辑
    struct StrategyConfig {
        PredictionStrategy strategy;
        double activationThreshold;
        double deactivationThreshold;
        int minSamplesRequired;
        bool requiresCalibration;
    };
    
    std::map<PredictionStrategy, StrategyConfig> strategyConfigs_;
    std::deque<PredictionPerformance> strategyPerformanceHistory_;

public:
    PredictiveAnalytics(float horizon = 5.0f, float timeStep = 0.1f, uint32_t historySize = 1000);
    ~PredictiveAnalytics();
    
    // 策略管理
    bool switchPredictionStrategy(PredictionStrategy newStrategy);
    PredictionStrategy getCurrentStrategy() const;
    PredictionStrategy evaluateBestStrategy() const;
    
    // NMPC设置
    bool setupNMPCProblem();
    bool initializeNMPCSolver();
    void updateNMPCReferences(const PerceptionData& perception, const PredictionResult& prediction);
    
    // 在线优化
    NMPCState solveNMPCOnline(const PerceptionData& currentState, const PredictionResult& reference);
    bool warmStartNMPC(const NMPCState& previousSolution);
    
    // 混合预测
    PredictionResult executeHybridPrediction(const PerceptionData& perception, 
                                           const PredictionResult& initialPrediction);
    
    // 性能监控
    PredictionPerformance evaluateStrategyPerformance(PredictionStrategy strategy) const;
    void updateStrategyWeightsBasedOnPerformance();
    
    // 故障处理
    bool checkNMPCConvergence() const;
    void activateFallbackStrategy();
    void recalibrateModels();

private:
    // NMPC问题定义
    void defineSystemDynamics();
    void setupCostFunction();
    void configureConstraints();
    void initializeNMPCWeights();
    
    // 模型切换逻辑
    bool validateStrategySwitch(PredictionStrategy newStrategy) const;
    void performStrategySwitch(PredictionStrategy newStrategy);
    void saveStrategyState(PredictionStrategy strategy);
    void restoreStrategyState(PredictionStrategy strategy);
    
    // 实时优化
    ACADO::DVector convertToNMPCState(const PerceptionData& perception) const;
    PredictionResult convertFromNMPCState(const ACADO::VariablesGrid& solution) const;
    
    // 性能评估
    double calculatePredictionError(const PredictionResult& prediction, 
                                  const PerceptionData& actual) const;
    double calculateComputationalCost(PredictionStrategy strategy) const;
    double calculateRobustnessMetric(PredictionStrategy strategy) const;
};

// 策略性能结构
struct PredictionPerformance {
    PredictionStrategy strategy;
    double averageError;
    double maxError;
    double computationalTime;
    double robustnessScore;
    uint32_t sampleCount;
    Timestamp lastUpdate;
};

} // namespace VCUCore