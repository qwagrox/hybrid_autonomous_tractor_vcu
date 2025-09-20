// include/prediction/predictive_analytics.hpp - 修复重复定义问题
#pragma once
#include "vcu_core_types.hpp"
#include "models/vehicle_dynamics_model.hpp"
#include "models/battery_model.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"

#ifdef WITH_LIBMPC
#include <mpc/NLMPC.hpp>
#include <mpc/StateVector.hpp>
#include <mpc/ControlVector.hpp>
#endif

#include <memory>
#include <deque>

namespace VCUCore {

// 移除重复的PredictionPerformance定义，使用vcu_core_types.hpp中的定义

#ifdef WITH_LIBMPC
// libmpc++相关类型定义
using StateVector = mpc::StateVector<double, 13>;  // 13维状态向量
using ControlVector = mpc::ControlVector<double, 4>; // 4维控制向量
using NMPCController = mpc::NLMPC<double, 13, 4>;
#else
// 备用类型定义
struct StateVector {
    std::vector<double> data;
    StateVector() : data(13, 0.0) {}
    StateVector(const std::vector<double>& d) : data(d) {}
    double& operator[](size_t i) { return data[i]; }
    const double& operator[](size_t i) const { return data[i]; }
    size_t size() const { return data.size(); }
};

struct ControlVector {
    std::vector<double> data;
    ControlVector() : data(4, 0.0) {}
    ControlVector(const std::vector<double>& d) : data(d) {}
    double& operator[](size_t i) { return data[i]; }
    const double& operator[](size_t i) const { return data[i]; }
    size_t size() const { return data.size(); }
};

struct NMPCState {
    StateVector state;
    ControlVector control;
    double cost;
    bool feasible;
};
#endif

class PredictiveAnalytics {
private:
    // 模型组件
    std::unique_ptr<VehicleDynamicsModel> dynamicsModel_;
    std::unique_ptr<BatteryModel> batteryModel_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    
    // 预测历史
    std::deque<PredictionResult> predictionHistory_;
    std::deque<PerceptionData> perceptionHistory_;
    std::deque<PredictionPerformance> strategyPerformanceHistory_;
    uint32_t maxHistorySize_;
    
    // 当前策略
    PredictionStrategy currentStrategy_;
    
    // 性能指标
    double averagePredictionError_;
    double maxPredictionError_;
    double computationalTime_;
    
#ifdef WITH_LIBMPC
    // NMPC控制器
    std::unique_ptr<NMPCController> nmpcController_;
    StateVector currentState_;
    std::vector<StateVector> stateTrajectory_;
    std::vector<ControlVector> controlTrajectory_;
#endif

public:
    PredictiveAnalytics(uint32_t historySize = 1000);
    ~PredictiveAnalytics() = default;
    
    // 初始化
    bool initialize();
    void setStrategy(PredictionStrategy strategy);
    PredictionStrategy getCurrentStrategy() const;
    
    // 主要预测功能
    PredictionResult predict(const PerceptionData& perception);
    std::vector<PredictionResult> predictMultiStep(const PerceptionData& perception, 
                                                 uint32_t steps, float timeStep);
    
    // 策略特定预测
    PredictionResult predictWithLinearModel(const PerceptionData& perception);
    PredictionResult predictWithKalmanFilter(const PerceptionData& perception);
    PredictionResult predictWithNeuralNetwork(const PerceptionData& perception);
    PredictionResult predictWithHybridApproach(const PerceptionData& perception);
    
#ifdef WITH_LIBMPC
    PredictionResult predictWithNMPC(const PerceptionData& perception);
#endif
    
    // 历史数据管理
    void updatePredictionHistory(const PredictionResult& result);
    void updatePerceptionHistory(const PerceptionData& perception);
    std::vector<PredictionResult> getPredictionHistory(uint32_t count) const;
    
    // 性能评估
    double evaluatePredictionAccuracy(const PredictionResult& prediction, 
                                    const PerceptionData& actual);
    void updatePerformanceMetrics();
    PredictionPerformance getCurrentPerformance() const;
    
    // 学习和适应
    void learnFromHistory();
    void adaptStrategy();
    void updateModelParameters(const PerceptionData& perception);
    
    // 校准功能
    void calibrateModels(const std::vector<PerceptionData>& trainingData);
    void validatePredictions(const std::vector<PerceptionData>& validationData);

private:
    // 初始化辅助函数
    void initializeModels();
    void initializeNMPC();
    
    // 预测辅助函数
    TractorVehicleState extrapolateLinear(const TractorVehicleState& current, 
                                        const TractorVehicleState& previous, 
                                        float timeStep);
    
    // Kalman滤波器
    void initializeKalmanFilter();
    void updateKalmanFilter(const PerceptionData& perception);
    TractorVehicleState getKalmanPrediction();
    
    // 神经网络（简化版本）
    std::vector<double> neuralNetworkPredict(const std::vector<double>& input);
    void trainNeuralNetwork(const std::vector<std::vector<double>>& trainingData);
    
    // 混合方法
    PredictionResult combineStrategies(const std::vector<PredictionResult>& predictions);
    double calculateStrategyWeight(PredictionStrategy strategy) const;
    
#ifdef WITH_LIBMPC
    // NMPC相关函数
    StateVector convertToNMPCState(const PerceptionData& perception) const;
    PredictionResult convertFromNMPCState(const std::vector<StateVector>& solution) const;
    void setupNMPCConstraints();
    void setupNMPCObjective();
#endif
    
    // 性能评估辅助函数
    double calculatePredictionError(const PredictionResult& prediction, 
                                  const PerceptionData& actual) const;
    double calculateComputationalCost(PredictionStrategy strategy) const;
    double calculateRobustnessMetric(PredictionStrategy strategy) const;
    PredictionPerformance evaluateStrategyPerformance(PredictionStrategy strategy) const;
};

} // namespace VCUCore
