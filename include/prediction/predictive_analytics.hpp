// include/prediction/predictive_analytics.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/vehicle_dynamics_model.hpp"
#include <Eigen/Dense>
#include <memory>
#include <deque>

#ifdef WITH_LIBMPC
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>
#endif

namespace VCUCore {

// 注意：PredictionStrategy已在vcu_core_types.hpp中定义，这里不重复定义

// libmpc++ MPC参数定义
static constexpr int STATE_DIM = 6;      // 状态维度 [x, y, theta, v, omega, a]
static constexpr int CONTROL_DIM = 2;    // 控制维度 [steering, throttle]
static constexpr int OUTPUT_DIM = 6;     // 输出维度
static constexpr int PREDICTION_HORIZON = 20;  // 预测时域
static constexpr int CONTROL_HORIZON = 10;     // 控制时域
static constexpr int INEQ_CONSTRAINTS = 40;    // 不等式约束数量
static constexpr int EQ_CONSTRAINTS = 0;       // 等式约束数量

#ifdef WITH_LIBMPC
// libmpc控制器类型定义
using MPCController = mpc::NLMPC<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, 
                               PREDICTION_HORIZON, CONTROL_HORIZON, 
                               INEQ_CONSTRAINTS, EQ_CONSTRAINTS>;

// 数据类型别名
using StateVector = mpc::cvec<STATE_DIM>;
using ControlVector = mpc::cvec<CONTROL_DIM>;
using OutputVector = mpc::cvec<OUTPUT_DIM>;
#else
// 备用类型定义（当没有libmpc++时）
using StateVector = Eigen::VectorXd;
using ControlVector = Eigen::VectorXd;
using OutputVector = Eigen::VectorXd;
#endif

// NMPC优化状态
struct NMPCState {
    std::vector<StateVector> states;
    std::vector<ControlVector> controls;
    std::vector<OutputVector> outputs;
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
    
    // MPC参数
    double samplingTime_;
    double wheelbase_;
    double maxSteeringAngle_;
    double maxVelocity_;
    double maxAcceleration_;
    
#ifdef WITH_LIBMPC
    // libmpc++ MPC相关成员
    std::unique_ptr<MPCController> mpcController_;
    
    // 权重矩阵
    mpc::mat<STATE_DIM, STATE_DIM> Q_;     // 状态权重矩阵
    mpc::mat<CONTROL_DIM, CONTROL_DIM> R_; // 控制权重矩阵
    mpc::mat<STATE_DIM, STATE_DIM> Qf_;    // 终端权重矩阵
#endif
    
    // 机器学习模型（简化版本）
    void* mlModel_;        // 简化为void*指针
    void* fallbackModel_;  // 简化为void*指针
    
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
    
    // libmpc++ 特定方法
    StateVector convertToMPCState(const PerceptionData& perception);
    ControlCommands convertToVCUControl(const ControlVector& mpcControl);
    
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
    
    // 实时优化 (libmpc++)
    StateVector convertToNMPCState(const PerceptionData& perception) const;
    PredictionResult convertFromNMPCState(const std::vector<StateVector>& solution) const;
    
    // 性能评估
    double calculatePredictionError(const PredictionResult& prediction, 
                                  const PerceptionData& actual) const;
    double calculateComputationalCost(PredictionStrategy strategy) const;
    double calculateRobustnessMetric(PredictionStrategy strategy) const;
};

// 注意：PredictionPerformance已在vcu_core_types.hpp中定义，这里不重复定义

} // namespace VCUCore