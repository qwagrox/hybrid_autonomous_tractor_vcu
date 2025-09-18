// src/prediction/predictive_analytics.cpp
#include "predictive_analytics.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "models/vehicle_dynamics_model.hpp"
#include "models/terrain_model.hpp"
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/kernels/register.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <thread>
#include <future>

namespace VCUCore {

// 常量定义
const int NMPC_NX = 8;  // 状态变量数量：x, y, heading, velocity, angular_vel, rolling_res, air_res, mass
const int NMPC_NU = 3;  // 控制变量数量：torque, steering, braking
const int NMPC_NP = 5;  // 参数数量：cr, cd, mass, slope, soil_resistance

PredictiveAnalytics::PredictiveAnalytics(float horizon, float timeStep, uint32_t historySize)
    : predictionHorizon_(horizon), timeStep_(timeStep), maxHistorySize_(historySize),
      currentStrategy_(PredictionStrategy::MACHINE_LEARNING),
      strategySwitchRequested_(false),
      predictionHorizonSteps_(static_cast<int>(horizon / timeStep)),
      controlInterval_(timeStep) {
    
    // 初始化策略配置
    initializeStrategyConfigs();
    
    // 初始化所有预测模型
    initializeMachineLearningModels();
    initializePhysicalModels();
    
    // 设置NMPC问题
    if (!setupNMPCProblem()) {
        std::cerr << "NMPC problem setup failed, using fallback mode" << std::endl;
        currentStrategy_ = PredictionStrategy::FALLBACK;
    }
    
    // 初始化NMPC求解器
    if (!initializeNMPCSolver()) {
        std::cerr << "NMPC solver initialization failed, using fallback mode" << std::endl;
        currentStrategy_ = PredictionStrategy::FALLBACK;
    }
    
    // 评估最佳策略
    currentStrategy_ = evaluateBestStrategy();
    std::cout << "Initialized with prediction strategy: " << static_cast<int>(currentStrategy_) << std::endl;
}

PredictiveAnalytics::~PredictiveAnalytics() {
    // 清理资源
    if (nmpcSolver_) {
        nmpcSolver_->shutdown();
    }
}

void PredictiveAnalytics::initializeStrategyConfigs() {
    strategyConfigs_ = {
        {PredictionStrategy::MACHINE_LEARNING, {
            .strategy = PredictionStrategy::MACHINE_LEARNING,
            .activationThreshold = 0.7,
            .deactivationThreshold = 0.4,
            .minSamplesRequired = 1000,
            .requiresCalibration = true
        }},
        {PredictionStrategy::NMPC_OPTIMIZATION, {
            .strategy = PredictionStrategy::NMPC_OPTIMIZATION,
            .activationThreshold = 0.8,
            .deactivationThreshold = 0.6,
            .minSamplesRequired = 100,
            .requiresCalibration = false
        }},
        {PredictionStrategy::HYBRID, {
            .strategy = PredictionStrategy::HYBRID,
            .activationThreshold = 0.75,
            .deactivationThreshold = 0.5,
            .minSamplesRequired = 500,
            .requiresCalibration = true
        }},
        {PredictionStrategy::FALLBACK, {
            .strategy = PredictionStrategy::FALLBACK,
            .activationThreshold = 0.3,
            .deactivationThreshold = 0.9,
            .minSamplesRequired = 0,
            .requiresCalibration = false
        }}
    };
}

void PredictiveAnalytics::initializeMachineLearningModels() {
    try {
        // 加载TensorFlow Lite模型
        loadForecastModel_ = tflite::FlatBufferModel::BuildFromFile("models/load_forecast_model.tflite");
        if (!loadForecastModel_) {
            throw std::runtime_error("Failed to load load forecast model");
        }
        
        // 创建解释器
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*loadForecastModel_, resolver);
        builder(&loadForecastInterpreter_);
        
        if (!loadForecastInterpreter_) {
            throw std::runtime_error("Failed to create interpreter");
        }
        
        // 分配张量
        if (loadForecastInterpreter_->AllocateTensors() != kTfLiteOk) {
            throw std::runtime_error("Failed to allocate tensors");
        }
        
        std::cout << "Machine learning models initialized successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ML model initialization failed: " << e.what() << std::endl;
        activateDegradedPredictionMode();
    }
}

void PredictiveAnalytics::initializePhysicalModels() {
    // 初始化物理模型
    vehicleDynamicsModel_ = std::make_unique<VehicleDynamicsModel>();
    engineModel_ = std::make_unique<EngineModel>();
    motorModel_ = std::make_unique<MotorModel>();
    terrainModel_ = std::make_unique<TerrainModel>();
    transmissionModel_ = std::make_unique<TransmissionModel>();
    
    // 加载模型参数
    if (!engineModel_->loadEngineParameters("config/engine_parameters.csv")) {
        std::cerr << "Failed to load engine parameters" << std::endl;
    }
    
    if (!motorModel_->loadMotorParameters("config/motor_parameters.csv")) {
        std::cerr << "Failed to load motor parameters" << std::endl;
    }
    
    std::cout << "Physical models initialized successfully" << std::endl;
}

bool PredictiveAnalytics::setupNMPCProblem() {
    try {
        // ==================== 1. 定义优化变量 ====================
        ACADO::DifferentialState x(NMPC_NX);    // 状态变量
        ACADO::Control u(NMPC_NU);              // 控制变量
        ACADO::Parameter p(NMPC_NP);            // 参数
        
        // ==================== 2. 定义系统动力学 ====================
        // 车辆动力学方程
        systemDynamics_ << dot(x[0]) == x[3] * cos(x[2]);  // dx/dt = v * cos(psi)
        systemDynamics_ << dot(x[1]) == x[3] * sin(x[2]);  // dy/dt = v * sin(psi)
        systemDynamics_ << dot(x[2]) == x[4];              // d(psi)/dt = omega
        systemDynamics_ << dot(x[3]) == (u[0] - x[5] - x[6]) / x[7];  // dv/dt = (F_traction - F_roll - F_drag) / m
        systemDynamics_ << dot(x[4]) == u[1];              // d(omega)/dt = steering_input
        systemDynamics_ << dot(x[5]) == 0;                 // 滚动阻力常数
        systemDynamics_ << dot(x[6]) == 0;                 // 空气阻力常数
        systemDynamics_ << dot(x[7]) == 0;                 // 质量常数
        
        // ==================== 3. 定义代价函数 ====================
        ACADO::Function h;
        h << x[0];                                  // 位置x跟踪
        h << x[1];                                  // 位置y跟踪  
        h << x[3];                                  // 速度跟踪
        h << x[2];                                  // 航向角跟踪
        h << u[0];                                  // 控制努力（扭矩）
        h << u[1];                                  // 控制努力（转向）
        h << u[2];                                  // 控制努力（制动）
        
        // 权重矩阵
        ACADO::Matrix Q = ACADO::Matrix::eye(h.getDim());
        Q(0,0) = 10.0;  // 位置x权重
        Q(1,1) = 10.0;  // 位置y权重
        Q(2,2) = 5.0;   // 速度权重
        Q(3,3) = 2.0;   // 航向权重
        Q(4,4) = 0.1;   // 扭矩权重
        Q(5,5) = 0.1;   // 转向权重
        Q(6,6) = 0.1;   // 制动权重
        
        // ==================== 4. 设置优化问题 ====================
        nmpcProblem_ = ACADO::OCP(0.0, predictionHorizon_, predictionHorizonSteps_);
        nmpcProblem_.subjectTo(systemDynamics_);
        
        // 代价函数
        nmpcProblem_.minimizeLSQ(Q, h);
        
        // ==================== 5. 设置约束 ====================
        // 控制输入约束
        nmpcProblem_.subjectTo(-600.0 <= u[0] <= 600.0);  // 扭矩约束 (Nm)
        nmpcProblem_.subjectTo(-0.5 <= u[1] <= 0.5);      // 转向角约束 (rad)
        nmpcProblem_.subjectTo(0.0 <= u[2] <= 1.0);       // 制动约束 (0-1)
        
        // 状态约束
        nmpcProblem_.subjectTo(0.0 <= x[3] <= 40.0);      // 速度约束 (m/s)
        nmpcProblem_.subjectTo(-M_PI <= x[2] <= M_PI);    // 航向角约束 (rad)
        nmpcProblem_.subjectTo(0.05 <= x[5] <= 0.15);     // 滚动阻力系数
        nmpcProblem_.subjectTo(0.3 <= x[6] <= 0.8);       // 空气阻力系数
        nmpcProblem_.subjectTo(7000.0 <= x[7] <= 9000.0); // 质量约束 (kg)
        
        // 参数约束
        nmpcProblem_.subjectTo(0.05 <= p[0] <= 0.15);     // 滚动阻力系数
        nmpcProblem_.subjectTo(0.3 <= p[1] <= 0.8);       // 空气阻力系数
        nmpcProblem_.subjectTo(7000.0 <= p[2] <= 9000.0); // 质量 (kg)
        nmpcProblem_.subjectTo(-0.3 <= p[3] <= 0.3);      // 坡度 (rad)
        nmpcProblem_.subjectTo(1000.0 <= p[4] <= 10000.0);// 土壤阻力 (N)
        
        std::cout << "NMPC problem setup completed successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "NMPC problem setup failed: " << e.what() << std::endl;
        return false;
    }
}

bool PredictiveAnalytics::initializeNMPCSolver() {
    try {
        // ==================== 1. 创建实时算法 ====================
        nmpcSolver_ = std::make_unique<ACADO::RealTimeAlgorithm>(nmpcProblem_, controlInterval_);
        
        // ==================== 2. 配置求解器参数 ====================
        nmpcSolver_->set(ACADO::MAX_NUM_ITERATIONS, 15);
        nmpcSolver_->set(ACADO::PRINTLEVEL, ACADO::NONE);
        nmpcSolver_->set(ACADO::PRINT_COPYRIGHT, ACADO::BT_FALSE);
        nmpcSolver_->set(ACADO::USE_REALTIME_ITERATIONS, ACADO::BT_TRUE);
        nmpcSolver_->set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
        
        // ==================== 3. 配置积分器 ====================
        ACADO::IntegrationAlgorithm integrator;
        integrator = ACADO::IntegrationAlgorithm(systemDynamics_);
        integrator.set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
        integrator.set(ACADO::ABS_TOLERANCE, 1e-6);
        integrator.set(ACADO::REL_TOLERANCE, 1e-6);
        integrator.set(ACADO::MAX_NUM_INTEGRATOR_STEPS, 10000);
        
        nmpcSolver_->setIntegrationAlgorithm(integrator);
        
        // ==================== 4. 初始化控制器 ====================
        nmpcController_ = std::make_unique<ACADO::Controller>(*nmpcSolver_);
        
        // ==================== 5. 初始化参考轨迹 ====================
        referenceTrajectory_ = std::make_unique<ACADO::StaticReferenceTrajectory>();
        
        std::cout << "NMPC solver initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "NMPC solver initialization failed: " << e.what() << std::endl;
        return false;
    }
}

PredictionResult PredictiveAnalytics::analyzeFuture(const PerceptionData& currentPerception,
                                                  const PredictionResult& initialPrediction) {
    PredictionResult result;
    
    // 根据当前策略选择预测方法
    switch (currentStrategy_) {
        case PredictionStrategy::MACHINE_LEARNING:
            result = analyzeFutureWithML(currentPerception, initialPrediction);
            break;
            
        case PredictionStrategy::NMPC_OPTIMIZATION:
            result = analyzeFutureWithNMPC(currentPerception, initialPrediction);
            break;
            
        case PredictionStrategy::HYBRID:
            result = executeHybridPrediction(currentPerception, initialPrediction);
            break;
            
        case PredictionStrategy::FALLBACK:
            result = analyzeFutureWithFallback(currentPerception, initialPrediction);
            break;
            
        default:
            result = analyzeFutureWithFallback(currentPerception, initialPrediction);
            break;
    }
    
    // 更新策略性能
    updateStrategyPerformance(result, currentPerception);
    
    // 检查是否需要策略切换
    if (shouldSwitchStrategy()) {
        PredictionStrategy newStrategy = evaluateBestStrategy();
        if (newStrategy != currentStrategy_) {
            switchPredictionStrategy(newStrategy);
        }
    }
    
    return result;
}

PredictionResult PredictiveAnalytics::analyzeFutureWithML(const PerceptionData& perception,
                                                        const PredictionResult& initialPrediction) {
    PredictionResult result;
    
    try {
        // 准备输入特征
        Eigen::VectorXf features = extractMLFeatures(perception);
        
        // 设置输入张量
        float* input = loadForecastInterpreter_->typed_input_tensor<float>(0);
        for (int i = 0; i < features.size(); ++i) {
            input[i] = features[i];
        }
        
        // 执行推理
        if (loadForecastInterpreter_->Invoke() != kTfLiteOk) {
            throw std::runtime_error("Failed to invoke TensorFlow Lite interpreter");
        }
        
        // 获取输出
        float* output = loadForecastInterpreter_->typed_output_tensor<float>(0);
        
        // 解析预测结果
        result = parseMLOutput(output, perception);
        result.predictionConfidence = calculateMLConfidence(output);
        
    } catch (const std::exception& e) {
        std::cerr << "ML prediction failed: " << e.what() << std::endl;
        result = analyzeFutureWithFallback(perception, initialPrediction);
        result.predictionConfidence *= 0.5; // 降低置信度
    }
    
    return result;
}

PredictionResult PredictiveAnalytics::analyzeFutureWithNMPC(const PerceptionData& perception,
                                                          const PredictionResult& initialPrediction) {
    PredictionResult result;
    
    try {
        // 执行NMPC在线优化
        NMPCState nmpcState = solveNMPCOnline(perception, initialPrediction);
        
        if (nmpcState.converged) {
            result = convertFromNMPCState(nmpcState.states);
            result.predictionConfidence = 0.9f; // NMPC高置信度
            
            // 记录优化性能
            strategyPerformanceHistory_.push_back({
                .strategy = PredictionStrategy::NMPC_OPTIMIZATION,
                .averageError = calculatePredictionError(result, perception),
                .maxError = 0.0f, // 需要实际计算
                .computationalTime = nmpcState.optimizationTime,
                .robustnessScore = 0.8f,
                .sampleCount = 1,
                .lastUpdate = std::chrono::duration_cast<Timestamp>(
                    std::chrono::system_clock::now().time_since_epoch())
            });
        } else {
            throw std::runtime_error("NMPC optimization did not converge");
        }
        
    } catch (const std::exception& e) {
        std::cerr << "NMPC prediction failed: " << e.what() << std::endl;
        result = analyzeFutureWithFallback(perception, initialPrediction);
        result.predictionConfidence *= 0.6f; // 降低置信度
    }
    
    return result;
}

PredictionResult PredictiveAnalytics::executeHybridPredection(const PerceptionData& perception,
                                                            const PredictionResult& initialPrediction) {
    PredictionResult hybridResult;
    
    try {
        // 并行执行ML和NMPC预测
        auto mlFuture = std::async(std::launch::async, [&]() {
            return analyzeFutureWithML(perception, initialPrediction);
        });
        
        auto nmpcFuture = std::async(std::launch::async, [&]() {
            return analyzeFutureWithNMPC(perception, initialPrediction);
        });
        
        // 等待结果
        PredictionResult mlResult = mlFuture.get();
        PredictionResult nmpcResult = nmpcFuture.get();
        
        // 融合预测结果
        hybridResult = fusePredictions(mlResult, nmpcResult);
        hybridResult.predictionConfidence = (mlResult.predictionConfidence + 
                                           nmpcResult.predictionConfidence) / 2.0f;
        
    } catch (const std::exception& e) {
        std::cerr << "Hybrid prediction failed: " << e.what() << std::endl;
        hybridResult = analyzeFutureWithFallback(perception, initialPrediction);
    }
    
    return hybridResult;
}

PredictionResult PredictiveAnalytics::analyzeFutureWithFallback(const PerceptionData& perception,
                                                              const PredictionResult& initialPrediction) {
    PredictionResult result;
    
    // 使用基于物理的简单预测模型
    result.loadForecast = predictLoadWithPhysics(perception, predictionHorizon_);
    result.energyDemand = predictEnergyWithPhysics(perception, result.loadForecast);
    result.terrainProfile = predictTerrainWithPhysics(perception, predictionHorizon_);
    result.predictedEfficiency = calculatePhysicsBasedEfficiency(perception);
    result.predictionConfidence = 0.5f; // 降级模式中等置信度
    
    return result;
}

NMPCState PredictiveAnalytics::solveNMPCOnline(const PerceptionData& currentState,
                                             const PredictionResult& reference) {
    NMPCState result;
    ACADO::VariablesGrid x0(NMPC_NX, ACADO::Vector(1, 0.0));
    
    try {
        // 1. 转换初始状态
        ACADO::DVector initialState = convertToNMPCState(currentState);
        x0.setVector(0, initialState);
        
        // 2. 更新参考轨迹
        updateNMPCReferences(currentState, reference);
        
        // 3. 设置初始状态
        nmpcController_->init(0.0, x0);
        
        // 4. 执行实时迭代
        ACADO::VariablesGrid uOpt;
        ACADO::VariablesGrid xOpt;
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        ACADO::returnValue status = nmpcController_->step(x0, uOpt);
        
        auto endTime = std::chrono::high_resolution_clock::now();
        result.optimizationTime = std::chrono::duration<double>(endTime - startTime).count();
        
        // 5. 处理优化结果
        if (status == ACADO::SUCCESSFUL_RETURN) {
            result.converged = true;
            result.states = xOpt;
            result.controls = uOpt;
            result.costValue = nmpcController_->getObjectiveValue();
            result.iterations = nmpcController_->getNumberOfSteps();
        } else {
            result.converged = false;
            std::cerr << "NMPC optimization failed with status: " << status << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "NMPC online solving failed: " << e.what() << std::endl;
        result.converged = false;
    }
    
    return result;
}

ACADO::DVector PredictiveAnalytics::convertToNMPCState(const PerceptionData& perception) const {
    ACADO::DVector state(NMPC_NX);
    
    // 状态变量映射
    state(0) = perception.vehicleState.position.x();  // x位置
    state(1) = perception.vehicleState.position.y();  // y位置
    state(2) = perception.vehicleState.heading;       // 航向角
    state(3) = perception.vehicleState.velocity.norm(); // 速度
    state(4) = perception.vehicleState.acceleration.norm(); // 角速度
    state(5) = perception.rollingResistance;          // 滚动阻力
    state(6) = perception.aerodynamicDrag;            // 空气阻力
    state(7) = perception.vehicleState.estimatedMass; // 质量
    
    return state;
}

PredictionResult PredictiveAnalytics::convertFromNMPCState(const ACADO::VariablesGrid& solution) const {
    PredictionResult result;
    
    if (solution.getNumPoints() > 0) {
        // 提取预测轨迹
        for (int i = 0; i < solution.getNumPoints(); ++i) {
            ACADO::DVector state = solution.getVector(i);
            
            // 位置预测
            Vector3d position(state(0), state(1), 0.0);
            result.pathProfile.push_back(position);
            
            // 速度预测
            result.energyDemand.push_back(state(3));
            
            // 坡度预测
            result.slopeProfile.push_back(state(5));
        }
        
        // 计算预测效率
        result.predictedEfficiency = calculateNMPCEfficiency(solution);
    }
    
    return result;
}

bool PredictiveAnalytics::switchPredictionStrategy(PredictionStrategy newStrategy) {
    // 检查策略切换是否有效
    if (!validateStrategySwitch(newStrategy)) {
        std::cerr << "Strategy switch validation failed" << std::endl;
        return false;
    }
    
    // 保存当前策略状态
    saveStrategyState(currentStrategy_);
    
    // 执行策略切换
    performStrategySwitch(newStrategy);
    
    // 更新当前策略
    currentStrategy_ = newStrategy;
    strategySwitchRequested_ = false;
    
    std::cout << "Switched to prediction strategy: " << static_cast<int>(newStrategy) << std::endl;
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
            nmpcSolver_->resume();
            // 停用ML模型
            mlModel_->setActive(false);
            break;
            
        case PredictionStrategy::HYBRID:
            // 保持两者都激活
            if (!nmpcSolver_->isInitialized()) {
                initializeNMPCSolver();
            }
            nmpcSolver_->resume();
            mlModel_->setActive(true);
            break;
            
        case PredictionStrategy::FALLBACK:
            // 使用降级模式
            if (nmpcSolver_) {
                nmpcSolver_->shutdown();
            }
            mlModel_->setActive(false);
            // 恢复策略状态
            restoreStrategyState(newStrategy);
            break;
    }
}

PredictionStrategy PredictiveAnalytics::evaluateBestStrategy() const {
    // 基于性能历史评估最佳策略
    if (strategyPerformanceHistory_.empty()) {
        return PredictionStrategy::HYBRID; // 默认混合模式
    }
    
    // 计算各策略的平均性能
    std::map<PredictionStrategy, PredictionPerformance> strategyStats;
    for (const auto& perf : strategyPerformanceHistory_) {
        strategyStats[perf.strategy].averageError += perf.averageError;
        strategyStats[perf.strategy].computationalTime += perf.computationalTime;
        strategyStats[perf.strategy].sampleCount++;
    }
    
    // 归一化性能指标
    float bestScore = -std::numeric_limits<float>::max();
    PredictionStrategy bestStrategy = currentStrategy_;
    
    for (auto& [strategy, stats] : strategyStats) {
        if (stats.sampleCount > 0) {
            stats.averageError /= stats.sampleCount;
            stats.computationalTime /= stats.sampleCount;
            
            // 综合评分：误差权重70%，计算时间权重30%
            float errorScore = 1.0f / (1.0f + stats.averageError);
            float timeScore = 1.0f / (1.0f + stats.computationalTime);
            float totalScore = 0.7f * errorScore + 0.3f * timeScore;
            
            if (totalScore > bestScore) {
                bestScore = totalScore;
                bestStrategy = strategy;
            }
        }
    }
    
    return bestStrategy;
}

bool PredictiveAnalytics::shouldSwitchStrategy() const {
    // 检查策略切换条件
    if (strategyPerformanceHistory_.size() < 50) {
        return false; // 样本不足
    }
    
    // 计算当前策略的性能
    PredictionPerformance currentPerf = evaluateStrategyPerformance(currentStrategy_);
    
    // 检查性能是否低于阈值
    const auto& config = strategyConfigs_.at(currentStrategy_);
    if (currentPerf.averageError > config.deactivationThreshold) {
        return true;
    }
    
    // 检查是否有更好的策略可用
    PredictionStrategy bestCandidate = evaluateBestStrategy();
    if (bestCandidate != currentStrategy_) {
        PredictionPerformance candidatePerf = evaluateStrategyPerformance(bestCandidate);
        if (candidatePerf.averageError < currentPerf.averageError * 0.8f) {
            return true; // 候选策略性能好20%以上
        }
    }
    
    return false;
}

void PredictiveAnalytics::updateStrategyPerformance(const PredictionResult& prediction,
                                                  const PerceptionData& actual) {
    // 计算预测误差
    float error = calculatePredictionError(prediction, actual);
    float compTime = calculateComputationalCost(currentStrategy_);
    float robustness = calculateRobustnessMetric(currentStrategy_);
    
    // 更新性能历史
    PredictionPerformance perf{
        .strategy = currentStrategy_,
        .averageError = error,
        .maxError = error,
        .computationalTime = compTime,
        .robustnessScore = robustness,
        .sampleCount = 1,
        .lastUpdate = std::chrono::duration_cast<Timestamp>(
            std::chrono::system_clock::now().time_since_epoch())
    };
    
    strategyPerformanceHistory_.push_back(perf);
    
    // 保持历史大小
    if (strategyPerformanceHistory_.size() > maxHistorySize_) {
        strategyPerformanceHistory_.pop_front();
    }
}

float PredictiveAnalytics::calculatePredictionError(const PredictionResult& prediction,
                                                  const PerceptionData& actual) const {
    // 计算多维度预测误差
    float positionError = 0.0f;
    float velocityError = 0.0f;
    float loadError = 0.0f;
    
    if (!prediction.pathProfile.empty() && !actual.vehicleState.position.isZero()) {
        positionError = (prediction.pathProfile[0] - actual.vehicleState.position).norm();
    }
    
    if (!prediction.energyDemand.empty()) {
        velocityError = std::abs(prediction.energyDemand[0] - actual.vehicleState.velocity.norm());
    }
    
    if (!prediction.loadForecast.empty()) {
        loadError = std::abs(prediction.loadForecast[0] - actual.implementForce);
    }
    
    // 加权综合误差
    return 0.4f * positionError + 0.3f * velocityError + 0.3f * loadError;
}

float PredictiveAnalytics::calculateComputationalCost(PredictionStrategy strategy) const {
    // 基于策略类型返回典型计算时间（秒）
    switch (strategy) {
        case PredictionStrategy::MACHINE_LEARNING: return 0.02f;
        case PredictionStrategy::NMPC_OPTIMIZATION: return 0.15f;
        case PredictionStrategy::HYBRID: return 0.08f;
        case PredictionStrategy::FALLBACK: return 0.01f;
        default: return 0.05f;
    }
}

float PredictiveAnalytics::calculateRobustnessMetric(PredictionStrategy strategy) const {
    // 基于历史数据计算鲁棒性评分
    int successCount = 0;
    int totalCount = 0;
    
    for (const auto& perf : strategyPerformanceHistory_) {
        if (perf.strategy == strategy) {
            totalCount++;
            if (perf.averageError < 1.0f) { // 误差小于1.0视为成功
                successCount++;
            }
        }
    }
    
    return totalCount > 0 ? static_cast<float>(successCount) / totalCount : 0.5f;
}

void PredictiveAnalytics::activateFallbackStrategy() {
    std::cout << "Activating fallback prediction strategy" << std::endl;
    switchPredictionStrategy(PredictionStrategy::FALLBACK);
}

void PredictiveAnalytics::recalibrateModels() {
    std::cout << "Recalibrating prediction models" << std::endl;
    
    // 重新校准ML模型
    if (mlModel_) {
        mlModel_->recalibrate();
    }
    
    // 重新初始化NMPC求解器
    if (nmpcSolver_) {
        nmpcSolver_->shutdown();
        initializeNMPCSolver();
    }
    
    // 清空性能历史
    strategyPerformanceHistory_.clear();
}

Eigen::VectorXf PredictiveAnalytics::extractMLFeatures(const PerceptionData& perception) const {
    // 提取机器学习特征向量
    Eigen::VectorXf features(20);
    int index = 0;
    
    // 车辆状态特征
    features[index++] = perception.vehicleState.velocity.norm();
    features[index++] = perception.vehicleState.acceleration.norm();
    features[index++] = perception.vehicleState.heading;
    features[index++] = perception.vehicleState.estimatedMass;
    
    // 地形特征
    features[index++] = perception.terrainSlope;
    features[index++] = perception.soilResistance;
    features[index++] = perception.rollingResistance;
    features[index++] = perception.aerodynamicDrag;
    
    // 负载特征
    features[index++] = perception.loadFactor;
    features[index++] = static_cast<float>(perception.loadChangeType);
    features[index++] = static_cast<float>(perception.loadTrend);
    
    // 环境特征
    features[index++] = perception.vehicleState.fuelConsumption;
    features[index++] = perception.vehicleState.energyEfficiency;
    
    // 历史特征（简化）
    for (int i = 0; i < 8; ++i) {
        features[index++] = 0.0f; // 实际中需要填充历史数据
    }
    
    return features;
}

PredictionResult PredictiveAnalytics::parseMLOutput(const float* output, 
                                                  const PerceptionData& perception) const {
    PredictionResult result;
    
    // 解析ML模型输出（假设输出格式为：位置x,y,速度,负载,能量）
    result.pathProfile.push_back(Vector3d(output[0], output[1], 0.0));
    result.energyDemand.push_back(output[2]);
    result.loadForecast.push_back(output[3]);
    result.predictedEfficiency = output[4];
    
    // 设置预测时域
    result.predictionHorizon = predictionHorizon_;
    
    return result;
}

float PredictiveAnalytics::calculateMLConfidence(const float* output) const {
    // 基于输出方差计算置信度（简化实现）
    float variance = 0.0f;
    float mean = 0.0f;
    int count = 5; // 假设有5个输出值
    
    for (int i = 0; i < count; ++i) {
        mean += output[i];
    }
    mean /= count;
    
    for (int i = 0; i < count; ++i) {
        variance += (output[i] - mean) * (output[i] - mean);
    }
    variance /= count;
    
    // 方差越小，置信度越高
    return std::exp(-variance);
}

std::vector<float> PredictiveAnalytics::predictLoadWithPhysics(const PerceptionData& perception,
                                                             float horizon) const {
    std::vector<float> loadForecast;
    
    // 基于物理模型的负载预测
    float currentLoad = perception.implementForce;
    float trend = 0.0f;
    
    // 根据负载趋势调整预测
    switch (perception.loadTrend) {
        case LoadTrend::INCREASING:
            trend = 0.1f; // 每步增加10%
            break;
        case LoadTrend::DECREASING:
            trend = -0.05f; // 每步减少5%
            break;
        case LoadTrend::OSCILLATING:
            trend = 0.05f * std::sin(2.0f * M_PI * 0.1f); // 振荡
            break;
        default:
            trend = 0.0f;
            break;
    }
    
    // 生成预测序列
    int steps = static_cast<int>(horizon / timeStep_);
    for (int i = 0; i < steps; ++i) {
        float predictedLoad = currentLoad * (1.0f + trend * i);
        loadForecast.push_back(predictedLoad);
    }
    
    return loadForecast;
}

std::vector<float> PredictiveAnalytics::predictEnergyWithPhysics(const PerceptionData& perception,
                                                               const std::vector<float>& loadForecast) const {
    std::vector<float> energyForecast;
    
    // 基于负载预测的能量需求预测
    float baseEnergy = perception.vehicleState.powerConsumption;
    
    for (float load : loadForecast) {
        float energy = baseEnergy * (1.0f + 0.5f * (load / 10000.0f)); // 简化模型
        energyForecast.push_back(energy);
    }
    
    return energyForecast;
}

std::vector<Vector3d> PredictiveAnalytics::predictTerrainWithPhysics(const PerceptionData& perception,
                                                                   float horizon) const {
    std::vector<Vector3d> terrainProfile;
    
    // 基于当前地形的简单预测
    Vector3d currentPos = perception.vehicleState.position;
    Vector3d currentVel = perception.vehicleState.velocity;
    
    int steps = static_cast<int>(horizon / timeStep_);
    for (int i = 0; i < steps; ++i) {
        Vector3d predictedPos = currentPos + currentVel * timeStep_ * i;
        terrainProfile.push_back(predictedPos);
    }
    
    return terrainProfile;
}

float PredictiveAnalytics::calculatePhysicsBasedEfficiency(const PerceptionData& perception) const {
    // 基于物理模型的效率预测
    float speed = perception.vehicleState.velocity.norm();
    float load = perception.implementForce;
    float slope = perception.terrainSlope;
    
    // 简化效率模型
    float speedEfficiency = 1.0f - 0.3f * std::abs(speed - 15.0f) / 15.0f;
    float loadEfficiency = 1.0f - 0.4f * std::abs(load - 5000.0f) / 5000.0f;
    float slopeEfficiency = 1.0f - 0.2f * std::abs(slope);
    
    return 0.85f * speedEfficiency * loadEfficiency * slopeEfficiency;
}

float PredictiveAnalytics::calculateNMPCEfficiency(const ACADO::VariablesGrid& solution) const {
    // 基于NMPC解计算预测效率
    if (solution.getNumPoints() == 0) {
        return 0.0f;
    }
    
    // 计算平均控制效率（简化）
    float totalEfficiency = 0.0f;
    int count = 0;
    
    for (int i = 0; i < solution.getNumPoints(); ++i) {
        ACADO::DVector state = solution.getVector(i);
        float speed = state(3);
        float efficiency = 0.9f - 0.2f * std::abs(speed - 20.0f) / 20.0f;
        totalEfficiency += efficiency;
        count++;
    }
    
    return count > 0 ? totalEfficiency / count : 0.8f;
}

void PredictiveAnalytics::updateNMPCReferences(const PerceptionData& perception,
                                             const PredictionResult& prediction) {
    if (!referenceTrajectory_) {
        return;
    }
    
    try {
        // 创建参考轨迹
        ACADO::VariablesGrid reference(NMPC_NX, predictionHorizonSteps_ + 1);
        
        for (int i = 0; i <= predictionHorizonSteps_; ++i) {
            ACADO::DVector refPoint(NMPC_NX);
            
            // 位置参考
            if (i < prediction.pathProfile.size()) {
                refPoint(0) = prediction.pathProfile[i].x();
                refPoint(1) = prediction.pathProfile[i].y();
            } else {
                // 外推
                refPoint(0) = perception.vehicleState.position.x() + 
                             perception.vehicleState.velocity.x() * i * timeStep_;
                refPoint(1) = perception.vehicleState.position.y() + 
                             perception.vehicleState.velocity.y() * i * timeStep_;
            }
            
            // 速度参考
            if (i < prediction.energyDemand.size()) {
                refPoint(3) = prediction.energyDemand[i];
            } else {
                refPoint(3) = perception.vehicleState.velocity.norm();
            }
            
            // 其他状态参考（使用当前值）
            refPoint(2) = perception.vehicleState.heading;
            refPoint(4) = 0.0f; // 零角速度
            refPoint(5) = perception.rollingResistance;
            refPoint(6) = perception.aerodynamicDrag;
            refPoint(7) = perception.vehicleState.estimatedMass;
            
            reference.setVector(i, refPoint);
        }
        
        // 更新参考轨迹
        referenceTrajectory_->setReference(reference);
        nmpcController_->setReference(*referenceTrajectory_);
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to update NMPC references: " << e.what() << std::endl;
    }
}

bool PredictiveAnalytics::checkNMPCConvergence() const {
    if (!nmpcSolver_) {
        return false;
    }
    
    // 检查NMPC收敛状态
    int iterations = nmpcSolver_->getNumberOfSteps();
    double cost = nmpcSolver_->getObjectiveValue();
    
    return iterations > 0 && cost < 1000.0; // 简单的收敛检查
}

void PredictiveAnalytics::activateDegradedPredictionMode() {
    std::cerr << "Activating degraded prediction mode" << std::endl;
    currentStrategy_ = PredictionStrategy::FALLBACK;
    
    // 禁用复杂模型
    if (nmpcSolver_) {
        nmpcSolver_->shutdown();
    }
    
    // 使用简单物理模型
    std::cout << "Using fallback physical models for prediction" << std::endl;
}

} // namespace VCUCore