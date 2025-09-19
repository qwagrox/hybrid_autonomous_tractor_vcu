// include/diagnostic/adaptive_learner.hpp
#pragma once
#include "vcu_core_types.hpp"
#include "models/engine_model.hpp"
#include "models/motor_model.hpp"
#include "control/torque_arbiter.hpp"
#include "control/cvt_controller.hpp"
#include <memory>
#include <map>
#include <deque>

namespace VCUCore {

class AdaptiveLearner {
private:
    struct LearningConfig {
        float learningRate;
        float discountFactor;
        float explorationRate;
        uint32_t batchSize;
        uint32_t memorySize;
        uint32_t updateInterval;
        bool enableOnlineLearning;
        bool enableTransferLearning;
    };
    
    LearningConfig config_;
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    std::unique_ptr<TorqueArbiter> torqueArbiter_;
    std::unique_ptr<CVTController> cvtController_;
    
    std::deque<LearningExperience> experienceBuffer_;
    std::map<std::string, LearningModel> learningModels_;
    
    uint32_t learningIterations_;
    float averageReward_;
    float explorationDecay_;
    
    // 性能跟踪
    struct LearningPerformance {
        float averageReward;
        float explorationRate;
        uint32_t experiencesProcessed;
        uint32_t modelUpdates;
        float learningProgress;
    };
    
    LearningPerformance performance_;

public:
    AdaptiveLearner(uint32_t memorySize = 10000);
    
    bool initialize(const std::string& modelPath = "models/learning/");
    void updateModels(const std::vector<SensorData>& sensorData,
                     const std::vector<TractorVehicleState>& states,
                     const std::vector<ControlCommands>& commands);
    
    // 强化学习
    float calculateReward(const TractorVehicleState& previousState,
                         const TractorVehicleState& currentState,
                         const ControlCommands& commands) const;
    void updateQValues(const LearningExperience& experience);
    void improveControlPolicy();
    
    // 模型适配
    void adaptToEnvironment(const EnvironmentData& environment);
    void adaptToOperator(const OperatorBehavior& behavior);
    void adaptToTask(const TaskRequirements& task);
    
    // 学习管理
    bool saveLearningState(const std::string& path) const;
    bool loadLearningState(const std::string& path);
    void resetLearning();
    
    // 性能监控
    LearningPerformance getPerformance() const;
    float getLearningProgress() const;
    bool isConverged() const;

private:
    void initializeLearningModels();
    void processExperienceBatch();
    LearningExperience createExperience(const SensorData& sensor,
                                      const TractorVehicleState& state,
                                      const ControlCommands& commands) const;
    
    float predictEfficiencyImprovement(const LearningExperience& experience) const;
    float predictFuelSavings(const LearningExperience& experience) const;
    float predictPerformanceGain(const LearningExperience& experience) const;
    
    void updateEfficiencyModel();
    void updateTorqueArbiterModel();
    void updateCVTControllerModel();
    
    void decayExplorationRate();
    void updateLearningRate();
    
    bool shouldUpdateModel() const;
    void performModelUpdate();
    
    void logLearningProgress(const LearningExperience& experience);
    void adjustLearningParameters();
};

} // namespace VCUCore