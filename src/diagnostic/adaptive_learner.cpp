// src/diagnostic/adaptive_learner.cpp
#include "adaptive_learner.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace VCUCore {

AdaptiveLearner::AdaptiveLearner(uint32_t memorySize) 
    : learningIterations_(0), averageReward_(0.0f), explorationDecay_(0.999f) {
    
    config_ = {
        .learningRate = 0.01f,
        .discountFactor = 0.9f,
        .explorationRate = 0.3f,
        .batchSize = 64,
        .memorySize = memorySize,
        .updateInterval = 1000,
        .enableOnlineLearning = true,
        .enableTransferLearning = true
    };
    
    performance_ = {
        .averageReward = 0.0f,
        .explorationRate = config_.explorationRate,
        .experiencesProcessed = 0,
        .modelUpdates = 0,
        .learningProgress = 0.0f
    };
}

bool AdaptiveLearner::initialize(const std::string& modelPath) {
    try {
        // 初始化模型
        engineModel_ = std::make_unique<EngineModel>();
        motorModel_ = std::make_unique<MotorModel>();
        torqueArbiter_ = std::make_unique<TorqueArbiter>();
        cvtController_ = std::make_unique<CVTController>();
        
        // 初始化学习模型
        initializeLearningModels();
        
        // 加载已有的学习状态
        if (!loadLearningState(modelPath + "learning_state.bin")) {
            std::cout << "No previous learning state found, starting fresh" << std::endl;
        }
        
        std::cout << "Adaptive learner initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Adaptive learner initialization failed: " << e.what() << std::endl;
        return false;
    }
}

void AdaptiveLearner::updateModels(const std::vector<SensorData>& sensorData,
                                 const std::vector<VehicleState>& states,
                                 const std::vector<ControlCommands>& commands) {
    
    if (sensorData.empty() || states.empty() || commands.empty()) {
        return;
    }
    
    try {
        // 创建学习经验
        for (size_t i = 0; i < sensorData.size(); ++i) {
            LearningExperience experience = createExperience(
                sensorData[i], states[i], commands[i]);
            
            // 添加到经验缓冲区
            experienceBuffer_.push_back(experience);
            if (experienceBuffer_.size() > config_.memorySize) {
                experienceBuffer_.pop_front();
            }
        }
        
        // 处理经验批次
        if (experienceBuffer_.size() >= config_.batchSize) {
            processExperienceBatch();
        }
        
        // 更新模型
        if (shouldUpdateModel()) {
            performModelUpdate();
        }
        
        // 更新学习参数
        decayExplorationRate();
        updateLearningRate();
        adjustLearningParameters();
        
    } catch (const std::exception& e) {
        std::cerr << "Model update failed: " << e.what() << std::endl;
    }
}

LearningExperience AdaptiveLearner::createExperience(const SensorData& sensor,
                                                   const VehicleState& state,
                                                   const ControlCommands& commands) const {
    
    LearningExperience exp;
    
    exp.timestamp = sensor.timestamp;
    exp.state = state;
    exp.commands = commands;
    exp.reward = calculateReward(VehicleState{}, state, commands); // 需要前一个状态
    
    // 提取特征
    exp.features.push_back(state.velocity.norm());
    exp.features.push_back(state.acceleration.norm());
    exp.features.push_back(state.drawbarPull);
    exp.features.push_back(state.energyEfficiency);
    exp.features.push_back(sensor.terrainSlope);
    exp.features.push_back(sensor.soilResistance);
    
    return exp;
}

float AdaptiveLearner::calculateReward(const VehicleState& previousState,
                                     const VehicleState& currentState,
                                     const ControlCommands& commands) const {
    
    float reward = 0.0f;
    
    // 效率奖励
    reward += currentState.energyEfficiency * 0.5f;
    
    // 燃油节省奖励
    if (previousState.fuelConsumption > 0) {
        float fuelSaved = previousState.fuelConsumption - currentState.fuelConsumption;
        reward += fuelSaved * 0.3f;
    }
    
    // 性能奖励
    reward += (currentState.drawbarPull / 10000.0f) * 0.2f;
    
    // 平滑性惩罚（减少剧烈变化）
    float torqueChange = std::abs(commands.engineTorqueRequest - previousState.actualTorque);
    reward -= torqueChange * 0.01f;
    
    return reward;
}

void AdaptiveLearner::processExperienceBatch() {
    // 随机采样批次
    std::vector<LearningExperience> batch;
    std::sample(experienceBuffer_.begin(), experienceBuffer_.end(),
               std::back_inserter(batch), config_.batchSize,
               std::mt19937{std::random_device{}()});
    
    float totalReward = 0.0f;
    
    for (const auto& exp : batch) {
        // 更新Q值
        updateQValues(exp);
        totalReward += exp.reward;
        
        // 记录学习进度
        logLearningProgress(exp);
    }
    
    // 更新平均奖励
    averageReward_ = 0.9f * averageReward_ + 0.1f * (totalReward / batch.size());
    performance_.averageReward = averageReward_;
    performance_.experiencesProcessed += batch.size();
}

void AdaptiveLearner::updateQValues(const LearningExperience& experience) {
    // 简化的Q学习更新
    // 实际实现会使用神经网络或更复杂的函数近似
    
    float predictedEfficiency = predictEfficiencyImprovement(experience);
    float predictedSavings = predictFuelSavings(experience);
    float predictedPerformance = predictPerformanceGain(experience);
    
    float totalGain = predictedEfficiency + predictedSavings + predictedPerformance;
    
    // 更新学习模型（简化实现）
    for (auto& [modelName, model] : learningModels_) {
        if (modelName == "efficiency_model") {
            model.update(experience.features, totalGain);
        }
    }
}

void AdaptiveLearner::improveControlPolicy() {
    // 改进扭矩分配策略
    if (torqueArbiter_) {
        // 基于学习经验更新仲裁策略
        float efficiencyImprovement = predictEfficiencyImprovement(LearningExperience{});
        torqueArbiter_->updateEfficiencyWeights(efficiencyImprovement);
    }
    
    // 改进CVT控制策略
    if (cvtController_) {
        float performanceGain = predictPerformanceGain(LearningExperience{});
        cvtController_->updateControlParams(performanceGain);
    }
}

void AdaptiveLearner::adaptToEnvironment(const EnvironmentData& environment) {
    // 根据环境调整学习参数
    if (environment.terrainType == TerrainType::HILLY) {
        config_.learningRate *= 1.2f; // 增加学习率
    } else if (environment.terrainType == TerrainType::FLAT) {
        config_.learningRate *= 0.8f; // 减少学习率
    }
    
    if (environment.weatherCondition == WeatherCondition::RAIN) {
        config_.explorationRate *= 1.1f; // 增加探索
    }
}

void AdaptiveLearner::initializeLearningModels() {
    // 初始化效率学习模型
    learningModels_["efficiency_model"] = LearningModel{
        .inputSize = 6,
        .outputSize = 1,
        .learningRate = config_.learningRate,
        .modelData = {}
    };
    
    // 初始化扭矩分配模型
    learningModels_["torque_model"] = LearningModel{
        .inputSize = 8,
        .outputSize = 2,
        .learningRate = config_.learningRate * 0.5f,
        .modelData = {}
    };
    
    // 初始化CVT控制模型
    learningModels_["cvt_model"] = LearningModel{
        .inputSize = 5,
        .outputSize = 1,
        .learningRate = config_.learningRate * 0.7f,
        .modelData = {}
    };
}

void AdaptiveLearner::decayExplorationRate() {
    config_.explorationRate *= explorationDecay_;
    config_.explorationRate = std::max(0.1f, config_.explorationRate);
    performance_.explorationRate = config_.explorationRate;
}

bool AdaptiveLearner::saveLearningState(const std::string& path) const {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        // 保存学习状态
        file.write(reinterpret_cast<const char*>(&learningIterations_), sizeof(learningIterations_));
        file.write(reinterpret_cast<const char*>(&averageReward_), sizeof(averageReward_));
        file.write(reinterpret_cast<const char*>(&config_.explorationRate), sizeof(config_.explorationRate));
        
        // 保存经验缓冲区大小
        uint32_t bufferSize = experienceBuffer_.size();
        file.write(reinterpret_cast<const char*>(&bufferSize), sizeof(bufferSize));
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving learning state: " << e.what() << std::endl;
        return false;
    }
}

bool AdaptiveLearner::loadLearningState(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    try {
        // 加载学习状态
        file.read(reinterpret_cast<char*>(&learningIterations_), sizeof(learningIterations_));
        file.read(reinterpret_cast<char*>(&averageReward_), sizeof(averageReward_));
        file.read(reinterpret_cast<char*>(&config_.explorationRate), sizeof(config_.explorationRate));
        
        // 加载经验缓冲区大小
        uint32_t bufferSize;
        file.read(reinterpret_cast<char*>(&bufferSize), sizeof(bufferSize));
        
        performance_.explorationRate = config_.explorationRate;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading learning state: " << e.what() << std::endl;
        return false;
    }
}

float AdaptiveLearner::getLearningProgress() const {
    // 基于经验数量和平均奖励计算学习进度
    float progress = std::min(1.0f, static_cast<float>(experienceBuffer_.size()) / config_.memorySize);
    progress = 0.7f * progress + 0.3f * (averageReward_ / 10.0f); // 假设最大奖励为10
    
    return std::clamp(progress, 0.0f, 1.0f);
}

bool AdaptiveLearner::isConverged() const {
    // 检查学习是否收敛
    return experienceBuffer_.size() >= config_.memorySize * 0.8f &&
           averageReward_ > 5.0f && // 达到一定的奖励阈值
           config_.explorationRate <= 0.15f;
}

} // namespace VCUCore