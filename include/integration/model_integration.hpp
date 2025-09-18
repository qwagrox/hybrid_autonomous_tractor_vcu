#ifndef MODEL_INTEGRATION_HPP
#define MODEL_INTEGRATION_HPP

#include "../models/engine_model.hpp"
#include "../models/motor_model.hpp"
#include "../prediction/predictive_analytics.hpp"
#include "../core/vcu_core_types.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <pthread.h>

namespace VCUCore {

class ModelIntegration {
public:
    using CommandCallback = std::function<void(const ControlCommands&)>;
    
    ModelIntegration();
    ~ModelIntegration();
    
    bool initialize(const std::string& configFile);
    bool start();
    bool stop();
    
    ControlCommands getLastControlCommands() const;
    IntegrationPerformance getPerformanceStats() const;
    
    void setCommandCallback(CommandCallback callback);
    bool saveDataToFile(const std::string& filename) const;
    
private:
    // 配置结构
    struct IntegrationConfig {
        int updateRate;
        float predictionHorizon;
        float controlHorizon;
        float maxTorqueError;
        float maxSpeedError;
        float thermalSafetyMargin;
        float batterySafetyMargin;
        float emergencyShutdownTemp;
        float minBatterySOC;
        std::string engineModelConfig;
        std::string motorModelConfig;
        std::string predictionConfig;
    };
    
    IntegrationConfig config_;
    bool isInitialized_;
    bool isRunning_;
    int updateRate_;
    uint64_t lastUpdateTime_;
    
    std::unique_ptr<EngineModel> engineModel_;
    std::unique_ptr<MotorModel> motorModel_;
    std::unique_ptr<PredictiveAnalytics> predictiveAnalytics_;
    
    // 数据缓冲区
    std::vector<SensorData> sensorBuffer_;
    std::vector<PerceptionData> perceptionBuffer_;
    std::vector<PredictionResult> predictionBuffer_;
    std::vector<ControlCommands> controlBuffer_;
    
    ControlCommands lastControlCommands_;
    IntegrationPerformance performanceStats_;
    
    // 线程和同步
    std::thread updateThread_;
    std::thread monitorThread_;
    mutable pthread_mutex_t dataMutex_;
    pthread_cond_t dataCondition_;
    
    CommandCallback commandCallback_;
    BatteryState batteryState_;
    
    void initializeDefaultConfig();
    bool loadConfiguration(const std::string& configFile);
    void initializeDataBuffers();
    
    void updateLoop();
    void monitorLoop();
    bool updateModels(float deltaTime);
    
    PerceptionData createPerceptionData(const SensorData& sensor,
                                      const BatteryData& battery,
                                      const TerrainData& terrain) const;
    
    ControlStrategy calculateOptimalStrategy(const PerceptionData& perception,
                                           const PredictionResult& prediction);
    
    ControlCommands generateControlCommands(const ControlStrategy& strategy,
                                          const PerceptionData& perception,
                                          const PredictionResult& prediction);
    
    bool validateControlCommands(const ControlCommands& commands) const;
    void activateFallbackMode();
    
    void updatePerformanceStats(float updateTime);
    void checkSystemHealth();
    void checkModelConvergence();
    void checkResourceUsage();
    
    float estimateVehicleMass(const SensorData& sensor) const;
    float calculateEnergyEfficiency(const SensorData& sensor, 
                                  const BatteryData& battery) const;
    
    OperatingMode determineOperatingMode(const EnergyRequirements& energyReq,
                                       const EngineState& engineState,
                                       const MotorState& motorState) const;
    
    TorqueSplit calculateTorqueSplit(OperatingMode mode,
                                   const EnergyRequirements& energyReq,
                                   const EngineState& engineState,
                                   const MotorState& motorState) const;
    
    EnergyRequirements calculateEnergyRequirements(const PerceptionData& perception,
                                                 const PredictionResult& prediction) const;
    
    void applySafetyLimits(ControlStrategy& strategy,
                         const EngineState& engineState,
                         const MotorState& motorState) const;
    
    void saveToBuffer(const SensorData& sensor, const PerceptionData& perception,
                    const PredictionResult& prediction, const ControlCommands& commands);
    
    void publishControlCommands(const ControlCommands& commands);
    
    // 辅助函数
    SensorData getLatestSensorData() const { return SensorData{}; } // 实现从传感器获取
    BatteryData getLatestBatteryData() const { return BatteryData{}; } // 实现从BMS获取
    TerrainData getLatestTerrainData() const { return TerrainData{}; } // 实现从地形传感器获取
    
    float calculateOptimalEngineSpeed(const ControlStrategy& strategy,
                                    const PerceptionData& perception) const {
        return 1500.0f; // 简化实现
    }
    
    float calculateOptimalMotorSpeed(const ControlStrategy& strategy,
                                  const PerceptionData& perception) const {
        return 2000.0f; // 简化实现
    }
    
    float calculateFuelInjection(float torque, float rpm) const {
        return torque * rpm / 9549.3f * 0.08f; // 简化燃油计算
    }
    
    MotorMode determineMotorMode(OperatingMode opMode) const {
        switch (opMode) {
            case OperatingMode::ELECTRIC_ONLY: return MotorMode::MOTORING;
            case OperatingMode::HYBRID_ECONOMY: return MotorMode::MOTORING;
            case OperatingMode::HYBRID_POWER: return MotorMode::MOTORING;
            case OperatingMode::POWER_BOOST: return MotorMode::MOTORING;
            default: return MotorMode::MOTORING;
        }
    }
    
    float calculateOptimalGearRatio(const ControlStrategy& strategy,
                                 const PerceptionData& perception) const {
        return 1.0f; // 简化实现
    }
    
    CVTMode determineCVTMode(TransmissionStrategy ts) const {
        return CVTMode::ECONOMY; // 简化实现
    }
    
    LoadTrend estimateLoadTrend() const {
        return LoadTrend::STEADY; // 简化实现
    }
    
    LoadChangeType estimateLoadChangeType() const {
        return LoadChangeType::GRADUAL; // 简化实现
    }
};

} // namespace VCUCore

#endif // MODEL_INTEGRATION_HPP