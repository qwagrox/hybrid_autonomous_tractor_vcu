// include/control/torque_arbiter.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <memory>
#include <map>

namespace VCUCore {

struct TorqueSplit {
    float engineTorque;
    float motorTorque;
    float totalTorque;
    float efficiency;
    float responseTime;
    uint32_t timestamp;
};

class TorqueArbiter {
private:
    struct ArbitrationPolicy {
        float enginePriority;
        float motorPriority;
        float efficiencyWeight;
        float responseWeight;
        float batterySOCWeight;
        float engineHealthWeight;
    };
    
    ArbitrationPolicy currentPolicy_;
    std::map<DriveMode, ArbitrationPolicy> policyMap_;
    
    // 系统限制
    float maxEngineTorque_;
    float maxMotorTorque_;
    float minEngineTorque_;
    float minMotorTorque_;
    float torqueChangeRateLimit_;
    
    // 历史数据
    std::deque<TorqueSplit> torqueHistory_;
    uint32_t maxHistorySize_;
    
    // 学习模型
    std::unique_ptr<EfficiencyModel> efficiencyModel_;
    std::unique_ptr<ResponseModel> responseModel_;

public:
    TorqueArbiter(uint32_t historySize = 500);
    
    TorqueSplit decideDistribution(const PerceptionData& perception,
                                 const PredictionResult& prediction);
    
    void updateArbitrationPolicy(DriveMode mode);
    void learnFromFeedback(const TorqueSplit& actualSplit, 
                         const PerceptionData& resultingPerception);
    
    // 限制检查
    bool checkTorqueLimits(float engineTorque, float motorTorque) const;
    TorqueSplit applyTorqueLimits(const TorqueSplit& split) const;
    
    // 效率优化
    float calculateOptimalEfficiency(float totalTorque, float speed, float batterySOC) const;
    float calculateResponseTime(float engineTorque, float motorTorque) const;
    
    // 历史分析
    float calculateAverageEfficiency() const;
    float calculateResponseQuality() const;

private:
    void initializePolicies();
    TorqueSplit calculateBaseSplit(float totalTorque, const PerceptionData& perception);
    TorqueSplit optimizeForEfficiency(const TorqueSplit& baseSplit, const PerceptionData& perception);
    TorqueSplit optimizeForResponse(const TorqueSplit& baseSplit, const PerceptionData& perception);
    TorqueSplit optimizeForBattery(const TorqueSplit& baseSplit, const PerceptionData& perception);
    
    float calculateEngineEfficiency(float torque, float rpm) const;
    float calculateMotorEfficiency(float torque, float rpm, float batterySOC) const;
    
    void updateEfficiencyModel(const TorqueSplit& split, float actualEfficiency);
    void updateResponseModel(const TorqueSplit& split, float actualResponse);
};

} // namespace VCUCore