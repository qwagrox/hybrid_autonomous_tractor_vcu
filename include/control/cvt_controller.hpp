// include/control/cvt_controller.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <memory>
#include <map>
#include <deque>

namespace VCUCore {

class CVTController {
private:
    struct CVTControlParams {
        float minRatio;
        float maxRatio;
        float ratioChangeRate;
        float slipRatioTarget;
        float efficiencyWeight;
        float comfortWeight;
        float responseWeight;
        float terrainAdaptationGain;
    };
    
    struct CVTState {
        float currentRatio;
        float targetRatio;
        float ratioRate;
        bool isShifting;
        float shiftProgress;
        uint32_t shiftStartTime;
    };
    
    CVTState currentState_;
    CVTControlParams controlParams_;
    std::map<DriveMode, CVTControlParams> modeParams_;
    
    // 学习参数
    std::deque<float> ratioHistory_;
    std::deque<float> efficiencyHistory_;
    uint32_t maxHistorySize_;
    
    // 制造商特定参数
    CVTManufacturer currentManufacturer_;
    std::map<CVTManufacturer, CVTControlParams> manufacturerParams_;

public:
    CVTController(uint32_t historySize = 1000);
    
    float calculateOptimalRatio(const PerceptionData& perception,
                              const PredictionResult& prediction);
    
    void updateControlParams(DriveMode mode);
    void adaptToManufacturer(CVTManufacturer manufacturer);
    
    // 状态查询
    CVTState getCurrentState() const;
    float getCurrentEfficiency() const;
    bool isShifting() const;
    
    // 限制检查
    bool checkRatioLimits(float ratio) const;
    float applyRateLimits(float targetRatio) const;
    
    // 学习功能
    void learnFromExperience(float actualRatio, float actualEfficiency);
    void optimizeControlParams();
    
    // 场景特定控制
    float calculatePlowingRatio(const PerceptionData& perception) const;
    float calculateSeedingRatio(const PerceptionData& perception) const;
    float calculateTransportRatio(const PerceptionData& perception) const;

private:
    void initializeModeParams();
    void initializeManufacturerParams();
    
    float calculateBaseRatio(const PerceptionData& perception) const;
    float optimizeForEfficiency(float baseRatio, const PerceptionData& perception) const;
    float optimizeForTraction(float baseRatio, const PerceptionData& perception) const;
    float optimizeForComfort(float baseRatio, const PerceptionData& perception) const;
    
    float calculateTheoreticalEfficiency(float ratio, const PerceptionData& perception) const;
    float calculateWheelSlip(float ratio, const PerceptionData& perception) const;
    float calculateShiftQuality(float ratioChange) const;
    
    void updateShiftState(float newRatio);
    void completeShift();
};

} // namespace VCUCore