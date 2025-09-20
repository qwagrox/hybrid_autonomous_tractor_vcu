// include/control/cvt_controller.hpp - 修复重复定义问题
#pragma once
#include "vcu_core_types.hpp"
#include "models/vehicle_dynamics_model.hpp"
#include <deque>
#include <array>

namespace VCUCore {

// 移除重复的CVTManufacturerParams定义，使用vcu_core_types.hpp中的定义

class CVTController {
public:
    CVTController(uint32_t history_size = 100);

    void setDriveMode(DriveMode mode);
    void update(const PerceptionData& perception, const PredictionResult& prediction);
    CVTState getCurrentState() const;
    
    // 制造商适配
    void adaptToManufacturer(CVTManufacturer manufacturer);
    CVTManufacturerParams getManufacturerParams() const;
    
    // 比率计算
    double calculateOptimalRatio(const PerceptionData& perception, 
                               const PredictionResult& prediction);
    
    // 历史数据管理
    void updateRatioHistory(double ratio);
    std::deque<double> getRatioHistory() const;
    
    // 性能优化
    void optimizeForEfficiency();
    void optimizeForPerformance();

private:
    CVTState currentState_;
    DriveMode currentDriveMode_;
    CVTManufacturer currentManufacturer_;
    CVTManufacturerParams manufacturerParams_;
    uint32_t historySize_;
    
    std::deque<double> ratioHistory_;
    std::array<double, 24> hourlyRatioPattern_;
    
    void initializeManufacturerParams();
    void updateRatioBasedOnLoad(double loadFactor);
    void updateRatioBasedOnTerrain(double terrainSlope);
    double calculateEfficiencyScore(double ratio, double load) const;
    
    // 学习算法
    void learnOptimalRatios();
    void updateAdaptiveParameters(const PerceptionData& perception);
};

} // namespace VCUCore
