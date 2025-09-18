// include/perception/load_detector.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <deque>
#include <array>
#include <cmath>

namespace VCUCore {

class LoadDetector {
private:
    struct LoadSignature {
        Timestamp timestamp;
        float engineTorque;
        float motorTorque;
        float implementForce;
        float wheelSlip;
        float powerConsumption;
        float torqueDerivative;
        float forceDerivative;
        Eigen::Vector3f frequencyComponents;
    };
    
    std::deque<LoadSignature> loadHistory_;
    size_t maxHistorySize_;
    
    // 检测参数
    float torqueChangeThreshold_;
    float forceChangeThreshold_;
    float slipChangeThreshold_;
    float minSignificantSlope_;
    
    // 统计信息
    std::array<float, 6> movingAverages_;
    std::array<float, 6> movingVariances_;
    
public:
    LoadDetector(size_t historySize = 500, 
                float torqueThreshold = 30.0f,
                float forceThreshold = 1000.0f,
                float slipThreshold = 0.05f,
                float slopeThreshold = 10.0f);
    
    LoadChangeResult detectLoadChange(const SensorData& sensorData, 
                                    const VehicleState& vehicleState);
    
    LoadType classifyLoadType(const LoadSignature& signature) const;
    LoadTrend analyzeLoadTrend() const;
    
    // 统计方法
    void updateMovingStatistics(const LoadSignature& signature);
    float calculateLoadVariability() const;
    float calculateLoadStability() const;
    
    // 历史数据访问
    std::vector<LoadSignature> getLoadHistory(int duration_ms) const;
    void clearHistory();
    
private:
    LoadSignature createLoadSignature(const SensorData& sensorData, 
                                    const VehicleState& vehicleState) const;
    
    bool detectAbruptChange(const LoadSignature& current, 
                          const LoadSignature& previous) const;
    bool detectGradualChange(const std::vector<LoadSignature>& window) const;
    bool detectCyclicVariation(const std::vector<LoadSignature>& window) const;
    
    float calculateLinearRegressionSlope(const std::vector<float>& x, 
                                       const std::vector<float>& y) const;
    float calculateAutocorrelation(const std::vector<float>& data, int lag) const;
    
    Eigen::Vector3f analyzeFrequencyComponents(const std::vector<float>& torqueSamples) const;
};

} // namespace VCUCore