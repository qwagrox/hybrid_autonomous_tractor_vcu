#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

class LoadDetector {
private:
    float currentLoad_;
    float maxLoad_;
    float loadThreshold_;
    uint64_t lastUpdateTime_;
    
    // 缺失的成员变量
    float torqueChangeThreshold_;
    float forceChangeThreshold_;
    float minSignificantSlope_;
    std::vector<LoadSignature> loadHistory_;
    std::array<float, 10> movingAverages_;
    std::array<float, 10> movingVariances_;
    size_t maxHistorySize_;
    float slipChangeThreshold_;
    
public:
    LoadDetector();
    LoadDetector(size_t historySize, float torqueThreshold, float forceThreshold, 
                float slipThreshold, float slopeThreshold);
    ~LoadDetector() = default;
    
    void updateLoadMeasurement(const TractorVehicleState& state);
    float getCurrentLoad() const;
    float getMaxLoad() const;
    bool isOverloaded() const;
    void setLoadThreshold(float threshold);
    void reset();
    
    
    // 新增的公共方法
    float calculateLoadStability() const;
    float calculateLoadVariability() const;
    std::vector<LoadSignature> getLoadHistory(int duration_ms) const;
    void clearHistory();
    LoadSignature createLoadSignature(const SensorData& sensorData, const TractorVehicleState& state) const;
    
    // 添加缺失的方法声明
    LoadChangeResult detectLoadChange(const SensorData& sensorData, const TractorVehicleState& state);
    LoadType classifyLoadType(const LoadSignature& signature) const;
    LoadTrend analyzeLoadTrend() const;
    void updateMovingStatistics(const LoadSignature& signature);
    
private:
    // 缺失的方法声明
    bool detectCyclicVariation(const std::vector<LoadSignature>& window) const;
    float calculateLinearRegressionSlope(const std::vector<float>& x, const std::vector<float>& y) const;
    float calculateAutocorrelation(const std::vector<float>& data, int lag) const;
    Eigen::Vector3f analyzeFrequencyComponents(const std::vector<float>& torqueSamples) const;
    bool detectAbruptChange(const LoadSignature& current, const LoadSignature& previous) const;
    bool detectGradualChange(const std::vector<LoadSignature>& window) const;
};

} // namespace VCUCore
