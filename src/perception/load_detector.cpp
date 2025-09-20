#include "perception/load_detector.hpp"
#include <chrono>
#include <algorithm>
#include <numeric>

namespace VCUCore {

LoadDetector::LoadDetector(size_t historySize, 
                          float torqueThreshold,
                          float forceThreshold,
                          float slipThreshold,
                          float slopeThreshold)
    : maxHistorySize_(historySize),
      torqueChangeThreshold_(torqueThreshold),
      forceChangeThreshold_(forceThreshold),
      slipChangeThreshold_(slipThreshold),
      minSignificantSlope_(slopeThreshold) {
    
    // 初始化移动平均和方差数组
    movingAverages_.fill(0.0f);
    movingVariances_.fill(0.0f);
}

LoadChangeResult LoadDetector::detectLoadChange(const SensorData& sensorData, 
                                               const TractorVehicleState& vehicleState) {
    LoadChangeResult result;
    result.hasChanged = false;
    result.changeType = LoadChangeType::NONE;
    result.magnitude = 0.0f;
    result.confidence = 0.0f;
    result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 创建当前负载签名
    LoadSignature currentSignature = createLoadSignature(sensorData, vehicleState);
    
    // 如果历史数据足够，进行变化检测
    if (!loadHistory_.empty()) {
        const LoadSignature& previous = loadHistory_.back();
        
        // 检测突变
        if (detectAbruptChange(currentSignature, previous)) {
            result.hasChanged = true;
            result.changeType = LoadChangeType::ABRUPT;
            result.magnitude = std::abs(currentSignature.engineTorque - previous.engineTorque);
            result.confidence = 0.8f;
        }
    }
    
    // 添加到历史记录
    loadHistory_.push_back(currentSignature);
    if (loadHistory_.size() > maxHistorySize_) {
        loadHistory_.erase(loadHistory_.begin());
    }
    
    // 更新统计信息
    updateMovingStatistics(currentSignature);
    
    return result;
}

LoadType LoadDetector::classifyLoadType(const LoadSignature& signature) const {
    // 简化的负载类型分类
    if (signature.implementForce > 15000.0f) {
        return LoadType::HEAVY_IMPLEMENT;
    } else if (signature.implementForce > 5000.0f) {
        return LoadType::MEDIUM_IMPLEMENT;
    } else {
        return LoadType::LIGHT_IMPLEMENT;
    }
}

LoadTrend LoadDetector::analyzeLoadTrend() const {
    LoadTrend trend;
    trend.direction = TrendDirection::STABLE;
    trend.rate = 0.0f;
    trend.confidence = 0.0f;
    trend.duration = 0;
    
    if (loadHistory_.size() < 10) {
        return trend;
    }
    
    // 分析最近的趋势
    std::vector<float> torqueValues;
    std::vector<float> timeValues;
    
    for (size_t i = std::max(0, (int)loadHistory_.size() - 10); i < loadHistory_.size(); ++i) {
        torqueValues.push_back(loadHistory_[i].engineTorque);
        timeValues.push_back(static_cast<float>(i));
    }
    
    float slope = calculateLinearRegressionSlope(timeValues, torqueValues);
    
    if (slope > minSignificantSlope_) {
        trend.direction = TrendDirection::INCREASING;
    } else if (slope < -minSignificantSlope_) {
        trend.direction = TrendDirection::DECREASING;
    }
    
    trend.rate = slope;
    trend.confidence = 0.7f;
    
    return trend;
}

void LoadDetector::updateMovingStatistics(const LoadSignature& signature) {
    // 简化的移动统计更新
    movingAverages_[0] = signature.engineTorque;
    movingAverages_[1] = signature.motorTorque;
    movingAverages_[2] = signature.implementForce;
    movingAverages_[3] = signature.wheelSlip;
    movingAverages_[4] = signature.powerConsumption;
    movingAverages_[5] = signature.torqueDerivative;
}

float LoadDetector::calculateLoadVariability() const {
    if (loadHistory_.size() < 2) return 0.0f;
    
    std::vector<float> torqueValues;
    for (const auto& signature : loadHistory_) {
        torqueValues.push_back(signature.engineTorque);
    }
    
    float mean = std::accumulate(torqueValues.begin(), torqueValues.end(), 0.0f) / torqueValues.size();
    float variance = 0.0f;
    
    for (float value : torqueValues) {
        variance += (value - mean) * (value - mean);
    }
    
    return std::sqrt(variance / torqueValues.size());
}

float LoadDetector::calculateLoadStability() const {
    float variability = calculateLoadVariability();
    return 1.0f / (1.0f + variability / 100.0f); // 归一化稳定性指标
}

std::vector<LoadSignature> LoadDetector::getLoadHistory(int duration_ms) const {
    std::vector<LoadSignature> result;
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    for (const auto& signature : loadHistory_) {
        if (currentTime - signature.timestamp <= static_cast<uint64_t>(duration_ms)) {
            result.push_back(signature);
        }
    }
    
    return result;
}

void LoadDetector::clearHistory() {
    loadHistory_.clear();
    movingAverages_.fill(0.0f);
    movingVariances_.fill(0.0f);
}

LoadSignature LoadDetector::createLoadSignature(const SensorData& sensorData, 
                                                             const TractorVehicleState& vehicleState) const {
    LoadSignature signature;
    signature.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    signature.engineTorque = vehicleState.actualTorque;
    signature.motorTorque = 0.0f; // 需要从传感器数据获取
    signature.implementForce = vehicleState.drawbarPull;
    signature.wheelSlip = vehicleState.wheelSlipRatio;
    signature.powerConsumption = vehicleState.powerConsumption;
    signature.torqueDerivative = 0.0f; // 需要计算
    signature.forceDerivative = 0.0f; // 需要计算
    signature.frequencyComponents = Eigen::Vector3f::Zero();
    
    return signature;
}

bool LoadDetector::detectAbruptChange(const LoadSignature& current, 
                                     const LoadSignature& previous) const {
    float torqueChange = std::abs(current.engineTorque - previous.engineTorque);
    float forceChange = std::abs(current.implementForce - previous.implementForce);
    
    return (torqueChange > torqueChangeThreshold_) || (forceChange > forceChangeThreshold_);
}

bool LoadDetector::detectGradualChange(const std::vector<LoadSignature>& window) const {
    if (window.size() < 5) return false;
    
    std::vector<float> torqueValues;
    std::vector<float> timeValues;
    
    for (size_t i = 0; i < window.size(); ++i) {
        torqueValues.push_back(window[i].engineTorque);
        timeValues.push_back(static_cast<float>(i));
    }
    
    float slope = calculateLinearRegressionSlope(timeValues, torqueValues);
    return std::abs(slope) > minSignificantSlope_;
}

bool LoadDetector::detectCyclicVariation(const std::vector<LoadSignature>& window) const {
    if (window.size() < 10) return false;
    
    std::vector<float> torqueValues;
    for (const auto& signature : window) {
        torqueValues.push_back(signature.engineTorque);
    }
    
    // 简化的周期性检测
    float autocorr = calculateAutocorrelation(torqueValues, torqueValues.size() / 4);
    return autocorr > 0.5f;
}

float LoadDetector::calculateLinearRegressionSlope(const std::vector<float>& x, 
                                                  const std::vector<float>& y) const {
    if (x.size() != y.size() || x.size() < 2) return 0.0f;
    
    float n = static_cast<float>(x.size());
    float sum_x = std::accumulate(x.begin(), x.end(), 0.0f);
    float sum_y = std::accumulate(y.begin(), y.end(), 0.0f);
    float sum_xy = 0.0f;
    float sum_x2 = 0.0f;
    
    for (size_t i = 0; i < x.size(); ++i) {
        sum_xy += x[i] * y[i];
        sum_x2 += x[i] * x[i];
    }
    
    float denominator = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) < 1e-6f) return 0.0f;
    
    return (n * sum_xy - sum_x * sum_y) / denominator;
}

float LoadDetector::calculateAutocorrelation(const std::vector<float>& data, int lag) const {
    if (data.size() <= static_cast<size_t>(lag)) return 0.0f;
    
    float mean = std::accumulate(data.begin(), data.end(), 0.0f) / data.size();
    float variance = 0.0f;
    float covariance = 0.0f;
    
    for (float value : data) {
        variance += (value - mean) * (value - mean);
    }
    
    for (size_t i = 0; i < data.size() - lag; ++i) {
        covariance += (data[i] - mean) * (data[i + lag] - mean);
    }
    
    if (variance < 1e-6f) return 0.0f;
    
    return covariance / variance;
}

Eigen::Vector3f LoadDetector::analyzeFrequencyComponents(const std::vector<float>& torqueSamples) const {
    // 简化的频率分析，返回零向量
    return Eigen::Vector3f::Zero();
}

} // namespace VCUCore
