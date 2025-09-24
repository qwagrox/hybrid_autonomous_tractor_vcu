// Copyright 2025 Manus AI

#include "vcu/prediction/platform_load_predictor.h"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace vcu {
namespace prediction {

PlatformLoadPredictor::PlatformLoadPredictor(PlatformInterface* platform)
    : PlatformLoadPredictor(platform, LoadPredictionConfig{}) {
}

PlatformLoadPredictor::PlatformLoadPredictor(PlatformInterface* platform, const LoadPredictionConfig& config)
    : platform_(platform),
      config_(config),
      last_update_time_ms_(0),
      is_initialized_(false) {
    
    time_interface_ = platform_->create_time_interface();
    data_mutex_ = platform_->create_mutex();
    history_.reserve(config_.history_window_size);
}

PlatformLoadPredictor::~PlatformLoadPredictor() {
    shutdown();
}

PredictionResult PlatformLoadPredictor::initialize() {
    if (is_initialized_) {
        return PredictionResult::SUCCESS;
    }

    // Validate configuration
    if (config_.history_window_size == 0 || 
        config_.prediction_horizon_ms == 0 ||
        config_.terrain_weight < 0.0f || config_.terrain_weight > 1.0f ||
        config_.speed_weight < 0.0f || config_.speed_weight > 1.0f ||
        config_.load_weight < 0.0f || config_.load_weight > 1.0f) {
        return PredictionResult::ERROR_INVALID_INPUT;
    }

    // Ensure weights sum to approximately 1.0
    float total_weight = config_.terrain_weight + config_.speed_weight + config_.load_weight;
    if (std::abs(total_weight - 1.0f) > 0.1f) {
        return PredictionResult::ERROR_INVALID_INPUT;
    }

    clear_history();
    last_update_time_ms_ = time_interface_->get_monotonic_time_ms();
    is_initialized_ = true;

    return PredictionResult::SUCCESS;
}

PredictionResult PlatformLoadPredictor::shutdown() {
    if (!is_initialized_) {
        return PredictionResult::SUCCESS;
    }

    data_mutex_->lock();
    clear_history();
    data_mutex_->unlock();
    
    is_initialized_ = false;
    return PredictionResult::SUCCESS;
}

PredictionResult PlatformLoadPredictor::update_perception_data(const common::PerceptionData& data) {
    if (!is_initialized_) {
        return PredictionResult::ERROR_NOT_INITIALIZED;
    }

    data_mutex_->lock();

    LoadDataPoint data_point;
    data_point.timestamp_ms = time_interface_->get_monotonic_time_ms();
    data_point.vehicle_speed_kmh = data.vehicle_speed_kmh;
    data_point.engine_load_percent = data.engine_load_percent;
    data_point.terrain_type = data.terrain_type;
    data_point.load_factor = data.load_factor;

    // Add to history
    history_.push_back(data_point);

    // Maintain window size
    if (history_.size() > config_.history_window_size) {
        history_.erase(history_.begin());
    }

    last_update_time_ms_ = data_point.timestamp_ms;

    data_mutex_->unlock();

    return PredictionResult::SUCCESS;
}

PredictionResult PlatformLoadPredictor::predict_load(LoadPrediction& prediction) {
    if (!is_initialized_) {
        return PredictionResult::ERROR_NOT_INITIALIZED;
    }

    data_mutex_->lock();

    if (history_.empty()) {
        data_mutex_->unlock();
        return PredictionResult::ERROR_INSUFFICIENT_DATA;
    }

    // Get latest data point
    const auto& latest = history_.back();

    // Calculate terrain factor
    float terrain_factor = calculate_terrain_factor(latest.terrain_type);

    // Calculate speed factor
    float speed_factor = calculate_speed_factor(latest.vehicle_speed_kmh);

    // Calculate load trend
    float load_trend = calculate_load_trend();

    // Weighted prediction
    float predicted_load = (terrain_factor * config_.terrain_weight +
                           speed_factor * config_.speed_weight +
                           load_trend * config_.load_weight) * 100.0f;

    // Clamp to valid range
    predicted_load = std::max(0.0f, std::min(100.0f, predicted_load));

    // Calculate confidence
    float variance = 0.0f;
    if (history_.size() > 1) {
        // 使用std::accumulate算法计算平均负载
        float mean_load = std::accumulate(history_.begin(), history_.end(), 0.0f,
            [](float sum, const LoadDataPoint& point) {
                return sum + point.engine_load_percent;
            }) / history_.size();

        // 使用std::accumulate算法计算方差
        variance = std::accumulate(history_.begin(), history_.end(), 0.0f,
            [mean_load](float sum, const LoadDataPoint& point) {
                float diff = point.engine_load_percent - mean_load;
                return sum + diff * diff;
            }) / history_.size();
    }

    float confidence = calculate_confidence(static_cast<uint32_t>(history_.size()), variance);

    // Predict terrain type
    common::TerrainType expected_terrain = predict_terrain_type();

    // Fill prediction result
    prediction.predicted_load_percent = predicted_load;
    prediction.confidence_level = confidence;
    prediction.prediction_horizon_ms = config_.prediction_horizon_ms;
    prediction.expected_terrain = expected_terrain;

    data_mutex_->unlock();

    return PredictionResult::SUCCESS;
}

PredictionResult PlatformLoadPredictor::get_statistics(float& avg_load, float& max_load, uint32_t& data_points) {
    if (!is_initialized_) {
        return PredictionResult::ERROR_NOT_INITIALIZED;
    }

    data_mutex_->lock();

    if (history_.empty()) {
        avg_load = 0.0f;
        max_load = 0.0f;
        data_points = 0;
        data_mutex_->unlock();
        return PredictionResult::SUCCESS;
    }

    // 使用std::accumulate算法计算总负载
    float total_load = std::accumulate(history_.begin(), history_.end(), 0.0f,
        [](float sum, const LoadDataPoint& point) {
            return sum + point.engine_load_percent;
        });

    // 使用std::max_element算法找到最大负载
    auto max_element = std::max_element(history_.begin(), history_.end(),
        [](const LoadDataPoint& a, const LoadDataPoint& b) {
            return a.engine_load_percent < b.engine_load_percent;
        });
    
    max_load = max_element->engine_load_percent;
    avg_load = total_load / history_.size();
    data_points = static_cast<uint32_t>(history_.size());

    data_mutex_->unlock();

    return PredictionResult::SUCCESS;
}

void PlatformLoadPredictor::clear_history() {
    history_.clear();
}

bool PlatformLoadPredictor::is_initialized() const {
    return is_initialized_;
}

float PlatformLoadPredictor::calculate_terrain_factor(common::TerrainType terrain) const {
    switch (terrain) {
        case common::TerrainType::FLAT:
            return 0.3f;
        case common::TerrainType::GENTLE_SLOPE:
            return 0.5f;
        case common::TerrainType::HILLY:
            return 0.7f;
        case common::TerrainType::STEEP_SLOPE:
            return 0.9f;
        default:
            return 0.5f;
    }
}

float PlatformLoadPredictor::calculate_speed_factor(float speed_kmh) const {
    // Normalize speed to 0-1 range (assuming max speed of 50 km/h for tractors)
    const float max_speed = 50.0f;
    float normalized_speed = std::min(speed_kmh / max_speed, 1.0f);
    
    // Higher speed generally requires more load
    return 0.2f + 0.6f * normalized_speed;
}

float PlatformLoadPredictor::calculate_load_trend() const {
    if (history_.size() < 2) {
        return history_.empty() ? 0.5f : history_.back().engine_load_percent / 100.0f;
    }

    // Calculate trend over recent data points
    const size_t trend_window = std::min(static_cast<size_t>(10), history_.size());
    float trend_sum = 0.0f;

    for (size_t i = history_.size() - trend_window; i < history_.size() - 1; ++i) {
        float load_change = history_[i + 1].engine_load_percent - history_[i].engine_load_percent;
        trend_sum += load_change;
    }

    float avg_trend = trend_sum / (trend_window - 1);
    float current_load = history_.back().engine_load_percent / 100.0f;

    // Project trend forward
    float projected_load = current_load + (avg_trend / 100.0f);
    return std::max(0.0f, std::min(1.0f, projected_load));
}

float PlatformLoadPredictor::calculate_confidence(uint32_t data_points, float variance) const {
    // Base confidence on data points available
    float data_confidence = std::min(1.0f, static_cast<float>(data_points) / config_.history_window_size);
    
    // Reduce confidence based on variance
    float variance_penalty = std::min(0.5f, variance / 1000.0f); // Normalize variance
    
    return std::max(0.1f, data_confidence - variance_penalty);
}

common::TerrainType PlatformLoadPredictor::predict_terrain_type() const {
    if (history_.empty()) {
        return common::TerrainType::FLAT;
    }

    // Use most recent terrain type as prediction
    // In a more sophisticated implementation, this could analyze terrain patterns
    return history_.back().terrain_type;
}

} // namespace prediction
} // namespace vcu
