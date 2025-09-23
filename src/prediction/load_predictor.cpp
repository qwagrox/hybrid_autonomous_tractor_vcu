#include "vcu/prediction/load_predictor.h"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace vcu {
namespace prediction {

LoadPredictor::LoadPredictor()
    : LoadPredictor(LoadPredictionConfig{}) {
}

LoadPredictor::LoadPredictor(const LoadPredictionConfig& config)
    : config_(config),
      is_initialized_(false) {
    history_.reserve(config_.history_window_size);
}

PredictionResult LoadPredictor::initialize() {
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
    last_update_time_ = std::chrono::steady_clock::now();
    is_initialized_ = true;

    return PredictionResult::SUCCESS;
}

PredictionResult LoadPredictor::shutdown() {
    if (!is_initialized_) {
        return PredictionResult::SUCCESS;
    }

    clear_history();
    is_initialized_ = false;

    return PredictionResult::SUCCESS;
}

PredictionResult LoadPredictor::update_data(const common::PerceptionData& perception_data) {
    if (!is_initialized_) {
        return PredictionResult::ERROR_INIT;
    }

    if (!perception_data.data_valid) {
        return PredictionResult::ERROR_INVALID_INPUT;
    }

    // Create new historical data point
    HistoricalDataPoint data_point;
    data_point.timestamp = std::chrono::steady_clock::now();
    data_point.perception_data = perception_data;
    data_point.actual_load = perception_data.engine_load_percent;

    add_historical_point(data_point);
    last_update_time_ = data_point.timestamp;

    return PredictionResult::SUCCESS;
}

PredictionResult LoadPredictor::predict_load(common::PredictionResult& prediction_result) {
    if (!is_initialized_) {
        return PredictionResult::ERROR_INIT;
    }

    if (!has_sufficient_data()) {
        return PredictionResult::ERROR_INSUFFICIENT_DATA;
    }

    // Get the most recent data point for current conditions
    const auto& latest_data = history_.back().perception_data;

    // Calculate base prediction from historical data
    float base_prediction = calculate_weighted_average();

    // Apply terrain and speed adjustments
    float adjusted_prediction = apply_adjustments(base_prediction, latest_data);

    // Clamp prediction to reasonable bounds (0-100%)
    adjusted_prediction = std::clamp(adjusted_prediction, 0.0f, 100.0f);

    // Store result
    prediction_result.predicted_load_percent = adjusted_prediction;

    return PredictionResult::SUCCESS;
}

const LoadPredictionConfig& LoadPredictor::get_config() const {
    return config_;
}

void LoadPredictor::set_config(const LoadPredictionConfig& config) {
    config_ = config;
    
    // Resize history buffer if needed
    if (history_.capacity() < config_.history_window_size) {
        history_.reserve(config_.history_window_size);
    }
    
    // Trim history if new window size is smaller
    if (history_.size() > config_.history_window_size) {
        history_.erase(history_.begin(), 
                      history_.begin() + (history_.size() - config_.history_window_size));
    }
}

bool LoadPredictor::has_sufficient_data() const {
    // Need at least 3 data points for meaningful prediction
    return history_.size() >= std::min(3u, config_.history_window_size);
}

size_t LoadPredictor::get_history_size() const {
    return history_.size();
}

void LoadPredictor::clear_history() {
    history_.clear();
}

void LoadPredictor::add_historical_point(const HistoricalDataPoint& data_point) {
    // Add new point
    history_.push_back(data_point);

    // Remove oldest point if we exceed window size
    if (history_.size() > config_.history_window_size) {
        history_.erase(history_.begin());
    }
}

float LoadPredictor::calculate_terrain_factor(common::TerrainType terrain_type) const {
    switch (terrain_type) {
        case common::TerrainType::SMOOTH:
            return 0.9f;  // Expect lower load on smooth terrain
        case common::TerrainType::MODERATE:
            return 1.1f;  // Slightly higher load on moderate terrain
        case common::TerrainType::ROUGH:
            return 1.3f;  // Significantly higher load on rough terrain
        default:
            return 1.0f;  // Default case
    }
}

float LoadPredictor::calculate_speed_trend(float current_speed) const {
    if (history_.size() < 2) {
        return 1.0f; // No trend available
    }

    // Calculate speed change over the last few data points
    const size_t trend_window = std::min(static_cast<size_t>(3), history_.size());
    float speed_sum = 0.0f;
    float weight_sum = 0.0f;

    for (size_t i = history_.size() - trend_window; i < history_.size(); ++i) {
        float weight = static_cast<float>(i + 1); // More recent data has higher weight
        speed_sum += history_[i].perception_data.vehicle_speed_mps * weight;
        weight_sum += weight;
    }

    float avg_recent_speed = speed_sum / weight_sum;
    float speed_change_rate = (current_speed - avg_recent_speed) / avg_recent_speed;

    // Convert speed change to load factor
    // Accelerating (positive change) typically increases load
    // Decelerating (negative change) typically decreases load
    return 1.0f + (speed_change_rate * 0.2f); // 20% influence factor
}

float LoadPredictor::calculate_weighted_average() const {
    if (history_.empty()) {
        return 50.0f; // Default assumption
    }

    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;

    // Use exponential weighting - more recent data has higher weight
    for (size_t i = 0; i < history_.size(); ++i) {
        float weight = std::exp(static_cast<float>(i) / static_cast<float>(history_.size()));
        weighted_sum += history_[i].actual_load * weight;
        weight_sum += weight;
    }

    return weighted_sum / weight_sum;
}

float LoadPredictor::apply_adjustments(float base_prediction, const common::PerceptionData& current_data) const {
    // Calculate individual adjustment factors
    float terrain_factor = calculate_terrain_factor(current_data.terrain_type);
    float speed_trend_factor = calculate_speed_trend(current_data.vehicle_speed_mps);
    
    // Current load influence (if current load is high, future load likely to remain high)
    float current_load_factor = 1.0f + (current_data.engine_load_percent - 50.0f) / 100.0f;

    // Apply weighted adjustments
    float terrain_adjustment = (terrain_factor - 1.0f) * config_.terrain_weight;
    float speed_adjustment = (speed_trend_factor - 1.0f) * config_.speed_weight;
    float load_adjustment = (current_load_factor - 1.0f) * config_.load_weight;

    // Combine adjustments with base prediction
    float total_adjustment = 1.0f + terrain_adjustment + speed_adjustment + load_adjustment;
    
    return base_prediction * total_adjustment;
}

} // namespace prediction
} // namespace vcu
