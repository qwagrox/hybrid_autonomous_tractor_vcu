#ifndef LOAD_PREDICTOR_H
#define LOAD_PREDICTOR_H

#include "vcu/common/vcu_data_types.h"
#include <vector>
#include <chrono>
#include <memory>

namespace vcu {
namespace prediction {

/**
 * @enum PredictionResult
 * @brief Result codes for prediction operations.
 */
enum class PredictionResult {
    SUCCESS = 0,        ///< Prediction completed successfully
    ERROR_INIT,         ///< Predictor not initialized
    ERROR_INSUFFICIENT_DATA, ///< Not enough historical data for prediction
    ERROR_INVALID_INPUT ///< Invalid input data provided
};

/**
 * @struct LoadPredictionConfig
 * @brief Configuration parameters for load prediction.
 */
struct LoadPredictionConfig {
    uint32_t history_window_size = 10;      ///< Number of historical samples to keep
    uint32_t prediction_horizon_ms = 2000;  ///< Prediction horizon in milliseconds
    float terrain_weight = 0.3f;            ///< Weight factor for terrain influence
    float speed_weight = 0.4f;              ///< Weight factor for speed influence
    float load_weight = 0.3f;               ///< Weight factor for current load influence
};

/**
 * @struct HistoricalDataPoint
 * @brief Represents a single historical data point for prediction.
 */
struct HistoricalDataPoint {
    std::chrono::steady_clock::time_point timestamp; ///< When the data was recorded
    common::PerceptionData perception_data;          ///< Sensor data at this time
    float actual_load;                               ///< Actual load observed
};

/**
 * @class LoadPredictor
 * @brief Predicts future engine load based on historical data and current conditions.
 *
 * This class implements a simple load prediction algorithm that considers:
 * - Historical load patterns
 * - Current terrain type
 * - Vehicle speed trends
 * - Engine operating conditions
 *
 * The predictor uses a weighted moving average approach with terrain-specific
 * adjustments to forecast future load requirements.
 */
class LoadPredictor {
public:
    /**
     * @brief Constructs a new LoadPredictor with default configuration.
     */
    LoadPredictor();

    /**
     * @brief Constructs a new LoadPredictor with custom configuration.
     * @param config Configuration parameters for the predictor.
     */
    explicit LoadPredictor(const LoadPredictionConfig& config);

    /**
     * @brief Destroys the LoadPredictor.
     */
    ~LoadPredictor() = default;

    /**
     * @brief Initializes the load predictor.
     * @return PredictionResult::SUCCESS on success, error code otherwise.
     */
    PredictionResult initialize();

    /**
     * @brief Shuts down the load predictor and clears historical data.
     * @return PredictionResult::SUCCESS on success, error code otherwise.
     */
    PredictionResult shutdown();

    /**
     * @brief Updates the predictor with new sensor data.
     * 
     * This method should be called regularly with fresh sensor data to
     * maintain the historical data window for accurate predictions.
     * 
     * @param perception_data Current sensor readings.
     * @return PredictionResult::SUCCESS on success, error code otherwise.
     */
    PredictionResult update_data(const common::PerceptionData& perception_data);

    /**
     * @brief Predicts future load based on current conditions and history.
     * 
     * @param prediction_result Output structure to store the prediction results.
     * @return PredictionResult::SUCCESS if prediction is available, error code otherwise.
     */
    PredictionResult predict_load(common::PredictionResult& prediction_result);

    /**
     * @brief Gets the current prediction configuration.
     * @return The current configuration parameters.
     */
    const LoadPredictionConfig& get_config() const;

    /**
     * @brief Updates the prediction configuration.
     * @param config New configuration parameters.
     */
    void set_config(const LoadPredictionConfig& config);

    /**
     * @brief Checks if the predictor has sufficient data for predictions.
     * @return True if enough historical data is available, false otherwise.
     */
    bool has_sufficient_data() const;

    /**
     * @brief Gets the number of historical data points currently stored.
     * @return Number of data points in the history buffer.
     */
    size_t get_history_size() const;

    /**
     * @brief Clears all historical data.
     */
    void clear_history();

private:
    /**
     * @brief Adds a new data point to the historical buffer.
     * @param data_point The data point to add.
     */
    void add_historical_point(const HistoricalDataPoint& data_point);

    /**
     * @brief Calculates terrain-based load adjustment factor.
     * @param terrain_type The current terrain type.
     * @return Adjustment factor (1.0 = no adjustment, >1.0 = higher load expected).
     */
    float calculate_terrain_factor(common::TerrainType terrain_type) const;

    /**
     * @brief Calculates speed-based load trend.
     * @param current_speed Current vehicle speed in m/s.
     * @return Speed trend factor for load prediction.
     */
    float calculate_speed_trend(float current_speed) const;

    /**
     * @brief Performs weighted moving average calculation on historical loads.
     * @return Predicted base load from historical data.
     */
    float calculate_weighted_average() const;

    /**
     * @brief Applies terrain and speed adjustments to base prediction.
     * @param base_prediction Base load prediction from historical data.
     * @param current_data Current sensor data for adjustments.
     * @return Adjusted load prediction.
     */
    float apply_adjustments(float base_prediction, const common::PerceptionData& current_data) const;

    LoadPredictionConfig config_;                    ///< Prediction configuration
    std::vector<HistoricalDataPoint> history_;      ///< Historical data buffer
    bool is_initialized_;                           ///< True if predictor is initialized
    std::chrono::steady_clock::time_point last_update_time_; ///< Timestamp of last update
};

} // namespace prediction
} // namespace vcu

#endif // LOAD_PREDICTOR_H
