#ifndef VCU_DATA_TYPES_H
#define VCU_DATA_TYPES_H

namespace vcu {
namespace common {

/**
 * @enum TerrainType
 * @brief Represents different terrain types for adaptive control.
 */
enum class TerrainType {
    SMOOTH = 0,     ///< Smooth terrain (roads, flat fields)
    MODERATE = 1,   ///< Moderate terrain (slight inclines, soft soil)
    ROUGH = 2       ///< Rough terrain (steep inclines, rocky ground)
};

/**
 * @brief Represents perception data from vehicle sensors.
 */
struct PerceptionData {
    // Basic sensor data
    float vehicle_speed_mps = 0.0f;              ///< Vehicle speed in meters per second
    float engine_speed_rpm = 0.0f;               ///< Engine speed in revolutions per minute
    float engine_load_percent = 0.0f;            ///< Engine load in percentage
    float accelerator_pedal_percent = 0.0f;      ///< Accelerator pedal position in percentage
    
    // Transmission data
    float current_transmission_ratio = 1.0f;     ///< Current CVT transmission ratio
    bool is_transmission_shifting = false;       ///< True if transmission is currently shifting
    
    // Derived data
    float load_factor = 0.0f;                    ///< Calculated load factor (0.0 - 1.0)
    TerrainType terrain_type = TerrainType::SMOOTH; ///< Detected terrain type
    
    // Data validity
    bool data_valid = false;                     ///< True if the data is valid and recent
};

/**
 * @brief Represents prediction results.
 * For now, this is a placeholder.
 */
struct PredictionResult {
    float predicted_load_percent = 0.0f; // Predicted engine load in percentage
};

} // namespace common
} // namespace vcu

#endif // VCU_DATA_TYPES_H
