#ifndef VCU_DATA_TYPES_H
#define VCU_DATA_TYPES_H

namespace vcu {
namespace common {

/**
 * @enum TerrainType
 * @brief Represents different terrain types for adaptive control.
 */
enum class TerrainType {
    FLAT = 0,           ///< Flat terrain (roads, flat fields)
    GENTLE_SLOPE = 1,   ///< Gentle slope terrain (slight inclines)
    HILLY = 2,          ///< Hilly terrain (moderate inclines, soft soil)
    STEEP_SLOPE = 3,    ///< Steep slope terrain (steep inclines, rocky ground)
    SMOOTH = 0,         ///< Alias for FLAT (backward compatibility)
    MODERATE = 2,       ///< Alias for HILLY (backward compatibility)
    ROUGH = 3           ///< Alias for STEEP_SLOPE (backward compatibility)
};

/**
 * @brief Represents perception data from vehicle sensors.
 */
struct PerceptionData {
    // Basic sensor data
    float vehicle_speed_mps = 0.0f;              ///< Vehicle speed in meters per second
    float vehicle_speed_kmh = 0.0f;              ///< Vehicle speed in kilometers per hour
    float engine_speed_rpm = 0.0f;               ///< Engine speed in revolutions per minute
    float engine_load_percent = 0.0f;            ///< Engine load in percentage
    float accelerator_pedal_percent = 0.0f;      ///< Accelerator pedal position in percentage
    
    // Additional sensor data
    float fuel_level_percent = 100.0f;           ///< Fuel level in percentage
    float coolant_temp_celsius = 20.0f;          ///< Coolant temperature in Celsius
    
    // Transmission data
    float current_transmission_ratio = 1.0f;     ///< Current CVT transmission ratio
    bool is_transmission_shifting = false;       ///< True if transmission is currently shifting
    
    // Derived data
    float load_factor = 0.0f;                    ///< Calculated load factor (0.0 - 1.0)
    TerrainType terrain_type = TerrainType::FLAT; ///< Detected terrain type
    
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
