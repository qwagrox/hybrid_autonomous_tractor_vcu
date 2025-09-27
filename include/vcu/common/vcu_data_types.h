#ifndef VCU_DATA_TYPES_H
#define VCU_DATA_TYPES_H

#include <cstdint>

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
 * @enum LiftMode
 * @brief Defines the lifting mode for hydraulic control.
 */
enum class LiftMode : uint8_t {
    MANUAL = 0,         ///< Manual mode - direct position control
    AUTO_DEPTH = 1,     ///< Automatic depth control based on load
    AUTO_LOAD = 2,      ///< Automatic load control
    SHOCK_ABSORB = 3,   ///< Shock absorption mode
    POSITION_HOLD = 4   ///< Hold current position
};

/**
 * @enum LiftAction
 * @brief Defines the lifting actions.
 */
enum class LiftAction : uint8_t {
    STOP = 0,           ///< Stop lifting
    LIFT_UP = 1,        ///< Lift up
    LIFT_DOWN = 2,      ///< Lift down
    SHOCK_ENABLE = 3,   ///< Enable shock absorption
    SHOCK_DISABLE = 4   ///< Disable shock absorption
};

/**
 * @enum MultiValveState
 * @brief Defines the state of multi-valve hydraulic system.
 */
enum class MultiValveState : uint8_t {
    NEUTRAL = 1,        ///< Neutral position
    EXTEND = 64,        ///< Extending
    RETRACT = 16,       ///< Retracting
    FLOAT = 4           ///< Floating
};

/**
 * @brief Represents hydraulic system state.
 */
struct HydraulicState {
    // Lift system state
    float lift_position = 0.0f;        ///< Current lift position (0-100%)
    LiftMode lift_mode = LiftMode::MANUAL; ///< Current lift mode
    bool lift_moving = false;           ///< True if lift is currently moving
    bool shock_absorb_active = false;   ///< True if shock absorption is active
    
    // Multi-valve system state
    MultiValveState valve_states[4] = { ///< State of 4 multi-valves
        MultiValveState::NEUTRAL,
        MultiValveState::NEUTRAL,
        MultiValveState::NEUTRAL,
        MultiValveState::NEUTRAL
    };
    bool valve_locked = false;          ///< True if valves are locked
    
    // Pressure and temperature monitoring
    float main_pressure_mpa = 0.0f;     ///< Main supply pressure (MPa)
    float lift_pressure_mpa = 0.0f;     ///< Lift system pressure (MPa)
    float pto_pressure_mpa = 0.0f;      ///< PTO clutch pressure (MPa)
    float rear_axle_temp_celsius = 20.0f; ///< Rear axle oil temperature (°C)
    float hst_temp_celsius = 20.0f;     ///< HST oil temperature (°C)
    
    // Filter and system status
    bool lift_filter_ok = true;         ///< Lift filter status
    bool suction_filter_ok = true;      ///< Suction filter status
    bool hydraulic_enabled = false;     ///< True if hydraulic control is enabled
    
    // Error flags
    uint32_t error_flags = 0;           ///< Error flags bitfield
    
    // Data validity
    bool data_valid = false;            ///< True if the data is valid and recent
};

/**
 * @brief Represents hydraulic control command.
 */
struct HydraulicCommand {
    // Lift control
    LiftAction lift_action = LiftAction::STOP; ///< Lift action command
    LiftMode lift_mode = LiftMode::MANUAL;     ///< Target lift mode
    float target_position = 0.0f;             ///< Target position (0-100%)
    float target_depth = 0.0f;                ///< Target working depth (0-100%)
    uint8_t lift_speed = 50;                  ///< Lift speed (0-100%)
    uint8_t force_position_mix = 50;          ///< Force/position mix ratio (0-100%)
    uint8_t upper_limit = 100;                ///< Upper position limit (0-100%)
    
    // Multi-valve control
    int8_t valve_flows[4] = {0, 0, 0, 0};     ///< Flow control for 4 valves (-100 to 100)
    bool valve_lock = false;                  ///< Lock all valves
    
    // Control flags
    bool hydraulic_enable = false;            ///< Enable hydraulic control
    bool emergency_stop = false;              ///< Emergency stop flag
    
    // Timestamp
    uint64_t timestamp_ms = 0;                ///< Command timestamp
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
    
    // Hydraulic sensor data
    HydraulicState hydraulic_state;              ///< Current hydraulic system state
    
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
