#ifndef VCU_TYPES_H
#define VCU_TYPES_H

namespace vcu {
namespace common {

/**
 * @brief Defines the driving mode of the vehicle.
 */
enum class DriveMode {
    MANUAL,         // Manual mode
    PLOWING,        // Plowing mode  
    SEEDING,        // Seeding mode
    TRANSPORT       // Transport mode
};

/**
 * @brief Defines the manufacturer of the CVT.
 */
enum class CvtManufacturer {
    UNKNOWN,        // Unknown manufacturer
    HMCVT_VENDOR1,  // HMCVT Vendor1 manufacturer
    BOSCH,          // Bosch
    ZF              // ZF
};

/**
 * @brief Represents the state of the CVT.
 */
struct CvtState {
    float current_ratio = 1.0f;     // Current transmission ratio
    float target_ratio = 1.0f;      // Target transmission ratio
    bool is_shifting = false;       // True if the CVT is currently shifting
};

// ========== 新增液压控制相关定义 ==========

/**
 * @enum TerrainType
 * @brief Represents different terrain types for adaptive control.
 */
enum class TerrainType {
    FLAT = 0,           ///< Flat terrain (roads, flat fields)
    GENTLE_SLOPE = 1,   ///< Gentle slope terrain (slight inclines)
    HILL = 2,           ///< Hilly terrain (moderate inclines, soft soil)
    STEEP_SLOPE = 3,    ///< Steep slope terrain (steep inclines, rocky ground)
    SMOOTH = 0,         ///< Alias for FLAT (backward compatibility)
    MODERATE = 2,       ///< Alias for HILLY (backward compatibility)
    ROUGH = 3           ///< Alias for STEEP_SLOPE (backward compatibility)
};

/**
 * @enum LiftMode
 * @brief Defines the lifting mode for hydraulic control.
 */
enum class LiftMode {
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
enum class LiftAction {
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
enum class MultiValveState {
    NEUTRAL = 1,        ///< Neutral position
    EXTEND = 64,        ///< Extending
    RETRACT = 16,       ///< Retracting
    FLOAT = 4           ///< Floating
};

/**
 * @enum HydraulicCommandType
 * @brief Defines the types of hydraulic commands.
 */
enum class HydraulicCommandType {
    LIFT_UP = 0,        ///< Lift up command
    LIFT_DOWN = 1,      ///< Lift down command
    LIFT_STOP = 2,      ///< Stop lift command
    SET_VALVE_FLOW = 3, ///< Set valve flow command
    EMERGENCY_STOP = 4  ///< Emergency stop command
};

/**
 * @struct HydraulicCommand
 * @brief Represents a hydraulic control command.
 */
struct HydraulicCommand {
    HydraulicCommandType type;      ///< Command type
    uint8_t valve_id = 0;          ///< Valve ID (0-3 for multi-valves)
    int8_t flow_percent = 0;       ///< Flow percentage (-100 to 100)
    float position = 0.0f;         ///< Target position (0-100%)
    bool valve_lock = false;       ///< Valve lock state
    uint32_t timestamp = 0;        ///< Command timestamp
};

/**
 * @struct HydraulicState
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
    
    // System status
    float pressure = 0.0f;              ///< System pressure (bar)
    float temperature = 0.0f;           ///< System temperature (°C)
    bool is_ready = false;              ///< System ready status
    uint32_t error_code = 0;            ///< Error code
};

/**
 * @struct PerceptionData
 * @brief Represents sensor perception data.
 */
struct PerceptionData {
    float vehicle_speed = 0.0f;         ///< Vehicle speed (km/h)
    float engine_speed = 0.0f;          ///< Engine speed (RPM)
    float load_factor = 0.0f;           ///< Current load factor (0-1)
    TerrainType terrain = TerrainType::FLAT; ///< Current terrain type
    bool obstacle_detected = false;      ///< Obstacle detection status
    float fuel_level = 0.0f;            ///< Fuel level (0-100%)
    uint32_t timestamp = 0;             ///< Data timestamp
    
    // 为了兼容现有代码，添加别名字段
    float vehicle_speed_mps = 0.0f;     ///< Vehicle speed (m/s) - 兼容字段
    float accelerator_pedal_percent = 0.0f; ///< Accelerator pedal position (%) - 兼容字段
    float engine_load_percent = 0.0f;   ///< Engine load percentage (%) - 兼容字段
    TerrainType terrain_type = TerrainType::FLAT; ///< Terrain type - 兼容字段
};

/**
 * @struct CvtConfig
 * @brief CVT configuration parameters.
 */
struct CvtConfig {
    float min_ratio = 0.5f;             ///< Minimum transmission ratio
    float max_ratio = 2.0f;             ///< Maximum transmission ratio
    float default_ratio = 1.0f;         ///< Default transmission ratio
    uint16_t status_period_ms = 100;    ///< Status message period (ms)
    uint16_t control_period_ms = 10;    ///< Control message period (ms)
    bool enable_auto_mode = true;       ///< Enable automatic mode
    bool enable_diagnostics = true;     ///< Enable diagnostics
};

/**
 * @struct VehicleState
 * @brief Overall vehicle state information.
 */
struct VehicleState {
    CvtState cvt_state;                 ///< CVT transmission state
    HydraulicState hydraulic_state;     ///< Hydraulic system state
    PerceptionData perception_data;     ///< Sensor perception data
    bool emergency_stop = false;        ///< Emergency stop status
    uint32_t system_timestamp = 0;      ///< System timestamp
};

} // namespace common
} // namespace vcu

#endif // VCU_TYPES_H
