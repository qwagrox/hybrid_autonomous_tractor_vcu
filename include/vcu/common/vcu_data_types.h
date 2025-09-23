#ifndef VCU_DATA_TYPES_H
#define VCU_DATA_TYPES_H

namespace vcu {
namespace common {

/**
 * @brief Represents perception data from vehicle sensors.
 */
struct PerceptionData {
    float vehicle_speed_mps = 0.0f;      // Vehicle speed in meters per second
    float engine_speed_rpm = 0.0f;       // Engine speed in revolutions per minute
    float engine_load_percent = 0.0f;    // Engine load in percentage
    float accelerator_pedal_percent = 0.0f; // Accelerator pedal position in percentage
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
