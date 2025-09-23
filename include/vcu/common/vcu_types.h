#ifndef VCU_TYPES_H
#define VCU_TYPES_H

namespace vcu {
namespace common {

/**
 * @brief Defines the driving mode of the vehicle.
 */
enum class DriveMode {
    MANUAL,      // Manual mode
    PLOWING,     // Plowing mode
    SEEDING,     // Seeding mode
    TRANSPORT    // Transport mode
};

/**
 * @brief Defines the manufacturer of the CVT.
 */
enum class CvtManufacturer {
    UNKNOWN,    // Unknown manufacturer
    BOSCH,      // Bosch
    ZF          // ZF
};

/**
 * @brief Represents the state of the CVT.
 */
struct CvtState {
    float current_ratio = 1.0f;       // Current transmission ratio
    float target_ratio = 1.0f;        // Target transmission ratio
    bool is_shifting = false;         // True if the CVT is currently shifting
};

}
}

#endif // VCU_TYPES_H
