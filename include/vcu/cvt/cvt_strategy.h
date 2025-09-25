#ifndef CVT_STRATEGY_H
#define CVT_STRATEGY_H

#include "vcu/common/vcu_data_types.h"
#include "vcu/common/vcu_types.h"

namespace vcu {
namespace cvt {

/**
 * @class CvtStrategy
 * @brief Abstract base class for CVT control strategies.
 *
 * This class defines the common interface for all CVT control strategies.
 * Each concrete strategy must implement these methods to provide
 * vendor-specific CVT control logic.
 */
class CvtStrategy {
public:
    virtual ~CvtStrategy() = default;

    /**
     * @brief Initializes the CVT controller.
     */
    virtual void init() = 0;

    /**
     * @brief Updates the CVT controller with a new target ratio.
     *
     * @param target_ratio The target transmission ratio.
     */
    virtual void set_target_ratio(float target_ratio) = 0;

    /**
     * @brief Updates the CVT controller based on the latest vehicle data.
     *
     * @param perception_data The current perception data from sensors.
     */
    virtual void update(const common::PerceptionData& perception_data) = 0;

    /**
     * @brief Gets the current state of the CVT.
     *
     * @return The current CVT state.
     */
    virtual common::CvtState get_current_state() const = 0;

    /**
     * @brief Sets the driving mode.
     *
     * @param mode The driving mode to set.
     */
    virtual void set_drive_mode(common::DriveMode mode) = 0;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_STRATEGY_H

