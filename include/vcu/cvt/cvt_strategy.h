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
 * vendor-specific CVT control logic and hydraulic control capabilities.
 */
class CvtStrategy {
public:
    virtual ~CvtStrategy() = default;

    // ========== CVT Control Interface ==========
    
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

    // ========== Hydraulic Control Interface ==========
    
    /**
     * @brief Enables or disables hydraulic control.
     *
     * @param enable True to enable hydraulic control, false to disable.
     */
    virtual void enable_hydraulic_control(bool enable) = 0;

    /**
     * @brief Sets the lift position.
     *
     * @param position Target lift position (0-100%).
     */
    virtual void set_lift_position(float position) = 0;

    /**
     * @brief Sets the lift mode.
     *
     * @param mode The lift mode to set.
     */
    virtual void set_lift_mode(common::LiftMode mode) = 0;

    /**
     * @brief Executes a lift action.
     *
     * @param action The lift action to execute.
     */
    virtual void execute_lift_action(common::LiftAction action) = 0;

    /**
     * @brief Sets the multi-valve flow control.
     *
     * @param valve_id Valve ID (0-3 for 4 valves).
     * @param flow_percent Flow percentage (-100 to 100, negative for retract, positive for extend).
     */
    virtual void set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent) = 0;

    /**
     * @brief Locks or unlocks all multi-valves.
     *
     * @param lock True to lock valves, false to unlock.
     */
    virtual void set_multi_valve_lock(bool lock) = 0;

    /**
     * @brief Gets the current hydraulic system state.
     *
     * @return The current hydraulic state.
     */
    virtual common::HydraulicState get_hydraulic_state() const = 0;

    /**
     * @brief Executes a hydraulic command.
     *
     * @param command The hydraulic command to execute.
     */
    virtual void execute_hydraulic_command(const common::HydraulicCommand& command) = 0;

    /**
     * @brief Checks if hydraulic system is ready for operation.
     *
     * @return True if hydraulic system is ready, false otherwise.
     */
    virtual bool is_hydraulic_ready() const = 0;

    /**
     * @brief Gets hydraulic system error flags.
     *
     * @return Error flags bitfield.
     */
    virtual uint32_t get_hydraulic_errors() const = 0;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_STRATEGY_H
