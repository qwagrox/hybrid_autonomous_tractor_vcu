#ifndef VCU_CORE_HYDRAULIC_SERVICE_H_
#define VCU_CORE_HYDRAULIC_SERVICE_H_

#include <memory>
#include <functional>
#include "vcu/common/vcu_data_types.h"
#include "vcu/cvt/cvt_strategy.h"

namespace vcu {
namespace core {

/**
 * @class HydraulicService
 * @brief Service class for hydraulic system control and monitoring.
 * 
 * This class provides a high-level interface for hydraulic control operations,
 * abstracting the underlying CVT strategy implementation details.
 */
class HydraulicService {
public:
    /**
     * @brief Callback function type for hydraulic state changes.
     */
    using HydraulicStateCallback = std::function<void(const common::HydraulicState&)>;

    /**
     * @brief Callback function type for hydraulic errors.
     */
    using HydraulicErrorCallback = std::function<void(uint32_t error_flags)>;

    /**
     * @brief Constructor.
     * 
     * @param cvt_strategy Shared pointer to the CVT strategy that provides hydraulic control.
     */
    explicit HydraulicService(std::shared_ptr<cvt::CvtStrategy> cvt_strategy);

    /**
     * @brief Destructor.
     */
    ~HydraulicService() = default;

    /**
     * @brief Initializes the hydraulic service.
     * 
     * @return True if initialization was successful, false otherwise.
     */
    bool initialize();

    /**
     * @brief Updates the hydraulic service (should be called periodically).
     * 
     * @param perception_data Current vehicle perception data.
     */
    void update(const common::PerceptionData& perception_data);

    /**
     * @brief Shuts down the hydraulic service.
     */
    void shutdown();

    // ========== Hydraulic Control Interface ==========

    /**
     * @brief Enables or disables hydraulic control.
     * 
     * @param enable True to enable, false to disable.
     * @return True if successful, false otherwise.
     */
    bool enable_hydraulic_control(bool enable);

    /**
     * @brief Executes a hydraulic command.
     * 
     * @param command The hydraulic command to execute.
     * @return True if successful, false otherwise.
     */
    bool execute_hydraulic_command(const common::HydraulicCommand& command);

    /**
     * @brief Sets the lift position.
     * 
     * @param position Target position (0-100%).
     * @return True if successful, false otherwise.
     */
    bool set_lift_position(float position);

    /**
     * @brief Sets the lift mode.
     * 
     * @param mode The lift mode to set.
     * @return True if successful, false otherwise.
     */
    bool set_lift_mode(common::LiftMode mode);

    /**
     * @brief Executes a lift action.
     * 
     * @param action The lift action to execute.
     * @return True if successful, false otherwise.
     */
    bool execute_lift_action(common::LiftAction action);

    /**
     * @brief Controls a multi-valve flow.
     * 
     * @param valve_id Valve ID (0-3).
     * @param flow_percent Flow percentage (-100 to 100).
     * @return True if successful, false otherwise.
     */
    bool set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent);

    /**
     * @brief Locks or unlocks all multi-valves.
     * 
     * @param lock True to lock, false to unlock.
     * @return True if successful, false otherwise.
     */
    bool set_multi_valve_lock(bool lock);

    // ========== Status and Monitoring ==========

    /**
     * @brief Gets the current hydraulic state.
     * 
     * @return The current hydraulic state.
     */
    common::HydraulicState get_hydraulic_state() const;

    /**
     * @brief Checks if the hydraulic system is ready for operation.
     * 
     * @return True if ready, false otherwise.
     */
    bool is_hydraulic_ready() const;

    /**
     * @brief Gets current hydraulic error flags.
     * 
     * @return Error flags bitfield.
     */
    uint32_t get_hydraulic_errors() const;

    /**
     * @brief Checks if hydraulic control is currently enabled.
     * 
     * @return True if enabled, false otherwise.
     */
    bool is_hydraulic_enabled() const;

    // ========== Callback Registration ==========

    /**
     * @brief Registers a callback for hydraulic state changes.
     * 
     * @param callback The callback function.
     */
    void register_state_callback(HydraulicStateCallback callback);

    /**
     * @brief Registers a callback for hydraulic errors.
     * 
     * @param callback The callback function.
     */
    void register_error_callback(HydraulicErrorCallback callback);

    // ========== Safety and Emergency ==========

    /**
     * @brief Executes emergency stop for all hydraulic operations.
     * 
     * @return True if successful, false otherwise.
     */
    bool emergency_stop();

    /**
     * @brief Resets hydraulic error flags after resolving issues.
     * 
     * @return True if successful, false otherwise.
     */
    bool reset_hydraulic_errors();

private:
    /**
     * @brief Validates hydraulic command before execution.
     * 
     * @param command The command to validate.
     * @return True if valid, false otherwise.
     */
    bool validate_hydraulic_command(const common::HydraulicCommand& command) const;

    /**
     * @brief Checks safety conditions for hydraulic operations.
     * 
     * @return True if safe, false otherwise.
     */
    bool check_safety_conditions() const;

    /**
     * @brief Monitors hydraulic system health.
     */
    void monitor_hydraulic_health();

    /**
     * @brief Notifies registered callbacks of state changes.
     * 
     * @param new_state The new hydraulic state.
     */
    void notify_state_callbacks(const common::HydraulicState& new_state);

    /**
     * @brief Notifies registered callbacks of errors.
     * 
     * @param error_flags The error flags.
     */
    void notify_error_callbacks(uint32_t error_flags);

    // Core components
    std::shared_ptr<cvt::CvtStrategy> cvt_strategy_;
    
    // Service state
    bool initialized_;
    bool hydraulic_enabled_;
    common::HydraulicState last_hydraulic_state_;
    uint32_t last_error_flags_;
    
    // Callbacks
    std::vector<HydraulicStateCallback> state_callbacks_;
    std::vector<HydraulicErrorCallback> error_callbacks_;
    
    // Safety and monitoring
    uint64_t last_update_time_;
    uint32_t consecutive_error_count_;
    static constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 5;
    static constexpr uint64_t HEALTH_CHECK_INTERVAL_MS = 1000;

    uint32_t get_current_time_ms() const;
};

} // namespace core
} // namespace vcu

#endif // VCU_CORE_HYDRAULIC_SERVICE_H_
