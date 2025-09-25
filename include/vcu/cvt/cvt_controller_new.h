#ifndef CVT_CONTROLLER_NEW_H
#define CVT_CONTROLLER_NEW_H

#include "vcu/common/vcu_types.h"
#include "vcu/common/vcu_data_types.h"
#include "vcu/cvt/cvt_strategy.h"
#include "vcu/cvt/cvt_config.h"
#include "vcu/can/can_interface.h"
#include <memory>

namespace vcu {
namespace cvt {

/**
 * @class CvtController
 * @brief Manages the Continuously Variable Transmission (CVT) of the vehicle using strategy pattern.
 *
 * This class is responsible for calculating the optimal transmission ratio based on
 * the vehicle's current state, perception data, and prediction results. It delegates
 * the actual CVT communication to a strategy object based on the CVT manufacturer.
 */
class CvtController {
public:
    /**
     * @brief Constructs a new CvtController object.
     *
     * @param can_interface Reference to the CAN interface for communication.
     * @param config CVT configuration parameters.
     */
    CvtController(can::CanInterface& can_interface, const CvtConfig& config = CvtConfig{});

    /**
     * @brief Destroys the CvtController object.
     */
    ~CvtController() = default;

    /**
     * @brief Initializes the CVT controller.
     *
     * This method must be called before using the controller.
     * It initializes the CVT strategy and performs necessary setup.
     *
     * @return True if initialization was successful, false otherwise.
     */
    bool init();

    /**
     * @brief Sets the current driving mode.
     *
     * @param mode The driving mode to set.
     */
    void set_drive_mode(const common::DriveMode mode);

    /**
     * @brief Updates the CVT controller with new data.
     *
     * This method should be called periodically to update the controller's state
     * and calculate the new target ratio.
     *
     * @param perception The current perception data.
     * @param prediction The current prediction result.
     */
    void update(const common::PerceptionData& perception, const common::PredictionResult& prediction);

    /**
     * @brief Gets the current state of the CVT.
     *
     * @return The current CVT state.
     */
    common::CvtState get_current_state() const;

    /**
     * @brief Sets the CVT manufacturer and recreates the strategy.
     *
     * @param manufacturer The new CVT manufacturer.
     * @return True if the strategy was successfully changed, false otherwise.
     */
    bool set_cvt_manufacturer(common::CvtManufacturer manufacturer);

    /**
     * @brief Gets the current CVT configuration.
     *
     * @return The current CVT configuration.
     */
    const CvtConfig& get_config() const;

    /**
     * @brief Updates the CVT configuration.
     *
     * @param config The new CVT configuration.
     * @return True if the configuration was successfully updated, false otherwise.
     */
    bool update_config(const CvtConfig& config);

    /**
     * @brief Checks if the CVT controller is initialized and ready.
     *
     * @return True if ready, false otherwise.
     */
    bool is_ready() const;

private:
    /**
     * @brief Calculates the optimal transmission ratio.
     *
     * @param perception The current perception data.
     * @param prediction The current prediction result.
     * @return The optimal transmission ratio.
     */
    float calculate_optimal_ratio(const common::PerceptionData& perception, const common::PredictionResult& prediction);

    /**
     * @brief Calculates the optimal ratio for plowing mode.
     *
     * @param perception The current perception data.
     * @return The optimal transmission ratio for plowing.
     */
    float calculate_plowing_ratio(const common::PerceptionData& perception);

    /**
     * @brief Calculates the optimal ratio for seeding mode.
     *
     * @param perception The current perception data.
     * @return The optimal transmission ratio for seeding.
     */
    float calculate_seeding_ratio(const common::PerceptionData& perception);

    /**
     * @brief Calculates the optimal ratio for transport mode.
     *
     * @param perception The current perception data.
     * @return The optimal transmission ratio for transport.
     */
    float calculate_transport_ratio(const common::PerceptionData& perception);

    /**
     * @brief Creates a CVT strategy based on the current configuration.
     *
     * @return True if the strategy was successfully created, false otherwise.
     */
    bool create_strategy();

    /**
     * @brief Validates the safety conditions before operation.
     *
     * @param perception The current perception data.
     * @return True if safe to operate, false otherwise.
     */
    bool validate_safety_conditions(const common::PerceptionData& perception) const;

    // Core components
    can::CanInterface& can_interface_;
    std::unique_ptr<CvtStrategy> cvt_strategy_;
    CvtConfig config_;

    // State management
    common::DriveMode drive_mode_;
    bool is_initialized_;
    bool is_ready_;

    // Safety and monitoring
    uint64_t last_update_time_;
    uint32_t safety_violation_count_;
    static constexpr uint32_t MAX_SAFETY_VIOLATIONS = 3;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_CONTROLLER_NEW_H
