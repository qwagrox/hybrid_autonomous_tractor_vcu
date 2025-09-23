#ifndef CVT_CONTROLLER_H
#define CVT_CONTROLLER_H

#include "vcu/common/vcu_types.h"
#include "vcu/common/vcu_data_types.h"

namespace vcu {
namespace cvt {

/**
 * @class CvtController
 * @brief Manages the Continuously Variable Transmission (CVT) of the vehicle.
 *
 * This class is responsible for calculating the optimal transmission ratio based on
 * the vehicle's current state, perception data, and prediction results. It also
 * handles the communication with the CVT hardware.
 */
class CvtController {
public:
    /**
     * @brief Constructs a new CvtController object.
     */
    CvtController();

    /**
     * @brief Destroys the CvtController object.
     */
    ~CvtController() = default;

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

    common::DriveMode drive_mode_;
    common::CvtState cvt_state_;
    common::CvtManufacturer cvt_manufacturer_;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_CONTROLLER_H
