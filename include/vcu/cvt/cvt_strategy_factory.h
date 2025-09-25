#ifndef CVT_STRATEGY_FACTORY_H
#define CVT_STRATEGY_FACTORY_H

#include <memory>
#include "vcu/cvt/cvt_strategy.h"
#include "vcu/can/can_interface.h"
#include "vcu/common/vcu_types.h"

namespace vcu {
namespace cvt {

/**
 * @class CvtStrategyFactory
 * @brief Factory class for creating CVT control strategies.
 *
 * This factory creates appropriate CVT strategy instances based on the
 * manufacturer type. It supports dynamic strategy creation and provides
 * a unified interface for different CVT vendors.
 */
class CvtStrategyFactory {
public:
    /**
     * @brief Creates a CVT strategy based on manufacturer type.
     *
     * @param manufacturer The CVT manufacturer type.
     * @param can_interface Reference to the CAN interface.
     * @return A unique pointer to the created CVT strategy.
     * @throws std::invalid_argument if the manufacturer is not supported.
     */
    static std::unique_ptr<CvtStrategy> create_strategy(
        common::CvtManufacturer manufacturer, 
        can::CanInterface& can_interface);

    /**
     * @brief Creates an HMCVT_Vendor1 strategy directly.
     *
     * This is a convenience method for creating HMCVT_Vendor1 strategies
     * without specifying the manufacturer enum.
     *
     * @param can_interface Reference to the CAN interface.
     * @return A unique pointer to the HMCVT_Vendor1 strategy.
     */
    static std::unique_ptr<CvtStrategy> create_hmcvt_vendor1_strategy(
        can::CanInterface& can_interface);

private:
    // Private constructor to prevent instantiation
    CvtStrategyFactory() = default;
};

} // namespace cvt
} // namespace vcu

#endif // CVT_STRATEGY_FACTORY_H

