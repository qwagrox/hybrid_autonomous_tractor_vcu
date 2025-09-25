#include "vcu/cvt/cvt_strategy_factory.h"
#include "vcu/cvt/hmcvt_vendor1_strategy.h"
#include <stdexcept>

namespace vcu {
namespace cvt {

std::unique_ptr<CvtStrategy> CvtStrategyFactory::create_strategy(
    common::CvtManufacturer manufacturer, 
    can::ICanInterface& can_interface) {
    
    switch (manufacturer) {
        case common::CvtManufacturer::HMCVT_VENDOR1:
            return std::make_unique<HMCVT_Vendor1_Strategy>(can_interface);
            
        case common::CvtManufacturer::UNKNOWN:
            // 对于未知制造商，返回默认的HMCVT_Vendor1策略
            return std::make_unique<HMCVT_Vendor1_Strategy>(can_interface);
            
        case common::CvtManufacturer::BOSCH:
            // 目前使用HMCVT_Vendor1作为Bosch的实现
            // 未来可以创建专门的BoschCvtStrategy
            return std::make_unique<HMCVT_Vendor1_Strategy>(can_interface);
            
        case common::CvtManufacturer::ZF:
            // 目前使用HMCVT_Vendor1作为ZF的实现
            // 未来可以创建专门的ZfCvtStrategy
            return std::make_unique<HMCVT_Vendor1_Strategy>(can_interface);
            
        default:
            throw std::invalid_argument("Unsupported CVT manufacturer");
    }
}

std::unique_ptr<CvtStrategy> CvtStrategyFactory::create_hmcvt_vendor1_strategy(
    can::ICanInterface& can_interface) {
    return std::make_unique<HMCVT_Vendor1_Strategy>(can_interface);
}

} // namespace cvt
} // namespace vcu
