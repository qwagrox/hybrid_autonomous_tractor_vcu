#include "vcu/adas_interface/adas_can_interface.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include <memory>

namespace vcu {
namespace adas_interface {

/**
 * @brief 创建智驾CAN接口实例
 * @param platform 平台接口指针
 * @param can_interface CAN接口
 * @return 智驾CAN接口实例
 */
std::unique_ptr<AdasCanInterface> create_adas_can_interface(
    PlatformInterface* platform,
    std::shared_ptr<can::ICanInterface> can_interface) {
    
    if (!platform || !can_interface) {
        return nullptr;
    }
    
    return std::make_unique<AdasCanInterface>(platform, can_interface);
}

/**
 * @brief 创建智驾接口 (通用工厂函数)
 * @param interface_type 接口类型 (目前只支持CAN)
 * @param platform 平台接口指针
 * @param can_interface CAN接口
 * @return 智驾接口实例
 */
std::unique_ptr<AdasCanInterface> create_adas_interface(
    const std::string& interface_type,
    PlatformInterface* platform,
    std::shared_ptr<can::ICanInterface> can_interface) {
    
    // 目前只支持CAN接口类型
    if (interface_type == "can" || interface_type == "CAN") {
        return create_adas_can_interface(platform, can_interface);
    }
    
    // 不支持的接口类型
    return nullptr;
}

} // namespace adas_interface
} // namespace vcu
