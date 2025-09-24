#ifndef VCU_ADAS_INTERFACE_FACTORY_H
#define VCU_ADAS_INTERFACE_FACTORY_H

#include "adas_can_interface.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include <memory>
#include <string>

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
    std::shared_ptr<can::ICanInterface> can_interface);

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
    std::shared_ptr<can::ICanInterface> can_interface);

} // namespace adas_interface
} // namespace vcu

#endif // VCU_ADAS_INTERFACE_FACTORY_H
