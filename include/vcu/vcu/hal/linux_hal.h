// Copyright 2025 Manus AI

#ifndef VCU_HAL_LINUX_HAL_H_
#define VCU_HAL_LINUX_HAL_H_

#include "vcu/hal/hardware_abstraction_layer.h"
#include <map>
#include <string>

namespace vcu {
namespace hal {

class LinuxHal : public HardwareAbstractionLayer {
public:
    LinuxHal();
    ~LinuxHal() override;

    bool initialize() override;
    void shutdown() override;

    std::shared_ptr<can::ICanInterface> get_can_interface(const std::string& can_bus_name) override;

private:
    bool check_system_requirements();
    bool setup_can_interfaces();
    
    std::map<std::string, std::shared_ptr<can::ICanInterface>> can_interfaces_;
    bool initialized_;
};

} // namespace hal
} // namespace vcu

#endif // VCU_HAL_LINUX_HAL_H_
