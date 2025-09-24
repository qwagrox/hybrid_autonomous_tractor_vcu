// Copyright 2025 Manus AI

#ifndef VCU_HAL_HARDWARE_ABSTRACTION_LAYER_H_
#define VCU_HAL_HARDWARE_ABSTRACTION_LAYER_H_

#include <memory>
#include "vcu/can/can_interface.h"

namespace vcu {
namespace hal {

class HardwareAbstractionLayer {
public:
    virtual ~HardwareAbstractionLayer() = default;

    virtual bool initialize() = 0;
    virtual void shutdown() = 0;

    virtual std::shared_ptr<can::ICanInterface> get_can_interface(const std::string& can_bus_name) = 0;
};

} // namespace hal
} // namespace vcu

#endif // VCU_HAL_HARDWARE_ABSTRACTION_LAYER_H_
