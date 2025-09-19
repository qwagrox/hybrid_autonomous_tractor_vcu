// include/hardware/gpio_driver.hpp
#pragma once
#include <cstdint>

namespace VCUCore {

class GPIODriver {
public:
    bool initialize();
    bool shutdown();
    bool setPinState(uint8_t pin, bool state);
};

} // namespace VCUCore

