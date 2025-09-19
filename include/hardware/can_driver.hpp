// include/hardware/can_driver.hpp
#pragma once
#include <string>
#include <vector>
#include <linux/can.h>

namespace VCUCore {

class CANDriver {
public:
    bool initialize(const std::string& interface, int bitrate);
    bool shutdown();
    bool sendFrame(const struct can_frame& frame);
};

} // namespace VCUCore

