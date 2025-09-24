// Copyright 2025 Manus AI

#ifndef VCU_AD_INTERFACE_AD_INTERFACE_H_
#define VCU_AD_INTERFACE_AD_INTERFACE_H_

#include "vcu/common/vcu_types.h"
#include "vcu/common/vcu_data_types.h"

namespace vcu {
namespace ad_interface {

// Placeholder for ROS2/DDS message types
struct AdCommand {
    common::DriveMode target_drive_mode;
    float target_speed_mps;
    float target_torque_nm;
};

struct VehicleState {
    common::DriveMode current_drive_mode;
    float current_speed_mps;
    float current_engine_rpm;
    float current_load_percent;
    float current_transmission_ratio;
};

class AdInterface {
public:
    virtual ~AdInterface() = default;

    virtual bool initialize() = 0;
    virtual void shutdown() = 0;

    virtual bool get_command(AdCommand& cmd) = 0;
    virtual void publish_state(const VehicleState& state) = 0;
};

} // namespace ad_interface
} // namespace vcu

#endif // VCU_AD_INTERFACE_AD_INTERFACE_H_
