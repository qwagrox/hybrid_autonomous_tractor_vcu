// include/models/motor_model.hpp
#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

class MotorModel {
public:
    MotorState getMotorState() const;
    float getMaxPower() const { return 0.0f; }
};

} // namespace VCUCore

