// include/models/engine_model.hpp
#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

class EngineModel {
public:
    EngineState getEngineState() const;
    float getMaxPower() const { return 0.0f; }
};

} // namespace VCUCore

