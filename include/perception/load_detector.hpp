#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

class LoadDetector {
private:
    float currentLoad_;
    float maxLoad_;
    float loadThreshold_;
    uint64_t lastUpdateTime_;
    
public:
    LoadDetector();
    ~LoadDetector() = default;
    
    void updateLoadMeasurement(const TractorVehicleState& state);
    float getCurrentLoad() const;
    float getMaxLoad() const;
    bool isOverloaded() const;
    void setLoadThreshold(float threshold);
    void reset();
};

} // namespace VCUCore
