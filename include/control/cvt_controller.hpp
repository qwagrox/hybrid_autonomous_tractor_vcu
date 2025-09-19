// include/control/cvt_controller.hpp
#pragma once
#include "vcu_core_types.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <map>

namespace VCUCore {

struct CVTManufacturerParams {
    float minRatio;
    float maxRatio;
    float defaultRatio;
    std::string name;
};

class CVTController {
public:
    CVTController(uint32_t history_size = 100);

    void setDriveMode(DriveMode mode);
    void update(const PerceptionData& perception, const PredictionResult& prediction);
    CVTState getCurrentState() const;
    bool isShifting() const;

private:
    CVTState currentState_;
    DriveMode currentDriveMode_;
    CVTManufacturer currentManufacturer_;
    CVTManufacturerParams manufacturerParams_;
    std::deque<float> ratioHistory_;
    uint32_t historySize_;

    float calculateOptimalRatio(const PerceptionData& perception, const PredictionResult& prediction);
    void adaptToManufacturer(CVTManufacturer manufacturer);
    void checkRatioLimits();

    float calculatePlowingRatio(const PerceptionData& perception) const;
    float calculateSeedingRatio(const PerceptionData& perception) const;
    float calculateTransportRatio(const PerceptionData& perception) const;
    float calculateBaseRatio(const PerceptionData& perception) const;

    float optimizeForEfficiency(float baseRatio, const PerceptionData& perception) const;
    float optimizeForTraction(float baseRatio, const PerceptionData& perception) const;
    float optimizeForComfort(float baseRatio, const PerceptionData& perception) const;

    float calculateWheelSlip(float ratio, const PerceptionData& perception) const;
};

} // namespace VCUCore

