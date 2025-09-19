// include/control/braking_controller.hpp
#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

/**
 * @class BrakingController
 * @brief 制动控制器，负责管理拖拉机的制动系统
 */
class BrakingController {
private:
    float maxBrakingForce_;      // 最大制动力 (N)
    float currentBrakingForce_;  // 当前制动力 (N)
    bool isEmergencyBraking_;    // 是否处于紧急制动状态
    bool isInitialized_;         // 是否已初始化

public:
    BrakingController();
    ~BrakingController() = default;

    bool initialize();
    void shutdown();
    
    bool applyBraking(float force);
    bool releaseBraking();
    bool emergencyBraking();
    
    float getCurrentBrakingForce() const;
    bool isEmergencyBraking() const;
    bool checkBrakingSystem() const;
    
    void setMaxBrakingForce(float maxForce);
};

} // namespace VCUCore
