// include/control/steering_controller.hpp
#pragma once
#include "vcu_core_types.hpp"

namespace VCUCore {

/**
 * @class SteeringController
 * @brief 转向控制器，负责管理拖拉机的转向系统
 */
class SteeringController {
private:
    float maxSteeringAngle_;     // 最大转向角 (度)
    float currentSteeringAngle_; // 当前转向角 (度)
    float targetSteeringAngle_;  // 目标转向角 (度)
    bool isInitialized_;         // 是否已初始化

public:
    SteeringController();
    ~SteeringController() = default;

    bool initialize();
    void shutdown();
    
    bool setSteeringAngle(float angle);
    void update();
    
    float getCurrentSteeringAngle() const;
    float getTargetSteeringAngle() const;
    bool checkSteeringSystem() const;
    
    void setMaxSteeringAngle(float maxAngle);
};

} // namespace VCUCore
