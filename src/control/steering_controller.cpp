// src/control/steering_controller.cpp
#include "control/steering_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCUCore {

SteeringController::SteeringController() 
    : maxSteeringAngle_(45.0f), currentSteeringAngle_(0.0f), 
      targetSteeringAngle_(0.0f), isInitialized_(false) {
}

bool SteeringController::initialize() {
    std::cout << "Initializing steering controller..." << std::endl;
    
    currentSteeringAngle_ = 0.0f;
    targetSteeringAngle_ = 0.0f;
    
    isInitialized_ = true;
    std::cout << "Steering controller initialized successfully" << std::endl;
    return true;
}

void SteeringController::shutdown() {
    if (!isInitialized_) return;
    
    currentSteeringAngle_ = 0.0f;
    targetSteeringAngle_ = 0.0f;
    
    isInitialized_ = false;
    std::cout << "Steering controller shutdown" << std::endl;
}

bool SteeringController::setSteeringAngle(float angle) {
    if (!isInitialized_) {
        return false;
    }
    
    // 限制转向角在有效范围内
    float limitedAngle = std::max(-maxSteeringAngle_, std::min(angle, maxSteeringAngle_));
    targetSteeringAngle_ = limitedAngle;
    
    std::cout << "Target steering angle set to: " << targetSteeringAngle_ << " degrees" << std::endl;
    return true;
}

void SteeringController::update() {
    if (!isInitialized_) {
        return;
    }
    
    // 简单的转向角跟踪控制
    float error = targetSteeringAngle_ - currentSteeringAngle_;
    float maxChange = 2.0f; // 最大变化率 (度/周期)
    
    if (std::abs(error) > 0.1f) {
        float change = std::max(-maxChange, std::min(error * 0.5f, maxChange));
        currentSteeringAngle_ += change;
        
        // 确保在有效范围内
        currentSteeringAngle_ = std::max(-maxSteeringAngle_, 
                                       std::min(currentSteeringAngle_, maxSteeringAngle_));
    }
}

float SteeringController::getCurrentSteeringAngle() const {
    return currentSteeringAngle_;
}

float SteeringController::getTargetSteeringAngle() const {
    return targetSteeringAngle_;
}

bool SteeringController::checkSteeringSystem() const {
    std::cout << "Checking steering system..." << std::endl;
    
    // 检查转向角范围
    if (maxSteeringAngle_ <= 0) {
        std::cerr << "Invalid maximum steering angle" << std::endl;
        return false;
    }
    
    std::cout << "Steering system check passed" << std::endl;
    return true;
}

void SteeringController::setMaxSteeringAngle(float maxAngle) {
    if (maxAngle > 0) {
        maxSteeringAngle_ = maxAngle;
        
        // 确保当前角度在新范围内
        currentSteeringAngle_ = std::max(-maxSteeringAngle_, 
                                       std::min(currentSteeringAngle_, maxSteeringAngle_));
        targetSteeringAngle_ = std::max(-maxSteeringAngle_, 
                                      std::min(targetSteeringAngle_, maxSteeringAngle_));
    }
}

} // namespace VCUCore
