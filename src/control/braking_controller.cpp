// src/control/braking_controller.cpp
#include "control/braking_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace VCUCore {

BrakingController::BrakingController() 
    : maxBrakingForce_(50000.0f), currentBrakingForce_(0.0f), 
      isEmergencyBraking_(false), isInitialized_(false) {
}

bool BrakingController::initialize() {
    std::cout << "Initializing braking controller..." << std::endl;
    
    currentBrakingForce_ = 0.0f;
    isEmergencyBraking_ = false;
    
    isInitialized_ = true;
    std::cout << "Braking controller initialized successfully" << std::endl;
    return true;
}

void BrakingController::shutdown() {
    if (!isInitialized_) return;
    
    currentBrakingForce_ = 0.0f;
    isEmergencyBraking_ = false;
    
    isInitialized_ = false;
    std::cout << "Braking controller shutdown" << std::endl;
}

bool BrakingController::applyBraking(float force) {
    if (!isInitialized_) {
        return false;
    }
    
    // 限制制动力在有效范围内
    float limitedForce = std::max(0.0f, std::min(force, maxBrakingForce_));
    currentBrakingForce_ = limitedForce;
    
    std::cout << "Applying braking force: " << currentBrakingForce_ << " N" << std::endl;
    return true;
}

bool BrakingController::releaseBraking() {
    if (!isInitialized_) {
        return false;
    }
    
    currentBrakingForce_ = 0.0f;
    isEmergencyBraking_ = false;
    
    std::cout << "Braking released" << std::endl;
    return true;
}

bool BrakingController::emergencyBraking() {
    if (!isInitialized_) {
        return false;
    }
    
    isEmergencyBraking_ = true;
    currentBrakingForce_ = maxBrakingForce_;
    
    std::cout << "Emergency braking activated!" << std::endl;
    return true;
}

float BrakingController::getCurrentBrakingForce() const {
    return currentBrakingForce_;
}

bool BrakingController::isEmergencyBraking() const {
    return isEmergencyBraking_;
}

bool BrakingController::checkBrakingSystem() const {
    std::cout << "Checking braking system..." << std::endl;
    
    // 检查制动力范围
    if (maxBrakingForce_ <= 0) {
        std::cerr << "Invalid maximum braking force" << std::endl;
        return false;
    }
    
    std::cout << "Braking system check passed" << std::endl;
    return true;
}

void BrakingController::setMaxBrakingForce(float maxForce) {
    if (maxForce > 0) {
        maxBrakingForce_ = maxForce;
        
        // 确保当前制动力不超过新的最大值
        if (currentBrakingForce_ > maxBrakingForce_) {
            currentBrakingForce_ = maxBrakingForce_;
        }
    }
}

} // namespace VCUCore
