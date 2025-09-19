// src/control/implement_control_manager.cpp
#include "control/implement_control_manager.hpp"
#include <iostream>

namespace VCUCore {

ImplementControlManager::ImplementControlManager() : isInitialized_(false) {
}

bool ImplementControlManager::initialize() {
    std::cout << "Initializing implement control manager..." << std::endl;
    isInitialized_ = true;
    return true;
}

void ImplementControlManager::shutdown() {
    if (!isInitialized_) return;
    std::cout << "Shutting down implement control manager..." << std::endl;
    controllers_.clear();
    controllerMap_.clear();
    isInitialized_ = false;
}

bool ImplementControlManager::addController(std::unique_ptr<IImplementController> controller) {
    if (!isInitialized_ || !controller) return false;
    
    std::string type = controller->getType();
    if (controllerMap_.find(type) != controllerMap_.end()) {
        std::cerr << "Controller type already exists: " << type << std::endl;
        return false;
    }
    
    controllers_.push_back(std::move(controller));
    controllerMap_[type] = controllers_.size() - 1;
    std::cout << "Added implement controller: " << type << std::endl;
    return true;
}

bool ImplementControlManager::removeController(const std::string& type) {
    if (!isInitialized_ || controllerMap_.find(type) == controllerMap_.end()) {
        return false;
    }
    
    size_t index = controllerMap_[type];
    controllers_.erase(controllers_.begin() + index);
    controllerMap_.erase(type);
    
    // 更新映射
    for (auto const& [key, val] : controllerMap_) {
        if (val > index) {
            controllerMap_[key] = val - 1;
        }
    }
    
    std::cout << "Removed implement controller: " << type << std::endl;
    return true;
}

IImplementController* ImplementControlManager::getController(const std::string& type) {
    if (!isInitialized_ || controllerMap_.find(type) == controllerMap_.end()) {
        return nullptr;
    }
    return controllers_[controllerMap_[type]].get();
}

std::vector<std::string> ImplementControlManager::getAvailableTypes() const {
    std::vector<std::string> types;
    for (auto const& [key, val] : controllerMap_) {
        (void)val; // 避免未使用参数警告
        types.push_back(key);
    }
    return types;
}

bool ImplementControlManager::startAll() {
    if (!isInitialized_) return false;
    for (auto& controller : controllers_) {
        controller->start();
    }
    return true;
}

bool ImplementControlManager::stopAll() {
    if (!isInitialized_) return false;
    for (auto& controller : controllers_) {
        controller->stop();
    }
    return true;
}

bool ImplementControlManager::setWorkParameter(const std::string& type, const std::string& key, double value) {
    IImplementController* controller = getController(type);
    if (controller) {
        return controller->setWorkParameter(key, value);
    }
    return false;
}

std::vector<ImplementStatus> ImplementControlManager::getAllStatus() const {
    std::vector<ImplementStatus> statuses;
    if (!isInitialized_) return statuses;
    for (const auto& controller : controllers_) {
        statuses.push_back(controller->getStatus());
    }
    return statuses;
}

std::vector<DiagnosticReport> ImplementControlManager::runAllDiagnostics() {
    std::vector<DiagnosticReport> reports;
    if (!isInitialized_) return reports;
    for (auto& controller : controllers_) {
        reports.push_back(controller->runDiagnostics());
    }
    return reports;
}

void ImplementControlManager::updateAll(double dt) {
    if (!isInitialized_) return;
    for (auto& controller : controllers_) {
        controller->update(dt);
    }
}

} // namespace VCUCore

