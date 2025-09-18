#include "../../include/control/implement_control_manager.hpp"
#include <stdexcept>

ImplementControlManager::ImplementControlManager(std::shared_ptr<ISOBUSAdapter> isobus_adapter)
    : isobus_adapter_(isobus_adapter), active_driver_(nullptr) {}

void ImplementControlManager::registerDriver(std::shared_ptr<IImplementController> driver) {
    if (driver) {
        std::string type = driver->getType();
        if (drivers_.find(type) == drivers_.end()) {
            drivers_[type] = driver;
        }
    }
}

bool ImplementControlManager::selectImplement(const std::string& type) {
    if (drivers_.find(type) != drivers_.end()) {
        active_driver_ = drivers_[type];
        // TODO: Add logic to load configuration and initialize the driver
        return true;
    }
    return false;
}

bool ImplementControlManager::startWork() {
    if (active_driver_) {
        return active_driver_->start();
    }
    return false;
}

bool ImplementControlManager::stopWork() {
    if (active_driver_) {
        return active_driver_->stop();
    }
    return false;
}

bool ImplementControlManager::setWorkParameter(const std::string& key, double value) {
    if (active_driver_) {
        return active_driver_->setWorkParameter(key, value);
    }
    return false;
}

ImplementStatus ImplementControlManager::getStatus() const {
    if (active_driver_) {
        return active_driver_->getStatus();
    }
    // Return a default/invalid status if no driver is active
    return ImplementStatus{}; 
}

void ImplementControlManager::update(double dt) {
    if (active_driver_) {
        active_driver_->update(dt);
    }
}

