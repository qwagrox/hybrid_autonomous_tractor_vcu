#include "vcu/sensors/sensor_data_manager.h"
#include <algorithm>

namespace vcu {
namespace sensors {

SensorDataManager::SensorDataManager(std::shared_ptr<can::ICanInterface> can_interface)
    : can_interface_(std::move(can_interface)),
      engine_data_{},
      vehicle_data_{},
      cvt_status_{},
      perception_data_{},
      last_update_time_(std::chrono::steady_clock::now()),
      data_timeout_ms_(1000),
      is_initialized_(false) {
}

SensorDataManager::~SensorDataManager() {
    shutdown();
}

SensorDataResult SensorDataManager::initialize(uint32_t data_timeout_ms) {
    if (is_initialized_) {
        return SensorDataResult::SUCCESS;
    }

    if (!can_interface_ || !can_interface_->is_ready()) {
        return SensorDataResult::ERROR_CAN_COMM;
    }

    data_timeout_ms_ = data_timeout_ms;

    // Set up CAN frame reception callback
    can_interface_->set_receive_callback(
        [this](const can::CanFrame& frame) {
            this->on_can_frame_received(frame);
        }
    );

    // Start receiving CAN frames
    auto result = can_interface_->start_receive();
    if (result != can::CanResult::SUCCESS) {
        return SensorDataResult::ERROR_CAN_COMM;
    }

    is_initialized_ = true;
    return SensorDataResult::SUCCESS;
}

SensorDataResult SensorDataManager::shutdown() {
    if (!is_initialized_) {
        return SensorDataResult::SUCCESS;
    }

    if (can_interface_) {
        can_interface_->stop_receive();
    }

    is_initialized_ = false;
    return SensorDataResult::SUCCESS;
}

SensorDataResult SensorDataManager::get_current_data(common::PerceptionData& data) const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!is_initialized_) {
        return SensorDataResult::ERROR_INIT;
    }

    if (!is_data_fresh()) {
        return SensorDataResult::ERROR_TIMEOUT;
    }

    data = perception_data_;
    return SensorDataResult::SUCCESS;
}

void SensorDataManager::set_data_callback(SensorDataCallback callback) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_callback_ = std::move(callback);
}

bool SensorDataManager::is_ready() const {
    return is_initialized_ && can_interface_ && can_interface_->is_ready();
}

uint32_t SensorDataManager::get_data_age_ms() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_);
    
    return static_cast<uint32_t>(age.count());
}

bool SensorDataManager::is_data_fresh() const {
    return get_data_age_ms() <= data_timeout_ms_;
}

SensorDataResult SensorDataManager::request_data_update() {
    if (!is_initialized_ || !can_interface_) {
        return SensorDataResult::ERROR_INIT;
    }

    // In a real implementation, this might send request messages for specific PGNs
    // For now, we rely on periodic broadcasts from ECUs
    
    return SensorDataResult::SUCCESS;
}

void SensorDataManager::on_can_frame_received(const can::CanFrame& frame) {
    // Only process supported J1939 messages
    if (!can::j1939::J1939Protocol::is_supported_message(frame)) {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Try to decode different types of sensor data
    can::j1939::EngineData engine_data;
    if (can::j1939::J1939Protocol::decode_engine_data(frame, engine_data)) {
        update_engine_data(engine_data);
        consolidate_perception_data();
        notify_data_callback();
        return;
    }

    can::j1939::VehicleData vehicle_data;
    if (can::j1939::J1939Protocol::decode_vehicle_data(frame, vehicle_data)) {
        update_vehicle_data(vehicle_data);
        consolidate_perception_data();
        notify_data_callback();
        return;
    }

    can::j1939::CvtStatusReport cvt_status;
    if (can::j1939::J1939Protocol::decode_cvt_status(frame, cvt_status)) {
        update_cvt_status(cvt_status);
        consolidate_perception_data();
        notify_data_callback();
        return;
    }
}

void SensorDataManager::update_engine_data(const can::j1939::EngineData& engine_data) {
    if (engine_data.data_valid) {
        // Update engine-related fields
        if (engine_data.engine_speed_rpm > 0) {
            engine_data_.engine_speed_rpm = engine_data.engine_speed_rpm;
        }
        
        if (engine_data.engine_load_percent >= 0 && engine_data.engine_load_percent <= 100) {
            engine_data_.engine_load_percent = engine_data.engine_load_percent;
        }
        
        if (engine_data.accelerator_pedal_percent >= 0 && engine_data.accelerator_pedal_percent <= 100) {
            engine_data_.accelerator_pedal_percent = engine_data.accelerator_pedal_percent;
        }
        
        engine_data_.data_valid = true;
        last_update_time_ = std::chrono::steady_clock::now();
    }
}

void SensorDataManager::update_vehicle_data(const can::j1939::VehicleData& vehicle_data) {
    if (vehicle_data.data_valid) {
        if (vehicle_data.vehicle_speed_mps >= 0) {
            vehicle_data_.vehicle_speed_mps = vehicle_data.vehicle_speed_mps;
        }
        
        vehicle_data_.data_valid = true;
        last_update_time_ = std::chrono::steady_clock::now();
    }
}

void SensorDataManager::update_cvt_status(const can::j1939::CvtStatusReport& cvt_status) {
    if (cvt_status.data_valid) {
        cvt_status_ = cvt_status;
        last_update_time_ = std::chrono::steady_clock::now();
    }
}

void SensorDataManager::consolidate_perception_data() {
    // Consolidate data from different sources into unified perception data
    
    // Engine data
    if (engine_data_.data_valid) {
        perception_data_.engine_speed_rpm = engine_data_.engine_speed_rpm;
        perception_data_.engine_load_percent = engine_data_.engine_load_percent;
        perception_data_.accelerator_pedal_percent = engine_data_.accelerator_pedal_percent;
    }
    
    // Vehicle data
    if (vehicle_data_.data_valid) {
        perception_data_.vehicle_speed_mps = vehicle_data_.vehicle_speed_mps;
    }
    
    // CVT status
    if (cvt_status_.data_valid) {
        perception_data_.current_transmission_ratio = cvt_status_.current_ratio;
        perception_data_.is_transmission_shifting = cvt_status_.is_shifting;
    }
    
    // Calculate derived values
    if (engine_data_.data_valid && vehicle_data_.data_valid) {
        // Calculate load factor based on engine load and vehicle speed
        float speed_factor = std::min(vehicle_data_.vehicle_speed_mps / 20.0f, 1.0f); // Normalize to 20 m/s max
        float load_factor = (engine_data_.engine_load_percent / 100.0f) * (1.0f + speed_factor);
        perception_data_.load_factor = std::min(load_factor, 1.0f);
    }
    
    // Update terrain type based on load and speed patterns
    if (perception_data_.load_factor > 0.8f && perception_data_.vehicle_speed_mps < 5.0f) {
        perception_data_.terrain_type = common::TerrainType::ROUGH;
    } else if (perception_data_.load_factor > 0.6f) {
        perception_data_.terrain_type = common::TerrainType::MODERATE;
    } else {
        perception_data_.terrain_type = common::TerrainType::SMOOTH;
    }
    
    // Mark perception data as valid if we have essential data
    perception_data_.data_valid = engine_data_.data_valid || vehicle_data_.data_valid;
}

void SensorDataManager::notify_data_callback() {
    if (data_callback_ && perception_data_.data_valid) {
        // Copy callback and data while holding the lock
        auto callback = data_callback_;
        auto data = perception_data_;
        
        // Note: This function is called while holding data_mutex_
        // The callback should be quick and non-blocking to avoid issues
        callback(data);
    }
}

} // namespace sensors
} // namespace vcu
