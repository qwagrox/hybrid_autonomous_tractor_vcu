
#include <cstddef>
#include "vcu/sensors/platform_sensor_data_manager.h"
#include "vcu/can/j1939_protocol.h"
#include <algorithm>

namespace vcu {
namespace sensors {

PlatformSensorDataManager::PlatformSensorDataManager(
    PlatformInterface* platform,
    std::shared_ptr<can::ICanInterface> can_interface)
    : platform_(platform),
      can_interface_(std::move(can_interface)),
      perception_data_{},
      last_update_time_ms_(0),
      data_timeout_ms_(1000),
      is_initialized_(false) {
    
    time_interface_ = platform_->create_time_interface();
    data_mutex_ = platform_->create_mutex();
    
    perception_data_.vehicle_speed_kmh = 0.0f;
    perception_data_.engine_speed_rpm = 0.0f;
    perception_data_.engine_load_percent = 0.0f;
    perception_data_.fuel_level_percent = 100.0f;
    perception_data_.coolant_temp_celsius = 20.0f;
    perception_data_.terrain_type = common::TerrainType::FLAT;
    perception_data_.load_factor = 0.0f;
    
    last_update_time_ms_ = time_interface_->get_monotonic_time_ms();
}

PlatformSensorDataManager::~PlatformSensorDataManager() {
    shutdown();
}

SensorDataResult PlatformSensorDataManager::initialize(uint32_t data_timeout_ms) {
    if (is_initialized_) {
        return SensorDataResult::SUCCESS;
    }

    if (!can_interface_ || !can_interface_->is_ready()) {
        return SensorDataResult::ERROR_CAN_COMM;
    }

    data_timeout_ms_ = data_timeout_ms;

    can_interface_->set_receive_callback(
        [this](const can::CanFrame& frame) {
            this->on_can_frame_received(frame);
        }
    );

    auto result = can_interface_->start_receive();
    if (result != can::CanResult::SUCCESS) {
        return SensorDataResult::ERROR_CAN_COMM;
    }

    is_initialized_ = true;
    return SensorDataResult::SUCCESS;
}

SensorDataResult PlatformSensorDataManager::shutdown() {
    if (!is_initialized_) {
        return SensorDataResult::SUCCESS;
    }

    if (can_interface_) {
        can_interface_->stop_receive();
    }

    is_initialized_ = false;
    return SensorDataResult::SUCCESS;
}

SensorDataResult PlatformSensorDataManager::get_current_data(common::PerceptionData& data) {
    if (!is_initialized_) {
        return SensorDataResult::ERROR_NOT_INITIALIZED;
    }

    data_mutex_->lock();
    data = perception_data_;
    data_mutex_->unlock();

    return SensorDataResult::SUCCESS;
}

bool PlatformSensorDataManager::is_data_fresh() const {
    uint32_t age_ms = get_data_age_ms();
    return age_ms < data_timeout_ms_;
}

uint32_t PlatformSensorDataManager::get_data_age_ms() const {
    uint64_t current_time = time_interface_->get_monotonic_time_ms();
    if (current_time >= last_update_time_ms_) {
        return static_cast<uint32_t>(current_time - last_update_time_ms_);
    }
    return 0;
}

void PlatformSensorDataManager::set_data_update_callback(
    std::function<void(const common::PerceptionData&)> callback) {
    data_callback_ = callback;
}

void PlatformSensorDataManager::on_can_frame_received(const can::CanFrame& frame) {
    uint32_t can_id = frame.get_id();
    uint32_t pgn = (can_id >> 8) & 0xFFFF;
    
    data_mutex_->lock();
    
    switch (pgn) {
        case can::j1939::pgn::ENGINE_SPEED_LOAD:
            parse_engine_data(frame);
            break;
        case can::j1939::pgn::VEHICLE_SPEED:
            parse_vehicle_data(frame);
            break;
        case can::j1939::pgn::CVT_STATUS_REPORT:
            parse_cvt_status(frame);
            break;
        default:
            break;
    }
    
    last_update_time_ms_ = time_interface_->get_monotonic_time_ms();
    
    data_mutex_->unlock();
    
    notify_data_callback();
}

void PlatformSensorDataManager::parse_engine_data(const can::CanFrame& frame) {
    can::j1939::EngineData engine_data;
    if (can::J1939Protocol::decode_engine_data(frame, engine_data)) {
        perception_data_.engine_speed_rpm = engine_data.engine_speed_rpm;
        perception_data_.engine_load_percent = engine_data.engine_load_percent;
    }
}

void PlatformSensorDataManager::parse_vehicle_data(const can::CanFrame& frame) {
    can::j1939::VehicleData vehicle_data;
    if (can::J1939Protocol::decode_vehicle_data(frame, vehicle_data)) {
        perception_data_.vehicle_speed_mps = vehicle_data.vehicle_speed_mps;
    }
}

void PlatformSensorDataManager::parse_cvt_status(const can::CanFrame& frame) {
    can::j1939::CvtStatusReport cvt_status;
    if (can::J1939Protocol::decode_cvt_status(frame, cvt_status)) {
        if (cvt_status.current_ratio > 2.5f) {
            perception_data_.terrain_type = common::TerrainType::STEEP_SLOPE;
            perception_data_.load_factor = 0.9f;
        } else if (cvt_status.current_ratio > 2.0f) {
            perception_data_.terrain_type = common::TerrainType::HILLY;
            perception_data_.load_factor = 0.7f;
        } else if (cvt_status.current_ratio > 1.5f) {
            perception_data_.terrain_type = common::TerrainType::GENTLE_SLOPE;
            perception_data_.load_factor = 0.5f;
        } else {
            perception_data_.terrain_type = common::TerrainType::FLAT;
            perception_data_.load_factor = 0.3f;
        }
    }
}

void PlatformSensorDataManager::notify_data_callback() {
    if (data_callback_) {
        data_callback_(perception_data_);
    }
}

} // namespace sensors
} // namespace vcu

