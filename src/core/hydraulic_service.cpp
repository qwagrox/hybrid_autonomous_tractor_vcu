#include "vcu/core/hydraulic_service.h"
#include <chrono>
#include <algorithm>

namespace vcu {
namespace core {

HydraulicService::HydraulicService(std::shared_ptr<cvt::CvtStrategy> cvt_strategy)
    : cvt_strategy_(cvt_strategy),
      initialized_(false),
      hydraulic_enabled_(false),
      last_error_flags_(0),
      last_update_time_(0),
      consecutive_error_count_(0) {
    
    // Initialize hydraulic state
    last_hydraulic_state_ = {};
}

bool HydraulicService::initialize() {
    if (!cvt_strategy_) {
        return false;
    }
    
    try {
        // Enable hydraulic control in the CVT strategy
        cvt_strategy_->enable_hydraulic_control(true);
        
        // Get initial hydraulic state
        last_hydraulic_state_ = cvt_strategy_->get_hydraulic_state();
        
        initialized_ = true;
        hydraulic_enabled_ = true;
        
        return true;
    } catch (...) {
        return false;
    }
}

void HydraulicService::shutdown() {
    if (cvt_strategy_) {
        // Disable hydraulic control
        cvt_strategy_->enable_hydraulic_control(false);
        
        // Send emergency stop command
        common::HydraulicCommand stop_command;
        stop_command.type = common::HydraulicCommandType::EMERGENCY_STOP;
        stop_command.timestamp = get_current_time_ms();
        
        cvt_strategy_->execute_hydraulic_command(stop_command);
    }
    
    initialized_ = false;
    hydraulic_enabled_ = false;
}

void HydraulicService::update(const common::PerceptionData& /* perception_data */) {
    if (!initialized_ || !hydraulic_enabled_) {
        return;
    }
    
    // Get current hydraulic state
    common::HydraulicState current_state = cvt_strategy_->get_hydraulic_state();
    
    // Check if state has changed significantly
    if (current_state.pressure != last_hydraulic_state_.pressure ||
        current_state.temperature != last_hydraulic_state_.temperature ||
        current_state.is_ready != last_hydraulic_state_.is_ready) {
        
        // Update last known state
        last_hydraulic_state_ = current_state;
        last_update_time_ = get_current_time_ms();
        
        // Notify state callbacks
        notify_state_callbacks(current_state);
    }
    
    // Monitor hydraulic health
    monitor_hydraulic_health();
    
    // Check safety conditions
    if (!check_safety_conditions()) {
        // Emergency stop if safety conditions are not met
        emergency_stop();
    }
}

bool HydraulicService::set_lift_mode(common::LiftMode mode) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    try {
        cvt_strategy_->set_lift_mode(mode);
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::set_lift_position(float position) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Validate position range
    if (position < 0.0f || position > 100.0f) {
        return false;
    }
    
    try {
        cvt_strategy_->set_lift_position(position);
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::execute_lift_action(common::LiftAction action) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Check safety conditions before executing action
    if (!check_safety_conditions()) {
        return false;
    }
    
    try {
        cvt_strategy_->execute_lift_action(action);
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Validate parameters
    if (valve_id >= 4 || flow_percent < -100 || flow_percent > 100) {
        return false;
    }
    
    try {
        cvt_strategy_->set_multi_valve_flow(valve_id, flow_percent);
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::set_multi_valve_lock(bool lock) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    try {
        cvt_strategy_->set_multi_valve_lock(lock);
        return true;
    } catch (...) {
        return false;
    }
}

common::HydraulicState HydraulicService::get_hydraulic_state() const {
    if (!initialized_) {
        return {};
    }
    
    return cvt_strategy_->get_hydraulic_state();
}

bool HydraulicService::is_hydraulic_ready() const {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    return state.is_ready;
}

uint32_t HydraulicService::get_hydraulic_errors() const {
    if (!initialized_) {
        return 0xFFFFFFFF; // All errors if not initialized
    }
    
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    return state.error_code;
}

bool HydraulicService::execute_hydraulic_command(const common::HydraulicCommand& command) {
    if (!initialized_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Validate command
    if (!validate_hydraulic_command(command)) {
        return false;
    }
    
    // Check safety conditions
    if (!check_safety_conditions()) {
        return false;
    }
    
    try {
        cvt_strategy_->execute_hydraulic_command(command);
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::emergency_stop() {
    if (!cvt_strategy_) {
        return false;
    }
    
    try {
        // Create emergency stop command
        common::HydraulicCommand stop_command;
        stop_command.type = common::HydraulicCommandType::EMERGENCY_STOP;
        stop_command.timestamp = get_current_time_ms();
        
        // Execute emergency stop
        cvt_strategy_->execute_hydraulic_command(stop_command);
        
        // Disable hydraulic system
        hydraulic_enabled_ = false;
        
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::validate_hydraulic_command(const common::HydraulicCommand& command) const {
    // Validate position range
    if (command.position < 0.0f || command.position > 100.0f) {
        return false;
    }
    
    // Validate flow percentage
    if (command.flow_percent < -100 || command.flow_percent > 100) {
        return false;
    }
    
    // Validate valve ID
    if (command.valve_id >= 4) {
        return false;
    }
    
    return true;
}

bool HydraulicService::check_safety_conditions() const {
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    
    // Check minimum pressure (18 bar = 1.8 MPa)
    if (state.pressure < 1.8f) {
        return false;
    }
    
    // Check maximum temperature (90°C)
    if (state.temperature > 90.0f) {
        return false;
    }
    
    // Check system readiness
    if (!state.is_ready) {
        return false;
    }
    
    // Check for critical errors
    if (state.error_code != 0) {
        return false;
    }
    
    return true;
}

void HydraulicService::monitor_hydraulic_health() {
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    uint32_t current_error_flags = 0;
    
    // Check for low pressure warning
    if (state.pressure < 1.0f) {
        consecutive_error_count_++;
        current_error_flags |= 0x01; // Low pressure flag
    } else {
        consecutive_error_count_ = 0;
    }
    
    // Check for high temperature warning
    if (state.temperature > 100.0f) {
        consecutive_error_count_++;
        current_error_flags |= 0x02; // High temperature flag
    }
    
    // Check if error flags have changed
    if (current_error_flags != last_error_flags_) {
        last_error_flags_ = current_error_flags;
        if (current_error_flags != 0) {
            notify_error_callbacks(current_error_flags);
        }
    }
    
    // If too many consecutive errors, disable hydraulic system
    if (consecutive_error_count_ > 10) {
        emergency_stop();
    }
}

uint32_t HydraulicService::get_current_time_ms() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void HydraulicService::register_state_callback(HydraulicStateCallback callback) {
    state_callbacks_.push_back(callback);
}

void HydraulicService::register_error_callback(HydraulicErrorCallback callback) {
    error_callbacks_.push_back(callback);
}

void HydraulicService::notify_state_callbacks(const common::HydraulicState& new_state) {
    for (const auto& callback : state_callbacks_) {
        if (callback) {
            callback(new_state);
        }
    }
}

void HydraulicService::notify_error_callbacks(uint32_t error_flags) {
    for (const auto& callback : error_callbacks_) {
        if (callback) {
            callback(error_flags);
        }
    }
}

bool HydraulicService::enable_hydraulic_control(bool enable) {
    if (!initialized_) {
        return false;
    }
    
    try {
        cvt_strategy_->enable_hydraulic_control(enable);
        hydraulic_enabled_ = enable;
        return true;
    } catch (...) {
        return false;
    }
}

bool HydraulicService::is_hydraulic_enabled() const {
    return hydraulic_enabled_;
}

} // namespace core
} // namespace vcu
