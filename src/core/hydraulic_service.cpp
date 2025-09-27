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
        last_error_flags_ = cvt_strategy_->get_hydraulic_errors();
        
        // Update timestamps
        auto now = std::chrono::steady_clock::now();
        last_update_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        initialized_ = true;
        hydraulic_enabled_ = true;
        
        return true;
    } catch (const std::exception& e) {
        initialized_ = false;
        return false;
    }
}

void HydraulicService::update(const common::PerceptionData& perception_data) {
    if (!initialized_ || !cvt_strategy_) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    uint64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    // Update hydraulic state from CVT strategy
    common::HydraulicState current_state = cvt_strategy_->get_hydraulic_state();
    uint32_t current_errors = cvt_strategy_->get_hydraulic_errors();
    
    // Check for state changes
    bool state_changed = false;
    if (current_state.lift_position != last_hydraulic_state_.lift_position ||
        current_state.lift_mode != last_hydraulic_state_.lift_mode ||
        current_state.lift_moving != last_hydraulic_state_.lift_moving ||
        current_state.main_pressure_mpa != last_hydraulic_state_.main_pressure_mpa ||
        current_state.hydraulic_enabled != last_hydraulic_state_.hydraulic_enabled) {
        state_changed = true;
    }
    
    // Check valve states
    for (int i = 0; i < 4; ++i) {
        if (current_state.valve_states[i] != last_hydraulic_state_.valve_states[i]) {
            state_changed = true;
            break;
        }
    }
    
    // Notify callbacks if state changed
    if (state_changed) {
        notify_state_callbacks(current_state);
        last_hydraulic_state_ = current_state;
    }
    
    // Check for error changes
    if (current_errors != last_error_flags_) {
        notify_error_callbacks(current_errors);
        last_error_flags_ = current_errors;
        
        if (current_errors != 0) {
            consecutive_error_count_++;
        } else {
            consecutive_error_count_ = 0;
        }
    }
    
    // Perform periodic health monitoring
    if (current_time - last_update_time_ >= HEALTH_CHECK_INTERVAL_MS) {
        monitor_hydraulic_health();
        last_update_time_ = current_time;
    }
}

void HydraulicService::shutdown() {
    if (!initialized_ || !cvt_strategy_) {
        return;
    }
    
    try {
        // Execute emergency stop
        emergency_stop();
        
        // Disable hydraulic control
        cvt_strategy_->enable_hydraulic_control(false);
        
        // Clear callbacks
        state_callbacks_.clear();
        error_callbacks_.clear();
        
        initialized_ = false;
        hydraulic_enabled_ = false;
    } catch (const std::exception& e) {
        // Log error but continue shutdown
    }
}

// ========== Hydraulic Control Implementation ==========

bool HydraulicService::enable_hydraulic_control(bool enable) {
    if (!initialized_ || !cvt_strategy_) {
        return false;
    }
    
    try {
        cvt_strategy_->enable_hydraulic_control(enable);
        hydraulic_enabled_ = enable;
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::execute_hydraulic_command(const common::HydraulicCommand& command) {
    if (!initialized_ || !cvt_strategy_ || !hydraulic_enabled_) {
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
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::set_lift_position(float position) {
    if (!initialized_ || !cvt_strategy_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Validate position range
    if (position < 0.0f || position > 100.0f) {
        return false;
    }
    
    try {
        cvt_strategy_->set_lift_position(position);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::set_lift_mode(common::LiftMode mode) {
    if (!initialized_ || !cvt_strategy_ || !hydraulic_enabled_) {
        return false;
    }
    
    try {
        cvt_strategy_->set_lift_mode(mode);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::execute_lift_action(common::LiftAction action) {
    if (!initialized_ || !cvt_strategy_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Check safety conditions for lift actions
    if (!check_safety_conditions()) {
        return false;
    }
    
    try {
        cvt_strategy_->execute_lift_action(action);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent) {
    if (!initialized_ || !cvt_strategy_ || !hydraulic_enabled_) {
        return false;
    }
    
    // Validate valve ID and flow range
    if (valve_id >= 4 || flow_percent < -100 || flow_percent > 100) {
        return false;
    }
    
    try {
        cvt_strategy_->set_multi_valve_flow(valve_id, flow_percent);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::set_multi_valve_lock(bool lock) {
    if (!initialized_ || !cvt_strategy_) {
        return false;
    }
    
    try {
        cvt_strategy_->set_multi_valve_lock(lock);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

// ========== Status and Monitoring Implementation ==========

common::HydraulicState HydraulicService::get_hydraulic_state() const {
    if (!initialized_ || !cvt_strategy_) {
        return {};
    }
    
    return cvt_strategy_->get_hydraulic_state();
}

bool HydraulicService::is_hydraulic_ready() const {
    if (!initialized_ || !cvt_strategy_) {
        return false;
    }
    
    return cvt_strategy_->is_hydraulic_ready();
}

uint32_t HydraulicService::get_hydraulic_errors() const {
    if (!initialized_ || !cvt_strategy_) {
        return 0xFFFFFFFF; // All errors if not initialized
    }
    
    return cvt_strategy_->get_hydraulic_errors();
}

bool HydraulicService::is_hydraulic_enabled() const {
    return initialized_ && hydraulic_enabled_;
}

// ========== Callback Registration Implementation ==========

void HydraulicService::register_state_callback(HydraulicStateCallback callback) {
    if (callback) {
        state_callbacks_.push_back(callback);
    }
}

void HydraulicService::register_error_callback(HydraulicErrorCallback callback) {
    if (callback) {
        error_callbacks_.push_back(callback);
    }
}

// ========== Safety and Emergency Implementation ==========

bool HydraulicService::emergency_stop() {
    if (!initialized_ || !cvt_strategy_) {
        return false;
    }
    
    try {
        // Stop all lift actions
        cvt_strategy_->execute_lift_action(common::LiftAction::STOP);
        
        // Set all multi-valve flows to neutral
        for (uint8_t i = 0; i < 4; ++i) {
            cvt_strategy_->set_multi_valve_flow(i, 0);
        }
        
        // Lock all valves for safety
        cvt_strategy_->set_multi_valve_lock(true);
        
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool HydraulicService::reset_hydraulic_errors() {
    if (!initialized_) {
        return false;
    }
    
    consecutive_error_count_ = 0;
    last_error_flags_ = 0;
    
    return true;
}

// ========== Private Methods Implementation ==========

bool HydraulicService::validate_hydraulic_command(const common::HydraulicCommand& command) const {
    // Validate lift position
    if (command.target_position < 0.0f || command.target_position > 100.0f) {
        return false;
    }
    
    // Validate lift speed
    if (command.lift_speed > 100) {
        return false;
    }
    
    // Validate valve flows
    for (int i = 0; i < 4; ++i) {
        if (command.valve_flows[i] < -100 || command.valve_flows[i] > 100) {
            return false;
        }
    }
    
    // Validate force/position mix
    if (command.force_position_mix > 100) {
        return false;
    }
    
    // Validate upper limit
    if (command.upper_limit > 100) {
        return false;
    }
    
    return true;
}

bool HydraulicService::check_safety_conditions() const {
    if (!cvt_strategy_) {
        return false;
    }
    
    // Check if hydraulic system is ready
    if (!cvt_strategy_->is_hydraulic_ready()) {
        return false;
    }
    
    // Check error count
    if (consecutive_error_count_ >= MAX_CONSECUTIVE_ERRORS) {
        return false;
    }
    
    // Get current hydraulic state
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    
    // Check pressure conditions
    if (state.main_pressure_mpa < 1.8f) { // Minimum 18 bar
        return false;
    }
    
    // Check temperature conditions
    if (state.rear_axle_temp_celsius > 90.0f || state.hst_temp_celsius > 90.0f) {
        return false;
    }
    
    // Check filter conditions
    if (!state.lift_filter_ok || !state.suction_filter_ok) {
        return false;
    }
    
    return true;
}

void HydraulicService::monitor_hydraulic_health() {
    if (!cvt_strategy_) {
        return;
    }
    
    // Get current state and errors
    common::HydraulicState state = cvt_strategy_->get_hydraulic_state();
    uint32_t errors = cvt_strategy_->get_hydraulic_errors();
    
    // Check for critical conditions
    bool critical_condition = false;
    
    // Critical pressure condition
    if (state.main_pressure_mpa < 1.0f) {
        critical_condition = true;
    }
    
    // Critical temperature condition
    if (state.rear_axle_temp_celsius > 100.0f || state.hst_temp_celsius > 100.0f) {
        critical_condition = true;
    }
    
    // If critical condition detected, execute emergency stop
    if (critical_condition) {
        emergency_stop();
        
        // Notify error callbacks
        notify_error_callbacks(errors | 0x80000000); // Set critical error bit
    }
}

void HydraulicService::notify_state_callbacks(const common::HydraulicState& new_state) {
    for (auto& callback : state_callbacks_) {
        try {
            callback(new_state);
        } catch (const std::exception& e) {
            // Log error but continue with other callbacks
        }
    }
}

void HydraulicService::notify_error_callbacks(uint32_t error_flags) {
    for (auto& callback : error_callbacks_) {
        try {
            callback(error_flags);
        } catch (const std::exception& e) {
            // Log error but continue with other callbacks
        }
    }
}

} // namespace core
} // namespace vcu
