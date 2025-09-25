#include "vcu/cvt/cvt_controller.h"
#include "vcu/cvt/cvt_strategy_factory.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace vcu {
namespace cvt {

CvtController::CvtController(can::CanInterface& can_interface, const CvtConfig& config)
    : can_interface_(can_interface),
      config_(config),
      drive_mode_(common::DriveMode::MANUAL),
      is_initialized_(false),
      is_ready_(false),
      last_update_time_(0),
      safety_violation_count_(0) {
}

bool CvtController::init() {
    if (is_initialized_) {
        return true; // Already initialized
    }

    // Validate configuration
    if (!CvtConfigManager::validate_config(config_)) {
        std::cerr << "CVT Controller: Invalid configuration" << std::endl;
        return false;
    }

    // Create CVT strategy
    if (!create_strategy()) {
        std::cerr << "CVT Controller: Failed to create CVT strategy" << std::endl;
        return false;
    }

    // Initialize the strategy
    try {
        cvt_strategy_->init();
    } catch (const std::exception& e) {
        std::cerr << "CVT Controller: Strategy initialization failed: " << e.what() << std::endl;
        return false;
    }

    // Set initial drive mode
    cvt_strategy_->set_drive_mode(drive_mode_);

    is_initialized_ = true;
    is_ready_ = true;

    if (config_.enable_debug_logging) {
        std::cout << "CVT Controller: Initialized successfully with manufacturer: " 
                  << static_cast<int>(config_.manufacturer) << std::endl;
    }

    return true;
}

void CvtController::set_drive_mode(const common::DriveMode mode) {
    drive_mode_ = mode;
    
    if (cvt_strategy_) {
        cvt_strategy_->set_drive_mode(mode);
    }

    if (config_.enable_debug_logging) {
        std::cout << "CVT Controller: Drive mode set to " << static_cast<int>(mode) << std::endl;
    }
}

void CvtController::update(const common::PerceptionData& perception, const common::PredictionResult& prediction) {
    if (!is_ready_) {
        return;
    }

    // Validate safety conditions
    if (!validate_safety_conditions(perception)) {
        safety_violation_count_++;
        if (safety_violation_count_ >= MAX_SAFETY_VIOLATIONS) {
            is_ready_ = false;
            std::cerr << "CVT Controller: Too many safety violations, disabling controller" << std::endl;
            return;
        }
    } else {
        safety_violation_count_ = 0; // Reset counter on successful validation
    }

    // Calculate optimal transmission ratio
    float optimal_ratio = calculate_optimal_ratio(perception, prediction);
    
    // Apply configuration limits
    optimal_ratio = std::clamp(optimal_ratio, config_.min_ratio, config_.max_ratio);

    // Set target ratio in strategy
    cvt_strategy_->set_target_ratio(optimal_ratio);

    // Update strategy with perception data
    cvt_strategy_->update(perception);

    // Update timestamp
    last_update_time_ = get_current_time_ms();

    if (config_.enable_debug_logging) {
        common::CvtState current_state = cvt_strategy_->get_current_state();
        std::cout << "CVT Controller: Target ratio: " << optimal_ratio 
                  << ", Current ratio: " << current_state.current_ratio
                  << ", Shifting: " << (current_state.is_shifting ? "Yes" : "No") << std::endl;
    }
}

common::CvtState CvtController::get_current_state() const {
    if (!cvt_strategy_) {
        common::CvtState empty_state;
        return empty_state;
    }
    
    return cvt_strategy_->get_current_state();
}

bool CvtController::set_cvt_manufacturer(common::CvtManufacturer manufacturer) {
    if (config_.manufacturer == manufacturer) {
        return true; // No change needed
    }

    config_.manufacturer = manufacturer;
    
    // Recreate strategy with new manufacturer
    if (!create_strategy()) {
        std::cerr << "CVT Controller: Failed to create strategy for new manufacturer" << std::endl;
        return false;
    }

    // Re-initialize if we were previously initialized
    if (is_initialized_) {
        try {
            cvt_strategy_->init();
            cvt_strategy_->set_drive_mode(drive_mode_);
        } catch (const std::exception& e) {
            std::cerr << "CVT Controller: Failed to initialize new strategy: " << e.what() << std::endl;
            is_ready_ = false;
            return false;
        }
    }

    if (config_.enable_debug_logging) {
        std::cout << "CVT Controller: Manufacturer changed to " << static_cast<int>(manufacturer) << std::endl;
    }

    return true;
}

const CvtConfig& CvtController::get_config() const {
    return config_;
}

bool CvtController::update_config(const CvtConfig& config) {
    if (!CvtConfigManager::validate_config(config)) {
        std::cerr << "CVT Controller: Invalid configuration provided" << std::endl;
        return false;
    }

    bool manufacturer_changed = (config_.manufacturer != config.manufacturer);
    config_ = config;

    if (manufacturer_changed) {
        return set_cvt_manufacturer(config.manufacturer);
    }

    return true;
}

bool CvtController::is_ready() const {
    return is_ready_ && is_initialized_ && cvt_strategy_ != nullptr;
}

float CvtController::calculate_optimal_ratio(const common::PerceptionData& perception, const common::PredictionResult& prediction) {
    float optimal_ratio = 1.0f;
    
    switch (drive_mode_) {
        case common::DriveMode::PLOWING:
            optimal_ratio = calculate_plowing_ratio(perception);
            break;
            
        case common::DriveMode::SEEDING:
            optimal_ratio = calculate_seeding_ratio(perception);
            break;
            
        case common::DriveMode::TRANSPORT:
            optimal_ratio = calculate_transport_ratio(perception);
            break;
            
        case common::DriveMode::MANUAL:
        default:
            // For manual mode, use a simple ratio based on accelerator pedal
            optimal_ratio = 0.5f + (perception.accelerator_pedal_percent / 100.0f) * 1.5f;
            break;
    }
    
    // Consider prediction results for optimization
    if (prediction.predicted_load_percent > 80.0f) {
        // High predicted load, reduce ratio for more torque
        optimal_ratio *= 0.9f;
    } else if (prediction.predicted_load_percent < 30.0f) {
        // Low predicted load, increase ratio for efficiency
        optimal_ratio *= 1.1f;
    }
    
    return optimal_ratio;
}

float CvtController::calculate_plowing_ratio(const common::PerceptionData& perception) {
    // For plowing, we want lower ratios (higher torque multiplication)
    float base_ratio = 0.8f;
    
    // Adjust based on engine load - higher load means lower ratio needed
    if (perception.engine_load_percent > 80.0f) {
        base_ratio = 0.6f;
    } else if (perception.engine_load_percent > 60.0f) {
        base_ratio = 0.7f;
    }
    
    // Adjust based on vehicle speed - slower speed means lower ratio
    if (perception.vehicle_speed_mps < 2.0f) {
        base_ratio *= 0.9f;
    }
    
    // Consider terrain type
    switch (perception.terrain_type) {
        case common::TerrainType::STEEP_SLOPE:
            base_ratio *= 0.8f; // More torque for steep terrain
            break;
        case common::TerrainType::HILLY:
            base_ratio *= 0.9f; // Slightly more torque for hilly terrain
            break;
        default:
            break;
    }
    
    return base_ratio;
}

float CvtController::calculate_seeding_ratio(const common::PerceptionData& perception) {
    // For seeding, we want moderate ratios for consistent speed
    float base_ratio = 1.2f;
    
    // Adjust based on accelerator pedal position
    base_ratio += (perception.accelerator_pedal_percent / 100.0f) * 0.5f;
    
    // Adjust based on engine load
    if (perception.engine_load_percent > 70.0f) {
        base_ratio *= 0.9f;
    }
    
    // Maintain consistent speed for seeding operations
    if (perception.vehicle_speed_mps > 0.5f && perception.vehicle_speed_mps < 3.0f) {
        // In optimal seeding speed range, maintain current ratio
        common::CvtState current_state = get_current_state();
        if (std::abs(current_state.current_ratio - base_ratio) < 0.1f) {
            base_ratio = current_state.current_ratio;
        }
    }
    
    return base_ratio;
}

float CvtController::calculate_transport_ratio(const common::PerceptionData& perception) {
    // For transport, we want higher ratios for fuel efficiency
    float base_ratio = 1.5f;
    
    // Adjust based on vehicle speed - higher speed allows higher ratio
    if (perception.vehicle_speed_mps > 10.0f) {
        base_ratio = 1.8f;
    } else if (perception.vehicle_speed_mps > 5.0f) {
        base_ratio = 1.6f;
    }
    
    // Adjust based on accelerator pedal position
    base_ratio += (perception.accelerator_pedal_percent / 100.0f) * 0.3f;
    
    // Consider terrain for transport efficiency
    switch (perception.terrain_type) {
        case common::TerrainType::FLAT:
            base_ratio *= 1.1f; // Higher ratio for flat terrain efficiency
            break;
        case common::TerrainType::STEEP_SLOPE:
            base_ratio *= 0.9f; // Lower ratio for climbing
            break;
        default:
            break;
    }
    
    return base_ratio;
}

bool CvtController::create_strategy() {
    try {
        cvt_strategy_ = CvtStrategyFactory::create_strategy(config_.manufacturer, can_interface_);
        return cvt_strategy_ != nullptr;
    } catch (const std::exception& e) {
        std::cerr << "CVT Controller: Failed to create strategy: " << e.what() << std::endl;
        cvt_strategy_.reset();
        return false;
    }
}

bool CvtController::validate_safety_conditions(const common::PerceptionData& perception) const {
    // Check oil temperature
    if (perception.coolant_temp_celsius > config_.max_oil_temp) {
        if (config_.enable_debug_logging) {
            std::cerr << "CVT Controller: Oil temperature too high: " 
                      << perception.coolant_temp_celsius << "°C" << std::endl;
        }
        return false;
    }
    
    // Check engine RPM
    if (perception.engine_speed_rpm < config_.min_engine_rpm && perception.engine_speed_rpm > 0) {
        if (config_.enable_debug_logging) {
            std::cerr << "CVT Controller: Engine RPM too low: " 
                      << perception.engine_speed_rpm << " RPM" << std::endl;
        }
        return false;
    }
    
    // Check data validity
    if (!perception.data_valid) {
        if (config_.enable_debug_logging) {
            std::cerr << "CVT Controller: Perception data is invalid" << std::endl;
        }
        return false;
    }
    
    return true;
}

uint64_t CvtController::get_current_time_ms() const {
    // This should be implemented using platform-specific time functions
    // For now, return a placeholder
    return 0;
}

} // namespace cvt
} // namespace vcu
