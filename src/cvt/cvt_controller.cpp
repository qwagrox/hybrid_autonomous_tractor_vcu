#include "vcu/cvt/cvt_controller.h"
#include <algorithm>
#include <cmath>

namespace vcu {
namespace cvt {

CvtController::CvtController()
    : drive_mode_(common::DriveMode::MANUAL),
      cvt_manufacturer_(common::CvtManufacturer::UNKNOWN) {
    // Initialize CVT state
    cvt_state_.current_ratio = 1.0f;
    cvt_state_.target_ratio = 1.0f;
    cvt_state_.is_shifting = false;
}

void CvtController::set_drive_mode(const common::DriveMode mode) {
    drive_mode_ = mode;
}

void CvtController::update(const common::PerceptionData& perception, const common::PredictionResult& prediction) {
    // Calculate the optimal transmission ratio
    float optimal_ratio = calculate_optimal_ratio(perception, prediction);
    
    // Update target ratio
    cvt_state_.target_ratio = optimal_ratio;
    
    // Check if shifting is needed
    const float ratio_tolerance = 0.05f; // 5% tolerance
    if (std::abs(cvt_state_.current_ratio - cvt_state_.target_ratio) > ratio_tolerance) {
        cvt_state_.is_shifting = true;
        
        // Gradually adjust current ratio towards target ratio
        const float adjustment_rate = 0.1f; // 10% adjustment per update
        float ratio_diff = cvt_state_.target_ratio - cvt_state_.current_ratio;
        cvt_state_.current_ratio += ratio_diff * adjustment_rate;
    } else {
        cvt_state_.is_shifting = false;
    }
}

common::CvtState CvtController::get_current_state() const {
    return cvt_state_;
}

float CvtController::calculate_optimal_ratio(const common::PerceptionData& perception, const common::PredictionResult& /* prediction */) {
    float optimal_ratio = 1.0f;
    
    switch (drive_mode_) {
        case common::DriveMode::PLOWING:
            // For plowing, prioritize torque over speed
            optimal_ratio = calculate_plowing_ratio(perception);
            break;
            
        case common::DriveMode::SEEDING:
            // For seeding, balance between torque and speed
            optimal_ratio = calculate_seeding_ratio(perception);
            break;
            
        case common::DriveMode::TRANSPORT:
            // For transport, prioritize speed and fuel efficiency
            optimal_ratio = calculate_transport_ratio(perception);
            break;
            
        case common::DriveMode::MANUAL:
        default:
            // For manual mode, use a simple ratio based on accelerator pedal
            optimal_ratio = 0.5f + (perception.accelerator_pedal_percent / 100.0f) * 1.5f;
            break;
    }
    
    // Clamp the ratio to valid range [0.5, 2.0]
    return std::clamp(optimal_ratio, 0.5f, 2.0f);
}

float CvtController::calculate_plowing_ratio(const common::PerceptionData& perception) {
    // For plowing, we want lower ratios (higher torque multiplication)
    // Base ratio on engine load and speed
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
    
    return base_ratio;
}

} // namespace cvt
} // namespace vcu
