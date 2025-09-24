#include "vcu/core/vcu_service.h"
#include <chrono>
#include <thread>

namespace vcu {
namespace core {

VcuService::VcuService()
    : state_(VcuState::OFF),
      running_(false) {
}

VcuService::~VcuService() {
    shutdown();
}

bool VcuService::initialize(const std::string& config_path) {
    if (state_ != VcuState::OFF) {
        return false;
    }

    state_ = VcuState::INITIALIZING;

    try {
        // Initialize configuration manager first
        config_manager_ = std::make_shared<config::JsonConfigManager>();
        if (!config_manager_->load_config(config_path)) {
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize diagnostic monitor
        diag_monitor_ = std::make_shared<diag::FileDiagnosticMonitor>();
        diag_monitor_->log(diag::LogLevel::INFO, "VCU Service initializing...");

        // Initialize hardware abstraction layer
        hal_ = std::make_shared<hal::LinuxHal>();
        if (!hal_->initialize()) {
            diag_monitor_->log(diag::LogLevel::ERROR, "HAL initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Get CAN interface from HAL
        auto config = config_manager_->get_vcu_config();
        can_interface_ = hal_->get_can_interface(config.can_bus_name);
        if (!can_interface_ || can_interface_->initialize("can0", 500000) != can::CanResult::SUCCESS) {
            diag_monitor_->log(diag::LogLevel::ERROR, "CAN interface initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize sensor data manager
        sensor_manager_ = std::make_shared<sensors::SensorDataManager>(can_interface_);
        if (sensor_manager_->initialize(config.data_timeout_ms) != sensors::SensorDataResult::SUCCESS) {
            diag_monitor_->log(diag::LogLevel::ERROR, "Sensor manager initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize load predictor
        load_predictor_ = std::make_shared<prediction::LoadPredictor>();
        if (load_predictor_->initialize() != prediction::PredictionResult::SUCCESS) {
            diag_monitor_->log(diag::LogLevel::ERROR, "Load predictor initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize CVT controller
        cvt_controller_ = std::make_shared<cvt::CvtController>();
        // CVT controller doesn't need explicit initialization

        // Initialize autonomous driving interface
        //ad_interface_ = std::make_shared<ad_interface::Ros2AdInterface>();
        //if (!ad_interface_->initialize()) {
        //    diag_monitor_->log(diag::LogLevel::ERROR, "AD interface initialization failed");
        //    state_ = VcuState::FAULT;
        //    return false;
        //}

        diag_monitor_->log(diag::LogLevel::INFO, "VCU Service initialized successfully");
        state_ = VcuState::RUNNING;
        return true;

    } catch (const std::exception& e) {
        diag_monitor_->log(diag::LogLevel::FATAL, "Exception during initialization: " + std::string(e.what()));
        state_ = VcuState::FAULT;
        return false;
    }
}

void VcuService::run() {
    if (state_ != VcuState::RUNNING) {
        return;
    }

    running_ = true;
    main_thread_ = std::make_unique<std::thread>(&VcuService::main_loop, this);
    
    diag_monitor_->log(diag::LogLevel::INFO, "VCU Service main loop started");
}

void VcuService::shutdown() {
    if (state_ == VcuState::OFF) {
        return;
    }

    state_ = VcuState::SHUTTING_DOWN;
    running_ = false;

    if (main_thread_ && main_thread_->joinable()) {
        main_thread_->join();
    }

    // Shutdown modules in reverse order
    //if (ad_interface_) {
    //    ad_interface_->shutdown();
    //}
    
    // CVT controller doesn't need explicit shutdown
    
    if (load_predictor_) {
        load_predictor_->shutdown();
    }
    
    if (sensor_manager_) {
        sensor_manager_->shutdown();
    }
    
    if (can_interface_) {
        can_interface_->shutdown();
    }
    
    if (hal_) {
        hal_->shutdown();
    }

    if (diag_monitor_) {
        diag_monitor_->log(diag::LogLevel::INFO, "VCU Service shutdown complete");
    }

    state_ = VcuState::OFF;
}

VcuState VcuService::get_state() const {
    return state_;
}

void VcuService::main_loop() {
    const auto loop_period = std::chrono::milliseconds(10); // 100Hz main loop
    
    while (running_ && state_ == VcuState::RUNNING) {
        auto loop_start = std::chrono::steady_clock::now();
        
        try {
            // Process autonomous driving commands
            process_ad_commands();
            
            // Update vehicle state and send to AD system
            update_vehicle_state();
            
        } catch (const std::exception& e) {
            diag_monitor_->log(diag::LogLevel::ERROR, "Exception in main loop: " + std::string(e.what()));
        }
        
        // Maintain loop timing
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - loop_start;
        
        if (elapsed < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed);
        } else {
            diag_monitor_->log(diag::LogLevel::WARN, "Main loop overrun detected");
        }
    }
}

void VcuService::process_ad_commands() {
    //ad_interface::AdCommand cmd;
    //if (ad_interface_->get_command(cmd)) {
        // Set drive mode
    //    cvt_controller_->set_drive_mode(cmd.target_drive_mode);
        
        // Update target parameters based on command
        // This is where the integration between AD and VCU happens
    //    diag_monitor_->log(diag::LogLevel::INFO, "Processing AD command");
    //}
}

void VcuService::update_vehicle_state() {
    // Get current sensor data
    common::PerceptionData perception_data;
    if (sensor_manager_->get_current_data(perception_data) == sensors::SensorDataResult::SUCCESS) {
        
        // Get current CVT state
        common::CvtState cvt_state = cvt_controller_->get_current_state();
        
        // Prepare vehicle state for AD system
        //ad_interface::VehicleState vehicle_state;
        //vehicle_state.current_drive_mode = common::DriveMode::MANUAL; // Default for now
        //vehicle_state.current_speed_mps = perception_data.vehicle_speed_mps;
        //vehicle_state.current_engine_rpm = perception_data.engine_speed_rpm;
        //vehicle_state.current_load_percent = perception_data.engine_load_percent;
        //vehicle_state.current_transmission_ratio = perception_data.current_transmission_ratio;
        
        // Send state to AD system
        //ad_interface_->publish_state(vehicle_state);
    }
}

} // namespace core
} // namespace vcu
