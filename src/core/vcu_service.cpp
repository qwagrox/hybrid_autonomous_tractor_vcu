#include "vcu/core/vcu_service.h"
#include "vcu/cvt/cvt_config.h"
#include "vcu/can/platform_can_interface.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace vcu {
namespace core {

VcuService::VcuService()
    : state_(VcuState::OFF),
      running_(false),
      last_cvt_update_time_(0),
      cvt_error_count_(0) {
}

VcuService::~VcuService() {
    if (running_) {
        shutdown();
    }
}

bool VcuService::initialize(const std::string& config_path) {
    state_ = VcuState::INITIALIZING;

    try {
        // Initialize platform
        platform_ = platform::PlatformFactory::create_platform();
        if (!platform_) {
            std::cerr << "VCU Service: Failed to create platform interface" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize configuration manager
        config_manager_ = std::make_shared<config::JsonConfigManager>();
        if (!config_manager_->load_config(config_path)) {
            std::cerr << "VCU Service: Failed to load configuration from " << config_path << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize diagnostic monitor
        diag_monitor_ = std::make_shared<diag::FileDiagnosticMonitor>();
        if (!diag_monitor_->init("/tmp/vcu_diagnostics.log")) {
            std::cerr << "VCU Service: Failed to initialize diagnostic monitor" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize HAL
        hal_ = std::make_shared<hal::LinuxHal>();
        if (!hal_->init()) {
            std::cerr << "VCU Service: Failed to initialize HAL" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize CAN interface
        can_interface_ = std::make_shared<can::PlatformCanInterface>();
        if (!can_interface_->init()) {
            std::cerr << "VCU Service: Failed to initialize CAN interface" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize ADAS interface
        adas_interface_ = std::make_shared<adas_interface::AdasCanInterface>(can_interface_);
        if (!adas_interface_->init()) {
            std::cerr << "VCU Service: Failed to initialize ADAS interface" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize sensor manager
        sensor_manager_ = std::make_shared<sensors::PlatformSensorDataManager>(platform_.get());
        if (!sensor_manager_->init()) {
            std::cerr << "VCU Service: Failed to initialize sensor manager" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize load predictor
        load_predictor_ = std::make_shared<prediction::LoadPredictor>();
        if (!load_predictor_->init()) {
            std::cerr << "VCU Service: Failed to initialize load predictor" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Initialize CVT controller with strategy factory
        if (!initialize_cvt_controller()) {
            std::cerr << "VCU Service: Failed to initialize CVT controller" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        // Create main thread
        main_thread_ = platform_->create_thread();
        if (!main_thread_) {
            std::cerr << "VCU Service: Failed to create main thread" << std::endl;
            state_ = VcuState::FAULT;
            return false;
        }

        state_ = VcuState::RUNNING;
        std::cout << "VCU Service: Initialization completed successfully" << std::endl;
        
        // Log CVT configuration
        std::cout << "CVT Configuration:" << std::endl;
        std::cout << "  Manufacturer: " << static_cast<int>(cvt_config_.manufacturer) << std::endl;
        std::cout << "  Ratio range: " << cvt_config_.min_ratio << " to " << cvt_config_.max_ratio << std::endl;
        std::cout << "  Control period: " << static_cast<int>(cvt_config_.control_period_ms) << "ms" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "VCU Service: Initialization failed with exception: " << e.what() << std::endl;
        state_ = VcuState::FAULT;
        return false;
    }
}

void VcuService::run() {
    if (state_ != VcuState::RUNNING) {
        std::cerr << "VCU Service: Cannot run, not in RUNNING state" << std::endl;
        return;
    }

    running_ = true;
    
    // Start main thread
    main_thread_->start([this]() { main_loop(); });
    
    std::cout << "VCU Service: Started main loop" << std::endl;
}

void VcuService::shutdown() {
    std::cout << "VCU Service: Shutting down..." << std::endl;
    state_ = VcuState::SHUTTING_DOWN;
    running_ = false;

    // Stop main thread
    if (main_thread_) {
        main_thread_->join();
    }

    // Shutdown modules in reverse order
    if (cvt_controller_) {
        // CVT controller doesn't have explicit shutdown, but we can log final state
        common::CvtState final_state = cvt_controller_->get_current_state();
        std::cout << "CVT final state - Ratio: " << final_state.current_ratio 
                  << ", Shifting: " << (final_state.is_shifting ? "Yes" : "No") << std::endl;
    }

    if (load_predictor_) {
        load_predictor_->shutdown();
    }

    if (sensor_manager_) {
        sensor_manager_->shutdown();
    }

    if (adas_interface_) {
        adas_interface_->shutdown();
    }

    if (can_interface_) {
        can_interface_->shutdown();
    }

    if (hal_) {
        hal_->shutdown();
    }

    if (diag_monitor_) {
        diag_monitor_->shutdown();
    }

    state_ = VcuState::OFF;
    std::cout << "VCU Service: Shutdown completed" << std::endl;
}

VcuState VcuService::get_state() const {
    return state_;
}

bool VcuService::set_cvt_manufacturer(common::CvtManufacturer manufacturer) {
    if (!cvt_controller_) {
        std::cerr << "VCU Service: CVT controller not initialized" << std::endl;
        return false;
    }

    if (!cvt_controller_->set_cvt_manufacturer(manufacturer)) {
        std::cerr << "VCU Service: Failed to set CVT manufacturer" << std::endl;
        cvt_error_count_++;
        return false;
    }

    cvt_config_.manufacturer = manufacturer;
    std::cout << "VCU Service: CVT manufacturer changed to " << static_cast<int>(manufacturer) << std::endl;
    
    return true;
}

const cvt::CvtConfig& VcuService::get_cvt_config() const {
    return cvt_config_;
}

bool VcuService::update_cvt_config(const cvt::CvtConfig& config) {
    if (!cvt_controller_) {
        std::cerr << "VCU Service: CVT controller not initialized" << std::endl;
        return false;
    }

    if (!cvt_controller_->update_config(config)) {
        std::cerr << "VCU Service: Failed to update CVT configuration" << std::endl;
        cvt_error_count_++;
        return false;
    }

    cvt_config_ = config;
    std::cout << "VCU Service: CVT configuration updated" << std::endl;
    
    return true;
}

common::CvtState VcuService::get_cvt_state() const {
    if (!cvt_controller_) {
        common::CvtState empty_state;
        return empty_state;
    }
    
    return cvt_controller_->get_current_state();
}

void VcuService::main_loop() {
    std::cout << "VCU Service: Main loop started" << std::endl;
    
    const auto loop_period = std::chrono::milliseconds(50); // 20Hz main loop
    auto next_loop_time = std::chrono::steady_clock::now();

    while (running_ && state_ == VcuState::RUNNING) {
        try {
            // Process ADAS commands
            process_ad_commands();

            // Update vehicle state
            update_vehicle_state();

            // Check for CVT errors and handle recovery
            if (cvt_error_count_ >= MAX_CVT_ERRORS) {
                if (!handle_cvt_error_recovery()) {
                    std::cerr << "VCU Service: CVT error recovery failed, entering fault state" << std::endl;
                    state_ = VcuState::FAULT;
                    break;
                }
            }

            // Sleep until next loop iteration
            next_loop_time += loop_period;
            std::this_thread::sleep_until(next_loop_time);

        } catch (const std::exception& e) {
            std::cerr << "VCU Service: Exception in main loop: " << e.what() << std::endl;
            cvt_error_count_++;
        }
    }

    std::cout << "VCU Service: Main loop ended" << std::endl;
}

void VcuService::process_ad_commands() {
    // Process commands from ADAS interface
    if (!adas_interface_) {
        return;
    }

    // Get latest ADAS commands (implementation depends on ADAS interface)
    // For now, this is a placeholder
    
    // Example: Check for drive mode changes from ADAS
    // common::DriveMode new_mode = adas_interface_->get_requested_drive_mode();
    // if (cvt_controller_ && new_mode != current_drive_mode) {
    //     cvt_controller_->set_drive_mode(new_mode);
    // }
}

void VcuService::update_vehicle_state() {
    if (!sensor_manager_ || !load_predictor_ || !cvt_controller_) {
        return;
    }

    // Get latest sensor data
    common::PerceptionData perception_data = sensor_manager_->get_latest_data();
    
    // Get load prediction
    common::PredictionResult prediction_result = load_predictor_->predict(perception_data);
    
    // Validate CVT operation conditions
    if (!validate_cvt_operation_conditions()) {
        return;
    }

    // Update CVT controller
    cvt_controller_->update(perception_data, prediction_result);
    
    last_cvt_update_time_ = get_current_time_ms();

    // Log CVT state periodically (every 1 second)
    static uint64_t last_log_time = 0;
    uint64_t current_time = get_current_time_ms();
    if (current_time - last_log_time > 1000) {
        if (cvt_config_.enable_debug_logging) {
            common::CvtState cvt_state = cvt_controller_->get_current_state();
            std::cout << "CVT State - Ratio: " << cvt_state.current_ratio 
                      << ", Target: " << cvt_state.target_ratio
                      << ", Shifting: " << (cvt_state.is_shifting ? "Yes" : "No")
                      << ", Speed: " << perception_data.vehicle_speed_mps << " m/s"
                      << ", Load: " << perception_data.engine_load_percent << "%" << std::endl;
        }
        last_log_time = current_time;
    }
}

bool VcuService::initialize_cvt_controller() {
    // Load CVT configuration
    cvt_config_ = load_cvt_configuration();

    // Create CVT controller with strategy factory support
    if (!can_interface_) {
        std::cerr << "VCU Service: CAN interface not available for CVT controller" << std::endl;
        return false;
    }

    try {
        cvt_controller_ = std::make_unique<cvt::CvtController>(*can_interface_, cvt_config_);
        
        if (!cvt_controller_->init()) {
            std::cerr << "VCU Service: Failed to initialize CVT controller" << std::endl;
            return false;
        }

        std::cout << "VCU Service: CVT controller initialized with manufacturer: " 
                  << static_cast<int>(cvt_config_.manufacturer) << std::endl;
        
        return true;

    } catch (const std::exception& e) {
        std::cerr << "VCU Service: Exception creating CVT controller: " << e.what() << std::endl;
        return false;
    }
}

cvt::CvtConfig VcuService::load_cvt_configuration() {
    // Try to load from configuration file first
    if (config_manager_) {
        // This would require extending JsonConfigManager to support CVT config
        // For now, use environment variables and defaults
    }

    // Load from environment variables
    cvt::CvtConfig config = cvt::CvtConfigManager::load_from_environment();
    
    // If environment doesn't provide config, use defaults
    if (config.manufacturer == common::CvtManufacturer::UNKNOWN) {
        config = cvt::CvtConfigManager::get_default_config();
        config.manufacturer = common::CvtManufacturer::HMCVT_VENDOR1; // Default to HMCVT_Vendor1
    }

    // Enable debug logging in development
    #ifdef DEBUG
    config.enable_debug_logging = true;
    #endif

    return config;
}

bool VcuService::handle_cvt_error_recovery() {
    std::cout << "VCU Service: Attempting CVT error recovery..." << std::endl;
    
    if (!cvt_controller_) {
        return false;
    }

    // Try to reinitialize CVT controller
    try {
        if (cvt_controller_->init()) {
            cvt_error_count_ = 0; // Reset error count on successful recovery
            std::cout << "VCU Service: CVT error recovery successful" << std::endl;
            return true;
        }
    } catch (const std::exception& e) {
        std::cerr << "VCU Service: CVT recovery failed: " << e.what() << std::endl;
    }

    return false;
}

bool VcuService::validate_cvt_operation_conditions() const {
    // Check if CVT controller is ready
    if (!cvt_controller_ || !cvt_controller_->is_ready()) {
        return false;
    }

    // Check for communication timeout
    uint64_t current_time = get_current_time_ms();
    if (current_time - last_cvt_update_time_ > CVT_UPDATE_TIMEOUT_MS) {
        return false;
    }

    // Check system state
    if (state_ != VcuState::RUNNING) {
        return false;
    }

    return true;
}

uint64_t VcuService::get_current_time_ms() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

} // namespace core
} // namespace vcu
