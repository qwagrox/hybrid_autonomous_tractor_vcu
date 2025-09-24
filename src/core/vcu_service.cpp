#include "vcu/core/vcu_service.h"
#include "vcu/platform/platform_factory.h"
#include <chrono>
#include <thread>

namespace vcu {
namespace core {

VcuService::VcuService()
    : state_(VcuState::OFF),
      running_(false),
      platform_(PlatformFactory::create_platform()) {
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
        // Create and initialize configuration manager
        config_manager_ = std::make_shared<config::JsonConfigManager>();
        
        // Create diagnostic monitor
        diag_monitor_ = std::make_shared<diag::FileDiagnosticMonitor>("vcu_diagnostics.log");

        // Create HAL
        hal_ = std::make_shared<hal::LinuxHal>();
        if (!hal_->initialize()) {
            diag_monitor_->report_fault(1001, "HAL initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Get CAN interface from HAL
        can_interface_ = hal_->get_can_interface("can0");
        if (!can_interface_) {
            diag_monitor_->report_fault(1002, "Failed to get CAN interface");
            state_ = VcuState::FAULT;
            return false;
        }

        // Create ADAS interface
        adas_interface_ = std::make_shared<adas_interface::AdasCanInterface>(platform_.get(), can_interface_);
        if (!adas_interface_->initialize()) {
            diag_monitor_->log(diag::LogLevel::WARNING, "ADAS interface initialization failed, continuing without ADAS");
            adas_interface_.reset();
        }

        // Create sensor manager
        sensor_manager_ = std::make_shared<sensors::PlatformSensorDataManager>(platform_.get(), can_interface_);
        if (sensor_manager_->initialize() != sensors::SensorDataResult::SUCCESS) {
            diag_monitor_->report_fault(1003, "Sensor manager initialization failed");
            state_ = VcuState::FAULT;
            return false;
        }

        // Create load predictor
        load_predictor_ = std::make_shared<prediction::LoadPredictor>();
        // LoadPredictor doesn't have initialize() method, it's ready to use

        // Create CVT controller
        cvt_controller_ = std::make_shared<cvt::CvtController>();
        // CvtController doesn't have initialize() method, it's ready to use

        state_ = VcuState::RUNNING;
        diag_monitor_->log(diag::LogLevel::INFO, "VCU Service initialized successfully");
        return true;

    } catch (const std::exception& e) {
        if (diag_monitor_) {
            diag_monitor_->report_fault(1000, std::string("Initialization exception: ") + e.what());
        }
        state_ = VcuState::FAULT;
        return false;
    }
}

void VcuService::run() {
    if (state_ != VcuState::RUNNING) {
        return;
    }

    running_ = true;

    // Create main thread
    main_thread_ = platform_->create_thread();
    main_thread_->start([this]() {
        main_loop();
    });

    diag_monitor_->log(diag::LogLevel::INFO, "VCU Service started");

    // Wait for main thread to complete
    main_thread_->join();
}

void VcuService::shutdown() {
    if (state_ == VcuState::OFF) {
        return;
    }

    state_ = VcuState::SHUTTING_DOWN;
    running_ = false;

    // Wait for main thread to finish
    if (main_thread_) {
        main_thread_->join();
        main_thread_.reset();
    }

    // Shutdown components in reverse order
    cvt_controller_.reset();
    load_predictor_.reset();

    if (sensor_manager_) {
        sensor_manager_->shutdown();
        sensor_manager_.reset();
    }

    if (adas_interface_) {
        adas_interface_->shutdown();
        adas_interface_.reset();
    }

    can_interface_.reset();

    if (hal_) {
        hal_->shutdown();
        hal_.reset();
    }

    diag_monitor_.reset();
    config_manager_.reset();
    platform_.reset();

    state_ = VcuState::OFF;
}

VcuState VcuService::get_state() const {
    return state_;
}

void VcuService::main_loop() {
    auto time_interface = platform_->create_time_interface();
    const uint32_t loop_period_ms = 50; // 20Hz control loop

    while (running_ && state_ == VcuState::RUNNING) {
        uint64_t loop_start = time_interface->get_current_time_ms();

        try {
            // Process ADAS commands
            process_ad_commands();

            // Update vehicle state
            update_vehicle_state();

        } catch (const std::exception& e) {
            diag_monitor_->log(diag::LogLevel::ERROR, std::string("Exception in main loop: ") + e.what());
        }

        // Maintain loop timing
        uint64_t loop_end = time_interface->get_current_time_ms();
        uint64_t elapsed = loop_end - loop_start;
        
        if (elapsed < loop_period_ms) {
            time_interface->sleep_ms(loop_period_ms - elapsed);
        }
    }
}

void VcuService::process_ad_commands() {
    if (!adas_interface_) {
        return;
    }

    // Get latest ADAS command - use correct method name
    auto command = adas_interface_->get_latest_adas_command();
    if (command.has_value()) {
        // Process the command with CVT controller
        if (cvt_controller_) {
            // Convert ADAS command to CVT control parameters
            diag_monitor_->log(diag::LogLevel::INFO, "Processing ADAS command");
        }
    }
}

void VcuService::update_vehicle_state() {
    if (!sensor_manager_ || !load_predictor_ || !cvt_controller_) {
        return;
    }

    // Get current sensor data
    common::PerceptionData perception_data;
    if (sensor_manager_->get_current_data(perception_data) == sensors::SensorDataResult::SUCCESS) {
        
        // Predict load based on sensor data - use correct method signature
        common::PredictionResult prediction;
        if (load_predictor_->predict_load(prediction) == prediction::PredictionResult::SUCCESS) {
            
            // Update CVT controller with new data - use correct method
            cvt_controller_->update(perception_data, prediction);
        }
    }
}

} // namespace core
} // namespace vcu
