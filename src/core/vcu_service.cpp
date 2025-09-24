#include "vcu/core/vcu_service.h"
#include "vcu/platform/platform_factory.h"
#include <chrono>
#include <thread>

namespace vcu {
namespace core {

// 修复：在初始化列表中初始化platform_
VcuService::VcuService()
    : platform_(PlatformFactory::create_platform()),
      state_(VcuState::OFF),
      is_running_(false),
      cvt_controller_(std::make_unique<cvt::CvtController>()),
      sensor_manager_(std::make_unique<sensors::PlatformSensorDataManager>(platform_)),
      load_predictor_(std::make_unique<prediction::PlatformLoadPredictor>(platform_)),
      adas_interface_(nullptr),
      config_manager_(std::make_unique<config::JsonConfigManager>()),
      diagnostic_monitor_(std::make_unique<diag::FileDiagnosticMonitor>("vcu_diagnostics.log")) {
}

VcuService::~VcuService() {
    shutdown();
}

bool VcuService::initialize() {
    if (state_ != VcuState::OFF) {
        return false;
    }

    // Initialize platform
    if (!platform_->initialize()) {
        diagnostic_monitor_->log_fault(1001, "Platform initialization failed");
        return false;
    }

    // Load configuration
    config::VcuConfig config;
    if (!config_manager_->load_configuration("vcu_config.json", config)) {
        diagnostic_monitor_->log_warning("Failed to load configuration, using defaults");
        config = config_manager_->get_default_configuration();
    }

    // Initialize CVT controller
    if (!cvt_controller_->initialize()) {
        diagnostic_monitor_->log_fault(1002, "CVT controller initialization failed");
        return false;
    }

    // Initialize sensor manager
    if (!sensor_manager_->initialize()) {
        diagnostic_monitor_->log_fault(1003, "Sensor manager initialization failed");
        return false;
    }

    // Initialize load predictor
    if (!load_predictor_->initialize()) {
        diagnostic_monitor_->log_fault(1004, "Load predictor initialization failed");
        return false;
    }

    // Initialize ADAS interface
    adas_interface_ = adas_interface::create_adas_interface(platform_);
    if (!adas_interface_) {
        diagnostic_monitor_->log_warning("ADAS interface creation failed, continuing without ADAS");
    } else if (!adas_interface_->initialize()) {
        diagnostic_monitor_->log_warning("ADAS interface initialization failed, continuing without ADAS");
        adas_interface_.reset();
    }

    state_ = VcuState::READY;
    diagnostic_monitor_->log_info("VCU Service initialized successfully");
    return true;
}

void VcuService::shutdown() {
    if (state_ == VcuState::OFF) {
        return;
    }

    stop();

    // Shutdown components in reverse order
    if (adas_interface_) {
        adas_interface_->shutdown();
        adas_interface_.reset();
    }

    if (load_predictor_) {
        load_predictor_->shutdown();
    }

    if (sensor_manager_) {
        sensor_manager_->shutdown();
    }

    if (cvt_controller_) {
        cvt_controller_->shutdown();
    }

    if (platform_) {
        platform_->shutdown();
    }

    state_ = VcuState::OFF;
    diagnostic_monitor_->log_info("VCU Service shutdown completed");
}

bool VcuService::start() {
    if (state_ != VcuState::READY) {
        return false;
    }

    is_running_ = true;
    state_ = VcuState::RUNNING;

    // Start main control loop
    main_thread_ = platform_->create_thread();
    main_thread_->start([this]() {
        main_control_loop();
    });

    diagnostic_monitor_->log_info("VCU Service started");
    return true;
}

void VcuService::stop() {
    if (state_ != VcuState::RUNNING) {
        return;
    }

    is_running_ = false;
    
    if (main_thread_) {
        main_thread_->join();
        main_thread_.reset();
    }

    state_ = VcuState::READY;
    diagnostic_monitor_->log_info("VCU Service stopped");
}

VcuState VcuService::get_state() const {
    return state_;
}

void VcuService::main_control_loop() {
    auto time_interface = platform_->create_time();
    const uint32_t loop_period_ms = 50; // 20Hz control loop

    while (is_running_) {
        uint64_t loop_start = time_interface->get_current_time_ms();

        try {
            // Get sensor data
            common::SensorData sensor_data;
            if (sensor_manager_->get_latest_sensor_data(sensor_data)) {
                // Predict load
                prediction::PredictionResult prediction;
                if (load_predictor_->predict_load(sensor_data, prediction)) {
                    // Update CVT controller
                    cvt_controller_->update_sensor_data(sensor_data);
                    cvt_controller_->update_prediction(prediction);

                    // Get ADAS commands if available
                    if (adas_interface_) {
                        auto adas_command = adas_interface_->get_latest_command();
                        if (adas_command.has_value()) {
                            cvt_controller_->update_adas_command(adas_command.value());
                        }
                    }

                    // Execute control
                    cvt_controller_->execute_control();
                }
            }
        } catch (...) {
            diagnostic_monitor_->log_error("Exception in main control loop");
        }

        // Maintain loop timing
        uint64_t loop_end = time_interface->get_current_time_ms();
        uint64_t elapsed = loop_end - loop_start;
        
        if (elapsed < loop_period_ms) {
            auto sleep_time = platform_->create_time();
            sleep_time->sleep_ms(loop_period_ms - elapsed);
        }
    }
}

} // namespace core
} // namespace vcu
