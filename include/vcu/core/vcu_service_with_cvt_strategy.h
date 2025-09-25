#ifndef VCU_CORE_VCU_SERVICE_WITH_CVT_STRATEGY_H_
#define VCU_CORE_VCU_SERVICE_WITH_CVT_STRATEGY_H_

#include <memory>

#include "vcu/adas_interface/adas_can_interface.h"
#include "vcu/config/json_config_manager.h"
#include "vcu/diag/file_diagnostic_monitor.h"
#include "vcu/hal/linux_hal.h"
#include "vcu/cvt/cvt_controller_new.h"
#include "vcu/cvt/cvt_config.h"
#include "vcu/sensors/platform_sensor_data_manager.h"
#include "vcu/prediction/load_predictor.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/platform/platform_factory.h"
#include "vcu/platform/thread_interface.h"

namespace vcu {
namespace core {

enum class VcuState {
    OFF,
    INITIALIZING,
    RUNNING,
    SHUTTING_DOWN,
    FAULT
};

/**
 * @class VcuService
 * @brief Main VCU service class with CVT strategy factory support.
 *
 * This class integrates the CVT strategy factory pattern into the VCU service,
 * allowing for flexible CVT vendor support and runtime strategy switching.
 */
class VcuService {
public:
    VcuService();
    ~VcuService();

    /**
     * @brief Initializes the VCU service with configuration.
     *
     * @param config_path Path to the configuration file.
     * @return True if initialization was successful, false otherwise.
     */
    bool initialize(const std::string& config_path);

    /**
     * @brief Runs the main VCU service loop.
     */
    void run();

    /**
     * @brief Shuts down the VCU service gracefully.
     */
    void shutdown();

    /**
     * @brief Gets the current VCU state.
     *
     * @return The current VCU state.
     */
    VcuState get_state() const;

    /**
     * @brief Sets the CVT manufacturer and reconfigures the CVT controller.
     *
     * @param manufacturer The new CVT manufacturer.
     * @return True if successful, false otherwise.
     */
    bool set_cvt_manufacturer(common::CvtManufacturer manufacturer);

    /**
     * @brief Gets the current CVT configuration.
     *
     * @return The current CVT configuration.
     */
    const cvt::CvtConfig& get_cvt_config() const;

    /**
     * @brief Updates the CVT configuration.
     *
     * @param config The new CVT configuration.
     * @return True if successful, false otherwise.
     */
    bool update_cvt_config(const cvt::CvtConfig& config);

    /**
     * @brief Gets the current CVT state.
     *
     * @return The current CVT state.
     */
    common::CvtState get_cvt_state() const;

private:
    /**
     * @brief Main service loop implementation.
     */
    void main_loop();

    /**
     * @brief Processes commands from the ADAS system.
     */
    void process_ad_commands();

    /**
     * @brief Updates the vehicle state based on sensor data.
     */
    void update_vehicle_state();

    /**
     * @brief Initializes the CVT controller with configuration.
     *
     * @return True if successful, false otherwise.
     */
    bool initialize_cvt_controller();

    /**
     * @brief Loads CVT configuration from files and environment.
     *
     * @return The loaded CVT configuration.
     */
    cvt::CvtConfig load_cvt_configuration();

    /**
     * @brief Handles CVT-related errors and recovery.
     *
     * @return True if recovery was successful, false otherwise.
     */
    bool handle_cvt_error_recovery();

    /**
     * @brief Validates system state before CVT operations.
     *
     * @return True if safe to operate CVT, false otherwise.
     */
    bool validate_cvt_operation_conditions() const;

    // Core state
    VcuState state_;
    std::unique_ptr<ThreadInterface> main_thread_;
    bool running_;

    // Platform abstraction
    std::unique_ptr<PlatformInterface> platform_;

    // Core Modules
    std::shared_ptr<config::JsonConfigManager> config_manager_;
    std::shared_ptr<diag::FileDiagnosticMonitor> diag_monitor_;
    std::shared_ptr<hal::LinuxHal> hal_;
    std::shared_ptr<adas_interface::AdasCanInterface> adas_interface_;

    // Control and Data Modules
    std::shared_ptr<can::ICanInterface> can_interface_;
    std::shared_ptr<sensors::PlatformSensorDataManager> sensor_manager_;
    std::shared_ptr<prediction::LoadPredictor> load_predictor_;
    
    // CVT Strategy-based Controller
    std::unique_ptr<cvt::CvtController> cvt_controller_;
    cvt::CvtConfig cvt_config_;

    // CVT monitoring and diagnostics
    uint64_t last_cvt_update_time_;
    uint32_t cvt_error_count_;
    static constexpr uint32_t MAX_CVT_ERRORS = 5;
    static constexpr uint64_t CVT_UPDATE_TIMEOUT_MS = 1000;
};

} // namespace core
} // namespace vcu

#endif // VCU_CORE_VCU_SERVICE_WITH_CVT_STRATEGY_H_
