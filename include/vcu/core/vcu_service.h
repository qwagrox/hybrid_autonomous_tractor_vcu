#ifndef VCU_CORE_VCU_SERVICE_H_
#define VCU_CORE_VCU_SERVICE_H_

#include <memory>

#include "vcu/adas_interface/adas_can_interface.h"
#include "vcu/config/json_config_manager.h"
#include "vcu/diag/file_diagnostic_monitor.h"
#include "vcu/hal/linux_hal.h"
#include "vcu/cvt/cvt_controller.h"
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

class VcuService {
public:
    VcuService();
    ~VcuService();

    bool initialize(const std::string& config_path);
    void run();
    void shutdown();

    VcuState get_state() const;

private:
    void main_loop();
    void process_ad_commands();
    void update_vehicle_state();

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
    std::shared_ptr<cvt::CvtController> cvt_controller_;
};

} // namespace core
} // namespace vcu

#endif // VCU_CORE_VCU_SERVICE_H_
