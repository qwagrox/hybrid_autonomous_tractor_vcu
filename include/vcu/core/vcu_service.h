#ifndef VCU_CORE_VCU_SERVICE_H_
#define VCU_CORE_VCU_SERVICE_H_

#include <memory>
#include <thread>
#include <atomic>

#include "vcu/config/json_config_manager.h"
#include "vcu/diag/file_diagnostic_monitor.h"
#include "vcu/hal/linux_hal.h"
#include "vcu/cvt/cvt_controller.h"
#include "vcu/sensors/sensor_data_manager.h"
#include "vcu/prediction/load_predictor.h"

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

    std::atomic<VcuState> state_;
    std::unique_ptr<std::thread> main_thread_;
    std::atomic<bool> running_;

    // Core Modules
    std::shared_ptr<config::JsonConfigManager> config_manager_;
    std::shared_ptr<diag::FileDiagnosticMonitor> diag_monitor_;
    std::shared_ptr<hal::LinuxHal> hal_;
    //std::shared_ptr<ad_interface::AdInterface> ad_interface_;

    // Control and Data Modules
    std::shared_ptr<can::ICanInterface> can_interface_;
    std::shared_ptr<sensors::SensorDataManager> sensor_manager_;
    std::shared_ptr<prediction::LoadPredictor> load_predictor_;
    std::shared_ptr<cvt::CvtController> cvt_controller_;
};

} // namespace core
} // namespace vcu

#endif // VCU_CORE_VCU_SERVICE_H_
