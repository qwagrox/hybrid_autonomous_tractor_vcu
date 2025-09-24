#include "vcu/core/vcu_service.h"
#include "vcu/adas_interface/adas_interface_factory.h"
#include "vcu/can/can_interface.h"

namespace vcu {
namespace core {

VcuService::VcuService()
    : state_(VcuState::OFF),
      running_(false) {
    // 创建平台抽象层
    platform_ = PlatformFactory::create_platform();
}

VcuService::~VcuService() {
    shutdown();
}

bool VcuService::initialize(const std::string& /* config_path */) {
    if (state_ != VcuState::OFF) {
        return false;
    }

    state_ = VcuState::INITIALIZING;

    try {
        // 初始化配置管理器
        config_manager_ = std::make_shared<config::JsonConfigManager>();
        // 简化：不检查配置加载结果

        // 初始化诊断监控器
        diag_monitor_ = std::make_shared<diag::FileDiagnosticMonitor>();
        // 简化：不检查初始化结果

        // 初始化硬件抽象层
        hal_ = std::make_shared<hal::LinuxHal>();
        // 简化：不检查初始化结果

        // 创建CAN接口
        can_interface_ = can::create_can_interface();
        if (!can_interface_) {
            state_ = VcuState::FAULT;
            return false;
        }

        // 初始化CAN接口
        if (can_interface_->initialize("can0", 500000) != can::CanResult::SUCCESS) {
            state_ = VcuState::FAULT;
            return false;
        }

        // 创建智驾接口
        adas_interface_ = adas_interface::create_adas_can_interface(
            platform_.get(), can_interface_);
        if (!adas_interface_) {
            state_ = VcuState::FAULT;
            return false;
        }

        // 初始化智驾接口
        if (!adas_interface_->initialize(0x20)) {  // VCU CAN地址
            state_ = VcuState::FAULT;
            return false;
        }

        // 初始化传感器数据管理器
        sensor_manager_ = std::make_shared<sensors::PlatformSensorDataManager>(
            platform_.get(), can_interface_);
        // 简化：不检查初始化结果

        // 初始化负载预测器
        load_predictor_ = std::make_shared<prediction::LoadPredictor>();
        // 简化：不检查初始化结果

        // 初始化CVT控制器
        cvt_controller_ = std::make_shared<cvt::CvtController>();
        // 简化：不检查初始化结果

        // 创建主线程
        main_thread_ = platform_->create_thread();
        if (!main_thread_) {
            state_ = VcuState::FAULT;
            return false;
        }

        state_ = VcuState::RUNNING;
        return true;

    } catch (const std::exception& e) {
        state_ = VcuState::FAULT;
        return false;
    }
}

void VcuService::run() {
    if (state_ != VcuState::RUNNING) {
        return;
    }

    running_ = true;

    // 启动主线程
    if (!main_thread_->start([this]() { main_loop(); })) {
        state_ = VcuState::FAULT;
        return;
    }

    // 等待主线程结束
    main_thread_->join();
}

void VcuService::shutdown() {
    if (state_ == VcuState::OFF) {
        return;
    }

    state_ = VcuState::SHUTTING_DOWN;
    running_ = false;

    // 等待主线程结束
    if (main_thread_) {
        main_thread_->join();
        main_thread_.reset();
    }

    // 关闭智驾接口
    if (adas_interface_) {
        adas_interface_->shutdown();
        adas_interface_.reset();
    }

    // 关闭其他组件
    cvt_controller_.reset();
    sensor_manager_.reset();
    load_predictor_.reset();

    if (can_interface_) {
        can_interface_->shutdown();
        can_interface_.reset();
    }

    hal_.reset();
    diag_monitor_.reset();
    config_manager_.reset();
    platform_.reset();

    state_ = VcuState::OFF;
}

VcuState VcuService::get_state() const {
    return state_;
}

void VcuService::main_loop() {
    while (running_ && state_ == VcuState::RUNNING) {
        try {
            // 处理智驾指令
            process_ad_commands();

            // 更新车辆状态
            update_vehicle_state();

            // 短暂休眠
            platform_->create_time_interface()->sleep_ms(10);  // 100Hz主循环

        } catch (const std::exception& e) {
            state_ = VcuState::FAULT;
            break;
        }
    }
}

void VcuService::process_ad_commands() {
    if (!adas_interface_) {
        return;
    }

    // 获取最新的智驾指令
    auto command = adas_interface_->get_latest_adas_command();
    if (command.has_value()) {
        // 处理紧急停车
        if (command->emergency_stop) {
            running_ = false;
        }
    }
}

void VcuService::update_vehicle_state() {
    if (!adas_interface_ || !sensor_manager_) {
        return;
    }

    try {
        // 获取传感器数据
        common::PerceptionData sensor_data;
        auto result = sensor_manager_->get_current_data(sensor_data);
        
        if (result == sensors::SensorDataResult::SUCCESS) {
            // 构建VCU状态报告
            adas_interface::AdasCanInterface::VcuStatusForAdas status;
            status.current_speed_mps = sensor_data.vehicle_speed_mps;
            status.engine_rpm = sensor_data.engine_speed_rpm;
            status.cvt_ratio = 1.0f;
            status.system_state = adas::AdasVcuState::ACTIVE;
            status.adas_ready = true;
            status.cvt_ready = true;
            status.load_factor = sensor_data.engine_load_percent / 100.0f;
            status.fault_codes = 0;

            // 发送状态到智驾系统
            adas_interface_->send_vcu_status(status);
        }

    } catch (const std::exception& e) {
        // 简化错误处理
    }
}

} // namespace core
} // namespace vcu
