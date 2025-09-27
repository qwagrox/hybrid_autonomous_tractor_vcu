#include "vcu/cvt/hmcvt_vendor1_strategy.h"
#include "vcu/can/can_frame.h"
#include <cstring>
#include <algorithm>

namespace vcu {
namespace cvt {

// HMCVT_Vendor1 协议常量定义
static constexpr uint32_t CVT_CONTROL_CAN_ID = 0x18FFF023;
static constexpr uint32_t CVT_STATUS_CAN_ID = 0x18FFF024;
static constexpr uint32_t CVT_SPEED_INFO_CAN_ID = 0x18FF6217;
static constexpr uint32_t CVT_PTO_STATUS_CAN_ID = 0x18FF6117;
static constexpr uint32_t CVT_SPEED_SETTING_CAN_ID = 0x18FF6317;
static constexpr uint32_t CVT_PRESSURE_CAN_ID = 0x18FF6617;
static constexpr uint32_t CVT_TEMP_STATUS_CAN_ID = 0x18FF6817;

static constexpr uint8_t CONTROL_MSG_PERIOD_MS = 10;
static constexpr uint8_t STATUS_MSG_PERIOD_MS = 100;

HMCVT_Vendor1_Strategy::HMCVT_Vendor1_Strategy(can::ICanInterface& can_interface)
    : can_interface_(can_interface),
      drive_mode_(common::DriveMode::MANUAL),
      target_ratio_(1.0f),
      control_enabled_(false),
      gear_position_(GearPosition::NEUTRAL),
      cvt_speed_value_(0),
      clutch_percentage_(0),
      brake_enabled_(false),
      last_cvt_control_send_time_(0),
      last_status_send_time_(0),
      // 液压控制相关初始化
      hydraulic_enabled_(false),
      lift_position_(0.0f),
      lift_mode_(common::LiftMode::MANUAL),
      multi_valve_lock_(false),
      hydraulic_ready_(false),
      hydraulic_errors_(0),
      last_hydraulic_control_send_time_(0) {
    
    // 初始化多路阀状态
    for (int i = 0; i < 4; ++i) {
        multi_valve_flows_[i] = 0;
    }
    
    // 初始化液压状态
    hydraulic_state_.lift_position = 0.0f;
    hydraulic_state_.lift_mode = common::LiftMode::MANUAL;
    hydraulic_state_.pressure = 0.0f;
    hydraulic_state_.temperature = 0.0f;
    hydraulic_state_.is_ready = false;
    hydraulic_state_.error_code = 0;
}

void HMCVT_Vendor1_Strategy::init() {
    // 发送初始控制消息
    send_cvt_control_message();
    
    // 初始化液压系统
    if (hydraulic_enabled_) {
        send_hydraulic_control_message();
    }
}

void HMCVT_Vendor1_Strategy::set_target_ratio(float target_ratio) {
    if (target_ratio >= 0.0f && target_ratio <= 2.0f) {
        target_ratio_ = target_ratio;
    }
}

void HMCVT_Vendor1_Strategy::set_drive_mode(common::DriveMode mode) {
    drive_mode_ = mode;
}

void HMCVT_Vendor1_Strategy::update(const common::PerceptionData& perception_data) {
    uint32_t current_time = get_current_time_ms();
    
    // 定期发送CVT控制消息
    if (current_time - last_cvt_control_send_time_ >= CONTROL_MSG_PERIOD_MS) {
        send_cvt_control_message();
        last_cvt_control_send_time_ = current_time;
    }
    
    // 定期发送液压控制消息
    if (hydraulic_enabled_ && 
        current_time - last_hydraulic_control_send_time_ >= CONTROL_MSG_PERIOD_MS) {
        send_hydraulic_control_message();
        last_hydraulic_control_send_time_ = current_time;
    }
    
    // 定期发送状态消息
    if (current_time - last_status_send_time_ >= STATUS_MSG_PERIOD_MS) {
        send_status_message();
        last_status_send_time_ = current_time;
    }
}

common::CvtState HMCVT_Vendor1_Strategy::get_current_state() const {
    common::CvtState state;
    state.current_ratio = target_ratio_;
    state.drive_mode = drive_mode_;
    state.is_ready = control_enabled_;
    state.error_code = 0;
    return state;
}

void HMCVT_Vendor1_Strategy::send_cvt_control_message() {
    can::CanFrame frame;
    frame.id = CVT_CONTROL_CAN_ID;
    frame.dlc = 8;
    
    // 构建CVT控制消息
    frame.data[0] = static_cast<uint8_t>(gear_position_);
    frame.data[1] = static_cast<uint8_t>(target_ratio_ * 100);
    frame.data[2] = cvt_speed_value_;
    frame.data[3] = clutch_percentage_;
    frame.data[4] = brake_enabled_ ? 1 : 0;
    frame.data[5] = control_enabled_ ? 1 : 0;
    frame.data[6] = 0; // 保留
    frame.data[7] = 0; // 保留
    
    can_interface_.send_frame(frame);
}

void HMCVT_Vendor1_Strategy::send_status_message() {
    can::CanFrame frame;
    frame.id = CVT_STATUS_CAN_ID;
    frame.dlc = 8;
    
    // 构建状态消息
    frame.data[0] = static_cast<uint8_t>(get_current_state().current_ratio * 100);
    frame.data[1] = static_cast<uint8_t>(drive_mode_);
    frame.data[2] = control_enabled_ ? 1 : 0;
    frame.data[3] = 0; // 错误代码
    frame.data[4] = 0; // 保留
    frame.data[5] = 0; // 保留
    frame.data[6] = 0; // 保留
    frame.data[7] = 0; // 保留
    
    can_interface_.send_frame(frame);
}

void HMCVT_Vendor1_Strategy::send_engine_start_command() {
    can::CanFrame frame;
    frame.id = CVT_CONTROL_CAN_ID;
    frame.dlc = 8;
    
    // 发动机启动命令
    frame.data[0] = 0x01; // 启动命令
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    
    can_interface_.send_frame(frame);
    send_cvt_control_message();
}

uint32_t HMCVT_Vendor1_Strategy::get_current_time_ms() const {
    // 这里应该返回系统时间，简化实现
    static uint32_t time_counter = 0;
    return ++time_counter;
}

// ========== 液压控制实现 ==========

void HMCVT_Vendor1_Strategy::enable_hydraulic_control(bool enable) {
    hydraulic_enabled_ = enable;
    if (enable) {
        send_hydraulic_control_message();
    }
}

void HMCVT_Vendor1_Strategy::set_lift_position(float position) {
    if (position >= 0.0f && position <= 100.0f) {
        lift_position_ = position;
        hydraulic_state_.lift_position = position;
    }
}

void HMCVT_Vendor1_Strategy::set_lift_mode(common::LiftMode mode) {
    lift_mode_ = mode;
    hydraulic_state_.lift_mode = mode;
}

void HMCVT_Vendor1_Strategy::execute_lift_action(common::LiftAction action) {
    can::CanFrame frame;
    frame.id = LIFT_CONTROL_CAN_ID;
    frame.dlc = 8;
    
    // 构建电控提升控制消息
    frame.data[0] = static_cast<uint8_t>(action);
    frame.data[1] = static_cast<uint8_t>(lift_mode_);
    frame.data[2] = static_cast<uint8_t>(lift_position_);
    frame.data[3] = 0; // 减震设置
    frame.data[4] = 0; // 力位综合设定
    frame.data[5] = 0; // 深度设定
    frame.data[6] = 0; // 保留
    frame.data[7] = 0; // 保留
    
    can_interface_.send_frame(frame);
}

void HMCVT_Vendor1_Strategy::set_multi_valve_flow(uint8_t valve_id, int8_t flow_percent) {
    if (valve_id < 4 && flow_percent >= -100 && flow_percent <= 100) {
        multi_valve_flows_[valve_id] = flow_percent;
    }
}

void HMCVT_Vendor1_Strategy::set_multi_valve_lock(bool lock) {
    multi_valve_lock_ = lock;
}

common::HydraulicState HMCVT_Vendor1_Strategy::get_hydraulic_state() const {
    return hydraulic_state_;
}

void HMCVT_Vendor1_Strategy::execute_hydraulic_command(const common::HydraulicCommand& command) {
    // 执行液压命令
    switch (command.type) {
        case common::HydraulicCommandType::LIFT_UP:
            execute_lift_action(common::LiftAction::LIFT_UP);
            break;
        case common::HydraulicCommandType::LIFT_DOWN:
            execute_lift_action(common::LiftAction::LIFT_DOWN);
            break;
        case common::HydraulicCommandType::LIFT_STOP:
            execute_lift_action(common::LiftAction::STOP);
            break;
        case common::HydraulicCommandType::SET_VALVE_FLOW:
            if (command.valve_id < 4) {
                set_multi_valve_flow(command.valve_id, command.flow_percent);
            }
            break;
        default:
            break;
    }
}

bool HMCVT_Vendor1_Strategy::is_hydraulic_ready() const {
    return hydraulic_ready_ && hydraulic_enabled_;
}

uint32_t HMCVT_Vendor1_Strategy::get_hydraulic_errors() const {
    return hydraulic_errors_;
}

void HMCVT_Vendor1_Strategy::send_hydraulic_control_message() {
    // 发送多路阀控制消息
    can::CanFrame frame;
    frame.id = MULTI_VALVE_CONTROL_CAN_ID;
    frame.dlc = 8;
    
    // 构建多路阀控制消息
    frame.data[0] = static_cast<uint8_t>(multi_valve_flows_[0] + 100); // 转换为0-200范围
    frame.data[1] = static_cast<uint8_t>(multi_valve_flows_[1] + 100);
    frame.data[2] = static_cast<uint8_t>(multi_valve_flows_[2] + 100);
    frame.data[3] = static_cast<uint8_t>(multi_valve_flows_[3] + 100);
    frame.data[4] = multi_valve_lock_ ? 1 : 0;
    frame.data[5] = 0; // 保留
    frame.data[6] = 0; // 保留
    frame.data[7] = 0; // 保留
    
    can_interface_.send_frame(frame);
}

void HMCVT_Vendor1_Strategy::process_can_message(const can::CanFrame& frame) {
    switch (frame.id) {
        case CVT_STATUS_CAN_ID:
            parse_cvt_status_message(frame);
            break;
        case CVT_SPEED_INFO_CAN_ID:
            parse_cvt_speed_info_message(frame);
            break;
        case PRESSURE_STATUS_CAN_ID:
            parse_pressure_status_message(frame);
            break;
        case TEMP_STATUS_CAN_ID:
            parse_temperature_status_message(frame);
            break;
        default:
            // 未知消息ID
            break;
    }
}

void HMCVT_Vendor1_Strategy::parse_cvt_status_message(const can::CanFrame& frame) {
    if (frame.dlc >= 8) {
        // 解析CVT状态消息
        float current_ratio = frame.data[0] / 100.0f;
        common::DriveMode mode = static_cast<common::DriveMode>(frame.data[1]);
        bool is_ready = frame.data[2] != 0;
        uint8_t error_code = frame.data[3];
        
        // 更新状态（这里可以添加状态更新逻辑）
    }
}

void HMCVT_Vendor1_Strategy::parse_cvt_speed_info_message(const can::CanFrame& frame) {
    if (frame.dlc >= 8) {
        // 解析速度信息消息
        uint16_t engine_speed = (frame.data[1] << 8) | frame.data[0];
        uint16_t output_speed = (frame.data[3] << 8) | frame.data[2];
        
        // 更新速度信息（这里可以添加速度信息处理逻辑）
    }
}

void HMCVT_Vendor1_Strategy::parse_pressure_status_message(const can::CanFrame& frame) {
    if (frame.dlc >= 8) {
        // 解析压力状态消息
        uint16_t pressure_raw = (frame.data[1] << 8) | frame.data[0];
        hydraulic_state_.pressure = pressure_raw / 10.0f; // 转换为实际压力值
        
        // 检查压力是否正常
        if (hydraulic_state_.pressure >= 18.0f) {
            hydraulic_ready_ = true;
            hydraulic_errors_ &= ~0x01; // 清除压力错误标志
        } else {
            hydraulic_ready_ = false;
            hydraulic_errors_ |= 0x01; // 设置压力错误标志
        }
    }
}

void HMCVT_Vendor1_Strategy::parse_temperature_status_message(const can::CanFrame& frame) {
    if (frame.dlc >= 8) {
        // 解析温度状态消息
        uint16_t temp_raw = (frame.data[1] << 8) | frame.data[0];
        hydraulic_state_.temperature = temp_raw / 10.0f - 40.0f; // 转换为实际温度值
        
        // 检查温度是否正常
        if (hydraulic_state_.temperature < 90.0f) {
            hydraulic_errors_ &= ~0x02; // 清除温度错误标志
        } else {
            hydraulic_errors_ |= 0x02; // 设置温度错误标志
        }
    }
}

} // namespace cvt
} // namespace vcu
