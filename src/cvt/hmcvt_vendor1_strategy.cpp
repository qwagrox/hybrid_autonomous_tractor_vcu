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

HMCVT_Vendor1_Strategy::HMCVT_Vendor1_Strategy(can::CanInterface& can_interface)
    : can_interface_(can_interface),
      drive_mode_(common::DriveMode::MANUAL),
      target_ratio_(1.0f),
      control_enabled_(false),
      gear_position_(GearPosition::NEUTRAL),
      cvt_speed_value_(0),
      clutch_percentage_(0),
      brake_enabled_(false),
      four_wheel_drive_(false),
      differential_lock_(false),
      engine_start_(false) {
    
    // 初始化CVT状态
    cvt_state_.current_ratio = 1.0f;
    cvt_state_.target_ratio = 1.0f;
    cvt_state_.is_shifting = false;
    
    // 初始化内部状态
    last_control_send_time_ = 0;
    last_status_update_time_ = 0;
    
    // 注册CAN消息回调
    register_can_callbacks();
}

void HMCVT_Vendor1_Strategy::init() {
    // 发送初始化序列
    send_engine_start_command();
    
    // 等待CVT响应（实际实现中可能需要状态机）
    // 这里简化处理，直接设置为工作状态
    control_enabled_ = true;
    gear_position_ = GearPosition::NEUTRAL;
    
    // 发送初始控制消息
    send_control_message();
}

void HMCVT_Vendor1_Strategy::set_target_ratio(float target_ratio) {
    // 限制传动比范围：-0.9 到 2.0
    target_ratio_ = std::clamp(target_ratio, -0.9f, 2.0f);
    cvt_state_.target_ratio = target_ratio_;
    
    // 根据目标传动比设置挡位
    if (target_ratio_ < 0) {
        gear_position_ = GearPosition::REVERSE;
    } else if (target_ratio_ > 0) {
        if (target_ratio_ <= 0.607f) {
            gear_position_ = GearPosition::FORWARD_1;
        } else {
            // F2区段暂不开放，限制在F1范围内
            gear_position_ = GearPosition::FORWARD_1;
            target_ratio_ = std::min(target_ratio_, 0.607f);
        }
    } else {
        gear_position_ = GearPosition::NEUTRAL;
    }
    
    // 计算CVT速度控制值
    calculate_cvt_speed_value();
}

void HMCVT_Vendor1_Strategy::update(const common::PerceptionData& perception_data) {
    // 更新内部状态
    update_internal_state(perception_data);
    
    // 检查是否需要发送控制消息
    uint64_t current_time = get_current_time_ms();
    if (current_time - last_control_send_time_ >= CONTROL_MSG_PERIOD_MS) {
        send_control_message();
        last_control_send_time_ = current_time;
    }
    
    // 更新CVT状态
    update_cvt_state();
}

common::CvtState HMCVT_Vendor1_Strategy::get_current_state() const {
    return cvt_state_;
}

void HMCVT_Vendor1_Strategy::set_drive_mode(common::DriveMode mode) {
    drive_mode_ = mode;
    
    // 根据驾驶模式调整控制参数
    switch (drive_mode_) {
        case common::DriveMode::PLOWING:
            // 犁地模式：启用四驱和差速锁
            four_wheel_drive_ = true;
            differential_lock_ = true;
            break;
            
        case common::DriveMode::SEEDING:
            // 播种模式：启用四驱，不锁差速
            four_wheel_drive_ = true;
            differential_lock_ = false;
            break;
            
        case common::DriveMode::TRANSPORT:
            // 运输模式：两驱，不锁差速
            four_wheel_drive_ = false;
            differential_lock_ = false;
            break;
            
        case common::DriveMode::MANUAL:
        default:
            // 手动模式：保持当前设置
            break;
    }
}

void HMCVT_Vendor1_Strategy::send_control_message() {
    can::CanFrame frame;
    frame.id = CVT_CONTROL_CAN_ID;
    frame.dlc = 8;
    std::memset(frame.data, 0, 8);
    
    // Byte0: 离合器分离控制 (0-1位)
    frame.data[0] |= static_cast<uint8_t>(gear_position_) & 0x03;
    
    // Byte1: 速度控制位 (暂时不使用脉冲控制)
    frame.data[1] = 0;
    
    // Byte2: 左转控制 (0-100)
    frame.data[2] = 0; // 由转向系统控制
    
    // Byte3: 右转控制 (0-100)
    frame.data[3] = 0; // 由转向系统控制
    
    // Byte4: 离合器百分比 (0-100)
    frame.data[4] = clutch_percentage_;
    
    // Byte5: 工作状态和其他控制位
    frame.data[5] = 0;
    frame.data[5] |= (control_enabled_ ? 1 : 0) << 0;  // 工作状态
    frame.data[5] |= (brake_enabled_ ? 0 : 1) << 2;    // 刹车控制 (0刹车, 1禁用)
    frame.data[5] |= (four_wheel_drive_ ? 1 : 0) << 4; // 四驱控制
    frame.data[5] |= (differential_lock_ ? 0 : 1) << 6; // 差速锁 (1不锁, 0锁定)
    
    // Byte6: 无级变速值 (-100 to 100)
    frame.data[6] = static_cast<uint8_t>(cvt_speed_value_ + 100); // 偏移100
    
    // Byte7: 启动和熄火控制
    frame.data[7] = 0;
    frame.data[7] |= (engine_start_ ? 1 : 0) << 0; // 启动控制
    
    // 发送CAN消息
    can_interface_.send_frame(frame);
}

void HMCVT_Vendor1_Strategy::send_engine_start_command() {
    engine_start_ = true;
    send_control_message();
    
    // 发送后重置启动标志
    engine_start_ = false;
}

void HMCVT_Vendor1_Strategy::calculate_cvt_speed_value() {
    // 根据当前传动比和目标传动比计算速度控制值
    float ratio_diff = target_ratio_ - cvt_state_.current_ratio;
    
    // 将传动比差值映射到-100到100的控制值
    // 这里使用简单的比例控制
    const float max_ratio_diff = 0.1f; // 最大单次调整幅度
    float normalized_diff = std::clamp(ratio_diff / max_ratio_diff, -1.0f, 1.0f);
    
    cvt_speed_value_ = static_cast<int8_t>(normalized_diff * 100);
}

void HMCVT_Vendor1_Strategy::update_internal_state(const common::PerceptionData& perception_data) {
    // 根据感知数据更新内部状态
    
    // 根据车速和负载调整离合器
    if (perception_data.vehicle_speed_mps < 0.5f && perception_data.engine_load_percent > 80.0f) {
        clutch_percentage_ = 90; // 高负载低速时增加离合器压力
    } else {
        clutch_percentage_ = 70; // 正常离合器压力
    }
    
    // 根据驾驶模式和地形调整刹车
    brake_enabled_ = (perception_data.vehicle_speed_mps > 15.0f); // 高速时启用刹车辅助
}

void HMCVT_Vendor1_Strategy::update_cvt_state() {
    // 模拟CVT状态更新（实际状态从CAN反馈消息中获取）
    const float adjustment_rate = 0.1f; // 10% adjustment per update
    float ratio_diff = cvt_state_.target_ratio - cvt_state_.current_ratio;
    
    if (std::abs(ratio_diff) > 0.01f) {
        cvt_state_.is_shifting = true;
        cvt_state_.current_ratio += ratio_diff * adjustment_rate;
    } else {
        cvt_state_.is_shifting = false;
        cvt_state_.current_ratio = cvt_state_.target_ratio;
    }
}

void HMCVT_Vendor1_Strategy::register_can_callbacks() {
    // 注册CAN消息接收回调
    // 这里简化处理，实际实现需要根据CAN接口的具体API
    
    // 注册状态反馈消息回调
    // can_interface_.register_callback(CVT_STATUS_CAN_ID, 
    //     [this](const can::CanFrame& frame) {
    //         parse_status_message(frame);
    //     });
    
    // 注册速度信息消息回调
    // can_interface_.register_callback(CVT_SPEED_INFO_CAN_ID,
    //     [this](const can::CanFrame& frame) {
    //         parse_speed_info_message(frame);
    //     });
}

void HMCVT_Vendor1_Strategy::parse_status_message(const can::CanFrame& frame) {
    if (frame.id == CVT_STATUS_CAN_ID && frame.dlc >= 8) {
        // 解析CVT状态反馈消息
        
        // Byte0 bit0-1: 开启状态
        bool cvt_enabled = (frame.data[0] & 0x03) == 0x01;
        
        // Byte1-2: 恒转速 (分辨率0.125)
        uint16_t rpm_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[1];
        float engine_rpm = rpm_raw * 0.125f;
        
        // 更新内部状态
        control_enabled_ = cvt_enabled;
        
        // 根据转速估算当前传动比（简化处理）
        if (engine_rpm > 100.0f) {
            // 这里需要根据实际的传动比计算公式来更新
            // cvt_state_.current_ratio = calculate_ratio_from_rpm(engine_rpm);
        }
    }
}

void HMCVT_Vendor1_Strategy::parse_speed_info_message(const can::CanFrame& frame) {
    if (frame.id == CVT_SPEED_INFO_CAN_ID && frame.dlc >= 8) {
        // 解析转速和车速信息
        
        // Byte0-1: 理论车速 (分辨率0.1 km/h)
        uint16_t speed_raw = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[0];
        float theoretical_speed = speed_raw * 0.1f;
        
        // Byte2-3: PTO转速 (分辨率0.125 rpm)
        uint16_t pto_rpm_raw = (static_cast<uint16_t>(frame.data[3]) << 8) | frame.data[2];
        float pto_rpm = pto_rpm_raw * 0.125f;
        
        // Byte6-7: 雷达车速 (分辨率0.1 km/h)
        uint16_t radar_speed_raw = (static_cast<uint16_t>(frame.data[7]) << 8) | frame.data[6];
        float radar_speed = radar_speed_raw * 0.1f;
        
        // 更新状态（这里可以用于验证控制效果）
        last_theoretical_speed_ = theoretical_speed;
        last_radar_speed_ = radar_speed;
    }
}

uint64_t HMCVT_Vendor1_Strategy::get_current_time_ms() const {
    // 获取当前时间（毫秒）
    // 实际实现需要使用平台相关的时间函数
    return 0; // 占位符
}

} // namespace cvt
} // namespace vcu
