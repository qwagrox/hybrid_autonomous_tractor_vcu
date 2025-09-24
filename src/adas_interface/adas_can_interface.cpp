#include "vcu/adas_interface/adas_can_interface.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/platform/thread_interface.h"
#include "vcu/platform/mutex_interface.h"
#include "vcu/platform/time_interface.h"
#include "vcu/can/can_frame.h"
#include <cstring>
#include <algorithm>

namespace vcu {
namespace adas_interface {

// CAN消息ID定义 (基于J1939扩展)
static constexpr uint32_t CAN_ID_ADAS_DRIVE_COMMAND = 0x18FF2000;    // PGN 0xFF20
static constexpr uint32_t CAN_ID_ADAS_STEERING_COMMAND = 0x18FF2100;  // PGN 0xFF21
static constexpr uint32_t CAN_ID_ADAS_HEARTBEAT = 0x18FF2200;         // PGN 0xFF22
static constexpr uint32_t CAN_ID_VCU_STATUS_REPORT = 0x18FF2300;      // PGN 0xFF23
static constexpr uint32_t CAN_ID_VCU_CAPABILITY = 0x18FF2400;         // PGN 0xFF24
static constexpr uint32_t CAN_ID_VCU_HEARTBEAT = 0x18FF2500;          // PGN 0xFF25
static constexpr uint32_t CAN_ID_EMERGENCY_STOP_ACK = 0x18FF2600;     // PGN 0xFF26
static constexpr uint32_t CAN_ID_VCU_FAULT_REPORT = 0x18FF2700;       // PGN 0xFF27

AdasCanInterface::AdasCanInterface(PlatformInterface* platform,
                                   std::shared_ptr<can::ICanInterface> can_interface)
    : platform_(platform)
    , can_interface_(can_interface)
    , vcu_can_address_(0x20)
    , adas_can_address_(0x10)
    , is_initialized_(false)
    , is_running_(false)
    , adas_mode_enabled_(false)
    , connection_state_(ConnectionState::DISCONNECTED)
    , last_command_time_ms_(0)
    , last_heartbeat_received_ms_(0)
    , last_heartbeat_sent_ms_(0)
    , expected_sequence_id_(0)
    , tx_sequence_id_(0) {
    
    // 初始化统计信息 - 避免在包含浮点数的结构体上使用memset
    comm_stats_.commands_received = 0;
    comm_stats_.status_messages_sent = 0;
    comm_stats_.heartbeats_sent = 0;
    comm_stats_.timeout_errors = 0;
    comm_stats_.checksum_errors = 0;
    comm_stats_.sequence_errors = 0;
    comm_stats_.average_latency_ms = 0.0f;
}

AdasCanInterface::~AdasCanInterface() {
    shutdown();
}

bool AdasCanInterface::initialize(uint8_t vcu_can_address) {
    if (is_initialized_) {
        return true;
    }
    
    if (!platform_ || !can_interface_) {
        return false;
    }
    
    vcu_can_address_ = vcu_can_address;
    
    // 创建平台抽象接口
    data_mutex_ = platform_->create_mutex();
    time_interface_ = platform_->create_time_interface();
    
    if (!data_mutex_ || !time_interface_) {
        return false;
    }
    
    // 初始化CAN接口
    if (can_interface_->initialize("can0", 500000) != can::CanResult::SUCCESS) {
        return false;
    }
    
    // 创建工作线程
    can_thread_ = platform_->create_thread();
    heartbeat_thread_ = platform_->create_thread();
    timeout_thread_ = platform_->create_thread();
    
    if (!can_thread_ || !heartbeat_thread_ || !timeout_thread_) {
        return false;
    }
    
    // 启动工作线程
    is_running_ = true;
    
    if (!can_thread_->start([this]() { can_receive_task(); })) {
        is_running_ = false;
        return false;
    }
    
    if (!heartbeat_thread_->start([this]() { heartbeat_task(); })) {
        is_running_ = false;
        return false;
    }
    
    if (!timeout_thread_->start([this]() { timeout_check_task(); })) {
        is_running_ = false;
        return false;
    }
    
    // 初始化连接状态
    update_connection_state(ConnectionState::CONNECTING);
    
    is_initialized_ = true;
    return true;
}

void AdasCanInterface::shutdown() {
    if (!is_initialized_) {
        return;
    }
    
    is_running_ = false;
    
    // 等待线程结束
    if (can_thread_) {
        can_thread_->join();
        can_thread_.reset();
    }
    
    if (heartbeat_thread_) {
        heartbeat_thread_->join();
        heartbeat_thread_.reset();
    }
    
    if (timeout_thread_) {
        timeout_thread_->join();
        timeout_thread_.reset();
    }
    
    // 关闭CAN接口
    if (can_interface_) {
        can_interface_->shutdown();
    }
    
    // 清理资源
    data_mutex_.reset();
    time_interface_.reset();
    
    is_initialized_ = false;
}

void AdasCanInterface::set_adas_command_callback(AdasCommandCallback callback) {
    data_mutex_->lock();
    command_callback_ = callback;
    data_mutex_->unlock();
}

void AdasCanInterface::set_connection_state_callback(ConnectionStateCallback callback) {
    data_mutex_->lock();
    connection_callback_ = callback;
    data_mutex_->unlock();
}

void AdasCanInterface::set_emergency_stop_callback(EmergencyStopCallback callback) {
    data_mutex_->lock();
    emergency_stop_callback_ = callback;
    data_mutex_->unlock();
}

bool AdasCanInterface::send_vcu_status(const VcuStatusForAdas& status) {
    if (!is_initialized_ || !can_interface_) {
        return false;
    }
    
    auto frame = encode_vcu_status(status);
    bool result = (can_interface_->send_frame(frame) == can::CanResult::SUCCESS);
    
    if (result) {
        data_mutex_->lock();
        comm_stats_.status_messages_sent++;
        data_mutex_->unlock();
    }
    
    return result;
}

bool AdasCanInterface::send_vcu_capability(const adas::VcuCapabilityInfo& capability) {
    if (!is_initialized_ || !can_interface_) {
        return false;
    }
    
    auto frame = encode_vcu_capability(capability);
    return (can_interface_->send_frame(frame) == can::CanResult::SUCCESS);
}

std::optional<AdasCanInterface::AdasCommand> AdasCanInterface::get_latest_adas_command() const {
    data_mutex_->lock();
    
    if (is_command_timeout()) {
        data_mutex_->unlock();
        return std::nullopt;
    }
    
    auto result = latest_command_;
    data_mutex_->unlock();
    return result;
}

AdasCanInterface::ConnectionState AdasCanInterface::get_connection_state() const {
    data_mutex_->lock();
    auto state = connection_state_;
    data_mutex_->unlock();
    return state;
}

bool AdasCanInterface::is_command_timeout() const {
    if (last_command_time_ms_ == 0) {
        return true;  // 从未收到指令
    }
    
    uint64_t current_time = time_interface_->get_monotonic_time_ms();
    return (current_time - last_command_time_ms_) > COMMAND_TIMEOUT_MS;
}

bool AdasCanInterface::enable_adas_mode(bool enable) {
    data_mutex_->lock();
    
    if (enable && connection_state_ != ConnectionState::CONNECTED) {
        data_mutex_->unlock();
        return false;  // 未连接时不能启用智驾模式
    }
    
    adas_mode_enabled_ = enable;
    data_mutex_->unlock();
    return true;
}

bool AdasCanInterface::is_adas_mode_enabled() const {
    data_mutex_->lock();
    bool enabled = adas_mode_enabled_;
    data_mutex_->unlock();
    return enabled;
}

bool AdasCanInterface::send_emergency_stop_ack() {
    if (!is_initialized_ || !can_interface_) {
        return false;
    }
    
    auto frame = encode_emergency_stop_ack();
    return (can_interface_->send_frame(frame) == can::CanResult::SUCCESS);
}

bool AdasCanInterface::send_fault_report(uint8_t fault_code, const char* description) {
    if (!is_initialized_ || !can_interface_) {
        return false;
    }
    
    auto frame = encode_fault_report(fault_code, description);
    return (can_interface_->send_frame(frame) == can::CanResult::SUCCESS);
}

AdasCanInterface::CommunicationStats AdasCanInterface::get_communication_stats() const {
    data_mutex_->lock();
    auto stats = comm_stats_;
    data_mutex_->unlock();
    return stats;
}

// 私有方法实现

void AdasCanInterface::can_receive_task() {
    // 设置接收回调
    can_interface_->set_receive_callback([this](const can::CanFrame& frame) {
        if (is_running_) {
            process_incoming_can_frame(frame);
        }
    });
    
    // 启动接收
    if (can_interface_->start_receive() != can::CanResult::SUCCESS) {
        data_mutex_->lock();
        comm_stats_.checksum_errors++;
        data_mutex_->unlock();
        return;
    }
    
    // 保持线程运行
    while (is_running_) {
        time_interface_->sleep_ms(100);
    }
    
    // 停止接收
    can_interface_->stop_receive();
}

void AdasCanInterface::process_incoming_can_frame(const can::CanFrame& frame) {
    uint32_t pgn = (frame.id >> 8) & 0xFFFF00;
    
    switch (pgn) {
        case 0xFF2000:  // ADAS驱动指令
            process_adas_drive_command(frame);
            break;
            
        case 0xFF2100:  // ADAS转向指令
            process_adas_steering_command(frame);
            break;
            
        case 0xFF2200:  // ADAS心跳
            process_adas_heartbeat(frame);
            break;
            
        default:
            // 未知消息，忽略
            break;
    }
}

void AdasCanInterface::process_adas_drive_command(const can::CanFrame& frame) {
    AdasCommand command;
    
    if (!decode_adas_drive_command(frame, command)) {
        data_mutex_->lock();
        comm_stats_.checksum_errors++;
        data_mutex_->unlock();
        return;
    }
    
    // 验证指令安全性
    if (!is_command_safe(command)) {
        return;
    }
    
    // 验证序列号
    if (!validate_command_sequence(command.sequence_id)) {
        data_mutex_->lock();
        comm_stats_.sequence_errors++;
        data_mutex_->unlock();
        return;
    }
    
    // 更新状态
    data_mutex_->lock();
    latest_command_ = command;
    last_command_time_ms_ = time_interface_->get_monotonic_time_ms();
    comm_stats_.commands_received++;
    
    // 更新连接状态
    if (connection_state_ != ConnectionState::CONNECTED) {
        update_connection_state(ConnectionState::CONNECTED);
    }
    data_mutex_->unlock();
    
    // 处理紧急停车
    if (command.emergency_stop && emergency_stop_callback_) {
        emergency_stop_callback_();
        send_emergency_stop_ack();
    }
    
    // 调用指令回调
    if (command_callback_) {
        command_callback_(command);
    }
}

void AdasCanInterface::process_adas_steering_command(const can::CanFrame& /* frame */) {
    // 转向指令处理 (如果需要)
    // 目前VCU主要处理驱动指令，转向由其他ECU处理
}

void AdasCanInterface::process_adas_heartbeat(const can::CanFrame& /* frame */) {
    data_mutex_->lock();
    last_heartbeat_received_ms_ = time_interface_->get_monotonic_time_ms();
    
    // 更新连接状态
    if (connection_state_ == ConnectionState::TIMEOUT || 
        connection_state_ == ConnectionState::DISCONNECTED) {
        update_connection_state(ConnectionState::CONNECTED);
    }
    data_mutex_->unlock();
}

void AdasCanInterface::heartbeat_task() {
    while (is_running_) {
        // 发送心跳
        auto frame = encode_heartbeat();
        if (can_interface_->send_frame(frame) == can::CanResult::SUCCESS) {
            data_mutex_->lock();
            last_heartbeat_sent_ms_ = time_interface_->get_monotonic_time_ms();
            comm_stats_.heartbeats_sent++;
            data_mutex_->unlock();
        }
        
        // 等待心跳间隔
        time_interface_->sleep_ms(HEARTBEAT_INTERVAL_MS);
    }
}

void AdasCanInterface::timeout_check_task() {
    while (is_running_) {
        uint64_t current_time = time_interface_->get_monotonic_time_ms();
        bool timeout_detected = false;
        
        data_mutex_->lock();
        
        // 检查心跳超时
        if (last_heartbeat_received_ms_ > 0 && 
            (current_time - last_heartbeat_received_ms_) > HEARTBEAT_TIMEOUT_MS) {
            timeout_detected = true;
        }
        
        // 检查指令超时
        if (last_command_time_ms_ > 0 && 
            (current_time - last_command_time_ms_) > COMMAND_TIMEOUT_MS) {
            timeout_detected = true;
        }
        
        data_mutex_->unlock();
        
        if (timeout_detected) {
            update_connection_state(ConnectionState::TIMEOUT);
            data_mutex_->lock();
            comm_stats_.timeout_errors++;
            data_mutex_->unlock();
        }
        
        // 等待检查间隔
        time_interface_->sleep_ms(100);  // 100ms检查间隔
    }
}

void AdasCanInterface::update_connection_state(ConnectionState new_state) {
    ConnectionState old_state;
    
    data_mutex_->lock();
    old_state = connection_state_;
    connection_state_ = new_state;
    data_mutex_->unlock();
    
    // 状态变化时调用回调
    if (old_state != new_state && connection_callback_) {
        connection_callback_(new_state);
    }
    
    // 连接断开时禁用智驾模式
    if (new_state == ConnectionState::DISCONNECTED || 
        new_state == ConnectionState::TIMEOUT) {
        data_mutex_->lock();
        adas_mode_enabled_ = false;
        data_mutex_->unlock();
    }
}

bool AdasCanInterface::decode_adas_drive_command(const can::CanFrame& frame, AdasCommand& command) {
    if (frame.dlc < 8) {
        return false;
    }
    
    const uint8_t* data = frame.data.data();
    
    // 解析数据字段
    command.timestamp_ms = time_interface_->get_monotonic_time_ms();
    
    // 字节0-1: 目标速度 (0.1 m/s精度)
    uint16_t speed_raw = (data[1] << 8) | data[0];
    command.target_speed_mps = speed_raw * 0.1f;
    
    // 字节2-3: 目标加速度 (0.01 m/s²精度, 有符号)
    int16_t accel_raw = (data[3] << 8) | data[2];
    command.target_acceleration = accel_raw * 0.01f;
    
    // 字节4: 驾驶模式和标志
    uint8_t mode_flags = data[4];
    command.drive_mode = static_cast<adas::AdasDriveMode>(mode_flags & 0x0F);
    command.emergency_stop = (mode_flags & 0x80) != 0;
    
    // 字节5: 序列号
    command.sequence_id = data[5];
    
    // 字节6: 置信度 (0-255 映射到 0.0-1.0)
    command.confidence = data[6] / 255.0f;
    
    // 字节7: 优先级
    command.priority = data[7];
    
    return true;
}

bool AdasCanInterface::validate_command_sequence(uint16_t sequence_id) {
    data_mutex_->lock();
    
    // 第一次接收或序列号重置
    if (expected_sequence_id_ == 0 || sequence_id == 0) {
        expected_sequence_id_ = sequence_id + 1;
        data_mutex_->unlock();
        return true;
    }
    
    // 检查序列号连续性
    if (sequence_id == expected_sequence_id_) {
        expected_sequence_id_ = (sequence_id + 1) % 256;
        data_mutex_->unlock();
        return true;
    }
    
    // 允许一定的序列号跳跃 (网络丢包)
    uint8_t diff = (sequence_id - expected_sequence_id_ + 256) % 256;
    if (diff <= 3) {  // 允许最多3个包的跳跃
        expected_sequence_id_ = (sequence_id + 1) % 256;
        data_mutex_->unlock();
        return true;
    }
    
    data_mutex_->unlock();
    return false;
}

can::CanFrame AdasCanInterface::encode_vcu_status(const VcuStatusForAdas& status) {
    can::CanFrame frame;
    frame.id = CAN_ID_VCU_STATUS_REPORT | vcu_can_address_;
    frame.dlc = 8;
    frame.is_extended = true;
    
    uint8_t* data = frame.data.data();
    
    // 字节0-1: 当前速度 (0.1 m/s精度)
    uint16_t speed_raw = static_cast<uint16_t>(status.current_speed_mps * 10);
    data[0] = speed_raw & 0xFF;
    data[1] = (speed_raw >> 8) & 0xFF;
    
    // 字节2-3: 发动机转速 (1 rpm精度)
    uint16_t rpm_raw = static_cast<uint16_t>(status.engine_rpm);
    data[2] = rpm_raw & 0xFF;
    data[3] = (rpm_raw >> 8) & 0xFF;
    
    // 字节4: CVT传动比 (0.01精度)
    uint8_t ratio_raw = static_cast<uint8_t>(status.cvt_ratio * 100);
    data[4] = ratio_raw;
    
    // 字节5: 状态标志
    uint8_t status_flags = 0;
    status_flags |= static_cast<uint8_t>(status.system_state) & 0x0F;
    if (status.adas_ready) status_flags |= 0x10;
    if (status.cvt_ready) status_flags |= 0x20;
    if (adas_mode_enabled_) status_flags |= 0x40;
    data[5] = status_flags;
    
    // 字节6: 负载因子 (0-255 映射到 0.0-1.0)
    data[6] = static_cast<uint8_t>(status.load_factor * 255);
    
    // 字节7: 故障码
    data[7] = status.fault_codes;
    
    return frame;
}

can::CanFrame AdasCanInterface::encode_vcu_capability(const adas::VcuCapabilityInfo& capability) {
    can::CanFrame frame;
    frame.id = CAN_ID_VCU_CAPABILITY | vcu_can_address_;
    frame.dlc = 8;
    frame.is_extended = true;
    
    uint8_t* data = frame.data.data();
    
    // 编码VCU能力信息
    data[0] = static_cast<uint8_t>(capability.max_speed_mps);
    data[1] = static_cast<uint8_t>(capability.min_speed_mps);
    data[2] = static_cast<uint8_t>(capability.max_cvt_ratio * 100);
    data[3] = static_cast<uint8_t>(capability.min_cvt_ratio * 100);
    data[4] = capability.supported_modes;
    data[5] = capability.protocol_version;
    data[6] = (capability.vcu_serial_number >> 8) & 0xFF;
    data[7] = capability.vcu_serial_number & 0xFF;
    
    return frame;
}

can::CanFrame AdasCanInterface::encode_heartbeat() {
    can::CanFrame frame;
    frame.id = CAN_ID_VCU_HEARTBEAT | vcu_can_address_;
    frame.dlc = 4;
    frame.is_extended = true;
    
    uint8_t* data = frame.data.data();
    
    // 时间戳 (毫秒)
    uint32_t timestamp = static_cast<uint32_t>(time_interface_->get_monotonic_time_ms());
    data[0] = timestamp & 0xFF;
    data[1] = (timestamp >> 8) & 0xFF;
    data[2] = (timestamp >> 16) & 0xFF;
    data[3] = (timestamp >> 24) & 0xFF;
    
    return frame;
}

can::CanFrame AdasCanInterface::encode_emergency_stop_ack() {
    can::CanFrame frame;
    frame.id = CAN_ID_EMERGENCY_STOP_ACK | vcu_can_address_;
    frame.dlc = 2;
    frame.is_extended = true;
    
    uint8_t* data = frame.data.data();
    
    // 确认码
    data[0] = 0xAA;  // 紧急停车确认
    data[1] = static_cast<uint8_t>(tx_sequence_id_++);
    
    return frame;
}

can::CanFrame AdasCanInterface::encode_fault_report(uint8_t fault_code, const char* description) {
    can::CanFrame frame;
    frame.id = CAN_ID_VCU_FAULT_REPORT | vcu_can_address_;
    frame.dlc = 8;
    frame.is_extended = true;
    
    uint8_t* data = frame.data.data();
    
    // 故障码
    data[0] = fault_code;
    
    // 时间戳 (简化版)
    uint32_t timestamp = static_cast<uint32_t>(time_interface_->get_monotonic_time_ms());
    data[1] = timestamp & 0xFF;
    data[2] = (timestamp >> 8) & 0xFF;
    
    // 故障描述 (前5个字符)
    for (int i = 0; i < 5; ++i) {
        if (description && description[i] != '\0') {
            data[3 + i] = description[i];
        } else {
            data[3 + i] = 0;
        }
    }
    
    return frame;
}

bool AdasCanInterface::is_command_safe(const AdasCommand& command) const {
    // 速度安全检查
    if (!is_speed_command_valid(command.target_speed_mps)) {
        return false;
    }
    
    // 加速度安全检查
    if (!is_acceleration_command_valid(command.target_acceleration)) {
        return false;
    }
    
    // 置信度检查
    if (command.confidence < 0.5f) {  // 置信度过低
        return false;
    }
    
    return true;
}

bool AdasCanInterface::is_speed_command_valid(float target_speed) const {
    return target_speed >= 0.0f && target_speed <= MAX_SAFE_SPEED_MPS;
}

bool AdasCanInterface::is_acceleration_command_valid(float target_acceleration) const {
    return target_acceleration >= MIN_SAFE_ACCELERATION && 
           target_acceleration <= MAX_SAFE_ACCELERATION;
}

} // namespace adas_interface
} // namespace vcu
