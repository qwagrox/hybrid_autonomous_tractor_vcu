#ifndef ADAS_VCU_PROTOCOL_H
#define ADAS_VCU_PROTOCOL_H

#include <cstdint>
#include "vcu/can/can_frame.h"

namespace vcu {
namespace adas {

// ADAS-VCU通信协议定义
namespace protocol {
    // PGN定义 (Parameter Group Numbers)
    constexpr uint32_t PGN_ADAS_DRIVE_COMMAND    = 0xFF20;  // 智驾驱动指令
    constexpr uint32_t PGN_ADAS_STEERING_COMMAND = 0xFF21;  // 智驾转向指令  
    constexpr uint32_t PGN_ADAS_BRAKE_COMMAND    = 0xFF22;  // 智驾制动指令
    constexpr uint32_t PGN_VCU_STATUS_REPORT     = 0xFF23;  // VCU状态报告
    constexpr uint32_t PGN_VCU_CAPABILITY_INFO   = 0xFF24;  // VCU能力信息
    constexpr uint32_t PGN_ADAS_VCU_HEARTBEAT    = 0xFF25;  // 心跳消息
    
    // 通信频率定义
    constexpr uint32_t DRIVE_COMMAND_FREQ_HZ = 20;    // 驱动指令频率
    constexpr uint32_t STATUS_REPORT_FREQ_HZ = 50;    // 状态报告频率
    constexpr uint32_t HEARTBEAT_FREQ_HZ = 1;         // 心跳频率
    
    // 超时定义
    constexpr uint32_t COMMAND_TIMEOUT_MS = 100;      // 指令超时
    constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;   // 心跳超时
}

// 驾驶模式枚举
enum class AdasDriveMode : uint8_t {
    MANUAL = 0,          // 手动模式
    CRUISE = 1,          // 巡航模式  
    AUTO_FOLLOW = 2,     // 自动跟车
    AUTO_NAVIGATE = 3,   // 自动导航
    EMERGENCY_STOP = 4   // 紧急停车
};

// 系统状态枚举
enum class AdasVcuState : uint8_t {
    DISCONNECTED = 0,    // 未连接
    CONNECTING = 1,      // 连接中
    STANDBY = 2,         // 待机
    ACTIVE = 3,          // 激活
    FAULT = 4,           // 故障
    EMERGENCY = 5        // 紧急状态
};

// 智驾驱动指令
struct AdasDriveCommand {
    float target_speed_mps;      // 目标速度 (m/s)
    AdasDriveMode drive_mode;    // 驾驶模式
    uint8_t priority;            // 指令优先级 (0-255)
    bool emergency_stop;         // 紧急停车标志
    uint16_t sequence_id;        // 序列号
    uint8_t checksum;            // 校验和
    
    // 构造函数
    AdasDriveCommand() 
        : target_speed_mps(0.0f)
        , drive_mode(AdasDriveMode::MANUAL)
        , priority(0)
        , emergency_stop(false)
        , sequence_id(0)
        , checksum(0) {}
        
    // 计算校验和
    uint8_t calculate_checksum() const;
    
    // 验证校验和
    bool validate_checksum() const;
};

// 智驾转向指令
struct AdasSteeringCommand {
    float target_steering_angle;  // 目标转向角度 (弧度)
    float steering_rate;          // 转向速率 (弧度/秒)
    uint16_t sequence_id;         // 序列号
    uint8_t checksum;             // 校验和
    
    AdasSteeringCommand()
        : target_steering_angle(0.0f)
        , steering_rate(0.0f)
        , sequence_id(0)
        , checksum(0) {}
        
    uint8_t calculate_checksum() const;
    bool validate_checksum() const;
};

// VCU状态报告
struct VcuStatusReport {
    float current_speed_mps;      // 当前速度 (m/s)
    float current_cvt_ratio;      // 当前CVT传动比
    float engine_rpm;             // 发动机转速
    float engine_load_percent;    // 发动机负载百分比
    AdasVcuState system_state;    // 系统状态
    uint8_t fault_codes;          // 故障码
    bool ready_for_adas;          // 智驾就绪标志
    bool cvt_ready;               // CVT就绪标志
    uint32_t timestamp_ms;        // 时间戳
    uint8_t checksum;             // 校验和
    
    VcuStatusReport()
        : current_speed_mps(0.0f)
        , current_cvt_ratio(1.0f)
        , engine_rpm(0.0f)
        , engine_load_percent(0.0f)
        , system_state(AdasVcuState::DISCONNECTED)
        , fault_codes(0)
        , ready_for_adas(false)
        , cvt_ready(false)
        , timestamp_ms(0)
        , checksum(0) {}
        
    uint8_t calculate_checksum() const;
    bool validate_checksum() const;
};

// VCU能力信息
struct VcuCapabilityInfo {
    float max_speed_mps;          // 最大速度
    float min_speed_mps;          // 最小速度
    float max_cvt_ratio;          // 最大传动比
    float min_cvt_ratio;          // 最小传动比
    uint8_t supported_modes;      // 支持的驾驶模式位掩码
    uint8_t protocol_version;     // 协议版本
    uint32_t vcu_serial_number;   // VCU序列号
    uint8_t checksum;             // 校验和
    
    VcuCapabilityInfo()
        : max_speed_mps(30.0f)
        , min_speed_mps(0.0f)
        , max_cvt_ratio(3.0f)
        , min_cvt_ratio(0.5f)
        , supported_modes(0x1F)  // 支持所有模式
        , protocol_version(1)
        , vcu_serial_number(0)
        , checksum(0) {}
        
    uint8_t calculate_checksum() const;
    bool validate_checksum() const;
};

// 心跳消息
struct AdasVcuHeartbeat {
    uint32_t sender_id;           // 发送者ID
    AdasVcuState sender_state;    // 发送者状态
    uint32_t timestamp_ms;        // 时间戳
    uint16_t sequence_id;         // 序列号
    uint8_t checksum;             // 校验和
    
    AdasVcuHeartbeat()
        : sender_id(0)
        , sender_state(AdasVcuState::DISCONNECTED)
        , timestamp_ms(0)
        , sequence_id(0)
        , checksum(0) {}
        
    uint8_t calculate_checksum() const;
    bool validate_checksum() const;
};

// 协议编解码器
class AdasVcuProtocol {
public:
    // 编码消息到CAN帧
    static can::CanFrame encode_drive_command(const AdasDriveCommand& cmd, uint8_t source_addr = 0x10);
    static can::CanFrame encode_steering_command(const AdasSteeringCommand& cmd, uint8_t source_addr = 0x10);
    static can::CanFrame encode_vcu_status(const VcuStatusReport& status, uint8_t source_addr = 0x20);
    static can::CanFrame encode_vcu_capability(const VcuCapabilityInfo& info, uint8_t source_addr = 0x20);
    static can::CanFrame encode_heartbeat(const AdasVcuHeartbeat& heartbeat, uint8_t source_addr = 0x10);
    
    // 从CAN帧解码消息
    static bool decode_drive_command(const can::CanFrame& frame, AdasDriveCommand& cmd);
    static bool decode_steering_command(const can::CanFrame& frame, AdasSteeringCommand& cmd);
    static bool decode_vcu_status(const can::CanFrame& frame, VcuStatusReport& status);
    static bool decode_vcu_capability(const can::CanFrame& frame, VcuCapabilityInfo& info);
    static bool decode_heartbeat(const can::CanFrame& frame, AdasVcuHeartbeat& heartbeat);
    
    // 消息验证
    static bool is_adas_vcu_message(const can::CanFrame& frame);
    static uint32_t extract_pgn(const can::CanFrame& frame);
    
private:
    // 辅助函数
    static void serialize_float(uint8_t* data, size_t offset, float value);
    static float deserialize_float(const uint8_t* data, size_t offset);
    static void serialize_uint32(uint8_t* data, size_t offset, uint32_t value);
    static uint32_t deserialize_uint32(const uint8_t* data, size_t offset);
    static void serialize_uint16(uint8_t* data, size_t offset, uint16_t value);
    static uint16_t deserialize_uint16(const uint8_t* data, size_t offset);
};

} // namespace adas
} // namespace vcu

#endif // ADAS_VCU_PROTOCOL_H
