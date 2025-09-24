#ifndef VCU_ADAS_CAN_INTERFACE_H
#define VCU_ADAS_CAN_INTERFACE_H

#include "adas_vcu_protocol.h"
#include "vcu/can/can_interface.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/common/vcu_data_types.h"
#include <functional>
#include <memory>
#include <optional>

namespace vcu {
namespace adas_interface {

/**
 * @class AdasCanInterface
 * @brief VCU侧的智驾CAN通信接口
 * 
 * 职责：
 * - 通过CAN总线接收智驾域控的驾驶指令
 * - 通过CAN总线发送VCU状态信息给智驾域控
 * - 处理智驾-VCU通信协议和安全机制
 * - 管理智驾连接状态和超时检测
 */
class AdasCanInterface {
public:
    // 智驾指令结构 (从CAN接收)
    struct AdasCommand {
        uint32_t timestamp_ms;        // 指令时间戳
        float target_speed_mps;       // 目标速度 (m/s)
        float target_acceleration;    // 目标加速度 (m/s²)
        adas::AdasDriveMode drive_mode; // 驾驶模式
        bool emergency_stop;          // 紧急停车标志
        uint16_t sequence_id;         // 序列号 (用于检测丢包)
        float confidence;             // 指令置信度 (0.0-1.0)
        uint8_t priority;            // 指令优先级 (0-255)
        
        AdasCommand() : timestamp_ms(0), target_speed_mps(0), 
                       target_acceleration(0), drive_mode(adas::AdasDriveMode::MANUAL),
                       emergency_stop(false), sequence_id(0), confidence(0), priority(0) {}
    };
    
    // VCU状态结构 (通过CAN发送)
    struct VcuStatusForAdas {
        uint32_t timestamp_ms;        // 状态时间戳
        float current_speed_mps;      // 当前速度 (m/s)
        float current_acceleration;   // 当前加速度 (m/s²)
        float engine_rpm;             // 发动机转速 (rpm)
        float cvt_ratio;             // CVT传动比
        float load_factor;           // 负载因子 (0.0-1.0)
        adas::AdasVcuState system_state; // 系统状态
        bool adas_ready;             // 智驾就绪标志
        bool cvt_ready;              // CVT就绪标志
        uint8_t fault_codes;         // 故障码位掩码
        float fuel_level_percent;    // 燃油液位百分比
        float coolant_temp_celsius;  // 冷却液温度
        
        VcuStatusForAdas() : timestamp_ms(0), current_speed_mps(0),
                            current_acceleration(0), engine_rpm(0),
                            cvt_ratio(1.0f), load_factor(0.5f),
                            system_state(adas::AdasVcuState::STANDBY),
                            adas_ready(false), cvt_ready(false), fault_codes(0),
                            fuel_level_percent(50.0f), coolant_temp_celsius(85.0f) {}
    };
    
    // 连接状态枚举
    enum class ConnectionState {
        DISCONNECTED = 0,    // 未连接
        CONNECTING = 1,      // 连接中
        CONNECTED = 2,       // 已连接
        TIMEOUT = 3,         // 超时
        ERROR = 4            // 错误
    };
    
    // 回调函数类型
    using AdasCommandCallback = std::function<void(const AdasCommand&)>;
    using ConnectionStateCallback = std::function<void(ConnectionState)>;
    using EmergencyStopCallback = std::function<void()>;
    
    /**
     * @brief 构造函数
     * @param platform 平台接口指针
     * @param can_interface CAN接口
     */
    AdasCanInterface(PlatformInterface* platform,
                     std::shared_ptr<can::ICanInterface> can_interface);
    
    /**
     * @brief 析构函数
     */
    ~AdasCanInterface();
    
    /**
     * @brief 初始化接口
     * @param vcu_can_address VCU的CAN地址
     * @return 初始化结果
     */
    bool initialize(uint8_t vcu_can_address = 0x20);
    
    /**
     * @brief 关闭接口
     */
    void shutdown();
    
    /**
     * @brief 设置智驾指令回调
     * @param callback 回调函数
     */
    void set_adas_command_callback(AdasCommandCallback callback);
    
    /**
     * @brief 设置连接状态回调
     * @param callback 回调函数
     */
    void set_connection_state_callback(ConnectionStateCallback callback);
    
    /**
     * @brief 设置紧急停车回调
     * @param callback 回调函数
     */
    void set_emergency_stop_callback(EmergencyStopCallback callback);
    
    /**
     * @brief 发送VCU状态给智驾系统
     * @param status VCU状态数据
     * @return 发送结果
     */
    bool send_vcu_status(const VcuStatusForAdas& status);
    
    /**
     * @brief 发送VCU能力信息
     * @param capability 能力信息
     * @return 发送结果
     */
    bool send_vcu_capability(const adas::VcuCapabilityInfo& capability);
    
    /**
     * @brief 获取最新的智驾指令
     * @return 最新指令 (如果有)
     */
    std::optional<AdasCommand> get_latest_adas_command() const;
    
    /**
     * @brief 获取连接状态
     * @return 当前连接状态
     */
    ConnectionState get_connection_state() const;
    
    /**
     * @brief 检查指令是否超时
     * @return true如果超时
     */
    bool is_command_timeout() const;
    
    /**
     * @brief 启用/禁用智驾模式
     * @param enable 是否启用
     * @return 操作结果
     */
    bool enable_adas_mode(bool enable);
    
    /**
     * @brief 检查智驾模式是否启用
     * @return true如果启用
     */
    bool is_adas_mode_enabled() const;
    
    /**
     * @brief 发送紧急停车确认
     * @return 发送结果
     */
    bool send_emergency_stop_ack();
    
    /**
     * @brief 发送故障报告
     * @param fault_code 故障码
     * @param description 故障描述
     * @return 发送结果
     */
    bool send_fault_report(uint8_t fault_code, const char* description);
    
    /**
     * @brief 获取通信统计信息
     */
    struct CommunicationStats {
        uint32_t commands_received;      // 接收的指令数
        uint32_t status_messages_sent;   // 发送的状态消息数
        uint32_t heartbeats_sent;        // 发送的心跳数
        uint32_t timeout_errors;         // 超时错误数
        uint32_t checksum_errors;        // 校验和错误数
        uint32_t sequence_errors;        // 序列号错误数
        float average_latency_ms;        // 平均延迟
    };
    
    CommunicationStats get_communication_stats() const;

private:
    // CAN消息处理任务
    void can_receive_task();
    void process_incoming_can_frame(const can::CanFrame& frame);
    void process_adas_drive_command(const can::CanFrame& frame);
    void process_adas_steering_command(const can::CanFrame& frame);
    void process_adas_heartbeat(const can::CanFrame& frame);
    
    // 状态管理任务
    void heartbeat_task();
    void timeout_check_task();
    void update_connection_state(ConnectionState new_state);
    
    // 协议处理
    bool decode_adas_drive_command(const can::CanFrame& frame, AdasCommand& command);
    bool validate_command_sequence(uint16_t sequence_id);
    bool validate_command_checksum(const can::CanFrame& frame);
    
    can::CanFrame encode_vcu_status(const VcuStatusForAdas& status);
    can::CanFrame encode_vcu_capability(const adas::VcuCapabilityInfo& capability);
    can::CanFrame encode_heartbeat();
    can::CanFrame encode_emergency_stop_ack();
    can::CanFrame encode_fault_report(uint8_t fault_code, const char* description);
    
    // 安全检查
    bool is_command_safe(const AdasCommand& command) const;
    bool is_speed_command_valid(float target_speed) const;
    bool is_acceleration_command_valid(float target_acceleration) const;
    
    // 成员变量
    PlatformInterface* platform_;
    std::shared_ptr<can::ICanInterface> can_interface_;
    
    // 平台抽象接口
    std::unique_ptr<ThreadInterface> can_thread_;
    std::unique_ptr<ThreadInterface> heartbeat_thread_;
    std::unique_ptr<ThreadInterface> timeout_thread_;
    std::unique_ptr<MutexInterface> data_mutex_;
    std::unique_ptr<TimeInterface> time_interface_;
    
    // 配置参数
    uint8_t vcu_can_address_;
    uint8_t adas_can_address_;       // 智驾系统CAN地址
    bool is_initialized_;
    bool is_running_;
    bool adas_mode_enabled_;
    
    // 状态数据
    AdasCommand latest_command_;
    ConnectionState connection_state_;
    uint64_t last_command_time_ms_;
    uint64_t last_heartbeat_received_ms_;
    uint64_t last_heartbeat_sent_ms_;
    uint16_t expected_sequence_id_;
    uint16_t tx_sequence_id_;
    
    // 回调函数
    AdasCommandCallback command_callback_;
    ConnectionStateCallback connection_callback_;
    EmergencyStopCallback emergency_stop_callback_;
    
    // 通信统计
    CommunicationStats comm_stats_;
    
    // 安全参数
    static constexpr float MAX_SAFE_SPEED_MPS = 30.0f;      // 最大安全速度
    static constexpr float MAX_SAFE_ACCELERATION = 3.0f;     // 最大安全加速度
    static constexpr float MIN_SAFE_ACCELERATION = -8.0f;    // 最大安全减速度
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 200;     // 指令超时时间
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000; // 心跳间隔
    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;  // 心跳超时
};

} // namespace adas_interface
} // namespace vcu

#endif // VCU_ADAS_CAN_INTERFACE_H
