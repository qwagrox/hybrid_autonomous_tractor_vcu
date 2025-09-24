#ifndef VCU_ADAS_INTERFACE_H
#define VCU_ADAS_INTERFACE_H

#include "adas_vcu_protocol.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/can/can_interface.h"
#include "vcu/common/vcu_data_types.h"
#include <functional>
#include <memory>

namespace vcu {
namespace adas {

// ADAS指令回调函数类型
using AdasDriveCommandCallback = std::function<void(const AdasDriveCommand&)>;
using AdasSteeringCommandCallback = std::function<void(const AdasSteeringCommand&)>;
using AdasConnectionStateCallback = std::function<void(AdasVcuState)>;

// VCU智驾接口结果枚举
enum class VcuAdasResult {
    SUCCESS = 0,
    ERROR_NOT_INITIALIZED,
    ERROR_CAN_COMM,
    ERROR_INVALID_COMMAND,
    ERROR_TIMEOUT,
    ERROR_CHECKSUM_FAILED,
    ERROR_SEQUENCE_ERROR
};

/**
 * @class VcuAdasInterface
 * @brief VCU侧的智驾系统接口
 * 
 * 负责与智驾域控制器进行通信，接收驾驶指令并发送VCU状态信息
 */
class VcuAdasInterface {
public:
    /**
     * @brief 构造函数
     * @param platform 平台接口指针
     * @param can_interface CAN接口
     */
    VcuAdasInterface(PlatformInterface* platform, 
                     std::shared_ptr<can::ICanInterface> can_interface);
    
    /**
     * @brief 析构函数
     */
    ~VcuAdasInterface();
    
    /**
     * @brief 初始化接口
     * @param vcu_address VCU的CAN地址
     * @return 初始化结果
     */
    VcuAdasResult initialize(uint8_t vcu_address = 0x20);
    
    /**
     * @brief 关闭接口
     * @return 操作结果
     */
    VcuAdasResult shutdown();
    
    /**
     * @brief 设置驾驶指令回调
     * @param callback 回调函数
     */
    void set_drive_command_callback(AdasDriveCommandCallback callback);
    
    /**
     * @brief 设置转向指令回调
     * @param callback 回调函数
     */
    void set_steering_command_callback(AdasSteeringCommandCallback callback);
    
    /**
     * @brief 设置连接状态回调
     * @param callback 回调函数
     */
    void set_connection_state_callback(AdasConnectionStateCallback callback);
    
    /**
     * @brief 发送VCU状态报告
     * @param vcu_data VCU当前数据
     * @return 发送结果
     */
    VcuAdasResult send_vcu_status(const common::VcuData& vcu_data);
    
    /**
     * @brief 发送VCU能力信息
     * @param capability 能力信息
     * @return 发送结果
     */
    VcuAdasResult send_vcu_capability(const VcuCapabilityInfo& capability);
    
    /**
     * @brief 获取当前连接状态
     * @return 连接状态
     */
    AdasVcuState get_connection_state() const;
    
    /**
     * @brief 获取最后接收到的驾驶指令
     * @return 驾驶指令
     */
    const AdasDriveCommand& get_last_drive_command() const;
    
    /**
     * @brief 获取最后接收到的转向指令
     * @return 转向指令
     */
    const AdasSteeringCommand& get_last_steering_command() const;
    
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
    VcuAdasResult enable_adas_mode(bool enable);
    
    /**
     * @brief 检查智驾模式是否启用
     * @return true如果启用
     */
    bool is_adas_mode_enabled() const;

private:
    // 内部方法
    void can_receive_thread_func();
    void heartbeat_thread_func();
    void timeout_check_thread_func();
    
    void on_can_frame_received(const can::CanFrame& frame);
    void process_drive_command(const can::CanFrame& frame);
    void process_steering_command(const can::CanFrame& frame);
    void process_heartbeat(const can::CanFrame& frame);
    
    void send_heartbeat();
    void update_connection_state(AdasVcuState new_state);
    
    VcuStatusReport create_status_report(const common::VcuData& vcu_data);
    
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
    uint8_t vcu_address_;
    bool is_initialized_;
    bool is_running_;
    bool adas_mode_enabled_;
    
    // 状态数据
    AdasVcuState connection_state_;
    AdasDriveCommand last_drive_command_;
    AdasSteeringCommand last_steering_command_;
    uint64_t last_command_time_ms_;
    uint64_t last_heartbeat_time_ms_;
    uint16_t tx_sequence_id_;
    
    // 回调函数
    AdasDriveCommandCallback drive_command_callback_;
    AdasSteeringCommandCallback steering_command_callback_;
    AdasConnectionStateCallback connection_state_callback_;
    
    // 统计信息
    struct Statistics {
        uint32_t messages_received;
        uint32_t messages_sent;
        uint32_t checksum_errors;
        uint32_t timeout_errors;
        uint32_t sequence_errors;
        
        Statistics() : messages_received(0), messages_sent(0), 
                      checksum_errors(0), timeout_errors(0), sequence_errors(0) {}
    } stats_;
};

} // namespace adas
} // namespace vcu

#endif // VCU_ADAS_INTERFACE_H
