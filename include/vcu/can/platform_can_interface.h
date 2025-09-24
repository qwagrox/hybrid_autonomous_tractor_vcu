// Copyright 2025 Manus AI

#ifndef VCU_CAN_PLATFORM_CAN_INTERFACE_H_
#define VCU_CAN_PLATFORM_CAN_INTERFACE_H_

#include "vcu/can/can_interface.h"
#include "vcu/platform/platform_interface.h"
#include "vcu/platform/thread_interface.h"
#include "vcu/platform/mutex_interface.h"
#include <memory>
#include <functional>

namespace vcu {
namespace can {

/**
 * @brief 使用平台抽象层的CAN接口实现
 * 
 * 这个类使用平台抽象接口来实现CAN通信，支持多平台部署
 */
class PlatformCanInterface : public ICanInterface {
public:
    explicit PlatformCanInterface(PlatformInterface* platform);
    virtual ~PlatformCanInterface();

    // ICanInterface implementation
    CanResult initialize(const std::string& interface_name, uint32_t bitrate) override;
    CanResult shutdown() override;
    bool is_ready() const override;
    std::string get_interface_name() const override;
    uint32_t get_bitrate() const override;

    CanResult send_frame(const CanFrame& frame) override;
    CanResult start_receive() override;
    CanResult stop_receive() override;
    void set_receive_callback(std::function<void(const CanFrame&)> callback) override;

private:
    void receive_thread_main();
    CanResult platform_specific_initialize(const std::string& interface_name, uint32_t bitrate);
    CanResult platform_specific_send(const CanFrame& frame);
    CanResult platform_specific_receive(CanFrame& frame);
    void platform_specific_cleanup();

    PlatformInterface* platform_;
    std::unique_ptr<ThreadInterface> receive_thread_;
    std::unique_ptr<MutexInterface> callback_mutex_;
    
    std::string interface_name_;
    uint32_t bitrate_;
    bool is_initialized_;
    bool is_receiving_;
    
    std::function<void(const CanFrame&)> receive_callback_;
    
    // Platform-specific data (will be cast to appropriate type)
    void* platform_data_;
};

} // namespace can
} // namespace vcu

#endif // VCU_CAN_PLATFORM_CAN_INTERFACE_H_
