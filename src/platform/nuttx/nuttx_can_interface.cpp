#include "nuttx_can_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

NuttxCanInterface::NuttxCanInterface() : can_fd_(-1), initialized_(false) {}

NuttxCanInterface::~NuttxCanInterface() {
    NuttxCanInterface::shutdown();  // 显式调用本类的shutdown方法
}

CanResult NuttxCanInterface::initialize(const std::string& interface_name) {
    if (initialized_) {
        return CanResult::SUCCESS;
    }
    
    // NuttX中CAN设备通常是 /dev/can0, /dev/can1 等
    std::string device_path = "/dev/" + interface_name;
    
    can_fd_ = open(device_path.c_str(), O_RDWR);
    if (can_fd_ < 0) {
        return CanResult::ERROR_INIT_FAILED;
    }
    
    initialized_ = true;
    return CanResult::SUCCESS;
}

CanResult NuttxCanInterface::shutdown() {
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
    }
    initialized_ = false;
    return CanResult::SUCCESS;
}

bool NuttxCanInterface::is_initialized() const {
    return initialized_;
}

CanResult NuttxCanInterface::send_frame(const CanFrame& frame) {
    if (!initialized_ || can_fd_ < 0) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }
    
    // 将CanFrame转换为NuttX CAN消息格式
    struct can_msg_s msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.cm_hdr.ch_id = frame.get_id();
    msg.cm_hdr.ch_dlc = frame.get_dlc();
    msg.cm_hdr.ch_rtr = frame.is_rtr() ? 1 : 0;
    msg.cm_hdr.ch_extid = frame.is_extended() ? 1 : 0;
    
    memcpy(msg.cm_data, frame.get_data(), frame.get_dlc());
    
    ssize_t bytes_written = write(can_fd_, &msg, sizeof(msg));
    if (bytes_written == sizeof(msg)) {
        return CanResult::SUCCESS;
    } else {
        return CanResult::ERROR_SEND_FAILED;
    }
}

CanResult NuttxCanInterface::receive_frame(CanFrame& frame) {
    if (!initialized_ || can_fd_ < 0) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }
    
    struct can_msg_s msg;
    ssize_t bytes_read = read(can_fd_, &msg, sizeof(msg));
    
    if (bytes_read == sizeof(msg)) {
        // 将NuttX CAN消息转换为CanFrame
        frame.set_id(msg.cm_hdr.ch_id);
        frame.set_dlc(msg.cm_hdr.ch_dlc);
        frame.set_rtr(msg.cm_hdr.ch_rtr != 0);
        frame.set_extended(msg.cm_hdr.ch_extid != 0);
        frame.set_data(msg.cm_data, msg.cm_hdr.ch_dlc);
        
        return CanResult::SUCCESS;
    } else {
        return CanResult::ERROR_RECEIVE_FAILED;
    }
}

CanResult NuttxCanInterface::set_receive_callback(std::function<void(const CanFrame&)> callback) {
    // NuttX版本暂时不支持异步回调，可以在后续版本中实现
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult NuttxCanInterface::start_receive() {
    // NuttX版本使用同步接收
    return CanResult::SUCCESS;
}

CanResult NuttxCanInterface::stop_receive() {
    // NuttX版本使用同步接收
    return CanResult::SUCCESS;
}
