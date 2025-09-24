#include "nuttx_can_interface.h"
#include <nuttx/can.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

namespace vcu {
namespace platform {
namespace nuttx {

NuttxCanInterface::NuttxCanInterface() : can_fd_(-1) {}

NuttxCanInterface::~NuttxCanInterface() {
    // 修复：显式调用本类的shutdown方法，避免虚函数调用
    NuttxCanInterface::shutdown();
}

CanResult NuttxCanInterface::initialize(const std::string& device_path, uint32_t bitrate) {
    if (can_fd_ >= 0) {
        return CanResult::ERROR_INIT;
    }

    can_fd_ = open(device_path.c_str(), O_RDWR);
    if (can_fd_ < 0) {
        return CanResult::ERROR_INIT;
    }

    // Configure bitrate if needed
    // Note: NuttX CAN configuration is typically done at compile time

    return CanResult::SUCCESS;
}

CanResult NuttxCanInterface::shutdown() {
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
    }
    return CanResult::SUCCESS;
}

bool NuttxCanInterface::is_ready() const {
    return can_fd_ >= 0;
}

CanResult NuttxCanInterface::send_frame(const CanFrame& frame) {
    if (can_fd_ < 0) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    struct can_msg_s msg;
    msg.cm_hdr.ch_id = frame.get_id();
    msg.cm_hdr.ch_dlc = frame.get_dlc();
    msg.cm_hdr.ch_rtr = false;
    msg.cm_hdr.ch_error = 0;
    msg.cm_hdr.ch_unused = 0;
    
    memcpy(msg.cm_data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(can_fd_, &msg, sizeof(struct can_msg_s));
    if (bytes_sent != sizeof(struct can_msg_s)) {
        return CanResult::ERROR_SEND;
    }

    return CanResult::SUCCESS;
}

CanResult NuttxCanInterface::receive_frame(CanFrame& frame, uint32_t timeout_ms) {
    if (can_fd_ < 0) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    struct can_msg_s msg;
    ssize_t bytes_received = read(can_fd_, &msg, sizeof(struct can_msg_s));
    
    if (bytes_received == sizeof(struct can_msg_s)) {
        frame = CanFrame(msg.cm_hdr.ch_id, msg.cm_data, msg.cm_hdr.ch_dlc);
        return CanResult::SUCCESS;
    } else if (bytes_received < 0) {
        return CanResult::ERROR_TIMEOUT;
    } else {
        return CanResult::ERROR_RECEIVE;
    }
}

} // namespace nuttx
} // namespace platform
} // namespace vcu
