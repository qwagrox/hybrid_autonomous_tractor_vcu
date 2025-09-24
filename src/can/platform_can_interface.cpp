#include "vcu/can/platform_can_interface.h"
#include "vcu/can/socketcan_interface.h"
#include <thread>
#include <chrono>

#ifdef PLATFORM_LINUX
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#endif

#ifdef PLATFORM_NUTTX
#include <nuttx/can.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace vcu {
namespace can {

struct PlatformCanData {
#ifdef PLATFORM_LINUX
    int socket_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
#endif
#ifdef PLATFORM_NUTTX
    int can_fd;
    std::string device_path;
#endif
};

PlatformCanInterface::PlatformCanInterface() 
    : is_initialized_(false), is_receiving_(false), platform_data_(std::make_unique<PlatformCanData>()) {
}

PlatformCanInterface::~PlatformCanInterface() {
    // 修复：显式调用本类的shutdown方法，避免虚函数调用
    PlatformCanInterface::shutdown();
}

CanResult PlatformCanInterface::initialize(const std::string& device_path, uint32_t bitrate) {
    if (is_initialized_) {
        return CanResult::ERROR_INIT;
    }

#if defined(PLATFORM_LINUX) || defined(PLATFORM_NUTTX)
    CanResult result = platform_specific_initialize(device_path, bitrate);
    if (result == CanResult::SUCCESS) {
        is_initialized_ = true;
        device_path_ = device_path;
        bitrate_ = bitrate;
    }
    return result;
#else
    // For unsupported platforms, return error
    (void)device_path;  // Suppress unused parameter warning
    (void)bitrate;      // Suppress unused parameter warning
    return CanResult::ERROR_NOT_SUPPORTED;
#endif
}

CanResult PlatformCanInterface::shutdown() {
    if (!is_initialized_) {
        return CanResult::SUCCESS;
    }

    stop_receive();

#ifdef PLATFORM_LINUX
    if (platform_data_->socket_fd >= 0) {
        close(platform_data_->socket_fd);
        platform_data_->socket_fd = -1;
    }
#endif

#ifdef PLATFORM_NUTTX
    if (platform_data_->can_fd >= 0) {
        close(platform_data_->can_fd);
        platform_data_->can_fd = -1;
    }
#endif

    is_initialized_ = false;
    return CanResult::SUCCESS;
}

bool PlatformCanInterface::is_ready() const {
    return is_initialized_;
}

CanResult PlatformCanInterface::send_frame(const CanFrame& frame) {
    if (!is_initialized_) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

#ifdef PLATFORM_LINUX
    struct can_frame can_frame;
    can_frame.can_id = frame.get_id();
    if (frame.is_extended()) {
        can_frame.can_id |= CAN_EFF_FLAG;
    }
    can_frame.can_dlc = frame.get_dlc();
    std::memcpy(can_frame.data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(platform_data_->socket_fd, &can_frame, sizeof(struct can_frame));
    if (bytes_sent != sizeof(struct can_frame)) {
        return CanResult::ERROR_SEND;
    }
    return CanResult::SUCCESS;
#endif

#ifdef PLATFORM_NUTTX
    struct can_msg_s msg;
    msg.cm_hdr.ch_id = frame.get_id();
    msg.cm_hdr.ch_dlc = frame.get_dlc();
    msg.cm_hdr.ch_rtr = false;
    msg.cm_hdr.ch_error = 0;
    msg.cm_hdr.ch_unused = 0;
    
    std::memcpy(msg.cm_data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(platform_data_->can_fd, &msg, sizeof(struct can_msg_s));
    if (bytes_sent != sizeof(struct can_msg_s)) {
        return CanResult::ERROR_SEND;
    }
    return CanResult::SUCCESS;
#endif

    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::start_receive(std::function<void(const CanFrame&)> callback) {
    if (!is_initialized_) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    if (is_receiving_) {
        return CanResult::ERROR_INIT;
    }

    receive_callback_ = callback;
    is_receiving_ = true;

    receive_thread_ = std::thread([this]() {
        while (is_receiving_) {
            CanFrame frame;
            // 修复：移除未使用的变量，直接调用函数
            platform_specific_receive(frame);
            
            if (receive_callback_) {
                receive_callback_(frame);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::stop_receive() {
    if (!is_receiving_) {
        return CanResult::SUCCESS;
    }

    is_receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    return CanResult::SUCCESS;
}

#ifdef PLATFORM_LINUX
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& device_path, uint32_t bitrate) {
    platform_data_->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (platform_data_->socket_fd < 0) {
        return CanResult::ERROR_INIT;
    }

    std::strcpy(platform_data_->ifr.ifr_name, device_path.c_str());
    if (ioctl(platform_data_->socket_fd, SIOCGIFINDEX, &platform_data_->ifr) < 0) {
        close(platform_data_->socket_fd);
        return CanResult::ERROR_INIT;
    }

    platform_data_->addr.can_family = AF_CAN;
    platform_data_->addr.can_ifindex = platform_data_->ifr.ifr_ifindex;

    // 修复：使用C++风格转换替代C风格转换
    if (bind(platform_data_->socket_fd, reinterpret_cast<struct sockaddr*>(&platform_data_->addr), sizeof(platform_data_->addr)) < 0) {
        close(platform_data_->socket_fd);
        return CanResult::ERROR_INIT;
    }

    // Note: Bitrate configuration is typically done via ip command in Linux
    (void)bitrate; // Suppress unused parameter warning

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    struct can_frame can_frame;
    ssize_t bytes_received = read(platform_data_->socket_fd, &can_frame, sizeof(struct can_frame));
    
    if (bytes_received == sizeof(struct can_frame)) {
        bool is_extended = (can_frame.can_id & CAN_EFF_FLAG) != 0;
        uint32_t id = can_frame.can_id & (is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        
        frame = CanFrame(id, can_frame.data, can_frame.can_dlc, is_extended);
        return CanResult::SUCCESS;
    }
    
    return CanResult::ERROR_TIMEOUT;
}
#endif

#ifdef PLATFORM_NUTTX
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& device_path, uint32_t bitrate) {
    platform_data_->device_path = device_path;
    platform_data_->can_fd = open(device_path.c_str(), O_RDWR);
    if (platform_data_->can_fd < 0) {
        return CanResult::ERROR_INIT;
    }

    // Note: NuttX CAN configuration is typically done at compile time
    (void)bitrate; // Suppress unused parameter warning

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    struct can_msg_s msg;
    ssize_t bytes_received = read(platform_data_->can_fd, &msg, sizeof(struct can_msg_s));
    
    if (bytes_received == sizeof(struct can_msg_s)) {
        frame = CanFrame(msg.cm_hdr.ch_id, msg.cm_data, msg.cm_hdr.ch_dlc);
        return CanResult::SUCCESS;
    }
    
    return CanResult::ERROR_TIMEOUT;
}
#endif

#if !defined(PLATFORM_LINUX) && !defined(PLATFORM_NUTTX)
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& /* device_path */, uint32_t /* bitrate */) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& /* frame */) {
    return CanResult::ERROR_NOT_SUPPORTED;
}
#endif

} // namespace can
} // namespace vcu
