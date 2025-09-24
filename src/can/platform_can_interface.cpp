#include "vcu/can/platform_can_interface.h"
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

PlatformCanInterface::PlatformCanInterface(PlatformInterface* platform)
    : platform_(platform),
      bitrate_(0),
      is_initialized_(false),
      is_receiving_(false),
      platform_data_(nullptr) {
    
    if (platform_) {
        callback_mutex_ = platform_->create_mutex();
    }
    
    // Allocate platform-specific data
    platform_data_ = new PlatformCanData();
    
#ifdef PLATFORM_LINUX
    static_cast<PlatformCanData*>(platform_data_)->socket_fd = -1;
#endif
#ifdef PLATFORM_NUTTX
    static_cast<PlatformCanData*>(platform_data_)->can_fd = -1;
#endif
}

PlatformCanInterface::~PlatformCanInterface() {
    shutdown();
    if (platform_data_) {
        delete static_cast<PlatformCanData*>(platform_data_);
        platform_data_ = nullptr;
    }
}

CanResult PlatformCanInterface::initialize(const std::string& interface_name, uint32_t bitrate) {
    if (is_initialized_) {
        return CanResult::ERROR_INIT;
    }

    interface_name_ = interface_name;
    bitrate_ = bitrate;

    CanResult result = platform_specific_initialize(interface_name, bitrate);
    if (result == CanResult::SUCCESS) {
        is_initialized_ = true;
    }

    return result;
}

CanResult PlatformCanInterface::shutdown() {
    if (!is_initialized_) {
        return CanResult::SUCCESS;
    }

    stop_receive();
    platform_specific_cleanup();
    
    is_initialized_ = false;
    return CanResult::SUCCESS;
}

bool PlatformCanInterface::is_ready() const {
    return is_initialized_;
}

std::string PlatformCanInterface::get_interface_name() const {
    return interface_name_;
}

uint32_t PlatformCanInterface::get_bitrate() const {
    return bitrate_;
}

CanResult PlatformCanInterface::send_frame(const CanFrame& frame) {
    if (!is_initialized_) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    return platform_specific_send(frame);
}

CanResult PlatformCanInterface::start_receive() {
    if (!is_initialized_) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    if (is_receiving_) {
        return CanResult::ERROR_INIT;
    }

    is_receiving_ = true;
    
    if (platform_) {
        receive_thread_ = platform_->create_thread();
        receive_thread_->start([this]() {
            receive_thread_main();
        });
    }

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::stop_receive() {
    if (!is_receiving_) {
        return CanResult::SUCCESS;
    }

    is_receiving_ = false;
    
    if (receive_thread_) {
        receive_thread_->join();
        receive_thread_.reset();
    }

    return CanResult::SUCCESS;
}

void PlatformCanInterface::set_receive_callback(std::function<void(const CanFrame&)> callback) {
    if (callback_mutex_) {
        callback_mutex_->lock();
        receive_callback_ = callback;
        callback_mutex_->unlock();
    } else {
        receive_callback_ = callback;
    }
}

void PlatformCanInterface::receive_thread_main() {
    while (is_receiving_) {
        CanFrame frame;
        CanResult result = platform_specific_receive(frame);
        
        if (result == CanResult::SUCCESS && receive_callback_) {
            if (callback_mutex_) {
                callback_mutex_->lock();
                if (receive_callback_) {
                    receive_callback_(frame);
                }
                callback_mutex_->unlock();
            } else {
                receive_callback_(frame);
            }
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

#ifdef PLATFORM_LINUX
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& interface_name, uint32_t bitrate) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    data->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->socket_fd < 0) {
        return CanResult::ERROR_INIT;
    }

    std::strcpy(data->ifr.ifr_name, interface_name.c_str());
    if (ioctl(data->socket_fd, SIOCGIFINDEX, &data->ifr) < 0) {
        close(data->socket_fd);
        data->socket_fd = -1;
        return CanResult::ERROR_INIT;
    }

    data->addr.can_family = AF_CAN;
    data->addr.can_ifindex = data->ifr.ifr_ifindex;

    if (bind(data->socket_fd, reinterpret_cast<struct sockaddr*>(&data->addr), sizeof(data->addr)) < 0) {
        close(data->socket_fd);
        data->socket_fd = -1;
        return CanResult::ERROR_INIT;
    }

    // Note: Bitrate configuration is typically done via ip command in Linux
    (void)bitrate; // Suppress unused parameter warning

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& frame) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    struct can_frame can_frame;
    can_frame.can_id = frame.get_id();
    if (frame.is_extended) {
        can_frame.can_id |= CAN_EFF_FLAG;
    }
    can_frame.can_dlc = frame.get_dlc();
    std::memcpy(can_frame.data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(data->socket_fd, &can_frame, sizeof(struct can_frame));
    if (bytes_sent != sizeof(struct can_frame)) {
        return CanResult::ERROR_SEND;
    }
    
    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    struct can_frame can_frame;
    ssize_t bytes_received = read(data->socket_fd, &can_frame, sizeof(struct can_frame));
    
    if (bytes_received == sizeof(struct can_frame)) {
        bool is_extended = (can_frame.can_id & CAN_EFF_FLAG) != 0;
        uint32_t id = can_frame.can_id & (is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        
        frame = CanFrame(id, can_frame.data, can_frame.can_dlc);
        frame.is_extended = is_extended;
        return CanResult::SUCCESS;
    }
    
    return CanResult::ERROR_TIMEOUT;
}

void PlatformCanInterface::platform_specific_cleanup() {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    if (data->socket_fd >= 0) {
        close(data->socket_fd);
        data->socket_fd = -1;
    }
}
#endif

#ifdef PLATFORM_NUTTX
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& interface_name, uint32_t bitrate) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    data->device_path = interface_name;
    data->can_fd = open(interface_name.c_str(), O_RDWR);
    if (data->can_fd < 0) {
        return CanResult::ERROR_INIT;
    }

    // Note: NuttX CAN configuration is typically done at compile time
    (void)bitrate; // Suppress unused parameter warning

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& frame) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    struct can_msg_s msg;
    msg.cm_hdr.ch_id = frame.get_id();
    msg.cm_hdr.ch_dlc = frame.get_dlc();
    msg.cm_hdr.ch_rtr = false;
    msg.cm_hdr.ch_error = 0;
    msg.cm_hdr.ch_unused = 0;
    
    std::memcpy(msg.cm_data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(data->can_fd, &msg, sizeof(struct can_msg_s));
    if (bytes_sent != sizeof(struct can_msg_s)) {
        return CanResult::ERROR_SEND;
    }
    
    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    struct can_msg_s msg;
    ssize_t bytes_received = read(data->can_fd, &msg, sizeof(struct can_msg_s));
    
    if (bytes_received == sizeof(struct can_msg_s)) {
        frame = CanFrame(msg.cm_hdr.ch_id, msg.cm_data, msg.cm_hdr.ch_dlc);
        return CanResult::SUCCESS;
    }
    
    return CanResult::ERROR_TIMEOUT;
}

void PlatformCanInterface::platform_specific_cleanup() {
    auto* data = static_cast<PlatformCanData*>(platform_data_);
    
    if (data->can_fd >= 0) {
        close(data->can_fd);
        data->can_fd = -1;
    }
}
#endif

#if !defined(PLATFORM_LINUX) && !defined(PLATFORM_NUTTX)
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& /* interface_name */, uint32_t /* bitrate */) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& /* frame */) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& /* frame */) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

void PlatformCanInterface::platform_specific_cleanup() {
    // Nothing to do for unsupported platforms
}
#endif

} // namespace can
} // namespace vcu
