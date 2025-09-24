#include "vcu/can/platform_can_interface.h"
#include "vcu/platform/time_interface.h"

#ifdef PLATFORM_LINUX
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#elif defined(PLATFORM_NUTTX)
#include <nuttx/can.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#endif

namespace vcu {
namespace can {

#ifdef PLATFORM_LINUX
struct LinuxCanData {
    int socket_fd;
    struct sockaddr_can addr;
};
#elif defined(PLATFORM_NUTTX)
struct NuttxCanData {
    int can_fd;
    struct can_msg_s msg;
};
#endif

PlatformCanInterface::PlatformCanInterface(PlatformInterface* platform)
    : platform_(platform),
      bitrate_(0),
      is_initialized_(false),
      is_receiving_(false),
      platform_data_(nullptr) {
    
    callback_mutex_ = platform_->create_mutex();
}

PlatformCanInterface::~PlatformCanInterface() {
    shutdown();
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
        return CanResult::SUCCESS;
    }

    is_receiving_ = true;
    receive_thread_ = platform_->create_thread();
    receive_thread_->start([this]() { this->receive_thread_main(); });

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
    callback_mutex_->lock();
    receive_callback_ = callback;
    callback_mutex_->unlock();
}

void PlatformCanInterface::receive_thread_main() {
    auto time_interface = platform_->create_time_interface();
    
    while (is_receiving_) {
        CanFrame frame;
        CanResult result = platform_specific_receive(frame);
        
        if (result == CanResult::SUCCESS) {
            callback_mutex_->lock();
            if (receive_callback_) {
                receive_callback_(frame);
            }
            callback_mutex_->unlock();
        } else if (result == CanResult::ERROR_TIMEOUT) {
            // Timeout is expected, continue
            time_interface->sleep_ms(10);
        } else {
            // Other errors, sleep and retry
            time_interface->sleep_ms(10);
        }
    }
}

#ifdef PLATFORM_LINUX
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& interface_name, uint32_t /* bitrate */) {
    auto* data = new LinuxCanData();
    platform_data_ = data;

    // Create SocketCAN socket
    data->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (data->socket_fd < 0) {
        delete data;
        platform_data_ = nullptr;
        return CanResult::ERROR_INIT;
    }

    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    
    if (ioctl(data->socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        close(data->socket_fd);
        delete data;
        platform_data_ = nullptr;
        return CanResult::ERROR_INIT;
    }

    // Bind socket to CAN interface
    data->addr.can_family = AF_CAN;
    data->addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(data->socket_fd, (struct sockaddr*)&data->addr, sizeof(data->addr)) < 0) {
        close(data->socket_fd);
        delete data;
        platform_data_ = nullptr;
        return CanResult::ERROR_INIT;
    }

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& frame) {
    auto* data = static_cast<LinuxCanData*>(platform_data_);
    if (!data) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    struct can_frame can_frame;
    can_frame.can_id = frame.get_id();
    can_frame.can_dlc = frame.get_dlc();
    memcpy(can_frame.data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(data->socket_fd, &can_frame, sizeof(can_frame));
    if (bytes_sent != sizeof(can_frame)) {
        return CanResult::ERROR_SEND;
    }

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    auto* data = static_cast<LinuxCanData*>(platform_data_);
    if (!data) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    struct can_frame can_frame;
    ssize_t bytes_received = read(data->socket_fd, &can_frame, sizeof(can_frame));
    
    if (bytes_received == sizeof(can_frame)) {
        frame = CanFrame(can_frame.can_id, can_frame.data, can_frame.can_dlc);
        return CanResult::SUCCESS;
    } else if (bytes_received < 0) {
        return CanResult::ERROR_TIMEOUT;
    } else {
        return CanResult::ERROR_RECEIVE;
    }
}

void PlatformCanInterface::platform_specific_cleanup() {
    auto* data = static_cast<LinuxCanData*>(platform_data_);
    if (data) {
        if (data->socket_fd >= 0) {
            close(data->socket_fd);
        }
        delete data;
        platform_data_ = nullptr;
    }
}

#elif defined(PLATFORM_NUTTX)
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& interface_name, uint32_t bitrate) {
    auto* data = new NuttxCanData();
    platform_data_ = data;

    // Open CAN device
    data->can_fd = open(interface_name.c_str(), O_RDWR);
    if (data->can_fd < 0) {
        delete data;
        platform_data_ = nullptr;
        return CanResult::ERROR_INIT;
    }

    // Configure bitrate if needed
    // Note: NuttX CAN configuration is typically done at compile time
    // or through board-specific initialization

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& frame) {
    auto* data = static_cast<NuttxCanData*>(platform_data_);
    if (!data) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    data->msg.cm_hdr.ch_id = frame.get_id();
    data->msg.cm_hdr.ch_dlc = frame.get_dlc();
    data->msg.cm_hdr.ch_rtr = false;
    data->msg.cm_hdr.ch_error = 0;
    data->msg.cm_hdr.ch_unused = 0;
    
    memcpy(data->msg.cm_data, frame.get_data(), frame.get_dlc());

    ssize_t bytes_sent = write(data->can_fd, &data->msg, sizeof(struct can_msg_s));
    if (bytes_sent != sizeof(struct can_msg_s)) {
        return CanResult::ERROR_SEND;
    }

    return CanResult::SUCCESS;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    auto* data = static_cast<NuttxCanData*>(platform_data_);
    if (!data) {
        return CanResult::ERROR_NOT_INITIALIZED;
    }

    ssize_t bytes_received = read(data->can_fd, &data->msg, sizeof(struct can_msg_s));
    
    if (bytes_received == sizeof(struct can_msg_s)) {
        frame = CanFrame(data->msg.cm_hdr.ch_id, data->msg.cm_data, data->msg.cm_hdr.ch_dlc);
        return CanResult::SUCCESS;
    } else if (bytes_received < 0) {
        return CanResult::ERROR_TIMEOUT;
    } else {
        return CanResult::ERROR_RECEIVE;
    }
}

void PlatformCanInterface::platform_specific_cleanup() {
    auto* data = static_cast<NuttxCanData*>(platform_data_);
    if (data) {
        if (data->can_fd >= 0) {
            close(data->can_fd);
        }
        delete data;
        platform_data_ = nullptr;
    }
}

#else
// Default implementation for unsupported platforms
CanResult PlatformCanInterface::platform_specific_initialize(const std::string& interface_name, uint32_t bitrate) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::platform_specific_send(const CanFrame& frame) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

CanResult PlatformCanInterface::platform_specific_receive(CanFrame& frame) {
    return CanResult::ERROR_NOT_SUPPORTED;
}

void PlatformCanInterface::platform_specific_cleanup() {
    // Nothing to do
}
#endif

} // namespace can
} // namespace vcu
