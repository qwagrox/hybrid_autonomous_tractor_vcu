#include "vcu/can/socketcan_interface.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

namespace vcu {
namespace can {

SocketCanInterface::SocketCanInterface()
    : socket_fd_(-1),
      bitrate_(0),
      is_initialized_(false),
      is_receiving_(false) {
}

SocketCanInterface::~SocketCanInterface() {
    // This is safe because we're in the destructor of the final class
    // cppcheck-suppress virtualCallInConstructor
    shutdown();
}

CanResult SocketCanInterface::initialize(const std::string& interface_name, uint32_t bitrate) {
    if (is_initialized_.load()) {
        return CanResult::ERROR_INIT;
    }

    interface_name_ = interface_name;
    bitrate_ = bitrate;

    // Create SocketCAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create SocketCAN socket" << std::endl;
        return CanResult::ERROR_INIT;
    }

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << interface_name << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return CanResult::ERROR_INIT;
    }

    // Bind socket to CAN interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket to " << interface_name << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return CanResult::ERROR_INIT;
    }

    // Enable reception of own messages (for loopback testing)
    int loopback = 1;
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // Disable filters to receive all messages
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);

    is_initialized_.store(true);
    return CanResult::SUCCESS;
}

CanResult SocketCanInterface::shutdown() {
    if (!is_initialized_.load()) {
        return CanResult::SUCCESS;
    }

    // Stop receiving first
    stop_receive();

    // Close socket
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    is_initialized_.store(false);
    return CanResult::SUCCESS;
}

CanResult SocketCanInterface::send_frame(const CanFrame& frame) {
    if (!is_initialized_.load()) {
        return CanResult::ERROR_INIT;
    }

    ::can_frame linux_frame;
    convert_to_linux_frame(frame, linux_frame);

    ssize_t bytes_sent = write(socket_fd_, &linux_frame, sizeof(linux_frame));
    if (bytes_sent != sizeof(linux_frame)) {
        std::cerr << "Failed to send CAN frame" << std::endl;
        return CanResult::ERROR_SEND;
    }

    return CanResult::SUCCESS;
}

void SocketCanInterface::set_receive_callback(CanReceiveCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    receive_callback_ = std::move(callback);
}

CanResult SocketCanInterface::start_receive() {
    if (!is_initialized_.load()) {
        return CanResult::ERROR_INIT;
    }

    if (is_receiving_.load()) {
        return CanResult::SUCCESS; // Already receiving
    }

    is_receiving_.store(true);
    receive_thread_ = std::thread(&SocketCanInterface::receive_thread_main, this);

    return CanResult::SUCCESS;
}

CanResult SocketCanInterface::stop_receive() {
    if (!is_receiving_.load()) {
        return CanResult::SUCCESS; // Not receiving
    }

    is_receiving_.store(false);

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    return CanResult::SUCCESS;
}

bool SocketCanInterface::is_ready() const {
    return is_initialized_.load() && socket_fd_ >= 0;
}

std::string SocketCanInterface::get_interface_name() const {
    return interface_name_;
}

uint32_t SocketCanInterface::get_bitrate() const {
    return bitrate_;
}

void SocketCanInterface::receive_thread_main() {
    ::can_frame linux_frame;
    
    while (is_receiving_.load()) {
        ssize_t bytes_received = read(socket_fd_, &linux_frame, sizeof(linux_frame));
        
        if (bytes_received == sizeof(linux_frame)) {
            CanFrame vcu_frame;
            convert_from_linux_frame(linux_frame, vcu_frame);

            // Invoke callback if set
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (receive_callback_) {
                receive_callback_(vcu_frame);
            }
        } else if (bytes_received < 0) {
            // Error occurred, but continue if still supposed to be receiving
            if (is_receiving_.load()) {
                std::cerr << "Error reading from CAN socket" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
}

void SocketCanInterface::convert_to_linux_frame(const CanFrame& vcu_frame, ::can_frame& linux_frame) {
    std::memset(&linux_frame, 0, sizeof(linux_frame));

    // Set CAN ID and flags
    linux_frame.can_id = vcu_frame.id;
    if (vcu_frame.is_extended) {
        linux_frame.can_id |= CAN_EFF_FLAG;
    }
    if (vcu_frame.is_rtr) {
        linux_frame.can_id |= CAN_RTR_FLAG;
    }
    if (vcu_frame.is_error) {
        linux_frame.can_id |= CAN_ERR_FLAG;
    }

    // Set data length and payload
    linux_frame.can_dlc = vcu_frame.dlc;
    std::memcpy(linux_frame.data, vcu_frame.data.data(), vcu_frame.dlc);
}

void SocketCanInterface::convert_from_linux_frame(const ::can_frame& linux_frame, CanFrame& vcu_frame) {
    // Extract CAN ID and flags
    vcu_frame.id = linux_frame.can_id & CAN_EFF_MASK;
    vcu_frame.is_extended = (linux_frame.can_id & CAN_EFF_FLAG) != 0;
    vcu_frame.is_rtr = (linux_frame.can_id & CAN_RTR_FLAG) != 0;
    vcu_frame.is_error = (linux_frame.can_id & CAN_ERR_FLAG) != 0;

    // Set data length and payload
    vcu_frame.dlc = linux_frame.can_dlc;
    vcu_frame.data.fill(0);
    std::memcpy(vcu_frame.data.data(), linux_frame.data, linux_frame.can_dlc);
}

} // namespace can
} // namespace vcu
