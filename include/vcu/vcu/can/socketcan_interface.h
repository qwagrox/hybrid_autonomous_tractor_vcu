#ifndef SOCKETCAN_INTERFACE_H
#define SOCKETCAN_INTERFACE_H

#include "can_interface.h"
#include <thread>
#include <atomic>
#include <mutex>

// Forward declaration to avoid including Linux headers in the header file
struct can_frame;

namespace vcu {
namespace can {

/**
 * @class SocketCanInterface
 * @brief Linux SocketCAN implementation of the CAN interface.
 *
 * This class provides a concrete implementation of the ICanInterface using
 * Linux SocketCAN. It supports both real CAN hardware and virtual CAN interfaces
 * for testing and simulation.
 */
class SocketCanInterface : public ICanInterface {
public:
    /**
     * @brief Constructs a new SocketCanInterface.
     */
    SocketCanInterface();

    /**
     * @brief Destroys the SocketCanInterface and cleans up resources.
     */
    ~SocketCanInterface() override;

    // ICanInterface implementation
    CanResult initialize(const std::string& interface_name, uint32_t bitrate) override;
    CanResult shutdown() override;
    CanResult send_frame(const CanFrame& frame) override;
    void set_receive_callback(CanReceiveCallback callback) override;
    CanResult start_receive() override;
    CanResult stop_receive() override;
    bool is_ready() const override;
    std::string get_interface_name() const override;
    uint32_t get_bitrate() const override;

private:
    /**
     * @brief Main loop for the receive thread.
     */
    void receive_thread_main();

    /**
     * @brief Converts a CanFrame to a Linux can_frame structure.
     * @param vcu_frame The VCU CAN frame to convert.
     * @param linux_frame Output Linux can_frame structure.
     */
    void convert_to_linux_frame(const CanFrame& vcu_frame, ::can_frame& linux_frame);

    /**
     * @brief Converts a Linux can_frame structure to a CanFrame.
     * @param linux_frame The Linux can_frame to convert.
     * @param vcu_frame Output VCU CAN frame.
     */
    void convert_from_linux_frame(const ::can_frame& linux_frame, CanFrame& vcu_frame);

    int socket_fd_;                     ///< SocketCAN file descriptor
    std::string interface_name_;        ///< CAN interface name (e.g., "can0")
    uint32_t bitrate_;                  ///< CAN bus bitrate
    std::atomic<bool> is_initialized_;  ///< True if the interface is initialized
    std::atomic<bool> is_receiving_;    ///< True if receiving is active
    std::thread receive_thread_;        ///< Background thread for receiving frames
    CanReceiveCallback receive_callback_; ///< Callback for received frames
    mutable std::mutex callback_mutex_; ///< Mutex to protect callback access
};

} // namespace can
} // namespace vcu

#endif // SOCKETCAN_INTERFACE_H
