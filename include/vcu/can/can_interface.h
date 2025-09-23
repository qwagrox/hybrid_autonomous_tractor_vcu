#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include "can_frame.h"
#include <functional>
#include <memory>
#include <string>

namespace vcu {
namespace can {

/**
 * @brief Callback function type for receiving CAN frames.
 * 
 * This callback is invoked whenever a CAN frame is received on the bus.
 * The callback should be non-blocking and thread-safe.
 */
using CanReceiveCallback = std::function<void(const CanFrame& frame)>;

/**
 * @enum CanResult
 * @brief Result codes for CAN operations.
 */
enum class CanResult {
    SUCCESS = 0,        ///< Operation completed successfully
    ERROR_INIT,         ///< Failed to initialize CAN interface
    ERROR_SEND,         ///< Failed to send CAN frame
    ERROR_RECEIVE,      ///< Failed to receive CAN frame
    ERROR_TIMEOUT,      ///< Operation timed out
    ERROR_INVALID_PARAM ///< Invalid parameter provided
};

/**
 * @class ICanInterface
 * @brief Abstract interface for CAN bus communication.
 *
 * This interface provides a hardware-agnostic abstraction for CAN bus operations.
 * Concrete implementations can support different CAN hardware (e.g., SocketCAN,
 * Peak CAN, Vector CAN) while maintaining the same API for the application layer.
 */
class ICanInterface {
public:
    virtual ~ICanInterface() = default;

    /**
     * @brief Initializes the CAN interface.
     * 
     * @param interface_name The name of the CAN interface (e.g., "can0", "vcan0").
     * @param bitrate The CAN bus bitrate in bits per second (e.g., 250000, 500000).
     * @return CanResult::SUCCESS on success, error code otherwise.
     */
    virtual CanResult initialize(const std::string& interface_name, uint32_t bitrate) = 0;

    /**
     * @brief Shuts down the CAN interface and releases resources.
     * 
     * @return CanResult::SUCCESS on success, error code otherwise.
     */
    virtual CanResult shutdown() = 0;

    /**
     * @brief Sends a CAN frame on the bus.
     * 
     * @param frame The CAN frame to send.
     * @return CanResult::SUCCESS on success, error code otherwise.
     */
    virtual CanResult send_frame(const CanFrame& frame) = 0;

    /**
     * @brief Sets a callback function for receiving CAN frames.
     * 
     * The callback will be invoked for each received CAN frame. Only one callback
     * can be active at a time; setting a new callback replaces the previous one.
     * 
     * @param callback The callback function to invoke on frame reception.
     */
    virtual void set_receive_callback(CanReceiveCallback callback) = 0;

    /**
     * @brief Starts receiving CAN frames asynchronously.
     * 
     * This method starts a background thread or process to receive CAN frames
     * and invoke the registered callback for each received frame.
     * 
     * @return CanResult::SUCCESS on success, error code otherwise.
     */
    virtual CanResult start_receive() = 0;

    /**
     * @brief Stops receiving CAN frames.
     * 
     * @return CanResult::SUCCESS on success, error code otherwise.
     */
    virtual CanResult stop_receive() = 0;

    /**
     * @brief Checks if the CAN interface is currently initialized and ready.
     * 
     * @return True if the interface is ready for communication, false otherwise.
     */
    virtual bool is_ready() const = 0;

    /**
     * @brief Gets the name of the CAN interface.
     * 
     * @return The interface name (e.g., "can0").
     */
    virtual std::string get_interface_name() const = 0;

    /**
     * @brief Gets the configured bitrate of the CAN interface.
     * 
     * @return The bitrate in bits per second.
     */
    virtual uint32_t get_bitrate() const = 0;
};

/**
 * @brief Factory function to create a CAN interface implementation.
 * 
 * This function creates the appropriate CAN interface implementation based on
 * the runtime environment. Currently supports SocketCAN on Linux.
 * 
 * @return A unique pointer to the CAN interface implementation.
 */
std::unique_ptr<ICanInterface> create_can_interface();

} // namespace can
} // namespace vcu

#endif // CAN_INTERFACE_H
