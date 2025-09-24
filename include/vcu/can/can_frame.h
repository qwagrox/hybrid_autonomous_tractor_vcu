#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <cstdint>
#include <array>
#include <string>
#include <cstring>
#include <cstddef>

namespace vcu {
namespace can {

/**
 * @struct CanFrame
 * @brief A standard, hardware-agnostic representation of a CAN data frame.
 *
 * This structure is used throughout the VCU software to represent CAN messages,
 * ensuring that the application logic remains independent of the underlying
 * CAN hardware or driver implementation.
 */
struct CanFrame {
    uint32_t id = 0;                        ///< CAN message identifier (Standard or Extended)
    uint8_t dlc = 0;                        ///< Data Length Code (0-8 bytes)
    std::array<uint8_t, 8> data{};          ///< CAN message data payload
    bool is_extended = false;               ///< True if the frame uses an extended 29-bit ID
    bool is_error = false;                  ///< True if the frame is an error frame
    bool is_rtr = false;                    ///< True if the frame is a Remote Transmission Request

    /**
     * @brief Default constructor
     */
    CanFrame() = default;

    /**
     * @brief Constructor with ID, data, and DLC
     */
    CanFrame(uint32_t frame_id, const uint8_t* frame_data, uint8_t frame_dlc)
        : id(frame_id), dlc(frame_dlc) {
        if (frame_dlc > 8) {
            dlc = 8;
        }
        if (frame_data && dlc > 0) {
            std::memcpy(data.data(), frame_data, dlc);
        }
    }

    /**
     * @brief Get CAN ID
     */
    uint32_t get_id() const { return id; }

    /**
     * @brief Get DLC
     */
    uint8_t get_dlc() const { return dlc; }

    /**
     * @brief Get data pointer
     */
    const uint8_t* get_data() const { return data.data(); }

    /**
     * @brief Provides a string representation of the CAN frame for logging.
     * @return A formatted string (e.g., "ID: 18FF0010, DLC: 8, Data: 01 02 03 04 05 06 07 08").
     */
    std::string to_string() const {
        std::string frame_str = "ID: " + std::to_string(id) + ", DLC: " + std::to_string(dlc) + ", Data:";
        for (uint8_t i = 0; i < dlc; ++i) {
            char hex_byte[4];
            snprintf(hex_byte, sizeof(hex_byte), " %02X", data[i]);
            frame_str += hex_byte;
        }
        return frame_str;
    }
};

} // namespace can
} // namespace vcu

#endif // CAN_FRAME_H

