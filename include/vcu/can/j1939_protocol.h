#ifndef J1939_PROTOCOL_H
#define J1939_PROTOCOL_H

#include "can_frame.h"
#include <cstdint>

namespace vcu {
namespace can {
namespace j1939 {

/**
 * @brief J1939 Parameter Group Numbers (PGNs) relevant to VCU operations.
 * 
 * These PGNs define the message types used for communication between
 * the VCU and other ECUs in the vehicle network.
 */
namespace pgn {
    constexpr uint32_t ENGINE_SPEED_LOAD = 0x00F004;           ///< Engine Speed and Load (EEC1)
    constexpr uint32_t VEHICLE_SPEED = 0x00FEF1;               ///< Vehicle Speed (CCVS1)
    constexpr uint32_t TRANSMISSION_FLUIDS = 0x00FEEE;         ///< Transmission Fluids (TF1)
    constexpr uint32_t ELECTRONIC_TRANSMISSION_CONTROLLER_1 = 0x00F003; ///< ETC1
    constexpr uint32_t ACCELERATOR_PEDAL_POSITION = 0x00F002;  ///< Accelerator Pedal Position (EEC2)
    constexpr uint32_t CVT_CONTROL_COMMAND = 0x00FF10;         ///< Custom PGN for CVT control commands
    constexpr uint32_t CVT_STATUS_REPORT = 0x00FF11;           ///< Custom PGN for CVT status reports
}

/**
 * @struct J1939Header
 * @brief Represents the header information of a J1939 message.
 */
struct J1939Header {
    uint8_t priority = 6;           ///< Message priority (0-7, lower is higher priority)
    uint32_t pgn = 0;               ///< Parameter Group Number
    uint8_t source_address = 0;     ///< Source ECU address
    uint8_t destination_address = 0xFF; ///< Destination ECU address (0xFF = broadcast)

    /**
     * @brief Constructs a 29-bit CAN ID from J1939 header components.
     * @return The constructed CAN ID.
     */
    uint32_t to_can_id() const {
        return (static_cast<uint32_t>(priority) << 26) |
               (pgn << 8) |
               source_address;
    }

    /**
     * @brief Parses a 29-bit CAN ID into J1939 header components.
     * @param can_id The CAN ID to parse.
     */
    void from_can_id(uint32_t can_id) {
        priority = (can_id >> 26) & 0x07;
        pgn = (can_id >> 8) & 0x3FFFF;
        source_address = can_id & 0xFF;
        
        // Extract destination address from PGN if it's a peer-to-peer message
        if ((pgn & 0xFF00) < 0xF000) {
            destination_address = pgn & 0xFF;
            pgn = pgn & 0xFF00;
        } else {
            destination_address = 0xFF; // Broadcast
        }
    }
};

/**
 * @struct EngineData
 * @brief Engine-related data extracted from J1939 messages.
 */
struct EngineData {
    float engine_speed_rpm = 0.0f;      ///< Engine speed in RPM
    float engine_load_percent = 0.0f;   ///< Engine load percentage (0-100%)
    float accelerator_pedal_percent = 0.0f; ///< Accelerator pedal position (0-100%)
    bool data_valid = false;            ///< True if the data is valid and recent
};

/**
 * @struct VehicleData
 * @brief Vehicle-related data extracted from J1939 messages.
 */
struct VehicleData {
    float vehicle_speed_mps = 0.0f;     ///< Vehicle speed in meters per second
    bool data_valid = false;            ///< True if the data is valid and recent
};

/**
 * @struct CvtControlCommand
 * @brief CVT control command to be sent via J1939.
 */
struct CvtControlCommand {
    float target_ratio = 1.0f;         ///< Target transmission ratio
    uint8_t drive_mode = 0;             ///< Drive mode (0=Manual, 1=Plowing, 2=Seeding, 3=Transport)
    bool enable_control = true;         ///< Enable/disable CVT control
};

/**
 * @struct CvtStatusReport
 * @brief CVT status report received via J1939.
 */
struct CvtStatusReport {
    float current_ratio = 1.0f;        ///< Current transmission ratio
    bool is_shifting = false;           ///< True if CVT is currently shifting
    uint8_t fault_code = 0;             ///< Fault code (0 = no fault)
    bool data_valid = false;            ///< True if the data is valid and recent
};

/**
 * @class J1939Protocol
 * @brief Handles encoding and decoding of J1939 messages for VCU operations.
 */
class J1939Protocol {
public:
    /**
     * @brief Decodes engine data from a J1939 CAN frame.
     * @param frame The received CAN frame.
     * @param engine_data Output structure to store decoded data.
     * @return True if decoding was successful, false otherwise.
     */
    static bool decode_engine_data(const CanFrame& frame, EngineData& engine_data);

    /**
     * @brief Decodes vehicle data from a J1939 CAN frame.
     * @param frame The received CAN frame.
     * @param vehicle_data Output structure to store decoded data.
     * @return True if decoding was successful, false otherwise.
     */
    static bool decode_vehicle_data(const CanFrame& frame, VehicleData& vehicle_data);

    /**
     * @brief Decodes CVT status from a J1939 CAN frame.
     * @param frame The received CAN frame.
     * @param cvt_status Output structure to store decoded data.
     * @return True if decoding was successful, false otherwise.
     */
    static bool decode_cvt_status(const CanFrame& frame, CvtStatusReport& cvt_status);

    /**
     * @brief Encodes a CVT control command into a J1939 CAN frame.
     * @param command The CVT control command to encode.
     * @param source_address The source ECU address (typically VCU address).
     * @return The encoded CAN frame.
     */
    static CanFrame encode_cvt_control_command(const CvtControlCommand& command, uint8_t source_address = 0x10);

    /**
     * @brief Checks if a CAN frame contains a supported J1939 message.
     * @param frame The CAN frame to check.
     * @return True if the frame contains a supported J1939 message, false otherwise.
     */
    static bool is_supported_message(const CanFrame& frame);

private:
    /**
     * @brief Extracts a 16-bit value from CAN data with little-endian byte order.
     * @param data Pointer to the data array.
     * @param offset Byte offset in the array.
     * @return The extracted 16-bit value.
     */
    static uint16_t extract_uint16_le(const uint8_t* data, size_t offset);

    /**
     * @brief Extracts a 32-bit value from CAN data with little-endian byte order.
     * @param data Pointer to the data array.
     * @param offset Byte offset in the array.
     * @return The extracted 32-bit value.
     */
    static uint32_t extract_uint32_le(const uint8_t* data, size_t offset);



    /**
     * @brief Inserts a 32-bit value into CAN data with little-endian byte order.
     * @param data Pointer to the data array.
     * @param offset Byte offset in the array.
     * @param value The value to insert.
     */
    static void insert_uint32_le(uint8_t* data, size_t offset, uint32_t value);
};

} // namespace j1939
} // namespace can
} // namespace vcu

#endif // J1939_PROTOCOL_H
