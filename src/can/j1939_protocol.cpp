#include "vcu/can/j1939_protocol.h"
#include <cstring>

namespace vcu {
namespace can {
namespace j1939 {

bool J1939Protocol::decode_engine_data(const CanFrame& frame, EngineData& engine_data) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    J1939Header header;
    header.from_can_id(frame.id);

    switch (header.pgn) {
        case pgn::ENGINE_SPEED_LOAD: {
            // EEC1 message format (SAE J1939-71)
            // Byte 3-4: Engine Speed (0.125 rpm/bit, offset 0)
            uint16_t raw_speed = extract_uint16_le(frame.data.data(), 3);
            engine_data.engine_speed_rpm = raw_speed * 0.125f;

            // Byte 2: Engine Load (0.5%/bit, offset 0)
            engine_data.engine_load_percent = frame.data[2] * 0.5f;
            
            engine_data.data_valid = true;
            return true;
        }

        case pgn::ACCELERATOR_PEDAL_POSITION: {
            // EEC2 message format
            // Byte 1: Accelerator Pedal Position (0.4%/bit, offset 0)
            engine_data.accelerator_pedal_percent = frame.data[1] * 0.4f;
            
            engine_data.data_valid = true;
            return true;
        }

        default:
            return false;
    }
}

bool J1939Protocol::decode_vehicle_data(const CanFrame& frame, VehicleData& vehicle_data) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    J1939Header header;
    header.from_can_id(frame.id);

    if (header.pgn == pgn::VEHICLE_SPEED) {
        // CCVS1 message format (SAE J1939-71)
        // Byte 1-2: Vehicle Speed (1/256 km/h/bit, offset 0)
        uint16_t raw_speed = extract_uint16_le(frame.data.data(), 1);
        float speed_kmh = raw_speed / 256.0f;
        vehicle_data.vehicle_speed_mps = speed_kmh / 3.6f; // Convert km/h to m/s
        
        vehicle_data.data_valid = true;
        return true;
    }

    return false;
}

bool J1939Protocol::decode_cvt_status(const CanFrame& frame, CvtStatusReport& cvt_status) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    J1939Header header;
    header.from_can_id(frame.id);

    if (header.pgn == pgn::CVT_STATUS_REPORT) {
        // Custom CVT status message format
        // Byte 0-3: Current ratio (IEEE 754 float)
        uint32_t raw_ratio = extract_uint32_le(frame.data.data(), 0);
        std::memcpy(&cvt_status.current_ratio, &raw_ratio, sizeof(float));

        // Byte 4: Status flags
        cvt_status.is_shifting = (frame.data[4] & 0x01) != 0;

        // Byte 5: Fault code
        cvt_status.fault_code = frame.data[5];

        cvt_status.data_valid = true;
        return true;
    }

    return false;
}

CanFrame J1939Protocol::encode_cvt_control_command(const CvtControlCommand& command, uint8_t source_address) {
    CanFrame frame;
    frame.is_extended = true;
    frame.dlc = 8;

    // Construct J1939 header
    J1939Header header;
    header.priority = 3; // High priority for control commands
    header.pgn = pgn::CVT_CONTROL_COMMAND;
    header.source_address = source_address;
    header.destination_address = 0xFF; // Broadcast

    frame.id = header.to_can_id();

    // Encode command data
    // Byte 0-3: Target ratio (IEEE 754 float)
    uint32_t raw_ratio;
    std::memcpy(&raw_ratio, &command.target_ratio, sizeof(float));
    insert_uint32_le(frame.data.data(), 0, raw_ratio);

    // Byte 4: Drive mode
    frame.data[4] = command.drive_mode;

    // Byte 5: Control flags
    frame.data[5] = command.enable_control ? 0x01 : 0x00;

    // Byte 6-7: Reserved
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFF;

    return frame;
}

bool J1939Protocol::is_supported_message(const CanFrame& frame) {
    if (!frame.is_extended) {
        return false;
    }

    J1939Header header;
    header.from_can_id(frame.id);

    switch (header.pgn) {
        case pgn::ENGINE_SPEED_LOAD:
        case pgn::VEHICLE_SPEED:
        case pgn::TRANSMISSION_FLUIDS:
        case pgn::ELECTRONIC_TRANSMISSION_CONTROLLER_1:
        case pgn::ACCELERATOR_PEDAL_POSITION:
        case pgn::CVT_CONTROL_COMMAND:
        case pgn::CVT_STATUS_REPORT:
            return true;
        default:
            return false;
    }
}

uint16_t J1939Protocol::extract_uint16_le(const uint8_t* data, size_t offset) {
    return static_cast<uint16_t>(data[offset]) |
           (static_cast<uint16_t>(data[offset + 1]) << 8);
}

uint32_t J1939Protocol::extract_uint32_le(const uint8_t* data, size_t offset) {
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}



void J1939Protocol::insert_uint32_le(uint8_t* data, size_t offset, uint32_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xFF);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

} // namespace j1939
} // namespace can
} // namespace vcu
