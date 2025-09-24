
#include "vcu/can/j1939_protocol.h"
#include <cstring>

namespace vcu {
namespace can {

// J1939Header method implementations
uint32_t j1939::J1939Header::to_can_id() const {
    return (static_cast<uint32_t>(priority) << 26) |
           (pgn << 8) |
           source_address;
}

void j1939::J1939Header::from_can_id(uint32_t can_id) {
    priority = (can_id >> 26) & 0x07;
    pgn = (can_id >> 8) & 0x3FFFF;
    source_address = can_id & 0xFF;
    
    if ((pgn & 0xFF00) < 0xF000) {
        destination_address = pgn & 0xFF;
        pgn = pgn & 0xFF00;
    } else {
        destination_address = 0xFF; // Broadcast
    }
}

// J1939Protocol method implementations
bool J1939Protocol::decode_engine_data(const CanFrame& frame, j1939::EngineData& engine_data) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    j1939::J1939Header header;
    header.from_can_id(frame.id);

    switch (header.pgn) {
        case j1939::pgn::ENGINE_SPEED_LOAD: {
            uint16_t raw_speed = extract_uint16_le(frame.data.data(), 3);
            engine_data.engine_speed_rpm = raw_speed * 0.125f;
            engine_data.engine_load_percent = frame.data[2] * 0.5f;
            engine_data.data_valid = true;
            return true;
        }
        case j1939::pgn::ACCELERATOR_PEDAL_POSITION: {
            engine_data.accelerator_pedal_percent = frame.data[1] * 0.4f;
            engine_data.data_valid = true;
            return true;
        }
        default:
            return false;
    }
}

bool J1939Protocol::decode_vehicle_data(const CanFrame& frame, j1939::VehicleData& vehicle_data) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    j1939::J1939Header header;
    header.from_can_id(frame.id);

    if (header.pgn == j1939::pgn::VEHICLE_SPEED) {
        uint16_t raw_speed = extract_uint16_le(frame.data.data(), 1);
        float speed_kmh = raw_speed / 256.0f;
        vehicle_data.vehicle_speed_mps = speed_kmh / 3.6f;
        vehicle_data.data_valid = true;
        return true;
    }

    return false;
}

bool J1939Protocol::decode_cvt_status(const CanFrame& frame, j1939::CvtStatusReport& cvt_status) {
    if (!frame.is_extended || frame.dlc < 8) {
        return false;
    }

    j1939::J1939Header header;
    header.from_can_id(frame.id);

    if (header.pgn == j1939::pgn::CVT_STATUS_REPORT) {
        uint32_t raw_ratio;
        std::memcpy(&raw_ratio, frame.data.data(), sizeof(float));
        std::memcpy(&cvt_status.current_ratio, &raw_ratio, sizeof(float));
        cvt_status.is_shifting = (frame.data[4] & 0x01) != 0;
        cvt_status.fault_code = frame.data[5];
        cvt_status.data_valid = true;
        return true;
    }

    return false;
}

CanFrame J1939Protocol::encode_cvt_control_command(const j1939::CvtControlCommand& command, uint8_t source_address) {
    CanFrame frame;
    frame.is_extended = true;
    frame.dlc = 8;

    j1939::J1939Header header;
    header.priority = 3;
    header.pgn = j1939::pgn::CVT_CONTROL_COMMAND;
    header.source_address = source_address;
    header.destination_address = 0xFF;

    frame.id = header.to_can_id();

    uint32_t raw_ratio;
    std::memcpy(&raw_ratio, &command.target_ratio, sizeof(float));
    insert_uint32_le(frame.data.data(), 0, raw_ratio);

    frame.data[4] = command.drive_mode;
    frame.data[5] = command.enable_control ? 0x01 : 0x00;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFF;

    return frame;
}

bool J1939Protocol::is_supported_message(const CanFrame& frame) {
    if (!frame.is_extended) {
        return false;
    }

    j1939::J1939Header header;
    header.from_can_id(frame.id);

    switch (header.pgn) {
        case j1939::pgn::ENGINE_SPEED_LOAD:
        case j1939::pgn::VEHICLE_SPEED:
        case j1939::pgn::TRANSMISSION_FLUIDS:
        case j1939::pgn::ELECTRONIC_TRANSMISSION_CONTROLLER_1:
        case j1939::pgn::ACCELERATOR_PEDAL_POSITION:
        case j1939::pgn::CVT_CONTROL_COMMAND:
        case j1939::pgn::CVT_STATUS_REPORT:
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

} // namespace can
} // namespace vcu

