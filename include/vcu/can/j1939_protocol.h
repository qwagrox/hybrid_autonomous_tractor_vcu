#ifndef J1939_PROTOCOL_H
#define J1939_PROTOCOL_H

#include "can_frame.h"
#include <cstdint>

namespace vcu {
namespace can {

// Forward declaration
class J1939Protocol;

namespace j1939 {

namespace pgn {
    constexpr uint32_t ENGINE_SPEED_LOAD = 0x00F004;
    constexpr uint32_t VEHICLE_SPEED = 0x00FEF1;
    constexpr uint32_t TRANSMISSION_FLUIDS = 0x00FEEE;
    constexpr uint32_t ELECTRONIC_TRANSMISSION_CONTROLLER_1 = 0x00F003;
    constexpr uint32_t ACCELERATOR_PEDAL_POSITION = 0x00F002;
    constexpr uint32_t CVT_CONTROL_COMMAND = 0x00FF10;
    constexpr uint32_t CVT_STATUS_REPORT = 0x00FF11;
}

struct J1939Header {
    uint8_t priority = 6;
    uint32_t pgn = 0;
    uint8_t source_address = 0;
    uint8_t destination_address = 0xFF;
    uint32_t to_can_id() const;
    void from_can_id(uint32_t can_id);
};

struct EngineData {
    float engine_speed_rpm = 0.0f;
    float engine_load_percent = 0.0f;
    float accelerator_pedal_percent = 0.0f;
    bool data_valid = false;
};

struct VehicleData {
    float vehicle_speed_mps = 0.0f;
    bool data_valid = false;
};

struct CvtControlCommand {
    float target_ratio = 1.0f;
    uint8_t drive_mode = 0;
    bool enable_control = true;
};

struct CvtStatusReport {
    float current_ratio = 1.0f;
    bool is_shifting = false;
    uint8_t fault_code = 0;
    bool data_valid = false;
};

} // namespace j1939

class J1939Protocol {
public:
    static bool decode_engine_data(const CanFrame& frame, j1939::EngineData& engine_data);
    static bool decode_vehicle_data(const CanFrame& frame, j1939::VehicleData& vehicle_data);
    static bool decode_cvt_status(const CanFrame& frame, j1939::CvtStatusReport& cvt_status);
    static CanFrame encode_cvt_control_command(const j1939::CvtControlCommand& command, uint8_t source_address = 0x10);
    static bool is_supported_message(const CanFrame& frame);

private:
    static uint16_t extract_uint16_le(const uint8_t* data, size_t offset);
    static uint32_t extract_uint32_le(const uint8_t* data, size_t offset);
    static void insert_uint32_le(uint8_t* data, size_t offset, uint32_t value);
};

} // namespace can
} // namespace vcu

#endif // J1939_PROTOCOL_H
