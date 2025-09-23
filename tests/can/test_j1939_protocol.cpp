#include <gtest/gtest.h>
#include "vcu/can/j1939_protocol.h"
#include <cstring>

namespace vcu {
namespace can {
namespace j1939 {
namespace test {

class J1939ProtocolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a test engine speed/load frame (EEC1)
        engine_frame_.id = 0x0CF00400; // Priority 3, PGN 0xF004, Source 0x00
        engine_frame_.is_extended = true;
        engine_frame_.dlc = 8;
        engine_frame_.data = {0x00, 0x00, 0x7D, 0x00, 0x20, 0x00, 0x00, 0x00};
        // Byte 2: Engine load = 125 * 0.5% = 62.5%
        // Byte 3-4: Engine speed = 0x2000 * 0.125 = 4096 RPM

        // Create a test vehicle speed frame (CCVS1)
        vehicle_frame_.id = 0x18FEF100; // Priority 6, PGN 0xFEF1, Source 0x00
        vehicle_frame_.is_extended = true;
        vehicle_frame_.dlc = 8;
        vehicle_frame_.data = {0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
        // Byte 1-2: Vehicle speed = 0x1000 / 256 = 64 km/h = 17.78 m/s
    }

    CanFrame engine_frame_;
    CanFrame vehicle_frame_;
};

TEST_F(J1939ProtocolTest, J1939HeaderConstruction) {
    J1939Header header;
    header.priority = 3;
    header.pgn = 0xF004;
    header.source_address = 0x10;
    header.destination_address = 0xFF;

    uint32_t can_id = header.to_can_id();
    EXPECT_EQ(can_id, 0x0CF00410u); // Priority 3, PGN F004, Source 0x10
}

TEST_F(J1939ProtocolTest, J1939HeaderParsing) {
    uint32_t can_id = 0x18FEF100; // Priority 6, PGN FEF1, Source 0x00
    
    J1939Header header;
    header.from_can_id(can_id);
    
    EXPECT_EQ(header.priority, 6u);
    EXPECT_EQ(header.pgn, 0xFEF1u);
    EXPECT_EQ(header.source_address, 0x00u);
    EXPECT_EQ(header.destination_address, 0xFFu); // Broadcast
}

TEST_F(J1939ProtocolTest, DecodeEngineSpeedLoad) {
    EngineData engine_data;
    bool result = J1939Protocol::decode_engine_data(engine_frame_, engine_data);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(engine_data.data_valid);
    EXPECT_FLOAT_EQ(engine_data.engine_speed_rpm, 1024.0f); // 0x2000 (little-endian) = 8192 * 0.125 = 1024
    EXPECT_FLOAT_EQ(engine_data.engine_load_percent, 62.5f); // 125 * 0.5
}

TEST_F(J1939ProtocolTest, DecodeAcceleratorPedal) {
    CanFrame pedal_frame;
    pedal_frame.id = 0x0CF00200; // Priority 3, PGN 0xF002 (EEC2), Source 0x00
    pedal_frame.is_extended = true;
    pedal_frame.dlc = 8;
    pedal_frame.data = {0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // Byte 1: Accelerator pedal = 200 * 0.4% = 80%

    EngineData engine_data;
    bool result = J1939Protocol::decode_engine_data(pedal_frame, engine_data);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(engine_data.data_valid);
    EXPECT_FLOAT_EQ(engine_data.accelerator_pedal_percent, 80.0f);
}

TEST_F(J1939ProtocolTest, DecodeVehicleSpeed) {
    VehicleData vehicle_data;
    bool result = J1939Protocol::decode_vehicle_data(vehicle_frame_, vehicle_data);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(vehicle_data.data_valid);
    EXPECT_NEAR(vehicle_data.vehicle_speed_mps, 4.44f, 0.01f); // 0x1000 / 256 = 16 km/h = 4.44 m/s
}

TEST_F(J1939ProtocolTest, DecodeCvtStatus) {
    CanFrame cvt_frame;
    cvt_frame.id = 0x0CFF1100; // Priority 3, PGN 0x00FF11, Source 0x00
    cvt_frame.is_extended = true;
    cvt_frame.dlc = 8;
    
    // Encode a float ratio of 1.5 in little-endian format
    float test_ratio = 1.5f;
    std::memcpy(cvt_frame.data.data(), &test_ratio, sizeof(float));
    cvt_frame.data[4] = 0x01; // is_shifting = true
    cvt_frame.data[5] = 0x02; // fault_code = 2

    CvtStatusReport cvt_status;
    bool result = J1939Protocol::decode_cvt_status(cvt_frame, cvt_status);
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(cvt_status.data_valid);
    EXPECT_FLOAT_EQ(cvt_status.current_ratio, 1.5f);
    EXPECT_TRUE(cvt_status.is_shifting);
    EXPECT_EQ(cvt_status.fault_code, 2u);
}

TEST_F(J1939ProtocolTest, EncodeCvtControlCommand) {
    CvtControlCommand command;
    command.target_ratio = 2.0f;
    command.drive_mode = 1; // Plowing
    command.enable_control = true;

    CanFrame frame = J1939Protocol::encode_cvt_control_command(command, 0x10);
    
    EXPECT_TRUE(frame.is_extended);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.id, 0x0CFF1010u); // Priority 3, PGN 0x00FF10, Source 0x10
    
    // Decode the ratio back
    float decoded_ratio;
    std::memcpy(&decoded_ratio, frame.data.data(), sizeof(float));
    EXPECT_FLOAT_EQ(decoded_ratio, 2.0f);
    
    EXPECT_EQ(frame.data[4], 1u); // drive_mode
    EXPECT_EQ(frame.data[5], 0x01u); // enable_control = true
    EXPECT_EQ(frame.data[6], 0xFFu); // Reserved
    EXPECT_EQ(frame.data[7], 0xFFu); // Reserved
}

TEST_F(J1939ProtocolTest, IsSupportedMessage) {
    // Test supported messages
    EXPECT_TRUE(J1939Protocol::is_supported_message(engine_frame_));
    EXPECT_TRUE(J1939Protocol::is_supported_message(vehicle_frame_));
    
    // Test unsupported message
    CanFrame unsupported_frame;
    unsupported_frame.id = 0x18FFFF00; // Unknown PGN
    unsupported_frame.is_extended = true;
    EXPECT_FALSE(J1939Protocol::is_supported_message(unsupported_frame));
    
    // Test standard (non-extended) frame
    CanFrame standard_frame;
    standard_frame.id = 0x123;
    standard_frame.is_extended = false;
    EXPECT_FALSE(J1939Protocol::is_supported_message(standard_frame));
}

TEST_F(J1939ProtocolTest, InvalidFrameDecoding) {
    // Test with non-extended frame
    CanFrame invalid_frame;
    invalid_frame.id = 0x123;
    invalid_frame.is_extended = false;
    invalid_frame.dlc = 8;

    EngineData engine_data;
    VehicleData vehicle_data;
    CvtStatusReport cvt_status;
    
    EXPECT_FALSE(J1939Protocol::decode_engine_data(invalid_frame, engine_data));
    EXPECT_FALSE(J1939Protocol::decode_vehicle_data(invalid_frame, vehicle_data));
    EXPECT_FALSE(J1939Protocol::decode_cvt_status(invalid_frame, cvt_status));
}

TEST_F(J1939ProtocolTest, InsufficientDataLength) {
    // Test with insufficient DLC
    engine_frame_.dlc = 4; // Too short for EEC1 message
    
    EngineData engine_data;
    bool result = J1939Protocol::decode_engine_data(engine_frame_, engine_data);
    
    EXPECT_FALSE(result);
    EXPECT_FALSE(engine_data.data_valid);
}

TEST_F(J1939ProtocolTest, UnknownPgn) {
    // Test with unknown PGN
    engine_frame_.id = 0x18FFFF00; // Unknown PGN
    
    EngineData engine_data;
    bool result = J1939Protocol::decode_engine_data(engine_frame_, engine_data);
    
    EXPECT_FALSE(result);
}

TEST_F(J1939ProtocolTest, ByteOrderHandling) {
    // Test little-endian byte order handling
    CanFrame test_frame;
    test_frame.id = 0x0CF00400; // EEC1
    test_frame.is_extended = true;
    test_frame.dlc = 8;
    test_frame.data = {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    // Engine speed = 0x0100 (little-endian) = 256 * 0.125 = 32 RPM

    EngineData engine_data;
    bool result = J1939Protocol::decode_engine_data(test_frame, engine_data);
    
    EXPECT_TRUE(result);
    EXPECT_FLOAT_EQ(engine_data.engine_speed_rpm, 0.125f); // 0x0100 (little-endian) = 1 * 0.125 = 0.125
}

} // namespace test
} // namespace j1939
} // namespace can
} // namespace vcu
