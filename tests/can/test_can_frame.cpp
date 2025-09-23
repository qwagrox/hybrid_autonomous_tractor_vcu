#include <gtest/gtest.h>
#include "vcu/can/can_frame.h"

namespace vcu {
namespace can {
namespace test {

class CanFrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up a standard test frame
        test_frame_.id = 0x123;
        test_frame_.dlc = 8;
        test_frame_.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        test_frame_.is_extended = false;
        test_frame_.is_error = false;
        test_frame_.is_rtr = false;
    }

    CanFrame test_frame_;
};

TEST_F(CanFrameTest, DefaultConstruction) {
    CanFrame frame;
    EXPECT_EQ(frame.id, 0u);
    EXPECT_EQ(frame.dlc, 0u);
    EXPECT_FALSE(frame.is_extended);
    EXPECT_FALSE(frame.is_error);
    EXPECT_FALSE(frame.is_rtr);
    
    // Check that data array is zero-initialized
    for (size_t i = 0; i < frame.data.size(); ++i) {
        EXPECT_EQ(frame.data[i], 0u);
    }
}

TEST_F(CanFrameTest, StandardFrameToString) {
    std::string frame_str = test_frame_.to_string();
    EXPECT_NE(frame_str.find("ID: 291"), std::string::npos); // 0x123 = 291
    EXPECT_NE(frame_str.find("DLC: 8"), std::string::npos);
    EXPECT_NE(frame_str.find("01 02 03 04 05 06 07 08"), std::string::npos);
}

TEST_F(CanFrameTest, ExtendedFrameToString) {
    test_frame_.id = 0x18FF1234;
    test_frame_.is_extended = true;
    test_frame_.dlc = 4;
    test_frame_.data = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00};
    
    std::string frame_str = test_frame_.to_string();
    EXPECT_NE(frame_str.find("ID: 419369524"), std::string::npos); // 0x18FF1234 = 419369524
    EXPECT_NE(frame_str.find("DLC: 4"), std::string::npos);
    EXPECT_NE(frame_str.find("AA BB CC DD"), std::string::npos);
}

TEST_F(CanFrameTest, EmptyFrameToString) {
    CanFrame empty_frame;
    std::string frame_str = empty_frame.to_string();
    EXPECT_NE(frame_str.find("ID: 0"), std::string::npos);
    EXPECT_NE(frame_str.find("DLC: 0"), std::string::npos);
    EXPECT_EQ(frame_str.find("Data:"), frame_str.length() - 5); // Should end with "Data:"
}

TEST_F(CanFrameTest, SingleByteFrameToString) {
    test_frame_.dlc = 1;
    test_frame_.data[0] = 0xFF;
    
    std::string frame_str = test_frame_.to_string();
    EXPECT_NE(frame_str.find("DLC: 1"), std::string::npos);
    EXPECT_NE(frame_str.find("FF"), std::string::npos);
    EXPECT_EQ(frame_str.find("02"), std::string::npos); // Should not contain second byte
}

TEST_F(CanFrameTest, MaxDlcFrame) {
    test_frame_.dlc = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        test_frame_.data[i] = i;
    }
    
    std::string frame_str = test_frame_.to_string();
    EXPECT_NE(frame_str.find("00 01 02 03 04 05 06 07"), std::string::npos);
}

TEST_F(CanFrameTest, FrameFlags) {
    // Test extended frame flag
    test_frame_.is_extended = true;
    EXPECT_TRUE(test_frame_.is_extended);
    
    // Test error frame flag
    test_frame_.is_error = true;
    EXPECT_TRUE(test_frame_.is_error);
    
    // Test RTR frame flag
    test_frame_.is_rtr = true;
    EXPECT_TRUE(test_frame_.is_rtr);
}

TEST_F(CanFrameTest, DataArrayAccess) {
    // Test direct array access
    test_frame_.data[0] = 0xAA;
    test_frame_.data[7] = 0xBB;
    
    EXPECT_EQ(test_frame_.data[0], 0xAA);
    EXPECT_EQ(test_frame_.data[7], 0xBB);
}

TEST_F(CanFrameTest, CopyConstruction) {
    CanFrame copied_frame = test_frame_;
    
    EXPECT_EQ(copied_frame.id, test_frame_.id);
    EXPECT_EQ(copied_frame.dlc, test_frame_.dlc);
    EXPECT_EQ(copied_frame.is_extended, test_frame_.is_extended);
    EXPECT_EQ(copied_frame.is_error, test_frame_.is_error);
    EXPECT_EQ(copied_frame.is_rtr, test_frame_.is_rtr);
    
    for (size_t i = 0; i < 8; ++i) {
        EXPECT_EQ(copied_frame.data[i], test_frame_.data[i]);
    }
}

TEST_F(CanFrameTest, Assignment) {
    CanFrame assigned_frame;
    assigned_frame = test_frame_;
    
    EXPECT_EQ(assigned_frame.id, test_frame_.id);
    EXPECT_EQ(assigned_frame.dlc, test_frame_.dlc);
    EXPECT_EQ(assigned_frame.is_extended, test_frame_.is_extended);
    EXPECT_EQ(assigned_frame.is_error, test_frame_.is_error);
    EXPECT_EQ(assigned_frame.is_rtr, test_frame_.is_rtr);
    
    for (size_t i = 0; i < 8; ++i) {
        EXPECT_EQ(assigned_frame.data[i], test_frame_.data[i]);
    }
}

} // namespace test
} // namespace can
} // namespace vcu
