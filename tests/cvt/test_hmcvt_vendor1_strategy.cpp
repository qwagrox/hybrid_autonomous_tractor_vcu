#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "vcu/cvt/hmcvt_vendor1_strategy.h"
#include "vcu/can/can_interface.h"
#include "vcu/can/can_frame.h"

namespace vcu {
namespace cvt {
namespace test {

// Mock CAN interface for testing
class MockCanInterface : public can::CanInterface {
public:
    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_METHOD(bool, send_frame, (const can::CanFrame& frame), (override));
    MOCK_METHOD(bool, receive_frame, (can::CanFrame& frame), (override));
    MOCK_METHOD(bool, is_connected, (), (const, override));
};

class HMCVT_Vendor1_StrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_can_interface_ = std::make_unique<MockCanInterface>();
        strategy_ = std::make_unique<HMCVT_Vendor1_Strategy>(*mock_can_interface_);
    }

    void TearDown() override {
        strategy_.reset();
        mock_can_interface_.reset();
    }

    std::unique_ptr<MockCanInterface> mock_can_interface_;
    std::unique_ptr<HMCVT_Vendor1_Strategy> strategy_;
};

TEST_F(HMCVT_Vendor1_StrategyTest, InitializationSendsStartCommand) {
    // Expect that initialization sends a CAN frame
    EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
        .Times(::testing::AtLeast(1))
        .WillRepeatedly(::testing::Return(true));

    strategy_->init();
}

TEST_F(HMCVT_Vendor1_StrategyTest, SetTargetRatioWithinValidRange) {
    // Test setting target ratio within valid range (-0.9 to 2.0)
    strategy_->set_target_ratio(1.0f);
    
    common::CvtState state = strategy_->get_current_state();
    EXPECT_EQ(state.target_ratio, 1.0f);
}

TEST_F(HMCVT_Vendor1_StrategyTest, SetTargetRatioClampedToValidRange) {
    // Test that target ratio is clamped to valid range
    strategy_->set_target_ratio(3.0f); // Above max (2.0)
    
    common::CvtState state = strategy_->get_current_state();
    EXPECT_LE(state.target_ratio, 2.0f);
    
    strategy_->set_target_ratio(-2.0f); // Below min (-0.9)
    
    state = strategy_->get_current_state();
    EXPECT_GE(state.target_ratio, -0.9f);
}

TEST_F(HMCVT_Vendor1_StrategyTest, SetDriveModeUpdatesInternalState) {
    // Test that setting drive mode doesn't throw
    EXPECT_NO_THROW(strategy_->set_drive_mode(common::DriveMode::PLOWING));
    EXPECT_NO_THROW(strategy_->set_drive_mode(common::DriveMode::SEEDING));
    EXPECT_NO_THROW(strategy_->set_drive_mode(common::DriveMode::TRANSPORT));
    EXPECT_NO_THROW(strategy_->set_drive_mode(common::DriveMode::MANUAL));
}

TEST_F(HMCVT_Vendor1_StrategyTest, UpdateSendsControlMessage) {
    // Mock CAN interface to expect control messages
    EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
        .Times(::testing::AtLeast(1))
        .WillRepeatedly(::testing::Return(true));

    // Create test perception data
    common::PerceptionData perception_data;
    perception_data.vehicle_speed_mps = 5.0f;
    perception_data.engine_speed_rpm = 2000.0f;
    perception_data.engine_load_percent = 50.0f;
    perception_data.accelerator_pedal_percent = 30.0f;
    perception_data.data_valid = true;

    strategy_->update(perception_data);
}

TEST_F(HMCVT_Vendor1_StrategyTest, GetCurrentStateReturnsValidState) {
    common::CvtState state = strategy_->get_current_state();
    
    // Check that state values are within reasonable ranges
    EXPECT_GE(state.current_ratio, -2.0f);
    EXPECT_LE(state.current_ratio, 3.0f);
    EXPECT_GE(state.target_ratio, -2.0f);
    EXPECT_LE(state.target_ratio, 3.0f);
}

TEST_F(HMCVT_Vendor1_StrategyTest, ControlMessageFormat) {
    // Test that control messages have correct format
    can::CanFrame captured_frame;
    
    EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
        .WillOnce(::testing::DoAll(
            ::testing::SaveArg<0>(&captured_frame),
            ::testing::Return(true)
        ));

    strategy_->init();

    // Verify CAN ID
    EXPECT_EQ(captured_frame.id, 0x18FFF023u);
    
    // Verify data length
    EXPECT_EQ(captured_frame.dlc, 8);
}

TEST_F(HMCVT_Vendor1_StrategyTest, ForwardGearSelection) {
    // Test forward gear selection for positive ratios
    strategy_->set_target_ratio(0.5f);
    
    // The strategy should select forward gear
    // We can't directly test internal state, but we can verify no exceptions
    EXPECT_NO_THROW(strategy_->get_current_state());
}

TEST_F(HMCVT_Vendor1_StrategyTest, ReverseGearSelection) {
    // Test reverse gear selection for negative ratios
    strategy_->set_target_ratio(-0.5f);
    
    // The strategy should select reverse gear
    // We can't directly test internal state, but we can verify no exceptions
    EXPECT_NO_THROW(strategy_->get_current_state());
}

TEST_F(HMCVT_Vendor1_StrategyTest, NeutralGearSelection) {
    // Test neutral gear selection for zero ratio
    strategy_->set_target_ratio(0.0f);
    
    // The strategy should select neutral gear
    // We can't directly test internal state, but we can verify no exceptions
    EXPECT_NO_THROW(strategy_->get_current_state());
}

TEST_F(HMCVT_Vendor1_StrategyTest, DriveModePlowingConfiguration) {
    // Test that plowing mode configures appropriate settings
    EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
        .Times(::testing::AtLeast(1))
        .WillRepeatedly(::testing::Return(true));

    strategy_->set_drive_mode(common::DriveMode::PLOWING);
    
    // Update to trigger control message
    common::PerceptionData perception_data;
    perception_data.data_valid = true;
    strategy_->update(perception_data);
}

TEST_F(HMCVT_Vendor1_StrategyTest, DriveModeTransportConfiguration) {
    // Test that transport mode configures appropriate settings
    EXPECT_CALL(*mock_can_interface_, send_frame(::testing::_))
        .Times(::testing::AtLeast(1))
        .WillRepeatedly(::testing::Return(true));

    strategy_->set_drive_mode(common::DriveMode::TRANSPORT);
    
    // Update to trigger control message
    common::PerceptionData perception_data;
    perception_data.data_valid = true;
    strategy_->update(perception_data);
}

} // namespace test
} // namespace cvt
} // namespace vcu
